//! BLE HID consumer-control business card firmware for the nRF52810.
//!
//! An AT42QT1070 touch controller provides 6 keys which are sent as HID
//! consumer control usages (volume, play/pause, ...) over HID-over-GATT.
//!
//! Bond information (including the peer IRK, required so centrals using
//! resolvable private addresses can reconnect) and the per-peer CCCD state
//! are persisted in internal flash via the MPSL-timeslot-safe Flash driver.
//! Up to [`MAX_BONDS`] peers are remembered; the least recently used bond is
//! evicted when a new peer pairs.
//!
//! Build with `just build`, flash with `just run` (see justfile).
#![no_std]
#![no_main]

use core::ops::Range;

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, rng};
use embassy_time::{Duration, Timer};
use nrf_sdc::mpsl::{Flash, MultiprotocolServiceLayer};
use nrf_sdc::{self as sdc, mpsl};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use trouble_host::config::CLIENT_ATT_TABLE_SIZE;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

extern "C" {
    static __storage_start: u8;
    static __storage_end: u8;
}

// The peripheral-only SoftDevice Controller library does not provide these
// central/scan HCI commands, but trouble's runner references them in code
// paths never taken by a legacy-advertising peripheral (cancelling a central
// connect, disabling ext advertising/scanning, central-initiated encryption).
// Stubs returning "Unknown HCI Command" (0x01) satisfy the linker without
// pulling in the much larger multirole controller library.
#[no_mangle]
extern "C" fn sdc_hci_cmd_le_create_conn_cancel() -> u8 {
    0x01
}
#[no_mangle]
extern "C" fn sdc_hci_cmd_le_enable_encryption(_params: *const u8) -> u8 {
    0x01
}
#[no_mangle]
extern "C" fn sdc_hci_cmd_le_set_scan_enable(_params: *const u8) -> u8 {
    0x01
}
#[no_mangle]
extern "C" fn sdc_hci_cmd_le_set_ext_scan_enable(_params: *const u8) -> u8 {
    0x01
}
#[no_mangle]
extern "C" fn sdc_hci_cmd_le_set_ext_adv_enable(_params: *const u8) -> u8 {
    0x01
}

fn storage_range() -> Range<u32> {
    unsafe {
        let start = &__storage_start as *const u8 as u32;
        let end = &__storage_end as *const u8 as u32;
        start..end
    }
}

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TWI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWI0>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2;

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 4;

/// Name used in the GAP service and advertising data
const DEVICE_NAME: &str = "Systemscape";

// AT42QT1070 I2C address
const QT1070_ADDR: u8 = 0x1B;

// AT42QT1070 registers
const QT1070_REG_DET_STATUS: u8 = 0x02;

// Consumer Control HID Report Descriptor: single 16-bit usage value for media keys
static REPORT_MAP: [u8; 23] = [
    0x05, 0x0C, // Usage Page (Consumer)
    0x09, 0x01, // Usage (Consumer Control)
    0xA1, 0x01, // Collection (Application)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x19, 0x00, //   Usage Minimum (0)
    0x2A, 0xFF, 0x03, //   Usage Maximum (1023)
    0x75, 0x10, //   Report Size (16)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x00, //   Input (Data,Array,Abs)
    0xC0, // End Collection
];

// Consumer Control usage codes
const CC_VOLUME_UP: u16 = 0x00E9;
const CC_VOLUME_DOWN: u16 = 0x00EA;
const CC_PLAY_PAUSE: u16 = 0x00CD;
const CC_SCAN_NEXT: u16 = 0x00B5;
const CC_MUTE: u16 = 0x00E2;
const CC_SCAN_PREV: u16 = 0x00B6;

fn key_to_usage(key: u8) -> u16 {
    match key {
        0 => CC_VOLUME_UP,
        1 => CC_VOLUME_DOWN,
        2 => CC_PLAY_PAUSE,
        3 => CC_SCAN_NEXT,
        4 => CC_MUTE,
        5 => CC_SCAN_PREV,
        _ => 0,
    }
}

// GATT Server definition. DIS and battery service are required by HOGP but
// never touched by the application, hence the underscore names.
#[gatt_server]
struct Server {
    _dis: DeviceInformationService,
    _battery_service: BatteryService,
    hid_service: HidService,
}

#[gatt_service(uuid = service::DEVICE_INFORMATION)]
struct DeviceInformationService {
    // PnP ID: vendor source 0x02 (USB-IF), vendor 0x1915 (Nordic), product 0x0001, version 0x0001
    #[characteristic(uuid = "2a50", read, value = [0x02, 0x15, 0x19, 0x01, 0x00, 0x01, 0x00])]
    pnp_id: [u8; 7],
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, value = 100)]
    level: u8,
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
struct HidService {
    // bcdHID 1.11, country code 0, flags: normally connectable
    #[characteristic(uuid = "2a4a", read, value = [0x11, 0x01, 0x00, 0x02], permissions(encrypted))]
    hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = REPORT_MAP, permissions(encrypted))]
    report_map: [u8; 23],
    #[characteristic(uuid = "2a4c", write_without_response, permissions(encrypted))]
    hid_control_point: u8,
    #[characteristic(
        uuid = "2a4e",
        read,
        write_without_response,
        value = 1,
        permissions(encrypted)
    )]
    protocol_mode: u8,
    // Report Reference descriptor: report ID 0, type Input
    #[descriptor(uuid = "2908", read = encrypted, value = [0x00, 0x01])]
    #[characteristic(uuid = "2a4d", read, notify, permissions(encrypted))]
    input_report: [u8; 2],
}

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(27, 27, L2CAP_TXQ, L2CAP_RXQ)?
        .build(p, rng, mpsl, mem)
}

/// Number of bonded peers remembered across power cycles.
const MAX_BONDS: usize = 4;

#[derive(Serialize, Deserialize)]
struct BondEntry {
    bond: BondInformation,
    cccd: heapless::Vec<u8, CLIENT_ATT_TABLE_SIZE>,
}

/// Bond table ordered most-recently-used first.
#[derive(Serialize, Deserialize, Default)]
struct BondTable(heapless::Vec<BondEntry, MAX_BONDS>);
impl<'a> PostcardValue<'a> for BondTable {}

const BONDS_KEY: u8 = 0;

impl BondTable {
    fn find(&mut self, identity: &Identity) -> Option<&mut BondEntry> {
        self.0
            .iter_mut()
            .find(|e| e.bond.identity.match_identity(identity))
    }

    /// Move the entry for `identity` to the front (most recently used).
    fn touch(&mut self, identity: &Identity) {
        if let Some(pos) = self
            .0
            .iter()
            .position(|e| e.bond.identity.match_identity(identity))
        {
            if pos != 0 {
                let entry = self.0.remove(pos);
                let _ = self.0.insert(0, entry);
            }
        }
    }

    /// Insert or update a bond, evicting the least recently used entry if full.
    fn upsert(&mut self, bond: BondInformation) {
        if let Some(pos) = self
            .0
            .iter()
            .position(|e| e.bond.identity.match_identity(&bond.identity))
        {
            let mut entry = self.0.remove(pos);
            entry.bond = bond;
            let _ = self.0.insert(0, entry);
        } else {
            if self.0.is_full() {
                let evicted = self.0.pop();
                if let Some(evicted) = evicted {
                    info!("[bond] table full, evicting {:?}", evicted.bond.identity);
                }
            }
            let _ = self.0.insert(
                0,
                BondEntry {
                    bond,
                    cccd: heapless::Vec::new(),
                },
            );
        }
    }
}

type Storage<'a> = MapStorage<u8, &'a mut Flash<'a>, NoCache>;

async fn save_bonds(storage: &mut Storage<'_>, buffer: &mut [u8], bonds: &BondTable) {
    match storage.store_item(buffer, &BONDS_KEY, bonds).await {
        Ok(()) => info!("[bond] table stored ({} entries)", bonds.0.len()),
        Err(_) => warn!("[bond] table store failed"),
    }
}

fn paint_stack() {
    extern "C" {
        static mut __sheap: u8;
    }
    unsafe {
        let sp: u32;
        core::arch::asm!("mov {}, sp", out(reg) sp);
        let mut p = core::ptr::addr_of_mut!(__sheap) as *mut u32;
        let end = (sp - 128) as *mut u32;
        while p < end {
            p.write_volatile(0x5757_5757);
            p = p.add(1);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    paint_stack();
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    let p = embassy_nrf::init(config);

    // Blue LED on P0.00 (active low)
    let mut led = Output::new(p.P0_00, Level::High, OutputDrive::Standard);

    // AT42QT1070 NRST on P0.12 - reset the touch controller
    let mut qt_nrst = Output::new(p.P0_12, Level::Low, OutputDrive::Standard);
    Timer::after(Duration::from_millis(10)).await;
    qt_nrst.set_high();
    // AT42QT1070 needs ~240ms calibration time after reset
    Timer::after(Duration::from_millis(300)).await;

    // AT42QT1070 NCHANGE on P0.10 (active low, signals key state change)
    let mut qt_change = Input::new(p.P0_10, Pull::Up);

    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static SESSION_MEM: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::with_timeslots(
        mpsl_p,
        Irqs,
        lfclk_cfg,
        SESSION_MEM.init(mpsl::SessionMem::new())
    )));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    static SDC_MEM: StaticCell<sdc::Mem<1560>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, sdc_mem));

    // Use internal flash (MPSL timeslot safe) for bond storage
    static FLASH: StaticCell<Flash> = StaticCell::new();
    let flash = FLASH.init(Flash::take(mpsl, p.NVMC));
    let mut map_storage: Storage =
        MapStorage::new(flash, MapConfig::new(storage_range()), NoCache::new());
    let mut data_buffer = [0; 640];

    // Init I2C after MPSL/SDC setup to avoid interrupt conflicts
    let twim_config = twim::Config::default();
    static TWIM_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let twim_buf = TWIM_BUF.init([0u8; 32]);
    let mut i2c = Twim::new(p.TWI0, Irqs, p.P0_15, p.P0_14, twim_config, twim_buf);

    // TWIM driver enables TWI0 interrupt at default P0 priority (same as MPSL's
    // RADIO/TIMER0/RTC0). Lower it to avoid interfering with radio timing.
    {
        use embassy_nrf::interrupt::typelevel::Interrupt as _;
        embassy_nrf::interrupt::typelevel::TWI0::set_priority(embassy_nrf::interrupt::Priority::P3);
    }

    let mut chip_id = [0u8; 1];
    match i2c.write_read(QT1070_ADDR, &[0x00], &mut chip_id).await {
        Ok(_) => info!("AT42QT1070 chip ID: {:#04x}", chip_id[0]),
        Err(e) => warn!("AT42QT1070 read failed: {:?}", defmt::Debug2Format(&e)),
    }

    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xfe]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<
        DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        1,
        MAX_BONDS,
    > = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(address)
        .set_io_capabilities(IoCapabilities::NoInputNoOutput)
        .build();

    let mut bonds: BondTable = match map_storage.fetch_item(&mut data_buffer, &BONDS_KEY).await {
        Ok(Some(table)) => table,
        _ => BondTable::default(),
    };
    info!("[bond] {} bond(s) loaded from flash", bonds.0.len());
    for entry in bonds.0.iter() {
        unwrap!(stack.add_bond_information(entry.bond.clone()));
    }

    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    info!("Starting advertising and GATT service");
    let server = unwrap!(Server::new_with_config(GapConfig::Peripheral(
        PeripheralConfig {
            name: DEVICE_NAME,
            appearance: &appearance::human_interface_device::KEYBOARD,
        }
    )));

    // Flash LED to indicate ready
    led.set_low();
    Timer::after(Duration::from_millis(200)).await;
    led.set_high();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(DEVICE_NAME, &mut peripheral, &server).await {
                Ok(conn) => {
                    if let Err(e) = conn.raw().set_bondable(true) {
                        warn!("Failed to set bondable: {:?}", e);
                    }
                    // Request encryption — prompts centrals to pair or use the stored LTK
                    if let Err(e) = conn.raw().request_security() {
                        warn!("Security request failed: {:?}", e);
                    }

                    // LED on while connected
                    led.set_low();
                    connection_task(
                        &server,
                        &conn,
                        &mut map_storage,
                        &mut data_buffer,
                        &mut bonds,
                        &mut i2c,
                        &mut qt_change,
                    )
                    .await;
                    led.set_high();
                }
                Err(e) => {
                    warn!("[adv] error: {:?}", defmt::Debug2Format(&e));
                    Timer::after(Duration::from_millis(500)).await;
                }
            }
        }
    })
    .await;
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            warn!("[ble_task] error: {:?}", defmt::Debug2Format(&e));
        }
    }
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::IncompleteServiceUuids16(&[service::HUMAN_INTERFACE_DEVICE.to_le_bytes()]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

/// Handle GATT events and touch input until the connection closes.
async fn connection_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    map_storage: &mut Storage<'_>,
    data_buffer: &mut [u8],
    bonds: &mut BondTable,
    i2c: &mut Twim<'_>,
    change_pin: &mut Input<'_>,
) {
    let input_report = &server.hid_service.input_report;
    let mut active_identity: Option<Identity> = None;
    loop {
        match select(conn.next(), change_pin.wait_for_low()).await {
            Either::First(event) => match event {
                GattConnectionEvent::Disconnected { reason } => {
                    info!("[gatt] disconnected: {:?}", reason);
                    break;
                }
                GattConnectionEvent::PairingComplete {
                    security_level,
                    bond,
                } => {
                    info!("[gatt] pairing complete: {:?}", security_level);
                    if let Some(bond) = bond {
                        active_identity = Some(bond.identity);
                        bonds.upsert(bond);
                        save_bonds(map_storage, data_buffer, bonds).await;
                    }
                }
                GattConnectionEvent::PairingFailed(err) => {
                    warn!("[gatt] pairing error: {:?}", err);
                }
                GattConnectionEvent::Encrypted {
                    security_level,
                    bond,
                } => {
                    info!("[gatt] link encrypted: {:?}", security_level);
                    // `bond` is only populated on fresh pairing; on bonded
                    // re-encryption look the peer up by identity instead.
                    let identity = bond
                        .map(|b| b.identity)
                        .unwrap_or_else(|| conn.raw().peer_identity());
                    if let Some(entry) = bonds.find(&identity) {
                        active_identity = Some(identity);
                        // Restore the CCCD state this bonded central set up in a
                        // previous connection (bonded centrals don't re-subscribe).
                        if !entry.cccd.is_empty() {
                            match ClientAttTableView::try_from_raw(&entry.cccd) {
                                Ok(view) => {
                                    server.set_client_att_table(conn.raw(), &view);
                                    info!("[gatt] restored client att table");
                                }
                                Err(_) => warn!("[gatt] stored client att table invalid"),
                            }
                        }
                        bonds.touch(&identity);
                    }
                }
                GattConnectionEvent::Gatt { event } => {
                    match &event {
                        GattEvent::Write(w) => {
                            info!("[gatt] write handle={}", w.handle());
                        }
                        GattEvent::Read(r) => {
                            info!("[gatt] read handle={}", r.handle());
                        }
                        GattEvent::NotAllowed(e) => {
                            info!("[gatt] disallowed request to handle {}", e.handle());
                        }
                        _ => (),
                    }
                    match event.accept() {
                        Ok(reply) => reply.send().await,
                        Err(e) => warn!("[gatt] error accepting event: {:?}", e),
                    }
                }
                _ => (),
            },
            Either::Second(_) => {
                // NCHANGE went low — read Detection Status + Key Status (2 bytes
                // starting at register 0x02). Both must be read to clear NCHANGE.
                let mut buf = [0u8; 2];
                if let Err(e) = i2c
                    .write_read(QT1070_ADDR, &[QT1070_REG_DET_STATUS], &mut buf)
                    .await
                {
                    warn!("[qt1070] read error: {:?}", defmt::Debug2Format(&e));
                    Timer::after(Duration::from_millis(50)).await;
                    continue;
                }

                let keys = buf[1] & 0x3F; // Key Status is second byte, mask to keys 0-5

                if keys != 0 {
                    let key_idx = keys.trailing_zeros() as u8;
                    let usage = key_to_usage(key_idx);
                    let report = usage.to_le_bytes();

                    match input_report.notify(conn, &report, false).await {
                        Ok(_) => info!("[qt1070] sent usage {:#06x}", usage),
                        Err(e) => warn!("[gatt] notify error: {:?}", e),
                    }

                    // Send release so the host doesn't auto-repeat
                    Timer::after(Duration::from_millis(30)).await;
                    if let Err(e) = input_report.notify(conn, &[0u8; 2], false).await {
                        warn!("[gatt] notify error: {:?}", e);
                    }
                }
            }
        }
    }

    // Persist the CCCD state for the bonded central so it survives a power
    // cycle; bonded centrals expect subscriptions to be remembered.
    if let Some(identity) = active_identity {
        if let Some(table) = server.get_client_att_table(conn.raw()) {
            if let Some(entry) = bonds.find(&identity) {
                if entry.cccd.as_slice() != table.raw() {
                    entry.cccd.clear();
                    let _ = entry.cccd.extend_from_slice(table.raw());
                    save_bonds(map_storage, data_buffer, bonds).await;
                }
            }
        }
    }
}

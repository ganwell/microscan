#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic::PanicInfo;

use cortex_m::interrupt::{free as section, Mutex};
use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use heapless::String;
use microbit::hal::clocks::Clocks;
use microbit::pac::{self, interrupt, TIMER0};
use microbit::Board;
use rubble::beacon::{Beacon, BeaconScanner, ScanCallback};
use rubble::link::ad_structure::AdStructure;
use rubble::link::filter::AllowAll;
use rubble::link::{CompanyId, DeviceAddress, Metadata, MIN_PDU_BUF};
use rubble::time::{Duration, Timer as _};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;
use rubble_nrf5x::utils::get_device_address;

pub struct BeaconScanCallback;

impl ScanCallback for BeaconScanCallback {
    fn beacon<'a, I>(&mut self, addr: DeviceAddress, data: I, metadata: Metadata)
    where
        I: Iterator<Item = AdStructure<'a>>,
    {
        println!(
            "[{:?}] CH:{:?} ",
            metadata.timestamp.unwrap().ticks(),
            metadata.channel,
        );
        if let Some(rssi) = metadata.rssi {
            println!("RSSI:{:?}dBm ", rssi);
        }
        let mut first = true;
        for packet in data {
            first = false;
        }
    }
}

struct Shared {
    radio: BleRadio,
    timer: BleTimer<TIMER0>,
    scanner: BeaconScanner<BeaconScanCallback, AllowAll>,
}

static SHARED: Mutex<RefCell<Option<Shared>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    static mut BLE_TX_BUF: PacketBuffer = [0; MIN_PDU_BUF];
    static mut BLE_RX_BUF: PacketBuffer = [0; MIN_PDU_BUF];
    if let Some(p) = microbit::Peripherals::take() {
        let _clocks = Clocks::new(p.CLOCK).enable_ext_hfosc();

        let mut timer = BleTimer::init(p.TIMER0);

        let mut radio = BleRadio::new(p.RADIO, &p.FICR, BLE_TX_BUF, BLE_RX_BUF);

        let mut scanner = BeaconScanner::new(BeaconScanCallback);
        let scanner_cmd = scanner.configure(timer.now(), Duration::millis(500));

        radio.configure_receiver(scanner_cmd.radio);
        timer.configure_interrupt(scanner_cmd.next_update);

        let shared = Shared {
            radio,
            timer,
            scanner,
        };
        section(|cs| SHARED.borrow(cs).replace(Some(shared)));
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIMER0);
            pac::NVIC::unmask(pac::Interrupt::RADIO);
        }
        println!("Scanner set up");
        loop {}
    }
    panic!("End");
}

#[interrupt]
fn TIMER0() {
    section(|cs| {
        if let Some(ref mut shared) = SHARED.borrow(cs).borrow_mut().deref_mut() {
            let timer = &mut shared.timer;
            let radio = &mut shared.radio;
            let scanner = &mut shared.scanner;
            if let Some(next_update) = radio.recv_beacon_interrupt(timer.now(), scanner)
            {
                timer.configure_interrupt(next_update);
            }
        }
    });
}

#[interrupt]
fn RADIO() {
    section(|cs| {
        if let Some(ref mut shared) = SHARED.borrow(cs).borrow_mut().deref_mut() {
            let timer = &mut shared.timer;
            let radio = &mut shared.radio;
            let scanner = &mut shared.scanner;
            if !timer.is_interrupt_pending() {
                return;
            }
            timer.clear_interrupt();
            let cmd = scanner.timer_update(timer.now());
            radio.configure_receiver(cmd.radio);
            timer.configure_interrupt(cmd.next_update);
        }
    });
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut s: String<1024> = String::new();
    write!(s, "Panic: {:?}", info).unwrap();
    println!("{}", s.as_str());
    loop {}
}

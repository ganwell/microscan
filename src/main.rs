#![no_std]
#![no_main]

use panic_rtt_target as _;
mod frames;

#[rtic::app(device = microbit::pac, peripherals = true)]
mod app {
    use core::cmp::{max, min};
    use core::mem::MaybeUninit;
    use core::sync::atomic::{AtomicU8, Ordering};

    use microbit::display::nonblocking::Display;
    use microbit::hal::clocks::Clocks;
    use microbit::{pac, Board};
    use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
    use rtt_target::{rprint, rprintln, rtt_init, set_print_channel, ChannelMode};
    use rubble::beacon::{BeaconScanner, ScanCallback};
    use rubble::link::ad_structure::AdStructure;
    use rubble::link::filter::AllowAll;
    use rubble::link::{DeviceAddress, Metadata, MIN_PDU_BUF};
    use rubble::time::{Duration, Timer};
    use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
    use rubble_nrf5x::timer::BleTimer;

    use crate::frames::FRAMES;

    static VALUE: AtomicU8 = AtomicU8::new(0);

    #[derive(Copy, Clone)]
    struct RSSIEntry {
        timestamp: u32,
        rssi: u8,
    }

    impl Default for RSSIEntry {
        fn default() -> Self {
            RSSIEntry {
                timestamp: 0,
                rssi: u8::MAX,
            }
        }
    }

    pub struct BeaconScanCallback {
        log: ConstGenericRingBuffer<RSSIEntry, 32>,
        rssi_window: ConstGenericRingBuffer<u8, 4>,
    }

    impl Default for BeaconScanCallback {
        fn default() -> Self {
            BeaconScanCallback {
                log: ConstGenericRingBuffer::new(),
                rssi_window: ConstGenericRingBuffer::new(),
            }
        }
    }

    impl ScanCallback for BeaconScanCallback {
        fn beacon<'a, I>(&mut self, addr: DeviceAddress, data: I, metadata: Metadata)
        where
            I: Iterator<Item = AdStructure<'a>>,
        {
            //rprint!(
            //    "[{:?}] CH:{:?} Type:{:?} ",
            //    metadata.timestamp.unwrap().ticks(),
            //    metadata.channel,
            //    metadata.pdu_type.unwrap(),
            //);
            if let Some(rssi) = metadata.rssi {
                let mut rssi = rssi.abs() as u8;
                rssi = rssi.saturating_sub(42);
                let entry = RSSIEntry {
                    timestamp: metadata.timestamp.unwrap().ticks(),
                    rssi: rssi,
                };
                self.log.enqueue(entry);
                let getstamp = match self.log.get_signed(-(self.log.len() as isize)) {
                    Some(get) => get.timestamp,
                    None => 0,
                };
                let diff = entry.timestamp.wrapping_sub(getstamp);

                const MAX_DELAY: u32 = 250_000;
                if self.log.is_full() || diff > MAX_DELAY {
                    let mut min_rssi = u8::MAX;
                    let mut valid_items: usize = 0;
                    for item in self.log.iter().rev() {
                        let diff = entry.timestamp.wrapping_sub(item.timestamp);

                        if diff < MAX_DELAY {
                            min_rssi = min(min_rssi, item.rssi);
                            if diff < MAX_DELAY {
                                valid_items += 1;
                            }
                        } else {
                            break;
                        }
                    }

                    while valid_items > 0 {
                        self.log.skip();
                        valid_items -= 1;
                    }

                    let mut avg_min_rssi: u32 = 0;
                    self.rssi_window.enqueue(min_rssi);
                    for i in self.rssi_window.iter() {
                        avg_min_rssi += *i as u32;
                    }
                    avg_min_rssi /= self.rssi_window.len() as u32;
                    if self.rssi_window.is_full() {
                        self.rssi_window.skip();
                    }

                    rprintln!("avg_min_rssi: {}", avg_min_rssi);
                    VALUE.store(avg_min_rssi as u8, Ordering::SeqCst);
                }
            }
            //rprint!("BDADDR:{:?} DATA:", addr);
            //let mut first = true;
            //for packet in data {
            //    rprint!("{}{:02x?}", if first { " " } else { " / " }, packet);
            //    first = false;
            //}
            //rprintln!("");
            //rprintln!("");
        }
    }

    #[shared]
    struct Shared {
        radio: BleRadio,
        ble_timer: BleTimer<pac::TIMER0>,
        scanner: BeaconScanner<BeaconScanCallback, AllowAll>,
        display: Display<pac::TIMER1>,
    }

    #[local]
    struct Local {
        last_rssi: u8,
    }

    #[init(local=[
        tx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit(),
        rx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit()
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rtt = rtt_init! {
            up: {
                0: {
                    size: 1024,
                    mode: ChannelMode::NoBlockTrim,
                    name: "Microscan Logs"
                }
            }
        };
        set_print_channel(rtt.up.0);
        let board = Board::new(ctx.device, ctx.core);

        let _clocks = Clocks::new(board.CLOCK).enable_ext_hfosc();

        let mut ble_timer = BleTimer::init(board.TIMER0);

        let ble_rx_buf: &'static mut _ = ctx.local.rx_buf.write([0; MIN_PDU_BUF]);
        let ble_tx_buf: &'static mut _ = ctx.local.tx_buf.write([0; MIN_PDU_BUF]);
        let mut radio = BleRadio::new(board.RADIO, &board.FICR, ble_tx_buf, ble_rx_buf);

        let mut scanner = BeaconScanner::new(BeaconScanCallback::default());
        let scanner_cmd = scanner.configure(ble_timer.now(), Duration::millis(500));

        radio.configure_receiver(scanner_cmd.radio);
        ble_timer.configure_interrupt(scanner_cmd.next_update);

        let display = Display::new(board.TIMER1, board.display_pins);

        rprintln!("nRF52 scanner ready!");

        (
            Shared {
                radio,
                scanner,
                ble_timer,
                display,
            },
            Local { last_rssi: 0 },
            init::Monotonics(),
        )
    }

    #[task(binds = TIMER1, priority = 2, shared = [display], local = [last_rssi])]
    fn timer1(mut ctx: timer1::Context) {
        let rssi = VALUE.load(Ordering::SeqCst);
        let frame = min(26, rssi);
        let last = *ctx.local.last_rssi;
        *ctx.local.last_rssi = frame;
        ctx.shared.display.lock(|display| {
            if last != frame {
                display.show(&FRAMES[frame as usize]);
            }
            display.handle_display_event();
        });
    }

    #[task(binds = RADIO, shared = [radio, scanner, ble_timer])]
    fn radio(ctx: radio::Context) {
        let timer = ctx.shared.ble_timer;
        let scanner = ctx.shared.scanner;
        let radio = ctx.shared.radio;

        (timer, scanner, radio).lock(|timer, scanner, radio| {
            if let Some(next_update) = radio.recv_beacon_interrupt(timer.now(), scanner)
            {
                timer.configure_interrupt(next_update);
            }
        });
    }

    #[task(binds = TIMER0, shared = [radio, ble_timer, scanner])]
    fn timer0(ctx: timer0::Context) {
        let timer = ctx.shared.ble_timer;
        let scanner = ctx.shared.scanner;
        let radio = ctx.shared.radio;

        (timer, scanner, radio).lock(|timer, scanner, radio| {
            if !timer.is_interrupt_pending() {
                return;
            }
            timer.clear_interrupt();

            let cmd = scanner.timer_update(timer.now());
            radio.configure_receiver(cmd.radio);
            timer.configure_interrupt(cmd.next_update);
        });
    }
}

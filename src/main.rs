#![no_std]
#![no_main]

use panic_rtt_target as _;

#[rtic::app(device = microbit::pac, peripherals = true)]
mod app {
    use core::mem::MaybeUninit;

    use microbit::hal::clocks::Clocks;
    use microbit::pac;
    use rtt_target::{rprint, rprintln, rtt_init, set_print_channel, ChannelMode};
    use rubble::beacon::{BeaconScanner, ScanCallback};
    use rubble::link::ad_structure::AdStructure;
    use rubble::link::filter::AllowAll;
    use rubble::link::{DeviceAddress, Metadata, MIN_PDU_BUF};
    use rubble::time::{Duration, Timer};
    use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
    use rubble_nrf5x::timer::BleTimer;

    pub struct BeaconScanCallback;

    impl ScanCallback for BeaconScanCallback {
        fn beacon<'a, I>(&mut self, addr: DeviceAddress, data: I, metadata: Metadata)
        where
            I: Iterator<Item = AdStructure<'a>>,
        {
            rprint!(
                "[{:?}] CH:{:?} Type:{:?} ",
                metadata.timestamp.unwrap().ticks(),
                metadata.channel,
                metadata.pdu_type.unwrap(),
            );
            if let Some(rssi) = metadata.rssi {
                rprint!("RSSI:{:?}dBm ", rssi);
            }
            rprint!("BDADDR:{:?} DATA:", addr);
            let mut first = true;
            for packet in data {
                rprint!("{}{:02x?}", if first { " " } else { " / " }, packet);
                first = false;
            }
            rprintln!("");
            rprintln!("");
        }
    }

    #[shared]
    struct Shared {
        radio: BleRadio,
        ble_timer: BleTimer<pac::TIMER0>,
        scanner: BeaconScanner<BeaconScanCallback, AllowAll>,
    }

    #[local]
    struct Local {}

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

        let _clocks = Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        let mut ble_timer = BleTimer::init(ctx.device.TIMER0);

        let ble_rx_buf: &'static mut _ = ctx.local.rx_buf.write([0; MIN_PDU_BUF]);
        let ble_tx_buf: &'static mut _ = ctx.local.tx_buf.write([0; MIN_PDU_BUF]);
        let mut radio =
            BleRadio::new(ctx.device.RADIO, &ctx.device.FICR, ble_tx_buf, ble_rx_buf);

        let mut scanner = BeaconScanner::new(BeaconScanCallback);
        let scanner_cmd = scanner.configure(ble_timer.now(), Duration::millis(500));

        radio.configure_receiver(scanner_cmd.radio);
        ble_timer.configure_interrupt(scanner_cmd.next_update);

        rprintln!("nRF52 scanner ready!");

        (
            Shared {
                radio,
                scanner,
                ble_timer,
            },
            Local {},
            init::Monotonics(),
        )
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

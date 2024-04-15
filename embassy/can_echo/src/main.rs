#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::frame::Envelope;
use embassy_stm32::can::{
    filter, Can, ExtendedId, Fifo, Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, StandardId, TxInterruptHandler
};
use embassy_stm32::peripherals::CAN;
use embassy_stm32::{bind_interrupts, Config};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;

use core::num::{NonZeroU16, NonZeroU8};

use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
    CAN1_RX1 => Rx1InterruptHandler<CAN>;
    CAN1_SCE => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
});


//cargo build --release
//cargo flash --chip stm32f103C8 --release
// This example is configured to work with real CAN transceivers on B8/B9.
// See other examples for loopback.

fn handle_frame(env: &Envelope, read_mode: &str) {
    match env.frame.id() {
        Id::Extended(id) => {
            defmt::println!(
                "{} Extended Frame id={:x} {:02x}",
                read_mode,
                id.as_raw(),
                env.frame.data()
            );
        }
        Id::Standard(id) => {
            defmt::println!(
                "{} Standard Frame id={:x} {:02x}",
                read_mode,
                id.as_raw(),
                env.frame.data()
            );
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {

    
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
    config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        // PLL uses HSE as the clock source
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            // 8 MHz clock source / 1 = 8 MHz PLL input
            prediv: PllPreDiv::DIV1,
            // 8 MHz PLL input * 7 = 56 MHz PLL VCO
            mul: PllMul::MUL7,
        });
        // System clock comes from PLL (= the 56 MHz main PLL output)
        config.rcc.sys = Sysclk::PLL1_P;
        // 56 MHz / 1 = 56 MHz AHB frequency
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        // 56 MHz / 2 = 28 MHz APB1 frequency
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        // 56 MHz / 1 = 56 MHz APB2 frequency
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        // 56 MHz / 4 = 14 MHz ADC frequency
        config.rcc.adc_pre = ADCPrescaler::DIV4;
    }

    let p = embassy_stm32::init(config);

    defmt::println!("Start");

    let led = Output::new(p.PC13, Level::High, Speed::Low);
    

    // Set alternate pin mapping to B8/B9
    //embassy_stm32::pac::AFIO.mapr().modify(|w| w.set_can1_remap(2));

    static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<10>> = StaticCell::new();
    static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<10>> = StaticCell::new();

    let mut can = Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all());

    can.modify_config()
        //.set_bitrate(250_000)

        .set_bit_timing(embassy_stm32::can::util::NominalBitTiming {
            prescaler: NonZeroU16::new(7).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        }) // http://www.bittiming.can-wiki.info/
        
        .set_automatic_retransmit(true)
        .set_loopback(false)
        .set_silent(false);

    can.enable().await;

    let mut rx = rx.buffered(RX_BUF.init(embassy_stm32::can::RxBuf::<10>::new()));
    let mut tx = tx.buffered(TX_BUF.init(embassy_stm32::can::TxBuf::<10>::new()));

    defmt::println!("Finished conf {} {}", embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::CAN>(), embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::USART1>());

    let (mut tx, mut rx) = can.split();

    spawner.spawn(led_blinker(led)).unwrap();

    loop {
        
        match rx.read().await {
            Ok(envelope) => {
                handle_frame(&envelope, "Buf");
                tx.write(&envelope.frame).await;
            }
            Err(err) => {
                defmt::println!("Error {}", err);
            }
        }
    }
}

#[embassy_executor::task]
async fn led_blinker(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(1000).await;

        led.set_low();
        Timer::after_millis(1000).await;
    }
}

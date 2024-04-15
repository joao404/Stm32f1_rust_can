
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::frame::Envelope;
use embassy_stm32::can::{
    filter, Can, Fifo, Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler
};
use embassy_stm32::can::{BufferedCanRx, BufferedCanTx};

use embassy_stm32::usart::{Config as UsartConfig, Uart, UartRx, UartTx};

use embassy_stm32::peripherals::{CAN, USART1, DMA1_CH4, DMA1_CH5};
use embassy_stm32::{bind_interrupts, Config as GlobalConfig};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

use core::num::{NonZeroU16, NonZeroU8};

use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
    CAN1_RX1 => Rx1InterruptHandler<CAN>;
    CAN1_SCE => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
    USART1 => embassy_stm32::usart::InterruptHandler<USART1>;
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

static _CAN_RX_CHANNEL: Channel<ThreadModeRawMutex, Envelope, 1> = Channel::new();

static _USART_RX_CHANNEL: Channel<ThreadModeRawMutex, u8, 8> = Channel::new();


#[embassy_executor::main]
async fn main(spawner: Spawner) {

    
    let mut config = GlobalConfig::default();
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
    

    // Set alternate pin mapping to PA11/PA12
    embassy_stm32::pac::AFIO.mapr().modify(|w| w.set_can1_remap(0));

    static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<20>> = StaticCell::new();
    static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<20>> = StaticCell::new();

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
        
        .set_loopback(false)
        .set_silent(false);

    can.enable().await;
    
    let (can_tx, can_rx) = can.split();
    let can_rx = can_rx.buffered(RX_BUF.init(embassy_stm32::can::RxBuf::<20>::new()));
    let mut can_tx = can_tx.buffered(TX_BUF.init(embassy_stm32::can::TxBuf::<20>::new()));
    
    let tx_frame = Frame::new_standard(0x30, &[0x30, 0, 1, 2, 3, 4, 5, 6]).unwrap();
    can_tx.write(&tx_frame).await;

    // DMA channel selection depends on the peripheral:
    // - USART1: TX = 4, RX = 5
    // - USART2: TX = 6, RX = 7
    // - USART3: TX = 2, RX = 3

    let mut uart_config = UsartConfig::default();
    uart_config.baudrate = 115200;
    uart_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    uart_config.parity = embassy_stm32::usart::Parity::ParityNone;
    uart_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    let usart = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, uart_config).unwrap();
    let (uart_tx, uart_rx) = usart.split();


    defmt::println!("Finished conf {} {}", embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::CAN>(), embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::USART1>());

    spawner.spawn(led_blinker(led)).unwrap();

    spawner.spawn(can_to_uart(can_rx, uart_tx)).unwrap();
    spawner.spawn(uart_to_can(can_tx, uart_rx)).unwrap();
    loop {
        Timer::after_millis(1000).await;
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

#[embassy_executor::task]
//async fn can_to_uart(mut can_rx : CanRx<'static, CAN>, mut uart_tx : UartTx<'static, USART1, DMA1_CH4>) {
async fn can_to_uart(mut can_rx : BufferedCanRx<'static, CAN, 20>, mut uart_tx : UartTx<'static, USART1, DMA1_CH4>) {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                handle_frame(&envelope, "Buf");
                //let buf = CHANNEL.receive().await;
                unwrap!(uart_tx.write(envelope.frame.data()).await);
            }
            Err(err) => {
                defmt::println!("Error {}", err);
            }
        }
        }
}

#[embassy_executor::task]
//async fn uart_to_can(mut can_tx : CanTx<'static, CAN>, mut uart_rx : UartRx<'static, USART1, DMA1_CH5>) {
async fn uart_to_can(mut can_tx : BufferedCanTx<'static, CAN, 20>, mut uart_rx : UartRx<'static, USART1, DMA1_CH5>) {
    
    let mut buf : [u8;8] = [0;8];
    loop {
    match uart_rx.read_until_idle(&mut buf).await
    {
        Ok(_size) => {
            defmt::println!("Uart {} {} {} {} {} {} {} {}", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
            let frame : Frame = unwrap!(Frame::new_extended(0x500, &buf));
            can_tx.write(&frame).await;
            //let status = can_tx.write(&frame).await;
            //can_tx.flush_all().await;
            //defmt::println!("CanTx {} {}", status.mailbox(), match status.dequeued_frame(){None => "None", Some(_x) => "Frame"});
        }
        Err(err) => {
            defmt::println!("Error {}", err);
        }
    }
    }
}

/*
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::frame::Envelope;
use embassy_stm32::can::{
    filter, Can, Fifo, Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, StandardId,
    TxInterruptHandler,
};
use embassy_stm32::peripherals::CAN;
use embassy_stm32::{bind_interrupts, Config};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
    CAN1_RX1 => Rx1InterruptHandler<CAN>;
    CAN1_SCE => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
});

// This example is configured to work with real CAN transceivers on B8/B9.
// See other examples for loopback.

fn handle_frame(env: Envelope, read_mode: &str) {
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
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    // Set alternate pin mapping to B8/B9
    embassy_stm32::pac::AFIO.mapr().modify(|w| w.set_can1_remap(0));

    static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<10>> = StaticCell::new();
    static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<10>> = StaticCell::new();

    let mut can = Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all());

    can.modify_config()
        .set_loopback(false)
        .set_silent(false)
        .set_bitrate(250_000);

    can.enable().await;
    let mut i: u8 = 0;

    /*
       // Example for using buffered Tx and Rx without needing to
       // split first as is done below.
       let mut can = can.buffered(
           TX_BUF.init(embassy_stm32::can::TxBuf::<10>::new()),
           RX_BUF.init(embassy_stm32::can::RxBuf::<10>::new()));
       loop {
           let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), &[i, 0, 1, 2, 3, 4, 5, 6]).unwrap();
           can.write(&tx_frame).await;

           match can.read().await {
               Ok((frame, ts)) => {
                   handle_frame(Envelope { ts, frame }, "Buf");
               }
               Err(err) => {
                   defmt::println!("Error {}", err);
               }
           }
           i += 1;
       }

    */
    let (mut tx, mut rx) = can.split();

    // This example shows using the wait_not_empty API before try read
    while i < 3 {
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), &[i, 0, 1, 2, 3, 4, 5, 6]).unwrap();
        tx.write(&tx_frame).await;

        rx.wait_not_empty().await;
        let env = rx.try_read().unwrap();
        handle_frame(env, "Wait");
        i += 1;
    }

    // This example shows using the full async non-buffered API
    while i < 6 {
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), &[i, 0, 1, 2, 3, 4, 5, 6]).unwrap();
        tx.write(&tx_frame).await;

        match rx.read().await {
            Ok(env) => {
                handle_frame(env, "NoBuf");
            }
            Err(err) => {
                defmt::println!("Error {}", err);
            }
        }
        i += 1;
    }

    // This example shows using buffered RX and TX. User passes in desired buffer (size)
    // It's possible this way to have just RX or TX buffered.
    let mut rx = rx.buffered(RX_BUF.init(embassy_stm32::can::RxBuf::<10>::new()));
    let mut tx = tx.buffered(TX_BUF.init(embassy_stm32::can::TxBuf::<10>::new()));

    loop {
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), &[i, 0, 1, 2, 3, 4, 5, 6]).unwrap();
        tx.write(&tx_frame).await;

        match rx.read().await {
            Ok(envelope) => {
                handle_frame(envelope, "Buf");
            }
            Err(err) => {
                defmt::println!("Error {}", err);
            }
        }
        i += 1;
    }
}
*/

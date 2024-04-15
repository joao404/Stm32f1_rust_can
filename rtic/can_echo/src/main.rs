/*********************************************************************
 * Stm32f1_rust_can
 *
 * Copyright (C) 2022 Marcel Maage
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */

//! Simple CAN example.
//! Requires a transceiver connected to PA11, PA12 (CAN1) or PB5 PB6 (CAN2).

//cargo build --release
//cargo flash --chip stm32f103C8 --release

//requires UART Rx on and Tx on

#![no_main]
#![no_std]

use panic_halt as _;

//use cortex_m::singleton;
//use cortex_m_semihosting::{hprintln};

use bxcan::Frame;

//use std::sync::mpsc;
//use cortex_m_rt::entry;

use core::cmp::Ordering;

#[derive(Debug)]
pub struct PriorityFrame(Frame);

/// Ordering is based on the Identifier and frame type (data vs. remote) and can be used to sort
/// frames by priority.
impl Ord for PriorityFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for PriorityFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for PriorityFrame {}

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [I2C1_EV], peripherals = true,)]
mod app {
    use super::PriorityFrame;
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx as canRx, StandardId, Tx as canTx};
    use nb::block;
    use stm32f1xx_hal::{
        can::Can,
        gpio::{gpioc, Output, PushPull}, //gpioa , Floating, Input, Alternate},
        pac::{Interrupt, CAN1, USART1, USART3},
        prelude::*,
        serial::{Config, Event, Rx as serialRx, Serial, Tx as serialTx /*TxDma1, RxDma1,*/},
    };
    //use rtic::{app};
    use heapless::{
        binary_heap::{BinaryHeap, Max},
        Vec,
    };

    use pid::Pid;   

    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    const PERIOD: u32 = 72_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<PERIOD>; // 8 MHz

    #[local]
    struct Local {
        can_tx: canTx<Can<CAN1>>,
        can_rx: canRx<Can<CAN1>>,
        serial1_rx: serialRx<USART1>,
        serial1_tx: serialTx<USART1>,
        serial3_rx: serialRx<USART3>,
        serial3_tx: serialTx<USART3>,
        led: gpioc::PC13<Output<PushPull>>,
    }
    #[shared]
    struct Shared {
        can_tx_queue: BinaryHeap<PriorityFrame, Max, 16>,
        can_rx_queue: BinaryHeap<PriorityFrame, Max, 16>,
        serial3_rx_queue: Vec<u8, 255>,
    }
    /*
    struct Resources {
        can_tx: canTx<Can<CAN1>>,
        can_tx_queue: BinaryHeap<PriorityFrame, Max, 16>,
        can_rx: canRx<Can<CAN1>>,
        can_rx_queue: BinaryHeap<PriorityFrame, Max, 16>,
        //serial: Serial<USART1, (gpioa::PA9<Alternate<PushPull>>,gpioa::PA10<Input<Floating>>)>,
        //serial_dma_rx : RxDma1,
        //serial_dma_tx : TxDma1,
        serial_rx : serialRx<USART1>,
        serial_tx : serialTx<USART1>,
        led: gpioc::PC13<Output<PushPull>>,
    }
    */

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // init::LateResources
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .hclk(72.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        let mut afio = cx.device.AFIO.constrain();

        let mut cp = cx.core;

        // Initialize the monotonic timer (CYCCNT)
        //cp.DCB.enable_trace();

        //cx.schedule.blinky(cx.start + PERIOD.cycles()).unwrap();

        // Initialize the monotonic
        let mono = DwtSystick::new(&mut cp.DCB, cp.DWT, cp.SYST, PERIOD);

        //cp.DWT.enable_cycle_counter();

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();

        //LED
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        //led.set_low().ok();

        // USART1
        let serial1_tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let serial1_rx_pin = gpioa.pa10;

        let mut serial1 = Serial::usart1(
            cx.device.USART1,
            (serial1_tx_pin, serial1_rx_pin),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            clocks,
        );

        // USART3
        let serial3_tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let serial3_rx_pin = gpiob.pb11;

        let mut serial3 = Serial::usart3(
            cx.device.USART3,
            (serial3_tx_pin, serial3_rx_pin),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            clocks,
        );

        block!(serial1.write(b'S')).ok();

        let _dma1 = cx.device.DMA1.split();

        // DMA channel selection depends on the peripheral:
        // - USART1: TX = 4, RX = 5
        // - USART2: TX = 6, RX = 7
        // - USART3: TX = 3, RX = 2

        let (mut serial1_tx, serial1_rx) = serial1.split();

        let serial3_rx_queue: Vec<u8, 255> = Vec::new();
        serial3.listen(Event::Rxne);

        let (serial3_tx, serial3_rx) = serial3.split();

        block!(serial1_tx.write(b't')).ok();
        //let serial_dma_rx = serial_rx.with_dma(dma1.5);
        //let serial_dma_tx = serial_tx.with_dma(dma1.4);

        let can = Can::new(cx.device.CAN1, cx.device.USB);

        // Select pins for CAN1.
        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);
        block!(serial1_tx.write(b'a')).ok();
        // APB1 (PCLK1): 36MHz, Bit rate: 250kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can1 = bxcan::Can::builder(can)
        	.set_bit_timing(0x001c0008)
            .leave_disabled();

        // APB1 (PCLK1): 36MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        // let mut can1 = bxcan::Can::builder(can)
        //     .set_bit_timing(0x001c0011)
        //     .leave_disabled();
        block!(serial1_tx.write(b'r')).ok();
        can1.modify_filters().enable_bank(0, Mask32::accept_all());
        
        // Sync to the bus and start normal operation.
        can1.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        block!(serial1_tx.write(b't')).ok();
        // Split the peripheral into transmitter and receiver parts.
        // block!(can1.enable()).unwrap();
        nb::block!(can1.enable_non_blocking()).unwrap();

        let (can_tx, can_rx) = can1.split();

        let can_tx_queue = BinaryHeap::new();
        let can_rx_queue = BinaryHeap::new();

        let mut pid: Pid<f32> = Pid::new(10.0, 0.0, 0.0, 100.0, 100.0, 100.0, 100.0, 15.0);

        let _output = pid.next_control_output(10.0);

        blinky::spawn().unwrap();

        (
            Shared {
                can_tx_queue: can_tx_queue,
                can_rx_queue: can_rx_queue,
                serial3_rx_queue: serial3_rx_queue,
            },
            Local {
                can_tx,
                can_rx,
                serial1_rx: serial1_rx,
                serial1_tx: serial1_tx,
                serial3_rx: serial3_rx,
                serial3_tx: serial3_tx,
                led,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [serial1_rx, serial1_tx, serial3_tx], shared = [can_tx_queue, can_rx_queue, serial3_rx_queue])]
    fn idle(cx: idle::Context) -> ! {

        let mut tx_queue = cx.shared.can_tx_queue;
        let mut rx_queue = cx.shared.can_rx_queue;
        let mut serial3_rx_queue = cx.shared.serial3_rx_queue;
        //let serial = cx.resources.serial;
        let serial1_rx = cx.local.serial1_rx;
        let serial1_tx = cx.local.serial1_tx;

        let mut data: [u8; 8] = [0; 8];
        let id: u16 = 0x500;

        data[1] = 1;
        data[2] = 2;
        data[3] = 3;
        data[4] = 4;
        data[5] = 5;
        data[6] = 6;
        data[7] = 7;

        //let buf = singleton!(: [u8; 8] = [0; 8]).unwrap();
        //let (_buf, _rx) = rx_channel.read(buf).wait();

        loop {
            // Read the byte that was just sent. Blocks until the read is complete

            //tx_channel.write(b"The quick brown fox");
            //rx_channel.ReadDma();

            if let Ok(recieved) = serial1_rx.read() {
                data[0] = recieved + 1;
                let frame = Frame::new_data(StandardId::new(id).unwrap(), data);
                tx_queue.lock(|tx_queue| {
                    tx_queue.push(PriorityFrame(frame)).unwrap();
                    rtic::pend(Interrupt::USB_HP_CAN_TX);
                });
            }

            //let received = block!(serial.read()).unwrap();

            rx_queue.lock(|rx_queue| {
                while let Some(frame) = rx_queue.peek() {
                    if let Some(frame_data) = frame.0.data() {
                        let sent = frame_data[0] + 1;
                        block!(serial1_tx.write(sent)).ok();
                    }
                    rx_queue.pop();
                }
            });

            serial3_rx_queue.lock(|serial3_rx_queue| {
                for x in serial3_rx_queue.iter() {
                    block!(serial1_tx.write(*x)).ok();
                }
                serial3_rx_queue.clear();
            });
        }
    }

    #[task(priority = 1, local = [led])]
    fn blinky(cx: blinky::Context) {
        // Periodic
        //blinky::spawn_after(Seconds(1_u32)).unwrap();
        cx.local.led.toggle();
        blinky::spawn_after(1.secs()).unwrap();
    }

    // This ISR is triggered by each finished frame transmission.
    #[task(binds = USB_HP_CAN_TX, priority = 2, local = [can_tx, ], shared = [can_tx_queue])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.local.can_tx;
        let mut tx_queue = cx.shared.can_tx_queue;

        tx.clear_interrupt_flags();

        tx_queue.lock(|tx_queue| {
            // There is now a free mailbox. Try to transmit pending frames until either
            // the queue is empty or transmission would block the execution of this ISR.
            while let Some(frame) = tx_queue.peek() {
                match tx.transmit(&frame.0) {
                    Ok(status) => match status.dequeued_frame() {
                        None => {
                            // Frame was successfully placed into a transmit buffer.
                            tx_queue.pop();
                        }
                        Some(pending_frame) => {
                            // A lower priority frame was replaced with our high priority frame.
                            // Put the low priority frame back in the transmit queue.
                            tx_queue.pop();
                            tx_queue.push(PriorityFrame(pending_frame.clone())).unwrap();
                            rtic::pend(Interrupt::USB_HP_CAN_TX);
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, local = [can_rx], shared = [can_rx_queue, can_tx_queue])]
    fn can_rx0(cx: can_rx0::Context) {
        let mut _rx_queue = cx.shared.can_rx_queue;
        let mut tx_queue = cx.shared.can_tx_queue;
        // Echo back received packages with correct priority ordering.

        loop {
            match cx.local.can_rx.receive() {
                Ok(frame) => {
                    // rx_queue.lock(|rx_queue| {
                    //     rx_queue.push(PriorityFrame(frame)).unwrap();
                    // });

                    tx_queue.lock(|tx_queue| {
                        tx_queue.push(PriorityFrame(frame)).unwrap();
                        rtic::pend(Interrupt::USB_HP_CAN_TX);
                    });
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }

    #[task(binds = USART3, priority = 2, local = [serial3_rx], shared = [serial3_rx_queue])]
    fn usart3_rx(cx: usart3_rx::Context) {
        let mut rx_queue = cx.shared.serial3_rx_queue;
        // // Echo back received packages with correct priority ordering.

        loop {
            match cx.local.serial3_rx.read() {
                Ok(b) => {
                    rx_queue.lock(|rx_queue| {
                        rx_queue.push(b).unwrap();
                    });
                    // cx.local.serial3_tx.write(b).unwrap();
                }
                Err(_err) => break,
            }
        }
    }
}

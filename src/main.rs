#![no_std]
#![no_main]

//! Space-Vector PWM (SVPWM) example for driving brushless DC (BLDC) motors.
//!
//! This uses a STM32 microcontroller to drive a STSPIN233-based motor driver,
//! to achieve low-speed control of a drone-class BLDC motor using SVPWM. A
//! unique feature of SVPWM is that it allows you to drive motors at low
//! rotation rate (or no rotation), bidirectionally, and absent any type of
//! position feedback like Hall sensors, encoders, or back EMF detection.
//!
//! Several good introductions to SVPWM can be found online. Here we use
//! terminology and notation from one in particular, Application Note AN2154
//! from STMicroelectronics.
//!
//! This runs as-is on a NUCLEO-F446RE development board with mounted
//! X-NUCLEO-IHM17M1 motor driver. Other hardware configurations may require
//! some adaptation.
//!
//! NOTE: During calibration this driver sends roughly 1.3 Amps rms through
//! the motor coils. Be sure that any motor you try this on can tolerate that
//! current. Virtually any drone-class motor is fine.

// Copyright 2020 by Jack Boyce (jboyce@gmail.com)
// Released under MIT License

use panic_itm as _;

use core::{
    cell::RefCell,
    f32::consts::{PI, FRAC_PI_3},
    ops::DerefMut,
    sync::atomic::{AtomicBool, Ordering},
};
use libm::sinf;

use cortex_m::{
    peripheral::{NVIC, ITM},
    interrupt::{free, Mutex},
    iprintln,
};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::{
    prelude::*,
    gpio::{
        gpioa::PA5, gpioc::PC13,
        ExtiPin, Edge, Output, PushPull, Input, Floating
    },
    interrupt,
    time::Hertz,
    stm32,
};

/// Low-level state of SVPWM motor drive
#[derive(Debug)]
struct SvpwmState {
    /// Motor position in electrical cycle (radians)
    theta: f32,

    /// Modulation depth (drive strength), 0 <= m <= 1
    mod_depth: f32,

    /// Duty cycle multiplier to prevent overcurrent faults
    duty_mult: f32,

    /// Timer period (clock ticks)
    tim_period: u16,
}

impl SvpwmState {
    pub fn new() -> Self {
        Self {
            theta: 0.0,
            mod_depth: 0.0,
            duty_mult: 1.0,
            tim_period: 0,
        }
    }

    /// Calculates UVW phase duty cycles needed to get desired motor position
    /// and driving strength (modulation depth).
    ///
    /// Uses the same notation as in Application Note AN2154 from
    /// STMicroelectronics (in particular see Figure 2 from that document).
    ///
    /// Returns a tuple of duty cycles for the UVW drive phases; these are in
    /// relation to the overall timer period `tim_period`.
    fn get_svpwm_duty_cycles(&mut self) -> (u16, u16, u16) {
        // states read off clockwise from Figure 2:
        const STATES: [u8; 7] = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0b001];

        let mut theta = self.theta;
        if theta < 0.0 {
            theta += 2.0 * PI * (1 + (-theta / (2.0 * PI)) as u16) as f32;
        } else if theta >= 2.0 * PI {
            theta -= 2.0 * PI * ((theta / (2.0 * PI)) as u16) as f32;
        }

        let sector = (theta / FRAC_PI_3) as usize;

        let state_a = STATES[sector];
        let state_b = STATES[sector + 1];

        // Work out the fraction of time we want to spend in each state. `alpha` is
        // the angle between `theta` and state A above. By construction 0 <= alpha < PI/3.
        let alpha = theta - FRAC_PI_3 * sector as f32;

        let state_a_duty = self.mod_depth * self.duty_mult * sinf(FRAC_PI_3 - alpha);
        let state_b_duty = self.mod_depth * self.duty_mult * sinf(alpha);

        // Calculate total fraction of time to drive each of the UVW phases.
        // Note that one of these will always be 0: for any given position SVPWM
        // in general drives two of the phases and keeps the other grounded.
        let phase_u_duty = if state_a & 4 != 0 { state_a_duty } else { 0.0 }
            + if state_b & 4 != 0 { state_b_duty } else { 0.0 };
        let phase_v_duty = if state_a & 2 != 0 { state_a_duty } else { 0.0 }
            + if state_b & 2 != 0 { state_b_duty } else { 0.0 };
        let phase_w_duty = if state_a & 1 != 0 { state_a_duty } else { 0.0 }
            + if state_b & 1 != 0 { state_b_duty } else { 0.0 };

        let phase_u_cmpr = (phase_u_duty * (self.tim_period as f32)) as u16;
        let phase_v_cmpr = (phase_v_duty * (self.tim_period as f32)) as u16;
        let phase_w_cmpr = (phase_w_duty * (self.tim_period as f32)) as u16;

        ( phase_u_cmpr, phase_v_cmpr, phase_w_cmpr )
    }
}

// The interrupt handlers need access to some global state. The safe ways to do it
// are Mutexes with interrupts disabled, or atomic types.
static MUTEX_SVPWM: Mutex<RefCell<Option<SvpwmState>>> = Mutex::new(RefCell::new(None));
static MUTEX_ITM: Mutex<RefCell<Option<ITM>>> = Mutex::new(RefCell::new(None));
static MUTEX_BUTTON: Mutex<RefCell<Option<PC13<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static MUTEX_LED: Mutex<RefCell<Option<PA5<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static SWITCH_PROGRAM: AtomicBool = AtomicBool::new(false);

/// Demonstrates SVPWM position control for a BLDC motor.
///
/// After doing some setup, execute a brief calibration phase where the maximum
/// driving signal is established. Then enter into some programmed demonstration
/// movement patterns. Pushing the user button on the NUCLEO board advances to
/// the next program.
///
/// If ITM debugging is enabled it will print some status messages.
#[entry]
fn main() -> ! {
    // Get our peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = stm32::Peripherals::take().unwrap();

    // enable SYSCFG clock so we can configure EXTI interrupts
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());

    // Configure clocks
    // If you change SYSCLK and use ITM debugging, remember to update TPIU
    // frequency in openocd.gdb
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(96.mhz()).freeze();

    // configure GPIO pins that interface with components on the NUCLEO board,
    // or the X-NUCLEO-IHM17M1 driver board
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa5.into_push_pull_output();
    let mut en_fault = gpioa.pa6.into_floating_input();
    let _drive_u = gpioa.pa8.into_alternate_af1();   // TIM1 outputs
    let _drive_v = gpioa.pa9.into_alternate_af1();
    let _drive_w = gpioa.pa10.into_alternate_af1();

    let gpiob = dp.GPIOB.split();
    let mut sby_reset = gpiob.pb5.into_push_pull_output();

    let gpioc = dp.GPIOC.split();
    let mut enable_u = gpioc.pc10.into_push_pull_output();
    let mut enable_v = gpioc.pc11.into_push_pull_output();
    let mut enable_w = gpioc.pc12.into_push_pull_output();
    let mut board_btn = gpioc.pc13.into_floating_input();

    // set "enable" and "standby/reset" outputs high
    enable_u.set_high().unwrap();
    enable_v.set_high().unwrap();
    enable_w.set_high().unwrap();
    sby_reset.set_high().unwrap();

    // connect en/fault input to EXTI6 interrupt controller, and enable as interrupt source
    en_fault.make_interrupt_source(&mut dp.SYSCFG);
    en_fault.enable_interrupt(&mut dp.EXTI);
    en_fault.trigger_on_edge(&mut dp.EXTI, Edge::FALLING);

    // connect button to EXTI13 interrupt controller, and enable as interrupt source
    board_btn.make_interrupt_source(&mut dp.SYSCFG);
    board_btn.enable_interrupt(&mut dp.EXTI);
    board_btn.trigger_on_edge(&mut dp.EXTI, Edge::RISING);

    led.set_low().unwrap();

    // move values into our static RefCells. Note `free()` disables interrupts.
    free(|cs| {
        MUTEX_ITM.borrow(cs).replace(Some(cp.ITM));
        MUTEX_BUTTON.borrow(cs).replace(Some(board_btn));
        MUTEX_LED.borrow(cs).replace(Some(led));
    });

    free(|cs| {
        if let Some(ref mut itm) = MUTEX_ITM.borrow(cs).borrow_mut().deref_mut() {
            iprintln!(&mut itm.stim[0], "SVPWM demo");
        }
    });

    start_tim1_pwm(&clocks);

    // enable interrupts
    unsafe { NVIC::unmask(interrupt::EXTI9_5) };
    unsafe { NVIC::unmask(interrupt::EXTI15_10) };

    calibrate_pwm();

    let mut program: u8 = 0;
    let mut next_program: u8 = 0;

    loop {
        while !SWITCH_PROGRAM.load(Ordering::Relaxed) {
            next_program = do_motor_program(program);
        }

        program = next_program;
        SWITCH_PROGRAM.store(false, Ordering::Relaxed);

        free(|cs| {
            if let Some(ref mut itm) = MUTEX_ITM.borrow(cs).borrow_mut().deref_mut() {
                iprintln!(&mut itm.stim[0], "switching program");
            }
            if let Some(ref mut led) = MUTEX_LED.borrow(cs).borrow_mut().deref_mut() {
                led.set_low().unwrap();
            }
        });
    }
}

/// Configures timer TIM1 for pwm output.
///
/// Channels 1, 2, 3 of TIM1 generate PWM signals for phases U, V, W respectively.
fn start_tim1_pwm(clocks: &hal::rcc::Clocks) {
    // enable TIM1 clock and reset to a clean state
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

    let tim1 = unsafe { &(*stm32::TIM1::ptr()) };

    // Set the PWM frequency. In our case it needs to be very fast because the
    // X-NUCLEO-IHM17M1 driver board has no current-limiting capability on its
    // own. When driving small motors with low coil inductance, current builds
    // up quickly to cause the STSPIN233 to trigger an overcurrent fault. For
    // example With an 1103-sized 8000 KV drone motor this happens in roughly
    // two microseconds. The board makes a current feedback signal available to
    // the microcontroller, but this timescale is too fast to do current chopping
    // in software (e.g., with an ISR). Instead we choose our PWM drive to have
    // a period on that timescale, so we can drive the motor at a reasonably
    // high duty cycle (and RMS current) without causing an overcurrent fault.
    let freq: Hertz = 400.khz().into();

    tim1.ccmr1_output().modify(|_, w| w
        .oc1pe().set_bit().oc1m().pwm_mode1()
        .oc2pe().set_bit().oc2m().pwm_mode1()
    );
    tim1.ccmr2_output().modify(|_, w| w
        .oc3pe().set_bit().oc3m().pwm_mode1()
    );

    // enable preload for timer ARR
    tim1.cr1.modify(|_, w| w.arpe().set_bit());

    // calculate timer prescaler and period to get the requested frequency
    let clk: u32 = clocks.pclk2().0 * if clocks.ppre2() == 1 { 1 } else { 2 };
    let ticks: u32 = clk / freq.0;
    let psc: u16 = ((ticks - 1) / (1 << 16)) as u16;
    let arr: u16 = (ticks / ((psc + 1) as u32)) as u16;
    tim1.psc.write(|w| w.psc().bits(psc));
    tim1.arr.write(|w| w.arr().bits(arr));

    let mut state = SvpwmState::new();
    state.tim_period = arr;
    free(|cs| MUTEX_SVPWM.borrow(cs).replace(Some(state)));

    update_motor_drive(0.0, 0.0);   // loads channel CCR registers

    // trigger update event to load the registers
    tim1.cr1.modify(|_, w| w.urs().set_bit());
    tim1.egr.write(|w| w.ug().set_bit());
    tim1.cr1.modify(|_, w| w.urs().clear_bit());

    tim1.bdtr.modify(|_, w| w.aoe().set_bit());

    // set up timer and start it
    tim1.cr1.write(|w| w
        .cms().bits(0b00)
        .dir().clear_bit()
        .opm().clear_bit()
        .cen().set_bit()
    );

    // enable output channels 1, 2, 3
    tim1.ccer.modify(|_, w| w
        .cc1e().set_bit()
        .cc2e().set_bit()
        .cc3e().set_bit()
    );
}

/// Recalculates duty cycles and loads into timer registers.
fn update_motor_drive(theta: f32, mod_depth: f32) {
    free(|cs| {
        if let Some(ref mut svpwm) = MUTEX_SVPWM.borrow(cs).borrow_mut().deref_mut() {
            svpwm.theta = theta;
            svpwm.mod_depth = mod_depth;
            let ccrs = svpwm.get_svpwm_duty_cycles();

            let tim1 = unsafe { &(*stm32::TIM1::ptr()) };
            tim1.ccr1.write(|w| w.ccr().bits(ccrs.0));
            tim1.ccr2.write(|w| w.ccr().bits(ccrs.1));
            tim1.ccr3.write(|w| w.ccr().bits(ccrs.2));

            // if let Some(ref mut itm) = MUTEX_ITM.borrow(cs).borrow_mut().deref_mut() {
            //     iprintln!(&mut itm.stim[0], "{:?}, {}", ccrs, svpwm.tim_period);
            // }
        }
    });
}

/// Determines the maximum `duty_mult` for the motor.
///
/// This runs the motor angleslowly through two full electrical cycles, at 1.0
/// modulation depth. This will likely trigger multiple overcurrent faults, which
/// will cause the interrupt routine below to back off the duty cycles until
/// there are no more overcurrents.
fn calibrate_pwm() {
    let max_theta: f32 = 2.0 * FRAC_PI_3;
    let steps = 100;

    for step in 0..=2*steps {
        let mut theta = (step as f32 / steps as f32) * max_theta;
        if theta > max_theta {
            theta = 2.0 * max_theta - theta;
        }
        update_motor_drive(theta, 1.0);
        cortex_m::asm::delay(300_000_000 / steps);
    }

    free(|cs| {
        if let Some(ref mut svpwm) = MUTEX_SVPWM.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut itm) = MUTEX_ITM.borrow(cs).borrow_mut().deref_mut() {
                iprintln!(&mut itm.stim[0],
                    "Calibration finished: Duty_mult = {}%",
                    (100.0 * svpwm.duty_mult) as u8
                );
            }
        }
    });
}

/// (Interrupt service routine) Handles when the motor driver has an overcurrent fault
/// and the EN/FAULT input is pulled low.
///
/// The idea here is to reduce the PWM duty cycle multiplier `duty_mult`, then wait
/// for the fault to clear and restart.
#[interrupt]
fn EXTI9_5() {
    let mut ccrs = (0_u16, 0_u16, 0_u16);

    free(|cs| {
        if let Some(ref mut svpwm) = MUTEX_SVPWM.borrow(cs).borrow_mut().deref_mut() {
            svpwm.duty_mult *= 0.9;
            ccrs = svpwm.get_svpwm_duty_cycles();

            if let Some(ref mut itm) = MUTEX_ITM.borrow(cs).borrow_mut().deref_mut() {
                iprintln!(&mut itm.stim[0],
                    "int--drive scaler = {}%",
                    (100.0 * svpwm.duty_mult) as u8
                );
            }
        }
    });

    let tim1 = unsafe { &(*stm32::TIM1::ptr()) };
    let exti = unsafe { &(*stm32::EXTI::ptr()) };

    // turn off the drive and wait for fault to recover
    tim1.ccr1.write(|w| w.ccr().bits(0));
    tim1.ccr2.write(|w| w.ccr().bits(0));
    tim1.ccr3.write(|w| w.ccr().bits(0));

    cortex_m::asm::delay(3_000_000);

    // unmask interrupt pending bit for line 6 in EXTI controller
    exti.pr.write(|w| w.pr6().set_bit());

    // resume drive at new reduced level
    tim1.ccr1.write(|w| w.ccr().bits(ccrs.0));
    tim1.ccr2.write(|w| w.ccr().bits(ccrs.1));
    tim1.ccr3.write(|w| w.ccr().bits(ccrs.2));
}

/// (ISR) Handles when the user button on the Nucleo board is pushed.
#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        if let Some(ref mut board_btn) = MUTEX_BUTTON.borrow(cs).borrow_mut().deref_mut() {
            // exti.pr.write(|w| w.pr13().set_bit());
            board_btn.clear_interrupt_pending_bit();
        }

        SWITCH_PROGRAM.store(true, Ordering::Relaxed);

        if let Some(ref mut led) = MUTEX_LED.borrow(cs).borrow_mut().deref_mut() {
            led.set_high().unwrap();
        }
    });
}

/// Moves the calibrated motor through a pre-defined demonstration program.
///
/// Returns the next program number in the sequence.
fn do_motor_program(program: u8) -> u8 {
    let pole_pairs = 6;

    return match program {
        0 => {
            let num_rot: f32 = 1.0;
            let steps = 5000;

            for step in 0..steps {
                cortex_m::asm::delay(150_000_000 / steps);
                let arg = (4.0 * step as f32 / steps as f32 - 1.0) * 0.5 * PI;
                let mut theta = PI * num_rot * pole_pairs as f32 * (1.0 + sinf(arg));
                while theta >= 2.0 * PI {
                    theta -= 2.0 * PI;
                }
                while theta < 0.0 {
                    theta += 2.0 * PI;
                }
                update_motor_drive(theta, 1.0);
            }

            1
        },
        _ => { 0 }
    };
}
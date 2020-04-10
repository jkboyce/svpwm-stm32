# svpwm-stm32
Space vector pulse width modulation (SVPWM) motor control with a STM32 microcontroller.

This code came out of a project I'm working on, which calls for a low-spin-rate reaction wheel controlled by a small, lightweight motor. The most attractive motors from a torque:weight standpoint are the brushless DC (BLDC) motors used on flying drones, which led to this investigation.

Controlling these motors at low speed is tricky. Typical motor controllers (for example drone ESCs) rely on detecting the back EMF from the spinning motor to sense motor position, and perform accurate commutation. However this back EMF is too small to detect at low spin rate. What ESCs typically do is execute an open-loop acceleration phase to get the motor spinning fast enough to sense back EMF, at which point they transition to closed-loop control.

This project controls BLDC motors in a very different way, using a space vector pulse width modulation (SVPWM) control scheme. It demonstrates accurate position control at very low speeds, as well as bidirectional control. SVPWM is described in several tutorials online, and for convenience we adopt the terminology and notation from one in particular, [Application Note AN2154](https://www.st.com/resource/en/application_note/cd00055518-space-vector-modulation-using-8bit-st7mc-microcontroller-and-st7mckitbldc-starter-kit-stmicroelectronics.pdf) from STMicroelectronics.

This code runs as-is on a NUCLEO-F446RE development board with mounted X-NUCLEO-IHM17M1 motor driver. Other hardware configurations may likely require some adaptation. A complete list of hardware needed:
- [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html).
- [X-NUCLEO-IHM17M1](https://www.st.com/en/ecosystems/x-nucleo-ihm17m1.html).
- Power supply for the motor driver, between 3 and 10 Volts, capable of supplying a few Amps peak. You could use a power supply, or a 1S or 2S LiPo battery.
- A drone motor. I've tested with small motors in the 4000-8000 KV range, and others should work.
- A mini USB cable (not micro USB!) to connect your computer to the NUCLEO's onboard ST-LINK debugger.

[The Discovery Book](https://docs.rust-embedded.org/discovery/) for embedded Rust development has detailed instructions on how to set up a Rust development environment targeting Arm microcontrollers. The book focuses on the [STM32F3 Discovery Kit](https://www.st.com/en/evaluation-tools/stm32f3discovery.html) board, which is very similar to the NUCLEO.

If you have OpenOCD, GDB, and Rust set up as described in the book, you can compile and run with:
```
cargo run --release
```

# svpwm-stm32
Space vector pulse width modulation (SVPWM) motor control with a STM32 microcontroller.

This code came out of a project I'm working on, which calls for a low-spin-rate reaction wheel controlled by a small, lightweight motor. The most attractive motors from a torque:weight standpoint are the brushless DC (BLDC) motors used on flying drones, which led to this investigation.

Controlling these sensorless motors at low speed is tricky. Motor controllers (for example drone ESCs) usually rely on detecting the back EMF from the spinning motor to sense motor position, and perform accurate commutation. However this back EMF is too small to detect at low spin rate. What ESCs typically do is execute an open-loop acceleration phase to get the motor spinning fast enough to sense back EMF, at which point they transition to closed-loop control using back EMF to sense rotor position.

This project controls BLDC motors in a different way, using a space vector pulse width modulation (SVPWM) control scheme. It demonstrates accurate position control at very low speeds, as well as bidirectional control. SVPWM is described in several tutorials online, and for convenience we adopt the terminology and notation from one in particular, [Application Note AN2154](https://www.st.com/resource/en/application_note/cd00055518-space-vector-modulation-using-8bit-st7mc-microcontroller-and-st7mckitbldc-starter-kit-stmicroelectronics.pdf) from STMicroelectronics.

This code runs as-is on a NUCLEO-F446RE development board with mounted X-NUCLEO-IHM17M1 motor driver. Other hardware configurations may likely require some adaptation. A complete list of hardware needed:
- [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html).
- [X-NUCLEO-IHM17M1](https://www.st.com/en/ecosystems/x-nucleo-ihm17m1.html).
- Power source for the motor driver: between 3 and 10 volts, and able to supply a few amps peak. You could use a power supply, or a 1S or 2S LiPo battery.
- A drone motor. I've tested with small motors in the 4000-8000 KV range, but others should work. NOTE: be sure your motor can handle 1.3 amps through its coils without generating too much heat. For drone motors this won't be an issue, but for motors with more turns and higher winding resistance (e.g., gimbal motors) this could be a problem. You can adjust the max. driving current in this line: `start_tim3_pwm(&clocks, 0.15)` where the 0.15 is the current limit in amps (peak, not rms).
- A mini USB cable (not micro USB!) to connect your computer to the NUCLEO's onboard ST-LINK debugger.

[The Discovery Book](https://docs.rust-embedded.org/discovery/) for embedded Rust development has detailed instructions on how to set up a Rust development environment targeting Arm microcontrollers. The book focuses on the [STM32F3 Discovery Kit](https://www.st.com/en/evaluation-tools/stm32f3discovery.html) board, which is very similar to the NUCLEO used in this project.

When OpenOCD, GDB, and Rust are set up as described in the book, you can compile and run with:
```
cargo run --release
```

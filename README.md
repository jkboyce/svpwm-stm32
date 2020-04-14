# svpwm-stm32
Space vector pulse width modulation (SVPWM) motor control with a STM32 microcontroller. See the related [blog post](https://jkboyce.github.io/electronics/2020/04/13/SVPWM-motor_control.html) for a video demo and more information.

This project is written in Rust. It specifically targets the [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) development board, with an attached [X-NUCLEO-IHM17M1](https://www.st.com/en/ecosystems/x-nucleo-ihm17m1.html) motor driver built around the STSPIN233 motor driver IC.
[The Discovery Book](https://docs.rust-embedded.org/discovery/) for embedded
Rust development has detailed instructions on how to set up a Rust development
environment targeting Arm microcontrollers. (The book focuses on the [STM32F3
Discovery Kit](https://www.st.com/en/evaluation-tools/stm32f3discovery.html)
board, which is very similar to the NUCLEO used in this project.)

When OpenOCD, GDB, and Rust are set up, you can compile and run with:

```
cargo run --release
```

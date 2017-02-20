Lab assignments for CS 6780. This is a class that I am currently taking. As such, I have only included the labs that we have done in the month and a half completed as of writing.

These labs largely revole around writing code to configure and perform actions on pins on the STM32F0 microcontroller. A large portion of the lab is reading documentation to learn how to use pins and protocols, and as such the actual code is fairly minimal.

Lab 1 is simple an introduction into using GPIO. The final code simply alternates blinking LEDs.

Lab 2 was learning how to use interrupts, such as SysTick and a physical button press. The code for this alternates two lights on SysTick and another two lights on button press. Another aspect of this code was interrupt priority. The button press has an infinite while loop, so when SysTick has a lower priority than the button, SysTick will get starved our of execution.

Lab 3 explores the various use of timers in the embedded system, and are used for both triggering an interrupt and for PWM to alternate between brightening and dimming lights.

Lab 4, which is still being worked on, has us learning various protocols, such as UART.
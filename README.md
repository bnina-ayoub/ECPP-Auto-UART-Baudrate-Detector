# ECPP Auto UART Baudrate Detector

This repository contains the code for an automatic UART baud rate detection algorithm implemented on an STM32 microcontroller as part of my embedded competitive programming (ECPP) preparation.

## Problem Context

The goal of this project is to automatically determine the baud rate of an incoming UART communication without prior knowledge. This is a common challenge in embedded systems when communicating with devices that might have a fixed but unknown baud rate. The approach used here involves capturing the timing of incoming data bits and analyzing these timings to infer the baud rate.

## Code Structure

The repository includes the following key files:

* `main.c`: Contains the main application logic, including UART initialization, the baud rate detection state machine, and the timing capture mechanism (likely using a Timer Input Capture).
* `stm32f4xx_hal_uart.c` and `stm32f4xx_hal_uart.h`: STM32 HAL UART driver files.
* `stm32f4xx_hal_tim.c` and `stm32f4xx_hal_tim.h`: STM32 HAL Timer driver files used for precise timing.
* `stm32f4xx_it.c`: Interrupt service routines, particularly for the UART receive and the Timer Input Capture.
* `stm32f4xx_hal_msp.c`: Low-level hardware initialization for the UART and Timer peripherals.
* `startup_stm32f401retx.s`: Startup assembly file.

## Implementation Details

The auto baud rate detection likely works through the following steps:

1.  **Initial UART Configuration:** The UART is initially set up at a very low, common baud rate or with the receiver enabled but without specific baud rate settings, relying on oversampling.
2.  **Edge Detection:** The system waits for the first falling or rising edge on the UART RX pin. This is often achieved using a Timer Input Capture configured to trigger on both edges.
3.  **Timing Measurement:** Once an edge is detected, the Timer is used in Input Capture mode to precisely measure the time intervals between subsequent edges of the incoming data stream.
4.  **Baud Rate Calculation:** By analyzing the measured time intervals (which correspond to the bit durations), the algorithm can infer the baud rate. The duration of a single bit is inversely proportional to the baud rate.
5.  **UART Reconfiguration:** Once a likely baud rate is detected, the UART is reconfigured with the determined baud rate.
6.  **Data Reception:** Normal UART data reception can then proceed at the correct baud rate.

The code likely involves a state machine (`RECEIVE` state as seen in previous discussions) to manage the different phases of the detection process. The timing calculations might involve comparing measured time intervals against expected bit durations for common baud rates.

## Key Challenges and Solutions Explored (Based on Conversation)

* **Precise Timing:** Using a Timer in Input Capture mode is crucial for accurately measuring the bit durations.
* **Handling Start Bit:** Correctly identifying the start bit and measuring the subsequent bit timings from there.
* **Synchronization:** Ensuring the Timer and UART are synchronized for accurate measurements.
* **Baud Rate Calculation Logic:** Implementing the correct mathematical logic to convert the measured time intervals to a baud rate. The calculation involving `HAL_RCC_GetPCLK2Freq()` and `elapsed_time` was likely part of this.
* **Tolerance:** Accounting for slight variations in the sender's baud rate. The `if(abs(baudrate - 115200) < 9)` check suggests a tolerance window.
* **Potential Issues:** The program getting stuck in the `Default_Handler` likely indicated an unhandled interrupt related to the Timer or UART during the detection process, possibly due to incorrect interrupt configuration or missing handlers.

## Getting Started (If Applicable for Others)

1.  Clone this repository: `git clone [repository URL]`
2.  This project targets an STM32F4 series microcontroller. Ensure you have the necessary development environment set up (e.g., STM32CubeIDE) along with the STM32 HAL library.
3.  The code might need adjustments based on the specific STM32F4 variant and the UART port being used for auto-detection.

## Further Development/Learning

* Implementing more robust error handling and timeout mechanisms for the baud rate detection.
* Supporting a wider range of baud rates.
* Optimizing the detection algorithm for speed and accuracy.

## License

[Your preferred license, e.g., MIT License]

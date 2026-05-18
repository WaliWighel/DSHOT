# DSHOT600 Motor Control Driver

A DMA-based DSHOT600 motor control driver for STM32 microcontrollers with DMA-accelerated signal generation and optional telemetry support.

## Features

- **DMA-based DSHOT600**: Efficient motor control using DMA for signal generation
- **Telemetry support**: Optional telemetry feedback from ESCs
- **Multi-motor support**: Simultaneous control of up to 4 motors
- **Simple API**: Straightforward initialization and control functions

## Quick Start

## configuration

I use timer with 300MHz clk inpout, so i set it like this : 

<img width="1526" height="1625" alt="image" src="https://github.com/user-attachments/assets/7a8f054a-823a-40da-a926-6e83cfed80ae" />

Prescaler = 3 makes my timer tick at 100MHz, if your timer can't be clocked at 100MHz with counter  = 168, you will have to addjust :
#define DSHOT600_TH1 	 (126U)
#define DSHOT600_TH0 	 (63U)

you have to make DSHOT600_TH1 to be 1,25us and DSHOT600_TH0 0.65us, +- 10% i guess. To make from it DSHOT1200 you will have to addjust your timer properlly. To make DSHOT300, you simply set clk division to 2, and nothin else, since this will
make timer rune 2times slower. Same for DSHOT150, but instead for divisio = 2, here division = 4.

### Initialization

Call `ESC_Init()` to initialize the ESC. This function:
1. Powers up the ESC and waits for stabilization (default: 4 seconds)
2. Sends calibration pulses (zero-throttle, ~5 seconds @ 8kHz)
3. Waits for ESC confirmation beeps (low tone, then high tone)

You can adjust the stabilization delay in the init function:
```c
/* Allow ESC power supply to stabilize */
HAL_Delay(4000);
```

### Main Loop

You must send DSHOT signals continuously to your ESC at a consistent rate. Failure to do so will cause the ESC to shut down as a safety measure.

**Recommended approach**: Use a timer interrupt (e.g., 8kHz timer):

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim16) {
        if (esc_ready) {
            ESC_EngineSetSpeedForAll(Throttle_all, 0);  // Normal operation
        } else {
            ESC_SetFlagForInit();         // Initialization sequence
        }
    }
}
```

**Important**: Keep the timer frequency consistent with the initialization rate (8kHz in this example).

### Telemetry (Optional)

To receive telemetry feedback from your ESC:

1. Connect the ESC telemetry pin to your microcontroller's UART RX pin
2. Set the telemetry bit to 1 when sending commands:
   ```c
   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim == &htim16) {
            if (esc_ready) {
                ESC_EngineSetSpeedForAll(Throttle_all, 1);  // Request telemetry
            } else {
                ESC_SetFlagForInit();         // Initialization sequence
            }
        }
    }  
   ```
3. Add a UART RX interrupt handler:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       if (huart == &huart1) {
           ESC_TelemetryHandling(EVENT_USART_RX);
       }
   }
   ```
4. Access telemetry data from the global structure:
   ```c
   ESC_Status_t ESC;  // Contains telemetry data for all 4 motors
   ```

## References

- [DSHOT Protocol Overview](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [STM32 HAL DSHOT Reference Implementation](https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

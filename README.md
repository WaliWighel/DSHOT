# DSHOT600 Motor Control Driver

A DMA-based DSHOT600 motor control driver for STM32 microcontrollers with DMA-accelerated signal generation and optional telemetry support.

## Features

- **DMA-based DSHOT600**: Efficient motor control using DMA for signal generation
- **Bi-directional support**: Optional telemetry feedback from ESCs
- **Multi-motor support**: Simultaneous control of up to 4 motors
- **Simple API**: Straightforward initialization and control functions

## Quick Start

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
            ESC_EngineSetSpeedForAll();  // Normal operation
        } else {
            ESC_SetFlagForInit();         // Initialization sequence
        }
    }
}
```

**Important**: Keep the timer frequency consistent with the initialization rate (8kHz in this example).

### Throttle Control

Set motor speeds using `ESC_EngineUpdateDMABuff()`:
```c
ESC_EngineUpdateDMABuff(throttle_value, telemetry_bit);
```

### Telemetry (Optional)

To receive telemetry feedback from your ESC:

1. Connect the ESC telemetry pin to your microcontroller's UART RX pin
2. Set the telemetry bit to 1 when sending commands:
   ```c
   ESC_EngineUpdateDMABuff(throttle_value, 1);  // Request telemetry
   ```
3. Add a UART RX interrupt handler:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       if (huart == &huart1) {
           HAL_UART_AbortReceive(huart);
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

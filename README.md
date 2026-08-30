# STM32 DSHOT Driver

A lightweight DSHOT driver for STM32 using **TIM + DMA in linked-list mode**.

The driver supports:

* **Normal DSHOT**
* **Bidirectional DSHOT telemetry**
* **USART-based ESC telemetry**
* Four ESC/motor channels
* DMA-driven DSHOT waveform generation
* DMA-based bidirectional telemetry capture
* DSHOT telemetry frame decoding, including GCR decoding and CRC checking

The driver is designed to keep CPU usage low by offloading the actual DSHOT signal generation and telemetry capture to the STM32 timers and DMA.

## Features

### DSHOT output

DSHOT frames are generated using a timer PWM peripheral and DMA. A lookup table is used to avoid rebuilding the DSHOT waveform for every transmitted frame.

The driver currently uses **DSHOT600**.

### Bidirectional DSHOT

Bidirectional mode switches the timer between:

1. DSHOT transmission
2. ESC telemetry reception
3. Telemetry decoding
4. Returning to DSHOT transmission

Telemetry capture is performed using timer input capture and DMA.

### USART telemetry

The driver can also receive ESC telemetry through USART using DMA.

The received telemetry includes:

* Temperature
* Voltage
* Current
* Consumption
* RPM

## Initialization

Initialize the driver once during startup:

```c
ESC_Init(1);
```

The argument selects the operating mode:

```c
ESC_Init(0);    // Normal DSHOT
ESC_Init(1);    // Bidirectional DSHOT
```

**Bidirectional DSHOT is significantly more CPU-intensive than normal DSHOT.** In my measurements, bidirectional operation is roughly 5× more expensive computationally than normal DSHOT.

USART telemetry also adds some CPU overhead.

## DSHOT update loop

The DSHOT update is triggered from a timer interrupt. In the example below, `TIM16` is configured to generate an **8 kHz trigger**.

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim14) {
        cpu_usage = ((float)total_cycles / 600000000.0f) * 100.0f;
        total_cycles = 0;
    }

    start_cnt = DWT->CYCCNT;

    if (htim == &htim16) {

        if (esc_ready) {
            ESC_EngineSetSpeedForAll(Throttle_all, 1);
        } else {
            ESC_SetFlagForInit();
        }
    }

    if (htim == &htim17) {
        ESC_BidirectionalTelemetryHandling(EVENT_DATA_RECIEVED);
    }

    cycles = DWT->CYCCNT - start_cnt;
    total_cycles += cycles;
}
```

`TIM16` is used as the DSHOT update trigger. In this example it runs at **8 kHz**.

From my measurements, increasing the update frequency beyond this is not particularly useful for this implementation, while CPU usage increases significantly.

## USART telemetry callback

If USART telemetry is enabled, the USART receive-complete callback should call:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    start_cnt = DWT->CYCCNT;

    if (huart == &huart1) {
        ESC_TelemetryHandling(EVENT_USART_RX);
    }

    cycles = DWT->CYCCNT - start_cnt;
    total_cycles += cycles;
}
```

The driver uses DMA for receiving the telemetry packet, so the CPU is only involved when the packet has been received and needs to be processed.

## CPU usage

CPU usage was measured using the Cortex-M **DWT cycle counter (`DWT->CYCCNT`)**.

Measurements were taken on an:

**STM32H7S3**

with compiler optimization:

```text
-O3
```

### Measured CPU usage

| DSHOT update rate | CPU usage |
| ----------------: | --------: |
|             1 kHz |     ~1.3% |
|             8 kHz |       ~9% |

The measurements include the driver processing associated with the DSHOT update and, when enabled, bidirectional telemetry processing.

The exact CPU usage will depend on the STM32 configuration, clock frequency, compiler version/options, HAL version, and whether bidirectional/USART telemetry is enabled.

### Example measurements

At 8 kHz, the example callback contains approximately:

```c
ESC_EngineSetSpeedForAll(Throttle_all, 1);
```

~**0.35% CPU**

and:

```c
ESC_BidirectionalTelemetryHandling(EVENT_DATA_RECIEVED);
```

~**0.9% CPU**

USART telemetry processing:

```c
ESC_TelemetryHandling(EVENT_USART_RX);
```

~**0.06% CPU**

These numbers are measurements from my setup and should be treated as reference values rather than guaranteed performance figures.

## ESC arming / initialization timing

During initialization, the driver sends zero-throttle DSHOT frames while synchronizing to the DSHOT update timer.

The current implementation contains:

```c
/* Send 40000 zero-throttle pulses for ESC arm (~5 seconds @ 8kHz) */
/* TODO: adjust for DSHOT update time */

for (uint32_t i = 0; i < 5000 * 8; i++) {

    ESC_EngineSetSpeedForAll(motor_speeds, 0);

    /* Synchronize iteration to the 8kHz tick from TIM16 IRQ */
    while (flag == 0);

    flag = 0;
}
```

If you change the DSHOT update frequency, this value may need to be adjusted.

For example, the `5000 * 8` iteration count assumes an **8 kHz** update rate.

## Configuration

Some of the main configuration values are:

```c
#define DSHOT_FRAME_SIZE        16
#define DSHOT_FULL_FRAME_SIZE   17
#define DTELE_FULL_FRAME_SIZE   21

#define DSHOT600_PERIOD         10U
#define DSHOT600_TH1            6U
#define DSHOT600_TH0            3U

#define DTELE_RX_ARR            254U
#define TELEMETRY_PACKET_SIZE   10U

#define ENGINE_POLES            14U

#define MIN_THROTTLE            70U
#define MAX_THROTTLE            2047U
```

`MIN_THROTTLE` is application-specific. In my setup, motors did not reliably spin below this value.

## Requirements

This driver relies on STM32 peripherals/features including:

* TIM
* DMA / GPDMA
* USART
* DWT cycle counter for CPU usage measurements

The implementation was developed and measured on an **STM32H7S3**.

Some peripheral configuration is device-specific, so the CubeMX/HAL configuration will need to be adapted to your STM32 target.

## Performance

The main goal of the implementation is to keep CPU usage low while maintaining relatively high DSHOT update rates.

Most of the DSHOT waveform generation and telemetry capture is performed by hardware:

```text
CPU
 │
 ├── Prepare DSHOT frame
 │
 └── Start DMA
       │
       ▼
     DMA
       │
       ▼
     TIM
       │
       ▼
     ESC
```

For bidirectional DSHOT, the direction is then switched:

```text
DSHOT TX
   │
   ▼
ESC response
   │
   ▼
TIM Input Capture + DMA
   │
   ▼
CPU decoding
   │
   ▼
GCR / CRC / RPM
```

This keeps the CPU largely out of the timing-critical waveform generation and capture process.

## Notes

This is a relatively low-level driver and is currently targeted at my STM32H7S3-based setup. Peripheral configuration, DMA requests, timer configuration, memory placement, and timing constants may need modification when porting it to another STM32 device.

The CPU measurements above are from my own hardware and software configuration and are provided primarily as a performance reference.

## Future improvements

Some areas that could potentially be optimized further include:

* Reducing HAL overhead when starting DMA
* DMA memory-to-memory transfers for DSHOT buffer preparation
* Further optimization of bidirectional telemetry frame reconstruction
* Reducing timer/DMA reconfiguration overhead
* More device-specific register-level optimizations

At the current measured CPU usage, however, the driver already leaves substantial CPU headroom for other control and application tasks.



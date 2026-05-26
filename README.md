# DSHOT600 Motor Control Driver

A high-performance, DMA-accelerated DSHOT motor control driver for STM32 microcontrollers. Provides efficient, non-blocking PWM signal generation with optional bidirectional telemetry support for quadcopter ESCs.

## Features

- **DMA-Accelerated DSHOT600**: Efficient motor control using DMA for signal generation, eliminating CPU overhead
- **Bidirectional Telemetry**: Non-blocking UART DMA-based telemetry feedback (temperature, voltage, current, RPM, etc.)
- **Multi-Motor Control**: Simultaneous control of up to 4 motors with parallel signal generation
- **Hardware Safety**: Built-in timeout detection and state recovery mechanisms
- **CRC Validation**: Automated DSHOT frame checksum computation
- **Simple API**: Straightforward initialization and control functions with minimal configuration

## Hardware Requirements

- **STM32 MCU**: tested on STM32H7S3, but compatible with other
- **Timer**: Timer for PWM signal generation on 4 channels
- **Synchronization Timer**: Timer 16 (TIM16) running at 8kHz
- **DMA Controller**: DMA for both PWM and UART transfers
- **UART**: UART1 for telemetry reception (optional)

## Configuration

### Timer Setup

The driver requires TIM5 configured for PWM generation and TIM16 as a synchronization clock.

**Critical Parameters** (for 100 MHz timer clock):
```c
#define DSHOT600_PERIOD  (168U)     // PWM period (1.68µs)
#define DSHOT600_TH1     (126U)     // Bit 1 duration (~1.25µs)
#define DSHOT600_TH0     (63U)      // Bit 0 duration (~0.625µs)
```

> **⚠️ Important**: These timings assume a 100 MHz timer clock. If your setup differs:
>
> For **other clock speeds**, adjust the prescaler to achieve 100 MHz:
> - Timer input clock ÷ prescaler = 100 MHz
> - Example: 300 MHz ÷ 3 = 100 MHz
>
> For **DSHOT1200** (2x speed): Halve the timer period and duty cycle values
>
> For **DSHOT300** (2x slower) or **DSHOT150** (4x slower): Multiply period and duty values accordingly

### STM32CubeMX Configuration Example


TIM5:
  - Mode: PWM Generation (CH1-4)
  - Prescaler: 3 (for 100 MHz clock from 300 MHz input)
  - Counter Period: 168
  - Pulse (CCR): Any value (overridden by DMA)

TIM16:
  - Mode: Timer (interrupt)
  - Prescaler: 12499 (for 8 kHz from 100 MHz)
  - Counter Period: 1
  - Interrupt: Enabled

DMA:
  - 4 channels for TIM5 PWM DMA transfers
<img width="1463" height="1533" alt="image" src="https://github.com/user-attachments/assets/c1a94513-072f-4d01-afd1-b3395babaf78" />


  - 1 channel for UART1 RX (telemetry)
<img width="1131" height="1629" alt="image" src="https://github.com/user-attachments/assets/a09c9f6d-3b8e-4933-8010-740acd79fd63" />



UART1:
  - Baud: 115200


## API Reference

### Initialization

#### `void ESC_Init(void)`

Initializes the ESC hardware and runs the startup calibration sequence.

**Sequence**:
1. Wait 4 seconds for ESC power supply stabilization
2. Register DMA completion callbacks
3. Enable PWM output on all 4 timer channels
4. Activate the 8kHz synchronization timer (TIM16)
5. Send 40,000 zero-throttle calibration pulses (~5 seconds @ 8kHz)

**Example**:
```c
// In main initialization
ESC_Init();
esc_ready = 1;  // Signal that ESC is ready for commands
```

### Motor Control

#### `void ESC_EngineSetSpeedForAll(uint16_t *motor_speeds, uint8_t telemetry)`

Encodes throttle values and triggers parallel DSHOT600 DMA transmissions.

**Parameters**:
- `motor_speeds`: Array of 4 throttle values [0-2047]
  - 0 = Motor off
  - 70-2047 = Active throttle (MIN_THROTTLE to MAX_THROTTLE)
- `telemetry`: 1 = Request telemetry from next motor in cycle, 0 = No telemetry

**DSHOT Protocol Details**:
- Frame structure: [11-bit Throttle] + [1-bit Telemetry Request] + [4-bit CRC]
- Bit encoding:
  - Logical 1: ~1.25µs high pulse
  - Logical 0: ~0.625µs high pulse
- CRC: XOR-based checksum across 12 bits

**Example**:
```c
// In timer interrupt (8kHz recommended)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim16) {
        if (esc_ready) {
            uint16_t throttles[4] = {500, 500, 500, 500};
            ESC_EngineSetSpeedForAll(throttles, 1);  // Request telemetry
        } else {
            ESC_SetFlagForInit();  // During initialization
        }
    }
}
```

### Telemetry

#### `uint8_t ESC_TelemetryHandling(Event_t event)`

Manages asynchronous telemetry reception with automatic state machine.

**Parameters**:
- `EVENT_START_CYCLE`: Arm a new UART DMA reception window
- `EVENT_USART_RX`: Process received telemetry packet (called from UART RX interrupt)

**Returns**:
- 1-4: Target motor index for current telemetry request
- 0: No active request or interface busy

**Telemetry Data Structure**:
```c
typedef struct {
    uint8_t temperature;    // [°C]
    uint16_t voltage;       // [mV]
    uint16_t current;       // [10mA units]
    uint16_t consumption;   // [mAh]
    uint16_t RPM;          // Rotations per minute
    uint16_t RPS;          // Rotations per second
    uint64_t deb_i_to;     // Timeout packet counter (debug)
} ESC_Telemetry_t;

typedef struct {
    ESC_Telemetry_t tele[4];     // Per-motor telemetry
    float current_total;          // Sum of all motor currents
    uint8_t average_temperature;  // System average temp
} ESC_Status_t;
```

**State Machine**:
- **Idle** → Request telemetry with telemetry bit set in DSHOT command
- **Waiting** → UART DMA receives 10-byte packet asynchronously
- **Timeout Recovery** → Auto-resets after 1000ms if no response
- **Cycle** → Rotates through motors 1→2→3→4→1

**Example**:
```c
// In UART RX interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        ESC_TelemetryHandling(EVENT_USART_RX);
    }
}

// Access telemetry data
extern ESC_Status_t ESC;

// Read motor 1 telemetry
uint8_t motor1_temp = ESC.tele[0].temperature;
uint16_t motor1_rpm = ESC.tele[0].RPM;
uint8_t avg_temp = ESC.average_temperature;
```

#### `void ESC_SetFlagForInit(void)`

Internal synchronization function called from TIM16 interrupt during initialization.

**Called automatically** by the initialization loop; no direct usage needed.

### Internal Functions

#### `void ESC_DmaTxCallback(DMA_HandleTypeDef *hdma)`

DMA transfer completion callback that safely terminates PWM signals.

**Behavior**:
- Disables active DMA channel
- Sets compare register to 0 (PWM low)
- Ensures clean signal termination before next cycle

**Registered automatically** during `ESC_Init()`.

## Usage Example

### Complete Application Flow

```c
#include "main.h"
#include "ESC.h"

uint16_t throttle[4] = {0, 0, 0, 0};
uint8_t esc_ready = 0;

int main(void) {
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Peripheral initialization (GPIO, timers, DMA, UART)
    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_TIM5_Init();
    MX_TIM16_Init();
    MX_USART1_UART_Init();
    
    // ESC initialization and calibration (blocks ~9 seconds)
    ESC_Init();
    esc_ready = 1;
    
    // Main loop
    while (1) {
        // Example: Ramp throttle from 0 to max
        if (throttle[0] < 1977) {
            throttle[0]++;
        } else {
            throttle[0] = 0;
        }
        throttle[1] = throttle[0];
        throttle[2] = throttle[0];
        throttle[3] = throttle[0];
        
        HAL_Delay(100);  // Update every 100ms
    }
}

// Timer interrupt at 8kHz (required for continuous ESC operation)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim16) {
        if (esc_ready) {
            // Request telemetry every frame
            ESC_EngineSetSpeedForAll(throttle, 1);
        } else {
            // Synchronization during initialization
            ESC_SetFlagForInit();
        }
    }
}

// UART RX callback for telemetry
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        ESC_TelemetryHandling(EVENT_USART_RX);
    }
}
```

## Implementation Details

### DMA-Based Signal Generation

The driver uses parallel DMA transfers on TIM5 channels to encode and transmit DSHOT signals without CPU intervention:

1. **Encoding**: DSHOT frame (throttle + telemetry bit + CRC) → 16-bit pattern
2. **DMA Array**: Each frame bit maps to a duty cycle value (TH1 or TH0)
3. **Parallel Transfers**: All 4 motors transmit simultaneously via independent DMA channels
4. **Callback**: DMA completion triggers PWM signal termination

**Timing per frame**:
- 16 bits × 1.68µs = ~26.88µs per DSHOT600 frame
- 8kHz update rate = 125µs between frames
- ~98µs idle time before next transmission

### Telemetry State Machine

The telemetry handler cycles through motors to reduce latency:

```
Frame 1: Send DSHOT with telemetry bit to Motor 1 → Listen for Motor 1 response
Frame 2: Send DSHOT with telemetry bit to Motor 2 → Listen for Motor 2 response
Frame 3: Send DSHOT with telemetry bit to Motor 3 → Listen for Motor 3 response
Frame 4: Send DSHOT with telemetry bit to Motor 4 → Listen for Motor 4 response
→ Repeat
```

**Timeout Recovery**:
- If no response within 1000ms, the state machine auto-recovers
- Timeout counter incremented for diagnostics (see `ESC.tele[x].deb_i_to`)
- Ensures system resilience to ESC communication failures

## Troubleshooting

### Motors Not Spinning

1. Verify ESC calibration completed (wait for confirmation beeps)
2. Check throttle values are above MIN_THROTTLE (70) for active thrust
3. Confirm TIM16 interrupt is firing at 8kHz
4. Verify DMA transfers are not stalled (check `deb_i_to` counters)

### Telemetry Not Received

1. Verify UART1 connection at 115200 baud
2. Check DMA RX channel is properly configured
3. Ensure `HAL_UART_RxCpltCallback()` is being invoked
4. Confirm ESC supports bidirectional DSHOT (some legacy ESCs don't)

### Signal Timing Issues

1. Verify system clock is 300 MHz
2. Confirm prescaler = 3 on TIM5 (100 MHz timer tick)
3. Check `DSHOT600_TH1` and `DSHOT600_TH0` match your clock configuration
4. Validate TIM16 produces 8kHz clock (prescaler = 12499 @ 100 MHz)

## References

- [DSHOT Protocol Overview](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [STM32H7RS Reference Manual](https://www.st.com/resource/en/reference_manual/rm0477-stm32h7r3-and-stm32h7s3-microcontroller-reference-manual-stmicroelectronics.pdf)
- [Bidirectional DSHOT Implementation](https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Project Structure

```
DSHOT/
├── Core/
│   ├── Inc/
│   │   ├── ESC.h           # ESC driver API and data structures
│   │   └── main.h          # MCU configuration
│   └── Src/
│       ├── ESC.c           # ESC driver implementation
│       ├── main.c          # Application entry point
│       ├── tim.c           # Timer initialization
│       └── stm32h7rsxx_it.c # Interrupt handlers
├── STM32H7S3Z8TX_FLASH.ld  # Linker script
├── QDF_ESC_TEST.ioc        # STM32CubeMX project
└── README.md               # This file
```

## Contributing

When modifying the DSHOT timing parameters or adding support for other variants:

1. Maintain the 1.25µs / 0.625µs pulse ratio for DSHOT600
2. Update both `.c` and `.h` timing macros together
3. Test with multiple ESC brands (Betaflight, DShot, etc.)
4. Verify telemetry timeout handling under communication loss

---

**Last Updated**: 2026-05-18  
**Target MCU**: STM32H7RS Series  
**Protocol**: DSHOT600 with Bidirectional Telemetry

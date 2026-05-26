/**
 * @file ESC.c
 * @brief Electronic Speed Controller (ESC) driver for DMA-based DSHOT600 motor control
 *
 * This module handles communication with a 4-channel quadcopter ESC using the DSHOT600 protocol.
 * Features:
 * - Non-blocking PWM signal generation via TIM5 DMA transfers triggered at 8kHz.
 * - Cyclic telemetry pooling via UART1 DMA with asynchronous processing.
 * - Hardware safety locks preventing telemetry request collisions.
 */

#include "main.h"
#include "tim.h"
#include "ESC.h"
#include "string.h"
#include "usart.h"

/* Main ESC status structure containing telemetry data for all 4 motors */
ESC_Status_t ESC;

/* Synchronization flag used exclusively by TIM16 blocking loops during initialization */
volatile uint8_t flag;

/**
 * @brief Initialize ESC hardware and run startup calibration sequence
 *
 * Sequence:
 * 1. Wait 4 seconds for the ESC hardware power rails to stabilize
 * 2. Link DMA completion callbacks on TIM5 to manage clean signal termination
 * 3. Arm PWM output generation across all 4 timer channels
 * 4. Activate the 8kHz synchronization timer (TIM16)
 * 5. Issue 40,000 zero-throttle synchronization pulses to complete ESC arming
 */
void ESC_Init (void) {
	uint16_t motor_speeds[4] = {0};

	/* Allow ESC power supply to stabilize */
	HAL_Delay(4000);

	/* Register callbacks to clear duty cycle immediately following DMA transmission end */
	(&htim5)->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = ESC_DmaTxCallback;

	/* Enable PWM generation on all motor channels */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	/* Start 8kHz timer for synchronization during init */
	HAL_TIM_Base_Start_IT(&htim16);

	/* Send 40000 zero-throttle pulses for ESC arm (~5 seconds @ 8kHz) */
	for (uint32_t i = 0; i < 40000; i ++) {
		ESC_EngineSetSpeedForAll(motor_speeds, 0);

		/* Synchronize iteration to the 8kHz tick from TIM16 IRQ */
		while (flag == 0);
		flag = 0;
	}
}

/**
 * @brief Handle the ESC asynchronous telemetry state machine
 *
 * State Machine Flow:
 * - EVENT_START_CYCLE: Evaluates whether a new UART DMA read window can be armed.
 * If ready, activates non-blocking UART DMA reception and returns the current target motor ID.
 * - EVENT_USART_RX: Triggered exclusively by the UART peripheral interrupt callback.
 * Parses the 10-byte data payload into physical metrics, shifts target focus to the next motor,
 * and frees the communication lock.
 * - Software Timeout: Rescues the interface if a packet drops, resetting state variables after 1000ms.
 *
 * @param event Determines whether to request/arm a new cycle or process an arrived data payload
 * @return uint8_t Target motor index (1 to 4) if a request is valid, 0 otherwise
 */
uint8_t ESC_TelemetryHandling (Event_t event) {
	/* Private RAM buffer dedicated to active UART DMA incoming streams */
	RAM1 static uint8_t usart_dma_buff[TELEMETRY_PACKET_SIZE];

	/* Tracks current motor target index [1-4] across iterations */
	static uint8_t engine = 1;

	/* Hardware interface lock: 1 = Idle/Ready, 0 = Active UART DMA transfer in progress */
	static uint8_t ready_flag = 1;

	/* Tracks background execution ticks for timeout safety checks */
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();

	/* Safety Reset: Recover state machine if an ESC fails to reply within 1 second */
	if ((current_time - last_time >= 1000) && ready_flag == 0) {
		ready_flag = 1;
		ESC.tele[engine - 1].deb_i_to++;  /* Track dropped frame fault counter */
		engine += (engine == 4) ? -3 : 1;  /* Safely rotate target engine window */
	}

	/* Fast-exit trap: Reject concurrent telemetry assertions if previous DMA is still busy */
	if (ready_flag == 0 && event == EVENT_START_CYCLE) {
		return 0;
	}

	/* Phase 1: Arm Non-blocking Reception Window */
	if (ready_flag && event == EVENT_START_CYCLE) {
		if (HAL_UART_Receive_DMA(&huart1, usart_dma_buff, TELEMETRY_PACKET_SIZE) != HAL_OK) {
			ready_flag = 1;  /* Recover state lock if peripheral driver fails */
			return 0;
		}

		last_time = current_time;
		ready_flag = 0; /* Engage interface lock */
		return engine;
	}

	/* Phase 2: Asynchronous Data Extraction (Triggered from UART IRQ context) */
	if (event == EVENT_USART_RX) {
		/* Parse raw telemetry packet fields */
		ESC.tele[engine - 1].temperature = usart_dma_buff[0];
		ESC.tele[engine - 1].voltage     = (usart_dma_buff[1] << 8) | usart_dma_buff[2];
		ESC.tele[engine - 1].current     = (usart_dma_buff[3] << 8) | usart_dma_buff[4];
		ESC.tele[engine - 1].consumption = (usart_dma_buff[5] << 8) | usart_dma_buff[6];

		/* Raw eRPM computation and scaling */
		uint32_t erpm = ((usart_dma_buff[7] << 8) | usart_dma_buff[8]) * 100;
		ESC.tele[engine - 1].RPM		 = erpm / (ENGINE_POLES / 2);
		ESC.tele[engine - 1].RPS		 = ESC.tele[engine - 1].RPM / 60;

		/* Compute arithmetic average of system operating temperatures */
		ESC.average_temperature = ((uint16_t)(ESC.tele[0].temperature + ESC.tele[1].temperature + ESC.tele[2].temperature + ESC.tele[3].temperature) >> 2);

		/* Reset control markers for next cycle */
		last_time = current_time;
		engine += (engine == 4) ? -3 : 1;
		ready_flag = 1;  /* Disengage interface lock */
	}

	return 0;
}

/**
 * @brief Encode throttle metrics and trigger parallel DSHOT600 DMA transmissions
 *
 * DSHOT600 Protocol Standard:
 * - Encoding schema utilizes 16 unique pulse-width durations per frame.
 * - Bit 1 Logic: ~1.2µs High Duration (DSHOT600_TH1)
 * - Bit 0 Logic: ~0.6µs High Duration (DSHOT600_TH0)
 * - Bit Array Structure: [11 Throttle Bits] + [1 Telemetry Request Bit] + [4 Checksum Bits]
 *
 * @param motor_speeds Array containing raw motor speed indexes [0 to 2047]
 * @param telemetry Set to 1 to query telemetry metrics from the rotating engine pool
 */
void ESC_EngineSetSpeedForAll (uint16_t *motor_speeds, uint8_t telemetry) {
	/* Local buffer isolating active transmissions from application race conditions */
	RAM1 static uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE];
	uint8_t telemetry_bit[4] = {0};

	/* Interrogate telemetry arbiter to check state viability */
	if (telemetry) {
		uint8_t temp = ESC_TelemetryHandling(EVENT_START_CYCLE);
		if (temp) {
			telemetry_bit[temp - 1] = 0x01; /* Isolate TRB bit directly to target engine array slot */
		}
	}

	/* Process formatting arrays across all channels */
	for (uint8_t j = 0; j < 4; j++) {
		uint16_t throttle;
		if (motor_speeds[j] != 0) {
			throttle = motor_speeds[j] + MIN_THROTTLE;
			throttle = throttle > MAX_THROTTLE ? MAX_THROTTLE : throttle;
		} else {
			throttle = 0;
		}

		/* Combine components: Frame = [Throttle (11-bits)] + [TRB (1-bit)] */
		uint16_t value = ((throttle << 1) | (telemetry_bit[j] & 0x01));

		/* Compute standardized DSHOT 4-bit cyclic redundancy check (CRC) */
		uint8_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
		uint16_t frame = (value << 4) | crc;

		/* Map the logical frame bitmask onto discrete timer duty cycle values */
		for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++) {
			if (frame & (0x8000 >> i)) {
				dma_buff[j][i] = DSHOT600_TH1;
			} else {
				dma_buff[j][i] = DSHOT600_TH0;
			}
		}
	}

	/* Launch parallel hardware DMA requests on TIM5 channels */
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC1], (uint32_t)dma_buff[0], (uint32_t)&(&htim5)->Instance->CCR1, DSHOT_FULL_FRAME_SIZE * sizeof(uint8_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC2], (uint32_t)dma_buff[1], (uint32_t)&(&htim5)->Instance->CCR2, DSHOT_FULL_FRAME_SIZE * sizeof(uint8_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC3], (uint32_t)dma_buff[2], (uint32_t)&(&htim5)->Instance->CCR3, DSHOT_FULL_FRAME_SIZE * sizeof(uint8_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC4], (uint32_t)dma_buff[3], (uint32_t)&(&htim5)->Instance->CCR4, DSHOT_FULL_FRAME_SIZE * sizeof(uint8_t));

	/* Commit DMA transfer requests to timer execution hardware registers */
	__HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);
}

/**
 * @brief Asserts state flag (Invoked directly from TIM16 Period Elapsed ISR)
 */
void ESC_SetFlagForInit (void) {
	flag = 1;
}

/**
 * @brief DMA transfer completion interrupt vector callback
 *
 * Invoked immediately as individual timer channel transmissions finish. Disables
 * the active DMA block channel and zeroes out comparing configurations to enforce clean
 * logic ground-state line terminations before subsequent cycle starts.
 *
 * @param hdma Handle reference targeting active finishing DMA stream
 */
void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma) {
	if (hdma == (&htim5)->hdma[TIM_DMA_ID_CC1]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC1);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	} else if (hdma == (&htim5)->hdma[TIM_DMA_ID_CC2]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC2);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	} else if(hdma == (&htim5)->hdma[TIM_DMA_ID_CC3]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC3);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
	} else if(hdma == (&htim5)->hdma[TIM_DMA_ID_CC4]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC4);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
	}
}

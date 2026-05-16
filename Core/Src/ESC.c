/**
 * @file ESC.c
 * @brief Electronic Speed Controller (ESC) driver for DMA-based DSHOT600 motor control
 *
 * This module handles communication with 4-channel ESCs using the DSHOT600 protocol.
 * Features:
 * - Non-blocking PWM signal generation via DMA transfers (TIM5)
 * - Per-motor telemetry reception via UART1 with cyclic polling
 * - Initialization sequence and timing synchronization
 */

#include "main.h"
#include "tim.h"
#include "ESC.h"
#include "string.h"
#include "usart.h"
#include "adc.h"

/* Main ESC status structure containing telemetry data for all 4 motors */
ESC_Status_t ESC;

/* DMA buffers in RAM1 - DMA accesable region */
RAM1 uint16_t global_dma_buff[4][DSHOT_FULL_FRAME_SIZE];

/* Current speed setpoint for each motor */
uint16_t Motor_Speeds[4];

/* Flag indicating next frame requires telemetry bit set */
uint8_t Telemetry_bit;

/* Synchronization flag from TIM16 IRQ during initialization */
uint8_t flag;


/**
 * @brief Initialize ESC hardware and run startup calibration sequence
 *
 * Sequence:
 * 1. Wait 4 seconds for ESC to power up
 * 2. Start TIM16 (8kHz tick for synchronization)
 * 3. Register DMA completion callbacks on TIM5 channels
 * 4. Enable PWM output on all 4 motor channels
 * 5. Send 40000 zero-throttle pulses at 8kHz (~5 seconds) for ESC calibration
 */
void ESC_Init (void) {
	uint16_t motor_speeds[4] = {0};

	/* Allow ESC power supply to stabilize */
	HAL_Delay(4000);

	/* Start 8kHz timer for synchronization during init */
	HAL_TIM_Base_Start_IT(&htim16);

	/* Register callbacks to disable PWM after each DMA transfer completes */
	(&htim5)->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = ESC_DmaTxCallback;
	(&htim5)->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = ESC_DmaTxCallback;

	/* Enable PWM generation on all motor channels */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	/* Prepare initial zero-speed DSHOT frames for all motors */
	ESC_EngineUpdateDMABuff(motor_speeds, 0);

	/* Send 40000 zero-throttle pulses for ESC calibration (~5 seconds @ 8kHz) */
	for (uint32_t i = 0; i < 40000; i ++) {
		ESC_EngineSetSpeedForAll();

		/* Synchronize to 8kHz tick from TIM16 */
		while (flag == 0);
		flag = 0;
	}
}

/**
 * @brief Encode motor speeds into DSHOT600 frames and prepare DMA buffers
 *
 * DSHOT600 Protocol:
 * - 16-bit frame: [11-bit throttle | 1-bit telemetry request | 4-bit CRC]
 * - Encoded as 16 pulse widths, MSB first
 * - HIGH pulse: ~1.2µs (DSHOT600_TH1), LOW pulse: ~0.6µs (DSHOT600_TH0)
 *
 * @param motor_speeds  Pointer to array of 4 speed values (range: 0-2047)
 * @param telemetry     Non-zero to request telemetry from one ESC (rotates each call)
 */
void ESC_EngineUpdateDMABuff (uint16_t *motor_speeds, uint8_t telemetry) {
	uint8_t telemetry_bit[4] = {0};

	if (telemetry) {
		Telemetry_bit = 1;
		/* Set telemetry bit for only one engine!!! */
		uint8_t temp = ESC_TelemetryHandling(EVENT_START_CYCLE);
		if (temp) {
			telemetry_bit[temp - 1] = 0x01;
		}
	}

	/* Encode each motor's speed into DSHOT frame */
	for (uint8_t j = 0; j < 4; j++) {
		Motor_Speeds[j] = motor_speeds[j];

		/* Clamp throttle to valid range [MIN_THROTTLE, MAX_THROTTLE] */
		uint16_t throttle;
		if (motor_speeds[j] != 0) {
			throttle = motor_speeds[j] + MIN_THROTTLE;
			throttle = throttle > MAX_THROTTLE ? MAX_THROTTLE : throttle;
		} else {
			throttle = 0;
		}

		/* Construct DSHOT frame: [throttle(11) | telemetry(1) | reserved(1) | CRC(4)] */
		uint16_t value = (((throttle) << 1) | (telemetry_bit[j] & 0x01));
		uint8_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
		uint16_t frame = (value << 4) | crc;

		/* Convert frame to pulse-width array for DMA transfer */
		for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++) {
			if (frame & (0x8000 >> i)) {
				global_dma_buff[j][i] = DSHOT600_TH1;
			} else {
				global_dma_buff[j][i] = DSHOT600_TH0;
			}
		}
	}
}

/**
 * @brief Handle ESC telemetry data exchange
 *
 * - Requests telemetry from one ESC per DSHOT frame
 * - Cycles through motors 1→2→3→4→1...
 * - 1-second timeout if ESC doesn't respond
 * - Parses received telemetry packet into ESC structure
 *
 * @param event  EVENT_START_CYCLE (initiated by ESC_EngineUpdateDMABuff) or
 *               EVENT_USART_RX (called from USART DMA callback)
 * @return Motor index (1-4) if requesting telemetry, 0 otherwise
 */
uint8_t ESC_TelemetryHandling (Event_t event) {
	/* DMA buffer for UART telemetry packet (10 bytes) */
	RAM1 static uint8_t usart_dma_buff[TELEMETRY_PACKET_SIZE];

	/* Currently polled motor (1-4), cycles through on successful reception */
	static uint8_t engine = 1;

	/* 0 = waiting for response, 1 = ready to request next telemetry */
	static uint8_t ready_flag = 1;

	/* Timestamp for timeout detection */
	static uint32_t last_time = 0;

	/* Check for 1-second timeout if no response received */
	uint32_t current_time = HAL_GetTick();
	if ((current_time - last_time >= 1000) && ready_flag == 0) {
		ready_flag = 1;  /* Ready for next attempt */
		ESC.tele[engine - 1].deb_i_to++;  /* Increment timeout counter */
		engine += (engine == 4) ? -3 : 1;  /* Move to next motor */
	}
	last_time = current_time;

	/* Phase 1: Initiate UART DMA reception (from ESC_EngineUpdateDMABuff) */
	if (ready_flag && event == EVENT_START_CYCLE) {
		if (HAL_UART_Receive_DMA(&huart1, usart_dma_buff, TELEMETRY_PACKET_SIZE) != HAL_OK) {
			ready_flag = 1;  /* Retry on error */
			return 0;
		}

		/* Mark as waiting and return motor index for telemetry request */
		ready_flag = 0;
		return engine;
	}

	/* Phase 2: Process received telemetry packet (from USART Rx IRQ) */
	if (event == EVENT_USART_RX) {
		/* convert usart_dma_buff to usable data */
		ESC.tele[engine - 1].temperature = usart_dma_buff[0];
		/* voltage in mV */
		ESC.tele[engine - 1].voltage     =	(usart_dma_buff[1] << 8) | usart_dma_buff[2];
		/* i have 0 here */
		ESC.tele[engine - 1].current     =	(usart_dma_buff[3] << 8) | usart_dma_buff[4];
		/* i have 0 here */
		ESC.tele[engine - 1].consumption = (usart_dma_buff[5] << 8) | usart_dma_buff[6];
		/* convert to real RPM */
		uint32_t erpm = ((usart_dma_buff[7] << 8) | usart_dma_buff[8]) * 100;
		ESC.tele[engine - 1].RPM		 = erpm / (ENGINE_POLES / 2);
		/* convert to RPS, could be used for engines noise filtering */
		ESC.tele[engine - 1].RPS		 = ESC.tele[engine - 1].RPM / 60;

		/* average temperature of all 4 ESCs */
		ESC.average_temperature = ((uint16_t)(ESC.tele[0].temperature + ESC.tele[1].temperature + ESC.tele[2].temperature + ESC.tele[3].temperature) >> 2);


		/* Move to next motor for next telemetry request */
		engine += engine == 4 ? -3 : 1;
		ready_flag = 1;  /* Ready to request telemetry from next motor */
	}

	return 0;
}

/**
 * @brief Transmit all 4 DSHOT600 frames via DMA and prepare next frame
 *
 * Called at 8kHz from TIM16 interrupt during initialization, or directly
 * from main loop during normal operation. Each call sends one complete
 * 16-pulse frame to each of the 4 motors simultaneously via DMA.
 *
 * Procedure:
 * 1. Copy current motor speeds to local buffer (prevents race condition)
 * 2. Start DMA transfers for all 4 PWM channels
 * 3. Enable TIM5 DMA requests
 * 4. If telemetry requested, prepare next frame without telemetry bit
 *
 * Input motor speeds range: 0 to (MAX_THROTTLE - MIN_THROTTLE)
 */
void ESC_EngineSetSpeedForAll () {
	/* Local DMA buffer - prevents tearing if global_dma_buff is updated mid-transfer */
	RAM1 static uint16_t dma_buff[4][DSHOT_FULL_FRAME_SIZE];
	memcpy(dma_buff, global_dma_buff, sizeof(dma_buff));

	/* Initiate DMA transfers for all 4 motor channels */
	/* DMA copies from dma_buff[ch] to TIM5 CCR register via pulse-width modulation */
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC1], (uint32_t)dma_buff[0], (uint32_t)&(&htim5)->Instance->CCR1, DSHOT_FULL_FRAME_SIZE * sizeof(uint16_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC2], (uint32_t)dma_buff[1], (uint32_t)&(&htim5)->Instance->CCR2, DSHOT_FULL_FRAME_SIZE * sizeof(uint16_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC3], (uint32_t)dma_buff[2], (uint32_t)&(&htim5)->Instance->CCR3, DSHOT_FULL_FRAME_SIZE * sizeof(uint16_t));
	HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC4], (uint32_t)dma_buff[3], (uint32_t)&(&htim5)->Instance->CCR4, DSHOT_FULL_FRAME_SIZE * sizeof(uint16_t));

	/* Enable DMA requests on all 4 channels */
	__HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);

	/* If telemetry was sent in this frame, prepare next frame without telemetry bit */
	if (Telemetry_bit) {
		ESC_EngineUpdateDMABuff(Motor_Speeds, 0);  /* Prepare frame with telemetry bit cleared */
		Telemetry_bit = 0;
	}
}

/**
 * @brief Set synchronization flag (called from TIM16 IRQ at 8kHz)
 *
 * Used during initialization to synchronize DSHOT frame transmission
 * to the 8kHz timer tick.
 */
void ESC_SetFlagForInit (void) {
	flag = 1;
}


/**
 * @brief DMA transfer completion callback - stop PWM output for completed channel
 *
 * Called by DMA interrupt when all 16 pulse-width values have been transferred
 * to a PWM channel. Disables DMA requests and sets PWM duty to 0 to ensure
 * clean signal termination before the next frame.
 *
 * Each channel has its own callback registered; they all point to this function.
 *
 * @param hdma  Handle of completed DMA channel
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

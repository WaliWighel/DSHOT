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
volatile uint8_t bidirectional_mode;
volatile ESC_Mode_t esc_mode;


extern DMA_HandleTypeDef handle_GPDMA1_Channel15;
extern DMA_HandleTypeDef handle_GPDMA1_Channel14;
extern DMA_HandleTypeDef handle_GPDMA1_Channel13;
extern DMA_HandleTypeDef handle_GPDMA1_Channel12;


static void ESC_SwitchToRXMode (void);
static void ESC_SwitchToTXMode (void);

// static void ESC_SendDshotCMD (uint16_t cmd);
static void ESC_PrepareDSHOTFrame (uint16_t *values, uint8_t *telemetry_bit, uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]);
static void ESC_SendSignalToESC (uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]);

static void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma);
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
void ESC_Init (uint8_t Bidirectional_mode) {
	uint16_t motor_speeds[4] = {0};
	bidirectional_mode = Bidirectional_mode;
	esc_mode = ESC_Tx_mode;

	if (bidirectional_mode) {
		/* Switch polarity for bidirectional mode */
		TIM_OC_InitTypeDef sConfigOC = {0};

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
	}


	/* Allow ESC power supply to stabilize */
	// HAL_Delay(2000);

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
	for (uint32_t i = 0; i < 5000; i ++) {
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
		ESC.tele[engine - 1].slow_RPM		 = erpm / (ENGINE_POLES / 2);
		ESC.tele[engine - 1].slow_RPS		 = ESC.tele[engine - 1].slow_RPM / 60;

		/* Compute arithmetic average of system operating temperatures */
		ESC.average_temperature = ((uint16_t)(ESC.tele[0].temperature + ESC.tele[1].temperature + ESC.tele[2].temperature + ESC.tele[3].temperature) >> 2);

		/* Reset control markers for next cycle */
		last_time = current_time;
		engine += (engine == 4) ? -3 : 1;
		ready_flag = 1;  /* Disengage interface lock */
	}

	return 0;
}


void ESC_BidirectionalTelemetryHandling (Event_t event) {
	RAM1 static uint8_t dma_buff[4][DTELE_FULL_FRAME_SIZE];
	static const int8_t gcr2bin[32] = {
	    -1, -1, -1, -1, -1, -1, -1, -1,
	    -1,  0x09, 0x0A, 0x0B, -1, 0x0D, 0x0E, 0x0F,
	    -1, -1,  0x02,  0x03,  -1, 0x05, 0x06, 0x07,
	    -1,  0x00,  0x08,  0x01,  -1,  0x04,  0x0C, -1
	};

	if (event == EVENT_START_CYCLE) {
		HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC1], (uint32_t)&(&htim5)->Instance->CCR1, (uint32_t)dma_buff[0], DTELE_FULL_FRAME_SIZE * sizeof(uint8_t));
		HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC2], (uint32_t)&(&htim5)->Instance->CCR2, (uint32_t)dma_buff[1], DTELE_FULL_FRAME_SIZE * sizeof(uint8_t));
		HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC3], (uint32_t)&(&htim5)->Instance->CCR3, (uint32_t)dma_buff[2], DTELE_FULL_FRAME_SIZE * sizeof(uint8_t));
		HAL_DMA_Start_IT((&htim5)->hdma[TIM_DMA_ID_CC4], (uint32_t)&(&htim5)->Instance->CCR4, (uint32_t)dma_buff[3], DTELE_FULL_FRAME_SIZE * sizeof(uint8_t));

 		__HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);

		/* Timeout for rx */
		HAL_TIM_Base_Start_IT(&htim17);
	} else if (event == EVENT_DATA_RECIEVED) {
		uint32_t received_frame[4] = {0};

		HAL_TIM_Base_Stop_IT(&htim17);


		uint8_t edges_cnt[4];
		edges_cnt[0] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel15);
		edges_cnt[1] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel14);
		edges_cnt[2] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel13);
		edges_cnt[3] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel12);


		for (uint8_t i = 0; i < 4; i++) {
			static uint64_t packets_cnt[4] = {0};
			static uint64_t gcr_d_cnt[4] = {0};
			static uint64_t crc_err_cnt[4] = {0};
			packets_cnt[i]++;

			uint8_t bit_count = 0;
			uint8_t current_level = 0;
			uint8_t prev_capture = dma_buff[i][0];

			/* decode frame form dma buffer */
			for (uint8_t j = 1; j < edges_cnt[i]; j++) {
				int16_t raw_diff = (int16_t)dma_buff[i][j] - (int16_t)prev_capture;
				uint8_t interval = (uint8_t)(((raw_diff % (TIM5->ARR + 1)) + (TIM5->ARR + 1)) % (TIM5->ARR + 1));

				prev_capture = dma_buff[i][j];

				uint8_t current_bit_count = (interval + 4) / (DSHOT600_PERIOD * 4 / 5);

				for (uint8_t n = 0; n < current_bit_count; n++) {
					received_frame[i] |= (current_level << (DTELE_FULL_FRAME_SIZE - 1 - n - bit_count));
				}

				bit_count += current_bit_count;
				current_level ^= 1;
			}

			/* fill remaining uncaptured trailing bits at the LSB end */
			while (bit_count < DTELE_FULL_FRAME_SIZE) {
			    received_frame[i] |= (current_level << (DTELE_FULL_FRAME_SIZE - 1 - bit_count));
			    bit_count++;
			}

			/* decode received_frame */
			uint32_t gcr = received_frame[i] ^ (received_frame[i] >> 1);

			/* decode gcr */
			uint16_t tele_payload = 0;
			uint8_t gcr_decode_error = 0;

			for (uint8_t n = 0; n < 4; n++) {
				uint8_t part = (gcr >> (n * 5)) & 0x1F;
				int8_t nibble = gcr2bin[part];

				if (nibble == -1) {
					gcr_decode_error = 1;
					break;

				}
				tele_payload |= (nibble << (n * 4));
			}

			if (gcr_decode_error) {
				gcr_d_cnt[i]++;
				ESC.tele[i].bi_gcr_dec_err_ratio = (float)((float)gcr_d_cnt[i] / (float)packets_cnt[i]);

				continue;
			}

			/* crc check */
			uint8_t crc_rx = (tele_payload & 0x000F);
			uint32_t value_for_crc = ((tele_payload >> 4) & 0x0FFF);
			uint8_t crc_expected = (~(value_for_crc ^ (value_for_crc >> 4) ^ (value_for_crc >> 8))) & 0x0F;

			if (crc_rx != crc_expected) {
				crc_err_cnt[i]++;
				ESC.tele[i].bi_crc_err_ratio = (float)((float)crc_err_cnt[i] / (float)packets_cnt[i]);

				continue;
			}

			/* decoding rpm */
			uint16_t period_base = ((tele_payload & 0x1FF0) >> 4);
			uint8_t erpm_shift = ((tele_payload & 0xE000) >> 13);
			uint32_t period_us = ((uint32_t)period_base << erpm_shift);
			uint32_t erpm = (period_us != 0) ? (60000000UL / period_us) : 0;

			ESC.tele[i].bi_RPM = erpm / (ENGINE_POLES / 2);
			ESC.tele[i].bi_gcr_dec_err_ratio = (float)((float)gcr_d_cnt[i] / (float)packets_cnt[i]);
			ESC.tele[i].bi_crc_err_ratio = (float)((float)crc_err_cnt[i] / (float)packets_cnt[i]);
		}
		ESC_SwitchToTXMode();
	}
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
	uint16_t tmp_speeds[4];

	/* Interrogate telemetry arbiter to check state viability */
	if (telemetry) {
		uint8_t temp = ESC_TelemetryHandling(EVENT_START_CYCLE);
		if (temp) {
			telemetry_bit[temp - 1] = 0x01; /* Isolate TRB bit directly to target engine array slot */
		}
	}

	/* cap speed */
	for (uint8_t j = 0; j < 4; j++) {
		if (motor_speeds[j] != 0) {
			tmp_speeds[j] = motor_speeds[j] + MIN_THROTTLE;
			tmp_speeds[j] = tmp_speeds[j] > MAX_THROTTLE ? MAX_THROTTLE : tmp_speeds[j];
		} else {
			tmp_speeds[j] = 0;
		}
	}

	/* Process formatting arrays across all channels */
	ESC_PrepareDSHOTFrame(tmp_speeds, telemetry_bit, dma_buff);

	/* Launch parallel hardware DMA requests on TIM5 channels */
	ESC_SendSignalToESC(dma_buff);
}

void ESC_SendCMDForAll (uint16_t cmd) {
	/* Local buffer isolating active transmissions from application race conditions */
	RAM1 static uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE];
	uint8_t telemetry_bit[4] = {0};
	uint16_t values[4];

	/* cap cmd */
	cmd = cmd > 47 ? 47 : cmd;

	memset(values, cmd, sizeof(values));

	/* Process formatting arrays across all channels */
	ESC_PrepareDSHOTFrame(values, telemetry_bit, dma_buff);

	/* Launch parallel hardware DMA requests on TIM5 channels */
	ESC_SendSignalToESC(dma_buff);
}

// prepare
static void ESC_PrepareDSHOTFrame (uint16_t *values, uint8_t *telemetry_bit, uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]) {
	for (uint8_t j = 0; j < 4; j++) {
		/* Combine components: Frame = [Throttle (11-bits)] + [dma_buff[i][j]TRB (1-bit)] */
		uint16_t value = ((values[j] << 1) | (telemetry_bit[j] & 0x01));

		/* Compute standardized DSHOT 4-bit cyclic redundancy check (CRC) */
		uint8_t crc = (value ^ (value >> 4) ^ (value >> 8));
		crc = bidirectional_mode ? ~crc : crc;
		crc &= 0x0F;

		uint16_t frame = (value << 4) | crc;

		/* Map the logical frame bitmask onto discrete timer duty cycle values */
		for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++) {
			if (frame & (0x8000 >> i)) {
				dma_buff[j][i] = DSHOT600_TH1;
			} else {
				dma_buff[j][i] = DSHOT600_TH0;
			}
		}

		memset((&dma_buff[j][16]), 0, (DSHOT_FULL_FRAME_SIZE - DSHOT_FRAME_SIZE));
	}
}

static void ESC_SendSignalToESC (uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]) {
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
static void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma) {
	static uint8_t DMA_finished = 0;

	/* if somehow in rx mode */
	if (esc_mode == ESC_Rx_mode) {
		return;
	}

	/* Set 0 (idle state for dshot) on line (esc has pull up resistor) */
	if (hdma == (&htim5)->hdma[TIM_DMA_ID_CC1]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC1);

		DMA_finished |= 1U;

		if (!bidirectional_mode) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		}
	} else if (hdma == (&htim5)->hdma[TIM_DMA_ID_CC2]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC2);

		DMA_finished |= (1U << 1);

		if (!bidirectional_mode) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
		}
	} else if(hdma == (&htim5)->hdma[TIM_DMA_ID_CC3]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC3);

		DMA_finished |= (1U << 2);

		if (!bidirectional_mode) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		}
	} else if(hdma == (&htim5)->hdma[TIM_DMA_ID_CC4]) {
		__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC4);

		DMA_finished |= (1U << 3);

		if (!bidirectional_mode) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		}
	}



	if (DMA_finished != 0x0F || !bidirectional_mode) {
		return;
	}

	/* all channels finished */
	DMA_finished = 0;

	ESC_SwitchToRXMode();
}

static void ESC_SwitchToRXMode (void) {
	TIM_IC_InitTypeDef sConfigIC = {0};

	esc_mode = ESC_Rx_mode;

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0x00;

	TIM5->ARR = 254;
	// TODO dma linked list mode

	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);

	/* Set dma channels */
	handle_GPDMA1_Channel15.Init.Direction = DMA_PERIPH_TO_MEMORY;
	handle_GPDMA1_Channel15.Init.SrcInc = DMA_SINC_FIXED;
	handle_GPDMA1_Channel15.Init.DestInc = DMA_DINC_INCREMENTED;

	handle_GPDMA1_Channel14.Init.Direction = DMA_PERIPH_TO_MEMORY;
	handle_GPDMA1_Channel14.Init.SrcInc = DMA_SINC_FIXED;
	handle_GPDMA1_Channel14.Init.DestInc = DMA_DINC_INCREMENTED;

	handle_GPDMA1_Channel13.Init.Direction = DMA_PERIPH_TO_MEMORY;
	handle_GPDMA1_Channel13.Init.SrcInc = DMA_SINC_FIXED;
	handle_GPDMA1_Channel13.Init.DestInc = DMA_DINC_INCREMENTED;

	handle_GPDMA1_Channel12.Init.Direction = DMA_PERIPH_TO_MEMORY;
	handle_GPDMA1_Channel12.Init.SrcInc = DMA_SINC_FIXED;
	handle_GPDMA1_Channel12.Init.DestInc = DMA_DINC_INCREMENTED;

	HAL_DMA_Init(&handle_GPDMA1_Channel15);
	HAL_DMA_Init(&handle_GPDMA1_Channel14);
	HAL_DMA_Init(&handle_GPDMA1_Channel13);
	HAL_DMA_Init(&handle_GPDMA1_Channel12);

	ESC_BidirectionalTelemetryHandling(EVENT_START_CYCLE);
}

/* Only used in bidirectional_mode, so polarity always is low here */
static void ESC_SwitchToTXMode (void) {
	/* Set timer channels */
	TIM_OC_InitTypeDef sConfigOC = {0};
	esc_mode = ESC_Tx_mode;

	TIM5->ARR = DSHOT600_PERIOD - 1;
	TIM5->CNT = 0;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = bidirectional_mode ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC2);
	__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC3);
	__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC4);

	/* Set dma channels */
	handle_GPDMA1_Channel15.Init.Direction = DMA_MEMORY_TO_PERIPH;
	handle_GPDMA1_Channel15.Init.SrcInc = DMA_SINC_INCREMENTED;
	handle_GPDMA1_Channel15.Init.DestInc = DMA_DINC_FIXED;

	handle_GPDMA1_Channel14.Init.Direction = DMA_MEMORY_TO_PERIPH;
	handle_GPDMA1_Channel14.Init.SrcInc = DMA_SINC_INCREMENTED;
	handle_GPDMA1_Channel14.Init.DestInc = DMA_DINC_FIXED;

	handle_GPDMA1_Channel13.Init.Direction = DMA_MEMORY_TO_PERIPH;
	handle_GPDMA1_Channel13.Init.SrcInc = DMA_SINC_INCREMENTED;
	handle_GPDMA1_Channel13.Init.DestInc = DMA_DINC_FIXED;

	handle_GPDMA1_Channel12.Init.Direction = DMA_MEMORY_TO_PERIPH;
	handle_GPDMA1_Channel12.Init.SrcInc = DMA_SINC_INCREMENTED;
	handle_GPDMA1_Channel12.Init.DestInc = DMA_DINC_FIXED;

	HAL_DMA_Init(&handle_GPDMA1_Channel15);
	HAL_DMA_Init(&handle_GPDMA1_Channel14);
	HAL_DMA_Init(&handle_GPDMA1_Channel13);
	HAL_DMA_Init(&handle_GPDMA1_Channel12);
}

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
#include "gpdma.h"


extern volatile uint32_t cycles;
extern volatile uint32_t start_cnt;
extern volatile uint64_t total_cycles;
extern volatile float cpu_usage;


ESC_Status_t ESC;

volatile uint8_t flag;
uint8_t bidirectional_mode;
ESC_Mode_t esc_mode;

volatile uint8_t tim_cnt_stamp[4];

// TODO, if low on ram, save output of ESC_PrepareLookUpTable(), and make dshot_lookup_table const
static uint8_t dshot_lookup_table[MAX_THROTTLE + 1][2][DSHOT_FULL_FRAME_SIZE];

/* DMA buffers */
RAM1 static uint8_t tele_dma_buff[4][DTELE_FULL_FRAME_SIZE];
RAM1 static uint8_t dshot_dma_buff[4][DSHOT_FULL_FRAME_SIZE];

extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;

DMA_NodeTypeDef TxNode[4];
DMA_NodeTypeDef	RxNode[4];
DMA_QListTypeDef Queue[4];



static void ESC_PrepareLookUpTable (uint8_t look_up_table[MAX_THROTTLE + 1][2][DSHOT_FULL_FRAME_SIZE]);

static void ESC_DMA_LinkedList_cfg (uint8_t ch, DMA_HandleTypeDef *hdma, uint32_t ccr_reg, uint32_t Request, uint8_t Bidirectional_mode);
static void ESC_DMA_Conf (uint8_t Bidirectional_mode);

static void ESC_SwitchToRXMode (void);
static void ESC_SwitchToTXMode (void);

// static void ESC_SendDshotCMD (uint16_t cmd);
static void ESC_PrepareDSHOTFrame (uint16_t *values, uint8_t *telemetry_bit, uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]);
static void ESC_SendSignalToESC (uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]);

static uint8_t Find_First_Edge (uint8_t *first_edge, uint8_t *edges_cnt, uint8_t i);
static uint8_t Validate_Frame (uint8_t edges_cnt, uint8_t i);
static void Build_Frame_From_Edges (uint32_t *received_frame, uint8_t first_edge, uint8_t edges_cnt, uint8_t i);
static uint8_t GCR_Decode (uint16_t *tele_payload, uint32_t received_frame, uint8_t i);
static uint8_t Check_for_CRC_error (uint16_t *payload, uint8_t i);
static void Decode_RPM (uint16_t tele_payload, uint8_t i);
static void Update_Error_Stats (uint8_t i);

static void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma);



void ESC_Init (uint8_t Bidirectional_mode) {
	uint16_t motor_speeds[4] = {0};
	bidirectional_mode = Bidirectional_mode;
	esc_mode = ESC_Tx_mode;

	ESC_PrepareLookUpTable(dshot_lookup_table);

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

	/* Set DMA */
	ESC_DMA_Conf(Bidirectional_mode);

	/* Enable PWM generation on all motor channels */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	/* Start 8kHz timer for synchronization during init */
	HAL_TIM_Base_Start_IT(&htim16);

	/* Send 40000 zero-throttle pulses for ESC arm (~5 seconds @ 8kHz) */
	// TODO adjust for dshot update time
	for (uint32_t i = 0; i < 5000 * 8; i ++) {
		ESC_EngineSetSpeedForAll(motor_speeds, 0);

		/* Synchronize iteration to the 8kHz tick from TIM16 IRQ */
		while (flag == 0);
		flag = 0;
	}
}

uint8_t ESC_TelemetryHandling (Event_t event) {
	RAM1 static uint8_t usart_dma_buff[TELEMETRY_PACKET_SIZE];
	static uint8_t engine = 1;
	static uint8_t ready_flag = 1;
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();

	if ((current_time - last_time >= 1000) && ready_flag == 0) {
		HAL_UART_AbortReceive(&huart1);
		ready_flag = 1;
		ESC.tele[engine - 1].errors.deb_i_to++;
		engine += (engine == 4) ? -3 : 1;
	}

	if (ready_flag == 0 && event == EVENT_START_CYCLE) {
		return 0;
	}

	if (ready_flag && event == EVENT_START_CYCLE) {
		if (HAL_UART_Receive_DMA(&huart1, usart_dma_buff, TELEMETRY_PACKET_SIZE) != HAL_OK) {
			HAL_UART_AbortReceive(&huart1);
			ready_flag = 1;  /* Recover state lock if peripheral driver fails */
			return 0;
		}

		last_time = current_time;
		ready_flag = 0;
		return engine;
	}

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
	if (event == EVENT_START_CYCLE) {
		__HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);
	} else if (event == EVENT_DATA_RECIEVED) {
		// from here to HAL_DMA_Abort(&handle_GPDMA1_Channel3);
		// 0.14 % cpu usage
		HAL_TIM_Base_Stop_IT(&htim17);

		uint8_t edges_cnt[4];

		edges_cnt[0] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel0);
		edges_cnt[1] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel1);
		edges_cnt[2] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel2);
		edges_cnt[3] = DTELE_FULL_FRAME_SIZE - __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel3);

		HAL_DMA_Abort(&handle_GPDMA1_Channel0);
		HAL_DMA_Abort(&handle_GPDMA1_Channel1);
		HAL_DMA_Abort(&handle_GPDMA1_Channel2);
		HAL_DMA_Abort(&handle_GPDMA1_Channel3);

		for (uint8_t i = 0; i < 4; i++) {
			ESC.tele[i].errors.packets_cnt++;


			// 0.11% cpu usage
			uint8_t first_edge = 0;
			if (Find_First_Edge(&first_edge, &edges_cnt[i], i)) {
				continue;
			}
			// 0.01% cpu usage
			if (Validate_Frame(edges_cnt[i], i)) {
				continue;
			}

			// 0.3% cpu usage
			uint32_t received_frame = 0;
			Build_Frame_From_Edges(&received_frame, first_edge, edges_cnt[i], i);

			// 0.07% cpu usage
			uint16_t tele_payload = 0;
			if (GCR_Decode(&tele_payload, received_frame, i)) {
				continue;
			}
			// 0.01% cpu usage
			if (Check_for_CRC_error(&tele_payload, i)) {
				continue;
			}
			// 0.06% cpu usage
			Decode_RPM(tele_payload, i);

			/* Update only once per 1000 packets */
			if ((ESC.tele[i].errors.packets_cnt % 1000) != 0) {
				continue;
			}
			// ~0% cpu usage
			Update_Error_Stats(i);
		}
		ESC_SwitchToTXMode();
	}
}

static uint8_t Find_First_Edge (uint8_t *first_edge, uint8_t *edges_cnt, uint8_t i) {
	uint8_t first_expected_edge = (tim_cnt_stamp[i] + (DSHOT600_PERIOD * 12)) % (DTELE_RX_ARR + 1);

	/* valid data only after 30us from last dshot edge */
	while ((int16_t)((int16_t)tele_dma_buff[i][*first_edge] - (int16_t)first_expected_edge) < 0) {
		(*first_edge)++;
		if (*first_edge >= DTELE_FULL_FRAME_SIZE) {
			return 1;
		}
	}

	(*edges_cnt) -= *first_edge;

	return 0;
}

static uint8_t Validate_Frame (uint8_t edges_cnt, uint8_t i) {
	if ((edges_cnt == 0 || edges_cnt > DTELE_FULL_FRAME_SIZE)) {
		ESC.tele[i].errors.bad_frame_cnt++;
		ESC.tele[i].errors.bi_bad_frame_ratio = (float)(((float)ESC.tele[i].errors.bad_frame_cnt / (float)ESC.tele[i].errors.packets_cnt));

		return 1;
	}
	return 0;
}

static void Build_Frame_From_Edges (uint32_t *received_frame, uint8_t first_edge, uint8_t edges_cnt, uint8_t i) {
	uint8_t bit_count = 0;
	uint8_t current_level = 0;

	uint8_t prev_capture = tele_dma_buff[i][first_edge];

	for (uint8_t j = first_edge + 1; j < edges_cnt + first_edge; j++) {
		int16_t raw_diff = (int16_t)tele_dma_buff[i][j] - (int16_t)prev_capture;
		prev_capture = tele_dma_buff[i][j];

		uint8_t interval = raw_diff < 0 ? (raw_diff + DTELE_RX_ARR + 1) : raw_diff;
		uint8_t current_bit_count = (interval + 4) / (DSHOT600_PERIOD * 4 / 5);

		*received_frame <<= current_bit_count;
		if (current_level) {
			*received_frame |= (1U << current_bit_count) - 1U;
		}

		bit_count += current_bit_count;
		current_level ^= 1;
	}

	/* fill remaining uncaptured trailing bits at the LSB end */
	uint8_t dif = DTELE_FULL_FRAME_SIZE - bit_count;
	if (!dif) {
		return;
	}
	*received_frame <<= dif;
	*received_frame |= (1U << dif) - 1U;
}

static uint8_t GCR_Decode (uint16_t *tele_payload, uint32_t received_frame, uint8_t i) {
	static const int8_t gcr2bin[32] = {
	    -1, -1, -1, -1, -1, -1, -1, -1,
	    -1,  0x09, 0x0A, 0x0B, -1, 0x0D, 0x0E, 0x0F,
	    -1, -1,  0x02,  0x03,  -1, 0x05, 0x06, 0x07,
	    -1,  0x00,  0x08,  0x01,  -1,  0x04,  0x0C, -1
	};

	uint32_t gcr = received_frame ^ (received_frame >> 1);

	for (uint8_t n = 0; n < 4; n++) {
		uint8_t part = (gcr >> (n * 5)) & 0x1F;
		int8_t nibble = gcr2bin[part];

		if (nibble == -1) {
			ESC.tele[i].errors.gcr_d_cnt++;
			ESC.tele[i].errors.bi_gcr_dec_err_ratio = (float)(((float)ESC.tele[i].errors.gcr_d_cnt / (float)ESC.tele[i].errors.packets_cnt));

			return 1;
		}
		*tele_payload |= (nibble << (n * 4));
	}
	return 0;
}

static uint8_t Check_for_CRC_error (uint16_t *payload, uint8_t i) {
	uint8_t crc_rx = (*payload & 0x000F);
	*payload = (*payload >> 4);
	uint32_t value_for_crc = (*payload & 0x0FFF);
	uint8_t crc_expected = (~(value_for_crc ^ (value_for_crc >> 4) ^ (value_for_crc >> 8))) & 0x0F;

	if (crc_rx != crc_expected) {
		ESC.tele[i].errors.crc_err_cnt++;
		ESC.tele[i].errors.bi_crc_err_ratio = (float)(((float)ESC.tele[i].errors.crc_err_cnt / (float)ESC.tele[i].errors.packets_cnt));

		return 1;
	}
	return 0;
}

static void Decode_RPM (uint16_t tele_payload, uint8_t i) {
	uint16_t period_base = ((tele_payload & 0x01FF));
	uint8_t erpm_shift = ((tele_payload & 0x0E00) >> 9);
	uint32_t period_us = ((uint32_t)period_base << erpm_shift);
	uint32_t erpm = (period_us != 0) ? (60000000UL / period_us) : 0;

	ESC.tele[i].bi_RPM = erpm / (ENGINE_POLES / 2);
}

static void Update_Error_Stats (uint8_t i) {
	ESC.tele[i].errors.bi_gcr_dec_err_ratio = (float)(((float)ESC.tele[i].errors.gcr_d_cnt / (float)ESC.tele[i].errors.packets_cnt));
	ESC.tele[i].errors.bi_crc_err_ratio = (float)(((float)ESC.tele[i].errors.crc_err_cnt / (float)ESC.tele[i].errors.packets_cnt));
	ESC.tele[i].errors.bi_bad_frame_ratio = (float)(((float)ESC.tele[i].errors.bad_frame_cnt / (float)ESC.tele[i].errors.packets_cnt));
}

void ESC_EngineSetSpeedForAll (uint16_t *motor_speeds, uint8_t telemetry) {
	uint8_t telemetry_bit[4] = {0};
	uint16_t tmp_speeds[4];

	// 0.11%
	if (telemetry) {
		uint8_t temp = ESC_TelemetryHandling(EVENT_START_CYCLE);
		if (temp) {
			telemetry_bit[temp - 1] = 0x01;
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
	// 0.05%
	ESC_PrepareDSHOTFrame(tmp_speeds, telemetry_bit, dshot_dma_buff);
	// 0.15%
	ESC_SendSignalToESC(dshot_dma_buff);
}



void ESC_SendCMDForAll (uint16_t cmd) {
	uint8_t telemetry_bit[4] = {0};
	uint16_t values[4];

	/* cap cmd */
	cmd = cmd > 47 ? 47 : cmd;
	memset(values, cmd, sizeof(values));

	ESC_PrepareDSHOTFrame(values, telemetry_bit, dshot_dma_buff);
	ESC_SendSignalToESC(dshot_dma_buff);
}

static void ESC_PrepareDSHOTFrame (uint16_t *values, uint8_t *telemetry_bit, uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]) {
	for (uint8_t j = 0; j < 4; j++) {
		memcpy(&dma_buff[j], &dshot_lookup_table[values[j]][telemetry_bit[j]], sizeof(dshot_lookup_table[values[j]][telemetry_bit[j]]));
	}
}

static void ESC_PrepareLookUpTable (uint8_t look_up_table[MAX_THROTTLE + 1][2][DSHOT_FULL_FRAME_SIZE]) {
	for (uint8_t tele = 0; tele < 2; tele++){
		for (uint16_t throttle = 0; throttle < 2048; throttle++) {
			uint16_t value = ((throttle << 1) | (tele & 0x01));

			uint8_t crc = (value ^ (value >> 4) ^ (value >> 8));
			crc = bidirectional_mode ? ~crc : crc;
			crc &= 0x0F;

			uint16_t frame = (value << 4) | crc;

			for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++) {
				if (frame & (0x8000 >> i)) {
					look_up_table[throttle][tele][i] = DSHOT600_TH1;
				} else {
					look_up_table[throttle][tele][i] = DSHOT600_TH0;
				}
			}
			memset((&look_up_table[throttle][tele][16]), 0, (DSHOT_FULL_FRAME_SIZE - DSHOT_FRAME_SIZE));
		}
	}
}

static void ESC_SendSignalToESC (uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]) {
	HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel0);
	HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel1);
	HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2);
	HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3);

	__HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);
}

void ESC_SetFlagForInit (void) {
	flag = 1;
}

static void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma) {
	static uint8_t DMA_finished = 0;

	/* if somehow in rx mode */
	if (esc_mode == ESC_Rx_mode) {
		return;
	}

	if (hdma == &handle_GPDMA1_Channel0) {
			DMA_finished |= 1U;
			tim_cnt_stamp[0] = TIM5->CNT;
			if (!bidirectional_mode) {
				__HAL_TIM_DISABLE_DMA((&htim5), TIM_DMA_CC1);
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			}
	    } else if (hdma == &handle_GPDMA1_Channel1) {
	        DMA_finished |= (1U << 1);
	        tim_cnt_stamp[1] = TIM5->CNT;
			if (!bidirectional_mode) {
				__HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC2);
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			}
	    } else if (hdma == &handle_GPDMA1_Channel2) {
	        DMA_finished |= (1U << 2);
	        tim_cnt_stamp[2] = TIM5->CNT;
			if (!bidirectional_mode) {
				__HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC3);
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
			}
	    } else if (hdma == &handle_GPDMA1_Channel3) {
	        DMA_finished |= (1U << 3);
	        tim_cnt_stamp[3] = TIM5->CNT;
			if (!bidirectional_mode) {
				__HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC4);
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

	/* Timeout for rx, 80us */
	HAL_TIM_Base_Start_IT(&htim17);

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0x00;

	TIM5->ARR = DTELE_RX_ARR;

	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);

	ESC_BidirectionalTelemetryHandling(EVENT_START_CYCLE);
}

/* Only used in bidirectional_mode, so polarity always is low here */
static void ESC_SwitchToTXMode (void) {
	TIM_OC_InitTypeDef sConfigOC = {0};
	esc_mode = ESC_Tx_mode;

	__HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);

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
}

static void ESC_DMA_Conf (uint8_t Bidirectional_mode) {
	ESC_DMA_LinkedList_cfg(0, &handle_GPDMA1_Channel0, (uint32_t)&TIM5->CCR1, GPDMA1_REQUEST_TIM5_CH1, Bidirectional_mode);
	ESC_DMA_LinkedList_cfg(1, &handle_GPDMA1_Channel1, (uint32_t)&TIM5->CCR2, GPDMA1_REQUEST_TIM5_CH2, Bidirectional_mode);
	ESC_DMA_LinkedList_cfg(2, &handle_GPDMA1_Channel2, (uint32_t)&TIM5->CCR3, GPDMA1_REQUEST_TIM5_CH3, Bidirectional_mode);
	ESC_DMA_LinkedList_cfg(3, &handle_GPDMA1_Channel3, (uint32_t)&TIM5->CCR4, GPDMA1_REQUEST_TIM5_CH4, Bidirectional_mode);

	HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &Queue[0]);
	HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel1, &Queue[1]);
	HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &Queue[2]);
	HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &Queue[3]);

	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel0, HAL_DMA_XFER_CPLT_CB_ID, ESC_DmaTxCallback);
	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel1, HAL_DMA_XFER_CPLT_CB_ID, ESC_DmaTxCallback);
	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel2, HAL_DMA_XFER_CPLT_CB_ID, ESC_DmaTxCallback);
	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel3, HAL_DMA_XFER_CPLT_CB_ID, ESC_DmaTxCallback);
}

static void ESC_DMA_LinkedList_cfg (uint8_t ch, DMA_HandleTypeDef *hdma, uint32_t ccr_reg, uint32_t Request, uint8_t Bidirectional_mode) {
	DMA_NodeConfTypeDef cfg = {0};
	cfg.NodeType               = DMA_GPDMA_LINEAR_NODE;
	cfg.Init.Request           = Request;
	cfg.Init.BlkHWRequest      = DMA_BREQ_SINGLE_BURST;
	cfg.Init.SrcDataWidth      = DMA_SRC_DATAWIDTH_BYTE;
	cfg.Init.DestDataWidth     = DMA_DEST_DATAWIDTH_BYTE;
	cfg.Init.SrcBurstLength    = 1;
	cfg.Init.DestBurstLength   = 1;
	cfg.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
	cfg.Init.Mode              = DMA_NORMAL;
	cfg.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
	cfg.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
	cfg.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;

	/* Config TX Node */
	cfg.SrcAddress             = (uint32_t)dshot_dma_buff[ch];
	cfg.DstAddress             = ccr_reg;
	cfg.DataSize               = DSHOT_FULL_FRAME_SIZE * sizeof(uint8_t);
	cfg.Init.Direction         = DMA_MEMORY_TO_PERIPH;
	cfg.Init.SrcInc            = DMA_SINC_INCREMENTED;
	cfg.Init.DestInc           = DMA_DINC_FIXED;

	HAL_DMAEx_List_BuildNode(&cfg, &TxNode[ch]);
	HAL_DMAEx_List_InsertNode_Tail(&Queue[ch], &TxNode[ch]);

	if (!Bidirectional_mode) {
		return;
	}

	/* Config RX Node */
	cfg.SrcAddress             = ccr_reg;
	cfg.DstAddress             = (uint32_t)tele_dma_buff[ch];
	cfg.DataSize               = DTELE_FULL_FRAME_SIZE * sizeof(uint8_t);
	cfg.Init.Direction         = DMA_PERIPH_TO_MEMORY;
	cfg.Init.SrcInc            = DMA_SINC_FIXED;
	cfg.Init.DestInc           = DMA_DINC_INCREMENTED;

	HAL_DMAEx_List_BuildNode(&cfg, &RxNode[ch]);
	HAL_DMAEx_List_InsertNode_Tail(&Queue[ch], &RxNode[ch]);
}

static void ESC_OG_PrepareDSHOTFrame (uint16_t *values, uint8_t *telemetry_bit, uint8_t dma_buff[4][DSHOT_FULL_FRAME_SIZE]) {
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

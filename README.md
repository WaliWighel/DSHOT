DSHOT driver for stm32, with TIM + DMA in linked list mode. Supports Bidirectional and normal DSHOT. USART telemetry is also handled. 

To start driver, place at the begingi of code : 
ESC_Init(1);
where 1 stands for Bidirectional mode. Use 0 for normal mode. Please not that Bidirectional mode is like 5 times more expencive compute whise, compared to normal mode. USART Telemetry also takes a bit of cpu time. 

Next, place this some where, it will handle things : 

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		cpu_usage = ((float)total_cycles / 600000000.0f) * 100.0f;
		total_cycles = 0;
	}

	start_cnt = DWT->CYCCNT;
	if (htim == &htim16) {
		// 0.35 % cpu usage
		if (esc_ready) {
			ESC_EngineSetSpeedForAll(Throttle_all, 1);
		} else {
			ESC_SetFlagForInit();
		}
	}

	if (htim == &htim17) {
		// 0.9 % cpu usage
		ESC_BidirectionalTelemetryHandling(EVENT_DATA_RECIEVED);
	}
	cycles = DWT->CYCCNT - start_cnt;
	total_cycles += cycles;
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart) {
	start_cnt = DWT->CYCCNT;
	if (huart == &huart1) {
		// 0.06% cpu usage, NOTHING
		ESC_TelemetryHandling(EVENT_USART_RX);
	}
	cycles = DWT->CYCCNT - start_cnt;
	total_cycles += cycles;
}

Here htim16 is being used as a triger for dshot signal, and in this example is 8kHz. From my measurements i can say, that it is not wise to make this loop faster. 

When setting update loop frequency, you might need to change in ESC_Init():

	/* Send 40000 zero-throttle pulses for ESC arm (~5 seconds @ 8kHz) */
	// TODO adjust for dshot update time
	for (uint32_t i = 0; i < 5000 * 8; i ++) {
		ESC_EngineSetSpeedForAll(motor_speeds, 0);

		/* Synchronize iteration to the 8kHz tick from TIM16 IRQ */
		while (flag == 0);
		flag = 0;
	}

  a

1.3% CPU usage at 1kHz dshot upadte loop with -O3. ~9% at 8kHz update loop. Measured on STM32H7S3.


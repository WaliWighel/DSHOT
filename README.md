# DSHOT
driver for DMA-based DSHOT600 motor control

# INIT

Call : 
ESC_Init();
to init ESC, this arms ESC and gets it ready for flight. Time for init can difer, so you can change in this function :

	/* Allow ESC power supply to stabilize */
	HAL_Delay(4000);

  and, during this one, your ESC have to emit one low tone bip and one hight tone bip, to tell you that it is ready.

  	/* Send 40000 zero-throttle pulses for ESC calibration (~5 seconds @ 8kHz) */
	for (uint32_t i = 0; i < 40000; i ++) {
		ESC_EngineSetSpeedForAll();

		/* Synchronize to 8kHz tick from TIM16 */
		while (flag == 0);
		flag = 0;
	}

# MAIN LOOP

You need to send signal sonstantly to your ESC, if you don't, it will think that something is wrong and will go off. So for example you can do it as i have done it, in irq handler from timer. 
My timer is trigered at 8kHz, only important think is thai it have to same timing as for init function : 

    void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
  	  if (htim == &htim16) {
  		  if (esc_ready) {
  			  ESC_EngineSetSpeedForAll(); // for operation
  		  } else {
  			  ESC_SetFlagForInit(); // for init function
  		  }
  	  }
    }

# TELEMETRY 

It is posible to get telemetry packets from your ESC. You have to send telemetry bit in your dshot frame. You can do it via, where 1 as a second argument is telemetry bit. : 

    ESC_EngineUpdateDMABuff(Throttle_all, 1);

Code will handle everything for you, you will see you telemetry data in :

     /* Main ESC status structure containing telemetry data for all 4 motors */
    ESC_Status_t ESC;

One importan thing is that you have to connect telemetry pin to UART RX pin to your MPU, and add this in you uart rx handler : 

    void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart) {
    	if (huart == &huart1) {
    		HAL_UART_AbortReceive(huart);
    		ESC_TelemetryHandling(EVENT_USART_RX);
    	}
    }

# MORE ABOUT DSHOT 

Great sorces for more about dshot : 

https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main

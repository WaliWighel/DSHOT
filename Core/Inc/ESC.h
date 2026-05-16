#define DSHOT_FRAME_SIZE 		(16)
#define DSHOT_FULL_FRAME_SIZE   (16)

#define DSHOT600_PERIOD  (168U)
#define DSHOT600_TH1 	 (126U)
#define DSHOT600_TH0 	 (63U)

#define TELEMETRY_PACKET_SIZE	  (10U)
#define ENGINE_POLES			  (14U)

#define CURRENT_ADC_READ_SIZE	  (8U)

/* my motors don't want to spin below this value */
#define MIN_THROTTLE	 (70U)

/* DSHOT max value */
#define MAX_THROTTLE	 (2047U)


typedef struct {
	/* temperature in [°C]*/
	uint8_t temperature;
	/* voltage in [mV] */
	uint16_t voltage;
	/* could be current in [10mA] */
	uint16_t current;
	/* could be consumption in [mAh] */
	uint16_t consumption;
	/* RPM is RPM */
	uint16_t RPM;
	/* RPS is rotates per second */
	uint16_t RPS;
	/* number of timeout packets, for debug purposes */
	uint64_t deb_i_to;
} ESC_Telemetry_t;


typedef struct {
	ESC_Telemetry_t tele[4];

	float current_total;

	uint8_t average_temperature;
} ESC_Status_t;


void ESC_Init (void);
void ESC_SetFlagForInit (void);

void ESC_EngineSetSpeedForAll (void);

void ESC_EngineUpdateDMABuff (uint16_t *motor_speeds, uint8_t telemetry);
uint8_t ESC_TelemetryHandling (Event_t event);

void ESC_DmaTxCallback (DMA_HandleTypeDef *hdma);


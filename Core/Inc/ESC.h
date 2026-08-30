#define DSHOT_FRAME_SIZE 		(16)
#define DSHOT_FULL_FRAME_SIZE   (17)
#define DTELE_FULL_FRAME_SIZE   (21)

/* 1.67us */
#define DSHOT600_PERIOD  (10U)
/* 1.25us */
#define DSHOT600_TH1 	 (6U)
/* 0.625us */
#define DSHOT600_TH0 	 (3U)

#define DTELE_RX_ARR	 (254U)

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
	/* RPM is RPM, received by usart */
	uint16_t slow_RPM;
	/* RPS is rotates per second, received by usart */
	uint16_t slow_RPS;
	/* RPM is RPM */
	uint16_t bi_RPM;
	/* RPS is rotates per second */
	uint16_t bi_RPS;

	float bi_gcr_dec_err_ratio;

	float bi_crc_err_ratio;

	float bi_bad_frame_ratio;
	/* number of timeout packets, for debug purposes */
	uint64_t deb_i_to;
} ESC_Telemetry_t;


typedef struct {
	ESC_Telemetry_t tele[4];

	float current_total;

	uint8_t average_temperature;
} ESC_Status_t;

typedef enum {
	ESC_Tx_mode,
	ESC_Rx_mode
} ESC_Mode_t;


void ESC_Init (uint8_t Bidirectional_mode);
void ESC_SetFlagForInit (void);

void ESC_EngineSetSpeedForAll (uint16_t *motor_speeds, uint8_t telemetry);

uint8_t ESC_TelemetryHandling (Event_t event);
void ESC_BidirectionalTelemetryHandling (Event_t event);



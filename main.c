/******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/


/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

#include "stdio.h"
#include "string.h"
#include <math.h>
#include <sys/stat.h>

static void UART1_Init(void);
static int  WiFi_Init(void);
static int  WiFi_SendFallAlert(float accel_mag, float gyro_mag, int pressure_confirmed);

extern void initialise_monitor_handles(void);

extern int mov_avg(int N, int* accel_buff); // asm implementation

UART_HandleTypeDef huart1;

/*===========================================================================*/
/*                         WIFI CONFIGURATION                                 */
/*===========================================================================*/

char WIFI_SSID[]     = "Paul’s iPhone";
char WIFI_PASSWORD[] = "dontwanttellyou";

static uint8_t server_ip[] = { 172, 20, 10, 2 };   // <-- your PC's IP as byte array
#define SERVER_PORT      5000
#define SERVER_PATH      "/fall-alert"
#define SERVER_HOST      "172.20.10.2"

/* ES-WiFi driver state */
static ES_WIFIObject_t  EsWifiObj;
static uint8_t          wifi_ok = 0;    // 1 = connected and TCP socket open

/* WiFi IO functions (defined in es_wifi_io.c) */
extern int8_t  SPI_WIFI_Init(uint16_t mode);
extern int8_t  SPI_WIFI_DeInit(void);
extern void    SPI_WIFI_Delay(uint32_t Delay);
extern int16_t SPI_WIFI_SendData(const uint8_t *pData, uint16_t len, uint32_t timeout);
extern int16_t SPI_WIFI_ReceiveData(uint8_t *pData, uint16_t len, uint32_t timeout);

/*===========================================================================*/
/*                    FALL DETECTION CONFIGURATION                            */
/*===========================================================================*/

typedef enum {
    STATE_NORMAL,
    STATE_FREEFALL,
    STATE_IMPACT,
    STATE_FALL_DETECTED
} FallState;

#define FREEFALL_THRESHOLD          4.0f
#define IMPACT_THRESHOLD            9.7f
#define GYRO_THRESHOLD            120.0f
#define PRESSURE_CHANGE_THRESHOLD   0.10f
#define FREEFALL_DEBOUNCE_COUNT     2
#define FREEFALL_TIMEOUT_MS         500
#define FALL_ALERT_DURATION_MS      3000
#define BLINK_FAST_MS               100
#define BLINK_SLOW_MS               1000
#define SAMPLE_PERIOD_MS             50


int main(void)
{
	const int N = 4;

	HAL_Init();
	initialise_monitor_handles();

	printf("printf \n");

	UART1_Init();

	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();
	BSP_LED_Off(LED2);

	/* WiFi initialization */
	if (WiFi_Init() != 0)
		printf("WiFi initialization failed — fall alerts will be skipped.\r\n");
	else
		printf("WiFi connected to: %s\r\n", WIFI_SSID);

	int accel_buff_x[4] = {0};
	int accel_buff_y[4] = {0};
	int accel_buff_z[4] = {0};
	int i = 0;

	FallState state           = STATE_NORMAL;
	int freefall_debounce     = 0;
	uint32_t freefall_start   = 0;
	uint32_t fall_alert_start = 0;
	float pressure_baseline   = 0.0f;
	int pressure_confirmed    = 0;
	int alert_sent            = 0;

	uint32_t last_led_toggle  = 0;
	int led_interval          = BLINK_SLOW_MS;

	float current_pressure = BSP_PSENSOR_ReadPressure();
	pressure_baseline = current_pressure;

	while (1)
	{
		uint32_t now = HAL_GetTick();

		/* ---- Non-blocking LED ---- */
		if ((now - last_led_toggle) >= (uint32_t)led_interval)
		{
			BSP_LED_Toggle(LED2);
			last_led_toggle = now;
		}

		/* ---- Accelerometer ---- */
		int16_t accel_data_i16[3] = {0};
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		accel_buff_x[i%4] = accel_data_i16[0];
		accel_buff_y[i%4] = accel_data_i16[1];
		accel_buff_z[i%4] = accel_data_i16[2];

		/* ---- Gyroscope ---- */
		float gyro_data[3] = {0.0};
		BSP_GYRO_GetXYZ(gyro_data);
		float gyro_velocity[3] = {0.0};
		gyro_velocity[0] = gyro_data[0] * 9.8f / 1000.0f;
		gyro_velocity[1] = gyro_data[1] * 9.8f / 1000.0f;
		gyro_velocity[2] = gyro_data[2] * 9.8f / 1000.0f;

		/* ---- Barometer ---- */
		current_pressure = BSP_PSENSOR_ReadPressure();

		/* ---- Filtered accelerometer (assembly) ---- */
		float accel_filt_asm[3] = {0};
		accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8f / 1000.0f);
		accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8f / 1000.0f);
		accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8f / 1000.0f);

		/* ---- Magnitudes ---- */
		float accel_mag = sqrtf(
			accel_filt_asm[0]*accel_filt_asm[0] +
			accel_filt_asm[1]*accel_filt_asm[1] +
			accel_filt_asm[2]*accel_filt_asm[2]);

		float gyro_mag = sqrtf(
			gyro_velocity[0]*gyro_velocity[0] +
			gyro_velocity[1]*gyro_velocity[1] +
			gyro_velocity[2]*gyro_velocity[2]);

		/* ================================================================ */
		/*                        STATE MACHINE                              */
		/* ================================================================ */
		switch (state)
		{
		case STATE_NORMAL:
			led_interval = BLINK_SLOW_MS;
			alert_sent   = 0;
			if (accel_mag < FREEFALL_THRESHOLD && gyro_mag > GYRO_THRESHOLD)
			{
				freefall_debounce++;
				if (freefall_debounce >= FREEFALL_DEBOUNCE_COUNT)
				{
					state             = STATE_FREEFALL;
					freefall_start    = now;
					pressure_baseline = current_pressure;
					pressure_confirmed= 0;
				}
			}
			else { freefall_debounce = 0; }
			break;

		case STATE_FREEFALL:
			if ((now - freefall_start) > FREEFALL_TIMEOUT_MS)
			{
				state = STATE_NORMAL;
				freefall_debounce = 0;
				break;
			}
			if ((current_pressure - pressure_baseline) > PRESSURE_CHANGE_THRESHOLD)
				pressure_confirmed = 1;
			if (accel_mag > IMPACT_THRESHOLD)
				state = STATE_IMPACT;
			break;

		case STATE_IMPACT:
			if (gyro_mag < 60 && accel_mag < 12 && accel_mag > 9.6)
			{
				state            = STATE_FALL_DETECTED;
				fall_alert_start = now;
			}
			else if ((now - freefall_start) > FREEFALL_TIMEOUT_MS)
			{
				state = STATE_NORMAL;
				freefall_debounce = 0;
			}
			break;

		case STATE_FALL_DETECTED:
			led_interval = BLINK_FAST_MS;
			if (!alert_sent)
			{
				printf("Sending fall alert via WiFi...\r\n");
				if (WiFi_SendFallAlert(accel_mag, gyro_mag, pressure_confirmed) == 0)
					printf("Fall alert sent successfully.\r\n");
				else
					printf("Failed to send fall alert.\r\n");
				alert_sent = 1;
			}
			if ((now - fall_alert_start) > FALL_ALERT_DURATION_MS)
			{
				state              = STATE_NORMAL;
				freefall_debounce  = 0;
				pressure_confirmed = 0;
			}
			break;
		}

		/* ================================================================ */
		/*                          UART PRINT                               */
		/* ================================================================ */
		if (i >= 3)
		{
			printf("Accel Mag: %f m/s^2 | Gyro Mag: %f dps | State: %d | Pressure: %f \n",
			       accel_mag, gyro_mag, (int)state, current_pressure);

			if (state == STATE_FALL_DETECTED)
			{
				printf(" | *** FALL DETECTED! ***");
				if (pressure_confirmed)
					printf(" [Barometer confirmed]");
			}
		}

		i++;
		HAL_Delay(SAMPLE_PERIOD_MS);
	}
}

/*===========================================================================*/
/*                         WIFI FUNCTIONS                                     */
/*===========================================================================*/

static int WiFi_Init(void)
{
	/* 1. Register SPI IO callbacks */
	ES_WIFI_RegisterBusIO(&EsWifiObj,
	                      SPI_WIFI_Init,
	                      SPI_WIFI_DeInit,
	                      SPI_WIFI_Delay,
	                      SPI_WIFI_SendData,
	                      SPI_WIFI_ReceiveData);

	/* 2. Init + connect to AP */
	if (ES_WIFI_Init(&EsWifiObj) != ES_WIFI_STATUS_OK)
	{
		printf("ES_WIFI_Init failed\r\n");
		return -1;
	}

    ES_WIFI_APs_t APs;
    if (ES_WIFI_ListAccessPoints(&EsWifiObj, &APs) == ES_WIFI_STATUS_OK)
    {
        printf("Found %d network(s):\r\n", APs.nbr);
        for (int k = 0; k < APs.nbr; k++)
        {
            printf("  [%d] SSID: %s | RSSI: %d dBm\r\n",
                   k, APs.AP[k].SSID, APs.AP[k].RSSI);
        }
    }
    else
    {
        printf("Network scan failed\r\n");
    }

	if (ES_WIFI_Connect(&EsWifiObj, WIFI_SSID, WIFI_PASSWORD, ES_WIFI_SEC_WPA2) != ES_WIFI_STATUS_OK)
	{
	    printf("ES_WIFI_Connect failed\r\n");
	    printf("  Tried SSID    : '%s' (%d chars)\r\n", WIFI_SSID, strlen(WIFI_SSID));
	    printf("  Tried password: '%s' (%d chars)\r\n", WIFI_PASSWORD, strlen(WIFI_PASSWORD));

		printf("ES_WIFI_Connect failed\r\n");
		return -2;
	}

	/* 3. Open TCP socket to server */
	ES_WIFI_Conn_t conn;
	conn.Number     = 0;
	conn.Type       = ES_WIFI_TCP_CONNECTION;
	conn.RemotePort = SERVER_PORT;
	conn.LocalPort  = 0;
	conn.Name       = NULL;
	memcpy(conn.RemoteIP, server_ip, 4);

	if (ES_WIFI_StartClientConnection(&EsWifiObj, &conn) != ES_WIFI_STATUS_OK)
	{
		printf("ES_WIFI_StartClientConnection failed\r\n");
		return -3;
	}

	wifi_ok = 1;
	return 0;
}

static int WiFi_SendFallAlert(float accel_mag, float gyro_mag, int pressure_confirmed)
{
	if (!wifi_ok)
	{
		printf("WiFi not available, skipping alert.\r\n");
		return -1;
	}

	/* Build JSON payload */
	char json_body[256];
	int json_len = snprintf(json_body, sizeof(json_body),
		"{\"event\":\"fall_detected\","
		"\"accel_mag\":%.2f,"
		"\"gyro_mag\":%.2f,"
		"\"pressure_confirmed\":%s}",
		accel_mag, gyro_mag,
		pressure_confirmed ? "true" : "false"
	);

	/* Build HTTP POST request */
	char http_request[512];
	int req_len = snprintf(http_request, sizeof(http_request),
		"POST %s HTTP/1.1\r\n"
		"Host: %s\r\n"
		"Content-Type: application/json\r\n"
		"Content-Length: %d\r\n"
		"Connection: close\r\n"
		"\r\n"
		"%s",
		SERVER_PATH, SERVER_HOST, json_len, json_body
	);

	/* Send over the already-open socket (conn 0) */
	uint16_t bytes_sent = 0;
	if (ES_WIFI_SendData(&EsWifiObj, 0, (uint8_t*)http_request,
	                     (uint16_t)req_len, &bytes_sent, 1000) != ES_WIFI_STATUS_OK)
	{
		printf("ES_WIFI_SendData failed\r\n");
		return -2;
	}

	printf("Sent %d bytes\r\n", bytes_sent);

	/* Read server response */
	uint8_t  rx_buf[256] = {0};
	uint16_t rx_count    = 0;
	ES_WIFI_ReceiveData(&EsWifiObj, 0, rx_buf, sizeof(rx_buf)-1, &rx_count, 2000);
	if (rx_count > 0)
		printf("Server response: %s\r\n", rx_buf);

	return 0;
}

/*===========================================================================*/

static void UART1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin       = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	huart1.Instance            = USART1;
	huart1.Init.BaudRate       = 115200;
	huart1.Init.WordLength     = UART_WORDLENGTH_8B;
	huart1.Init.StopBits       = UART_STOPBITS_1;
	huart1.Init.Parity         = UART_PARITY_NONE;
	huart1.Init.Mode           = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) { while(1); }
}

// Do not modify these lines of code.
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
//int _lseek(int file, int ptr, int dir) { return 0; }
//int _isatty(int file) { return 1; }
//int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }

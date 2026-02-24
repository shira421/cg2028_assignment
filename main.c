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

#include "stdio.h"
#include "string.h"
#include <math.h>
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;

/*===========================================================================*/
/*                    FALL DETECTION CONFIGURATION                            */
/*===========================================================================*/

/* State machine states */
typedef enum {
    STATE_NORMAL,           // normal activity, slow LED blink
    STATE_FREEFALL,         // free-fall detected, waiting for impact
    STATE_IMPACT,           // impact detected, confirming with gyroscope
    STATE_FALL_DETECTED     // fall confirmed, fast LED blink + alert
} FallState;

/* Accelerometer thresholds (m/s^2) */
#define FREEFALL_THRESHOLD      4.0f
#define IMPACT_THRESHOLD        19.6f

/* Gyroscope threshold (degrees per second) */
#define GYRO_THRESHOLD          120.0f

/* Barometer threshold (hPa) */
#define PRESSURE_CHANGE_THRESHOLD  0.10f

/* Debounce and timing */
#define FREEFALL_DEBOUNCE_COUNT    2
#define FREEFALL_TIMEOUT_MS        500
#define FALL_ALERT_DURATION_MS     10000

/* LED blink rates */
#define BLINK_FAST_MS              100
#define BLINK_SLOW_MS              1000

/* Sensor sampling period */
#define SAMPLE_PERIOD_MS           50


int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	initialise_monitor_handles();
	/* UART initialization  */
	UART1_Init();

	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();

	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	int i=0;

	/* Fall detection state variables */
	FallState state = STATE_NORMAL;
	int freefall_debounce     = 0;
	uint32_t freefall_start   = 0;
	uint32_t fall_alert_start = 0;
	float pressure_baseline   = 0.0f;
	int pressure_confirmed    = 0;

	/* Non-blocking LED timing */
	uint32_t last_led_toggle = 0;
	int led_interval = BLINK_SLOW_MS;

	/* Initial pressure reading */
	float current_pressure = BSP_PSENSOR_ReadPressure();
	pressure_baseline = current_pressure;

	while (1)
	{
		uint32_t now = HAL_GetTick();

		/* ---- Non-blocking LED control ---- */
		if ((now - last_led_toggle) >= (uint32_t)led_interval)
		{
			BSP_LED_Toggle(LED2);
			last_led_toggle = now;
		}

		/* ---- Read accelerometer ---- */
		int16_t accel_data_i16[3] = { 0 };
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		accel_buff_x[i%4]=accel_data_i16[0];
		accel_buff_y[i%4]=accel_data_i16[1];
		accel_buff_z[i%4]=accel_data_i16[2];

		/* ---- Read gyroscope ---- */
		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));

		/* ---- Read barometer ---- */
		current_pressure = BSP_PSENSOR_ReadPressure();

		/* ---- Filtered accelerometer (assembly) ---- */
		float accel_filt_asm[3]={0};
		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);

		/* ---- Filtered accelerometer (C reference) ---- */
		float accel_filt_c[3]={0};
		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		/* ---- Compute magnitudes ---- */
		float accel_mag = sqrtf(
			accel_filt_asm[0] * accel_filt_asm[0] +
			accel_filt_asm[1] * accel_filt_asm[1] +
			accel_filt_asm[2] * accel_filt_asm[2]
		);

		float gyro_mag = sqrtf(
			gyro_velocity[0] * gyro_velocity[0] +
			gyro_velocity[1] * gyro_velocity[1] +
			gyro_velocity[2] * gyro_velocity[2]
		);

		/* ================================================================ */
		/*                   STATE MACHINE FALL DETECTION                    */
		/* ================================================================ */

		switch (state)
		{
		case STATE_NORMAL:
			led_interval = BLINK_SLOW_MS;

			if (accel_mag < FREEFALL_THRESHOLD)
			{
				freefall_debounce++;
				if (freefall_debounce >= FREEFALL_DEBOUNCE_COUNT)
				{
					state = STATE_FREEFALL;
					freefall_start = now;
					pressure_baseline = current_pressure;
					pressure_confirmed = 0;
				}
			}
			else
			{
				freefall_debounce = 0;
			}
			break;

		case STATE_FREEFALL:
			if ((now - freefall_start) > FREEFALL_TIMEOUT_MS)
			{
				state = STATE_NORMAL;
				freefall_debounce = 0;
				break;
			}

			if ((current_pressure - pressure_baseline) > PRESSURE_CHANGE_THRESHOLD)
			{
				pressure_confirmed = 1;
			}

			if (accel_mag > IMPACT_THRESHOLD)
			{
				state = STATE_IMPACT;
			}
			break;

		case STATE_IMPACT:
			if (gyro_mag > GYRO_THRESHOLD)
			{
				state = STATE_FALL_DETECTED;
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

			if ((now - fall_alert_start) > FALL_ALERT_DURATION_MS)
			{
				state = STATE_NORMAL;
				freefall_debounce = 0;
				pressure_confirmed = 0;
			}
			break;
		}

		/* ================================================================ */
		/*                       UART TRANSMISSION                          */
		/* ================================================================ */
		char buffer[200];

		if(i>=3)
		{
			// C filtered accelerometer
			printf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			printf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
					accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// Assembly filtered accelerometer
			printf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			printf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
					accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// Gyroscope
			printf(buffer, "Gyroscope sensor readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			printf(buffer, "Gyro X : %f; Gyro Y : %f; Gyro Z : %f;\r\n",
					gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// Barometer
			printf(buffer, "Pressure : %f hPa\r\n", current_pressure);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// Detection status
			printf(buffer, "Accel Mag: %f m/s^2 | Gyro Mag: %f dps | State: %d",
					accel_mag, gyro_mag, (int)state);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			if (state == STATE_FALL_DETECTED)
			{
				printf(buffer, " | *** FALL DETECTED! ***");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

				if (pressure_confirmed)
				{
					printf(buffer, " [Barometer confirmed]");
					HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				}
			}

			printf(buffer, "\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		}

		i++;

		HAL_Delay(SAMPLE_PERIOD_MS);
	}


}

int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result=0;
	for(int i=0; i<N;i++)
	{
		result+=accel_buff[i];
	}

	result=result/4;

	return result;
}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
//int _lseek(int file, int ptr, int dir) { return 0; }
//int _isatty(int file) { return 1; }
//int _close(int file) { return -1; }
int _getpid(void) { return 1; }
//int _kill(int pid, int sig) { return -1; }

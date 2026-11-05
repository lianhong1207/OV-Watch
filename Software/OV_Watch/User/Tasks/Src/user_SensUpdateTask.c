/* Private includes -----------------------------------------------------------*/
//includes
#include "user_TasksInit.h"
#include "user_ScrRenewTask.h"
#include "user_SensUpdateTask.h"
#include "ui_HomePage.h"
#include "ui_MenuPage.h"
#include "ui_SetPage.h"
#include "ui_HRPage.h"
#include "ui_SPO2Page.h"
#include "ui_ENVPage.h"
#include "ui_CompassPage.h"
#include "main.h"

#include "AHT21.h"
#include "LSM303.h"
#include "SPL06_001.h"
#include "em70x8.h"
#include "HrAlgorythm.h"

#include "HWDataAccess.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COMPASS_PAGE_SAMPLE_PERIOD_MS 500U
#define ENV_PAGE_SAMPLE_PERIOD_MS     3000U
#define HR_SAMPLE_PERIOD_MS           50U

/* Private variables ---------------------------------------------------------*/
uint32_t user_HR_timecount=0;         //给心率算法用的时间计数,STM32/FreeRTOS 自己累计出来的当前时间

/* Private function prototypes -----------------------------------------------*/
// 这是EM7028官方lib的库函数, 没有lib用不了
extern uint8_t GET_BP_MAX (void);
extern uint8_t GET_BP_MIN (void);
extern void Blood_Process (void);
extern void Blood_50ms_process (void);
extern void Blood_500ms_process(void);
extern int em70xx_bpm_dynamic(int RECEIVED_BYTE, int g_sensor_x, int g_sensor_y, int g_sensor_z);
extern int em70xx_reset(int ref);


/**
  * @brief  MPU6050 Check the state
  * @param  argument: Not used
  * @retval None
  */
void MPUCheckTask(void *argument)
{
	while(1)
	{
		if(HWInterface.IMU.wrist_is_enabled)
		{
			if(MPU_isHorizontal())
			{
				HWInterface.IMU.wrist_state = WRIST_UP;
			}
			else
			{
				if(WRIST_UP == HWInterface.IMU.wrist_state)
				{
					HWInterface.IMU.wrist_state = WRIST_DOWN;
					if( Page_Get_NowPage()->page_obj == &ui_HomePage ||
						Page_Get_NowPage()->page_obj == &ui_MenuPage ||
						Page_Get_NowPage()->page_obj == &ui_SetPage )
					{
						uint8_t Stopstr;
						osMessageQueuePut(Stop_MessageQueue, &Stopstr, 0, 1);//sleep
					}
				}
				HWInterface.IMU.wrist_state = WRIST_DOWN;
			}
		}
		osDelay(300);
	}
}

/**
  * @brief  Send a command to the HR data task
  * @param  event: HR task event type
  * @retval None
  */
void HRTask_SendEvent(HRTaskEventType_t event)
{
	osMessageQueuePut(HRTaskEvent_MessageQueue, &event, 0, 0);
}

static void Sensor_UpdateHRData(void)
{
	uint8_t idle_break_message = 0;
	uint8_t hr_temp;

	osMessageQueuePut(IdleBreak_MessageQueue,
	                  &idle_break_message,
	                  0,
	                  1);

	if(!HWInterface.HR_meter.ConnectionError)
	{
		vTaskSuspendAll();
		hr_temp = HR_Calculate(EM7028_Get_HRS1(), user_HR_timecount);
		xTaskResumeAll();

		if((HWInterface.HR_meter.HrRate != hr_temp) &&
		   (hr_temp > 50) && (hr_temp < 120))
		{
			HWInterface.HR_meter.HrRate = hr_temp;
		}
	}
}

/**
  * @brief  HR data renew task
  * @param  argument: Not used
  * @retval None
  */
void HRDataUpdateTask(void *argument)
{
	HRTaskEventType_t event;
	uint8_t hr_active = 0;

	while(1)
	{
		uint32_t wait_time = hr_active ? HR_SAMPLE_PERIOD_MS : osWaitForever;
		osStatus_t status = osMessageQueueGet(HRTaskEvent_MessageQueue,
		                                      &event,
		                                      NULL,
		                                      wait_time);

		if(status == osOK)
		{
			switch(event)
			{
				case HR_TASK_EVENT_PAGE_ENTER:
					if(!HWInterface.HR_meter.ConnectionError)
					{
						EM7028_hrs_Enable();
						hr_active = 1;
					}
					else
					{
						hr_active = 0;
					}
					break;

				case HR_TASK_EVENT_PAGE_EXIT:
					hr_active = 0;
					if(!HWInterface.HR_meter.ConnectionError)
					{
						EM7028_hrs_DisEnable();
					}
					break;

				default:
					break;
			}
		}
		else if((status == osErrorTimeout) && hr_active)
		{
			Sensor_UpdateHRData();
		}
	}
}

/**
  * @brief  Send a command to the sensor data task
  * @param  event: Sensor event type
  * @retval None
  */
void SensorData_SendEvent(SensorEventType_t event)
{
	osMessageQueuePut(SensorEvent_MessageQueue, &event, 0, 0);
}

static void Sensor_UpdateHomePageData(void)
{
	HWInterface.Power.power_remain = HWInterface.Power.BatCalculate();
	if((HWInterface.Power.power_remain == 0) ||
	   (HWInterface.Power.power_remain > 100))
	{
		HWInterface.Power.power_remain = 0;
	}

	if(!HWInterface.IMU.ConnectionError)
	{
		HWInterface.IMU.Steps = HWInterface.IMU.GetSteps();
	}

	if(!HWInterface.AHT21.ConnectionError)
	{
		float humi;
		float temp;

		HWInterface.AHT21.GetHumiTemp(&humi, &temp);
		if((temp > -10) && (temp < 50) &&
		   (humi > 0) && (humi < 100))
		{
			HWInterface.AHT21.humidity = humi;
			HWInterface.AHT21.temperature = temp;
		}
	}

	{
		uint8_t data_save_message = 3;
		osMessageQueuePut(DataSave_MessageQueue,
		                  &data_save_message,
		                  0,
		                  1);
	}
}

static void Sensor_UpdateEnvPageData(void)
{
	uint8_t idle_break_message = 0;
	osMessageQueuePut(IdleBreak_MessageQueue,
	                  &idle_break_message,
	                  0,
	                  1);

	if(!HWInterface.AHT21.ConnectionError)
	{
		float humi;
		float temp;

		HWInterface.AHT21.GetHumiTemp(&humi, &temp);
		if((temp > -10) && (temp < 50) &&
		   (humi > 0) && (humi < 100))
		{
			HWInterface.AHT21.temperature = (int8_t)temp;
			HWInterface.AHT21.humidity = (int8_t)humi;
		}
	}
}

static void Sensor_UpdateCompassPageData(void)
{
	uint8_t idle_break_message = 0;
	osMessageQueuePut(IdleBreak_MessageQueue,
	                  &idle_break_message,
	                  0,
	                  1);

	if(!HWInterface.Ecompass.ConnectionError)
	{
		int16_t Xa, Ya, Za, Xm, Ym, Zm;
		float direction;

		LSM303_ReadAcceleration(&Xa, &Ya, &Za);
		LSM303_ReadMagnetic(&Xm, &Ym, &Zm);
		direction = Azimuth_Calculate(Xa, Ya, Za, Xm, Ym, Zm);
		if(direction < 0)
		{
			direction += 360;
		}
		if((direction >= 0) && (direction <= 360))
		{
			HWInterface.Ecompass.direction = (uint16_t)direction;
		}
	}

	if(!HWInterface.Barometer.ConnectionError)
	{
		HWInterface.Barometer.GetPreTempAlti(
			&HWInterface.Barometer.pressure,
			&HWInterface.Barometer.temperature,
			&HWInterface.Barometer.altitude
		);
	}
}

/**
  * @brief  Sensor data update task
  * @param  argument: Not used
  * @retval None
  */
void SensorDataUpdateTask(void *argument)
{
	SensorEventType_t event;
	uint8_t env_page_active = 0;
	uint8_t compass_page_active = 0;

	while(1)
	{
		uint32_t wait_time = osWaitForever;

		if(compass_page_active)
		{
			wait_time = COMPASS_PAGE_SAMPLE_PERIOD_MS;
		}
		else if(env_page_active)
		{
			wait_time = ENV_PAGE_SAMPLE_PERIOD_MS;
		}

		if(osMessageQueueGet(SensorEvent_MessageQueue,
		                     &event,
		                     NULL,
		                     wait_time) == osOK)
		{
			switch(event)
			{
				case SENSOR_EVENT_HOME_PAGE_REFRESH:
					Sensor_UpdateHomePageData();
					break;

				case SENSOR_EVENT_ENV_PAGE_ENTER:
					env_page_active = 1;
					Sensor_UpdateEnvPageData();
					break;

				case SENSOR_EVENT_ENV_PAGE_EXIT:
					env_page_active = 0;
					break;

				case SENSOR_EVENT_COMPASS_PAGE_ENTER:
					compass_page_active = 1;
					if(!HWInterface.Ecompass.ConnectionError)
					{
						LSM303DLH_Wakeup();
					}
					if(!HWInterface.Barometer.ConnectionError)
					{
						HWInterface.Barometer.Wakeup();
					}
					Sensor_UpdateCompassPageData();
					break;

				case SENSOR_EVENT_COMPASS_PAGE_EXIT:
					compass_page_active = 0;
					if(!HWInterface.Ecompass.ConnectionError)
					{
						LSM303DLH_Sleep();
					}
					if(!HWInterface.Barometer.ConnectionError)
					{
						HWInterface.Barometer.Sleep();
					}
					break;

				default:
					break;
			}
		}
		else
		{
			if(compass_page_active)
			{
				Sensor_UpdateCompassPageData();
			}
			else if(env_page_active)
			{
				Sensor_UpdateEnvPageData();
			}
		}
	}
}

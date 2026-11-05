#ifndef __USER_SENSUPDATETASK_H__
#define __USER_SENSUPDATETASK_H__

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t user_HR_timecount;

typedef enum
{
	SENSOR_EVENT_HOME_PAGE_REFRESH = 0,
	SENSOR_EVENT_ENV_PAGE_ENTER,
	SENSOR_EVENT_ENV_PAGE_EXIT,
	SENSOR_EVENT_COMPASS_PAGE_ENTER,
	SENSOR_EVENT_COMPASS_PAGE_EXIT
} SensorEventType_t;

typedef enum
{
	HR_TASK_EVENT_PAGE_ENTER = 0,
	HR_TASK_EVENT_PAGE_EXIT
} HRTaskEventType_t;

void SensorDataUpdateTask(void *argument);
void SensorData_SendEvent(SensorEventType_t event);
void HRDataUpdateTask(void *argument);
void HRTask_SendEvent(HRTaskEventType_t event);
void MPUCheckTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif


#include "reAlarm.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "esp_err.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rLog.h"
#include "reRx433.h"
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#include "def_alarm.h"

static const char* logTAG = "ALARM";
static const char* alarmTaskName = "alarm";

TaskHandle_t _alarmTask;
QueueHandle_t _alarmQueue = nullptr;
ledQueue_t _ledRx433 = nullptr;
ledQueue_t _ledAlarm = nullptr;
ledQueue_t _siren    = nullptr;
ledQueue_t _flasher  = nullptr;

#define ALARM_QUEUE_ITEM_SIZE sizeof(reciever_data_t)
#if CONFIG_ALARM_STATIC_ALLOCATION
StaticQueue_t _alarmQueueBuffer;
StaticTask_t _alarmTaskBuffer;
StackType_t _alarmTaskStack[CONFIG_ALARM_STACK_SIZE];
uint8_t _alarmQueueStorage [CONFIG_ALARM_QUEUE_SIZE * ALARM_QUEUE_ITEM_SIZE];
#endif // CONFIG_ALARM_STATIC_ALLOCATION

#define ERR_CHECK(err, str) if (err != ESP_OK) rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err));
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"
#define ERR_GPIO_SET_ISR  "Failed to set ISR handler"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Zones ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

STAILQ_HEAD(alarmZoneHead_t, alarmZone_t);
typedef struct alarmZoneHead_t *alarmZoneHeadHandle_t;

static alarmZoneHeadHandle_t alarmZones = nullptr;

void alarmZonesInit()
{
  if (!alarmZones) {
    alarmZones = (alarmZoneHeadHandle_t)calloc(1, sizeof(alarmZoneHead_t));
    if (alarmZones) {
      STAILQ_INIT(alarmZones);
    };
  };
}

void alarmZonesFree()
{
  if (alarmZones) {
    alarmZoneHandle_t itemZ, tmpZ;
    STAILQ_FOREACH_SAFE(itemZ, alarmZones, next, tmpZ) {
      STAILQ_REMOVE(alarmZones, itemZ, alarmZone_t, next);
      free(itemZ);
    };
    free(alarmZones);
  };
}

alarmZoneHandle_t alarmZoneAdd(alarm_zone_t type, const char* name)
{
  if (!alarmZones) {
    alarmZonesInit();
  };
  if (alarmZones) {
    alarmZoneHandle_t item = (alarmZoneHandle_t)calloc(1, sizeof(alarmZone_t));
    if (item) {
      item->type = type;
      item->name = name;
      STAILQ_INSERT_TAIL(alarmZones, item, next);
      return item;
    };
  };
  return nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Responses ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmResponsesSet(alarmZoneHandle_t zone, alarm_mode_t mode, uint16_t responses)
{
  if (zone) {
    zone->responses[mode] = responses;
  }
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Sensors ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

STAILQ_HEAD(alarmSensorHead_t, alarmSensor_t);
typedef struct alarmSensorHead_t *alarmSensorHeadHandle_t;

static alarmSensorHeadHandle_t alarmSensors = nullptr;

void alarmSensorsInit()
{
  if (!alarmSensors) {
    alarmSensors = (alarmSensorHeadHandle_t)calloc(1, sizeof(alarmSensorHead_t));
    if (alarmSensors) {
      STAILQ_INIT(alarmSensors);
    };
  };
}

void alarmSensorsFree()
{
  if (alarmSensors) {
    alarmSensorHandle_t itemS, tmpS;
    STAILQ_FOREACH_SAFE(itemS, alarmSensors, next, tmpS) {
      STAILQ_REMOVE(alarmSensors, itemS, alarmSensor_t, next);
      free(itemS);
    };
    free(alarmSensors);
  };
}

alarmSensorHandle_t alarmSensorAdd(alarm_sensor_type_t type, const char* name, uint32_t address)
{
  if (!alarmSensors) {
    alarmSensorsInit();
  };
  if (alarmSensors) {
    alarmSensorHandle_t item = (alarmSensorHandle_t)calloc(1, sizeof(alarmSensor_t));
    if (item) {
      item->name = name;
      item->type = type;
      item->address = address;
      for (uint8_t i = 0; i < CONFIG_ALARM_MAX_EVENTS; i++) {
        item->events[i].type = ASE_EMPTY;
        item->events[i].zone = nullptr;
        item->events[i].value_set = 0;
        item->events[i].value_clr = 0;
        item->events[i].threshold = 0;
        item->events[i].timeout = 0;
        item->events[i].events_count = 0;
        item->events[i].event_last = 0;
      };
      STAILQ_INSERT_TAIL(alarmSensors, item, next);
      return item;
    };
  };
  return nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Sensor events -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmEventSet(alarmSensorHandle_t sensor, alarmZoneHandle_t zone, uint8_t index, 
  alarm_event_t type, uint32_t value_set, uint32_t value_clear, uint16_t threshold, uint32_t timeout)
{
  if ((sensor) && (zone) && (index<CONFIG_ALARM_MAX_EVENTS)) {
    sensor->events[index].type = type;
    sensor->events[index].value_set = value_set;
    sensor->events[index].value_clr = value_clear;
    sensor->events[index].threshold = threshold;
    sensor->events[index].timeout = timeout;
    sensor->events[index].value_set_count = 0;
    sensor->events[index].value_clr_count = 0;
    sensor->events[index].events_count = 0;
    sensor->events[index].event_last = 0;
  };
}

bool alarmEventCheckAddress(reciever_data_t data, alarmSensorHandle_t sensor)
{
  switch (sensor->type) {
    case AST_RX433_GENERIC:
      return (data.source == RTM_RX433) && (data.value == sensor->address);
    case AST_RX433_20A4C:
      return (data.source == RTM_RX433) && ((data.value >> 4) == sensor->address);
    default:
      return (data.source == RTM_WIRED) && (data.address == sensor->address);
  };
  return false;
}

bool alarmEventCheckValueSet(reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
{
  switch (type) {
    case AST_RX433_GENERIC:
      return true;
    case AST_RX433_20A4C:
      return (data.value & 0x0f) == event->value_set;
    default:
      return data.value == event->value_set;
  };
  return false;
}

bool alarmEventCheckValueClr(reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
{
  switch (type) {
    case AST_RX433_GENERIC:
      return false;
    case AST_RX433_20A4C:
      return (data.value & 0x0f) == event->value_clr;
    default:
      return data.value == event->value_clr;
  };
  return false;
}

void alarmEventProcessValueSet(reciever_data_t data, alarmEventHandle_t event)
{
}

void alarmEventProcessValueClr(reciever_data_t data, alarmEventHandle_t event)
{
  
}

void alarmProcessIncomingData(reciever_data_t data)
{
  // Scanning the entire list of sensors
  alarmSensorHandle_t item, sensor = nullptr;
  STAILQ_FOREACH(item, alarmSensors, next) {
    // Check sensor type and address
    if (alarmEventCheckAddress(data, item)) {
      sensor = item;
      // Check values
      for (uint8_t i = 0; i < CONFIG_ALARM_MAX_EVENTS; i++) {
        if (alarmEventCheckValueSet(data, sensor->type, &sensor->events[i])) {
          alarmEventProcessValueSet(data, &sensor->events[i]);
          return;
        } else if (alarmEventCheckValueClr(data, sensor->type, &sensor->events[i])) {
          alarmEventProcessValueClr(data, &sensor->events[i]);
          return;
        };
      };
    };
  };

  if (data.value > 0xff) {
    if (sensor) {
      // Sensor found, but no command defined
      rlog_w(logTAG, "Failed to identify command [0x%.8X / %d] for sensor [%s]!", data.value, data.value, sensor->name);
      #if CONFIG_TELEGRAM_ENABLE && defined(CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED) && CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED
        tgSend(CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED_STATE, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED_TEMPLATE, sensor->name, data.value, data.value);
      #endif // CONFIG_TELEGRAM_ENABLE
    } else {
      // Sensor not found
      rlog_w(logTAG, "Failed to identify event [%d : 0x%.8X]!", data.source, data.value);
      #if CONFIG_TELEGRAM_ENABLE && defined(CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED) && CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED
        tgSend(CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED_TEMPLATE, data.value, data.value);
      #endif // CONFIG_TELEGRAM_ENABLE
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Task function ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmIncomingDataTimer(void* arg)
{

}

void alarmTaskExec(void *pvParameters)
{
  reciever_data_t data;
  esp_timer_create_args_t timer_args;
  memset(&timer_args, 0, sizeof(esp_timer_create_args_t));
  timer_args.callback = &alarmIncomingDataTimer;
  timer_args.name = "alarm_incoming_timer";

  while (1) {
    if (xQueueReceive(_alarmQueue, &data, portMAX_DELAY) == pdPASS) {
      // Send signal to LED
      if (_ledRx433) {
        ledTaskSend(_ledRx433, lmFlash, CONFIG_ALARM_INCOMING_QUANTITY, CONFIG_ALARM_INCOMING_DURATION, CONFIG_ALARM_INCOMING_INTERVAL);
      };

      // Reading the GPIO level for wired events
      if (data.source == RTM_WIRED) {
        data.value = gpio_get_level((gpio_num_t)data.address);
      };

      // Log
      rlog_w(logTAG, "Incoming message:: source: %d, protocol: %d, length: %d, value: 0x%.8X / %d, address: %.8X, command: %X", 
        data.source, data.address, data.length, data.value, data.value, data.value >> 4, data.value & 0x0f);

      // Process message
      alarmProcessIncomingData(data);
    };
  };
  alarmTaskDelete();
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Task routines ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool alarmTaskCreate(ledQueue_t siren, ledQueue_t flasher, ledQueue_t ledAlarm, ledQueue_t ledRx433) 
{
  if (!_alarmTask) {
    _siren = siren;
    _flasher = flasher;
    _ledAlarm = ledAlarm;
    _ledRx433 = ledRx433;
    
    alarmZonesInit();
    alarmSensorsInit();

    if (!_alarmQueue) {
      #if CONFIG_ALARM_STATIC_ALLOCATION
      _alarmQueue = xQueueCreateStatic(CONFIG_ALARM_QUEUE_SIZE, ALARM_QUEUE_ITEM_SIZE, &(_alarmQueueStorage[0]), &_alarmQueueBuffer);
      #else
      _alarmQueue = xQueueCreate(CONFIG_ALARM_QUEUE_SIZE, ALARM_QUEUE_ITEM_SIZE);
      #endif // CONFIG_ALARM_STATIC_ALLOCATION
      if (!_alarmQueue) {
        rloga_e("Failed to create a queue for fire-alarm task!");
        return false;
      };
    };
    
    #if CONFIG_ALARM_STATIC_ALLOCATION
    _alarmTask = xTaskCreateStaticPinnedToCore(alarmTaskExec, alarmTaskName, CONFIG_ALARM_STACK_SIZE, nullptr, CONFIG_ALARM_PRIORITY, _alarmTaskStack, &_alarmTaskBuffer, CONFIG_ALARM_CORE); 
    #else
    xTaskCreatePinnedToCore(alarmTaskExec, alarmTaskName, CONFIG_ALARM_STACK_SIZE, nullptr, CONFIG_ALARM_PRIORITY, &_alarmTask, CONFIG_ALARM_CORE); 
    #endif // CONFIG_ALARM_STATIC_ALLOCATION
    if (_alarmTask == nullptr) {
      vQueueDelete(_alarmQueue);
      rloga_e("Failed to create fire-alarm task!");
      return false;
    }
    else {
      rloga_i("Task [ %s ] has been successfully started", alarmTaskName);
      return true;
    };
  };
  return false;
}

bool alarmTaskSuspend()
{
  if ((_alarmTask) && (eTaskGetState(_alarmTask) != eSuspended)) {
    vTaskSuspend(_alarmTask);
    if (eTaskGetState(_alarmTask) == eSuspended) {
      rloga_d("Task [ %s ] has been suspended", alarmTaskName);
      return true;
    } else {
      rloga_e("Failed to suspend task [ %s ]!", alarmTaskName);
    };
  };
  return false;  
}

bool alarmTaskResume()
{
  if ((_alarmTask) && (eTaskGetState(_alarmTask) == eSuspended)) {
    if (eTaskGetState(_alarmTask) != eSuspended) {
      rloga_i("Task [ %s ] has been successfully resumed", alarmTaskName);
      return true;
    } else {
      rloga_e("Failed to resume task [ %s ]!", alarmTaskName);
    };
  };
  return false;  
}

void alarmTaskDelete()
{
  if (_alarmTask != nullptr) {
    if (_alarmQueue != nullptr) {
      vQueueDelete(_alarmQueue);
      _alarmQueue = nullptr;
    };

    vTaskDelete(_alarmTask);
    _alarmTask = nullptr;
    rloga_d("Task [ %s ] was deleted", alarmTaskName);

    alarmSensorsFree();
    alarmZonesFree();
  };
}

QueueHandle_t alarmTaskQueue()
{
  return _alarmQueue;
}


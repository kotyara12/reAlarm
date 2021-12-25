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
#include "reParams.h"
#include "reEvents.h"
#include "reBeep.h"
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#include "project_config.h"
#include "def_alarm.h"


static const char* logTAG = "ALARM";
static const char* alarmTaskName = "alarm";

TaskHandle_t _alarmTask;
QueueHandle_t _alarmQueue = nullptr;
ledQueue_t _ledRx433 = nullptr;
ledQueue_t _ledAlarm = nullptr;

#define ALARM_QUEUE_ITEM_SIZE sizeof(reciever_data_t)
#if CONFIG_ALARM_STATIC_ALLOCATION
StaticQueue_t _alarmQueueBuffer;
StaticTask_t _alarmTaskBuffer;
StackType_t _alarmTaskStack[CONFIG_ALARM_STACK_SIZE];
uint8_t _alarmQueueStorage[CONFIG_ALARM_QUEUE_SIZE * ALARM_QUEUE_ITEM_SIZE];
#endif // CONFIG_ALARM_STATIC_ALLOCATION

#define ERR_CHECK(err, str) if (err != ESP_OK) rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err));

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Modes ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static alarm_mode_t _alarmMode = ASM_DISABLED;
static paramsEntryHandle_t _alarmParamMode = nullptr;
static cb_alarm_change_mode_t _alarmOnChangeMode = nullptr;
static uint32_t _alarmCount = 0;

static void alarmSirenAlarmOff();
static void alarmFlasherAlarmOff();
static void alarmSirenChangeMode();
static void alarmFlasherChangeMode();
static void alarmBuzzerChangeMode();

static void alarmModeChange(alarm_mode_t newMode, bool forced)
{
  bool alarmModeChanged = newMode != _alarmMode;
  if (forced || alarmModeChanged) {
    if (alarmModeChanged) {
      // Save and publish new value
      _alarmMode = newMode;
      if (_alarmParamMode) {
        paramsValueStore(_alarmParamMode, false);
      };
      // Disable siren if ASM_DISABLED mode is set
      if (newMode == ASM_DISABLED) {
        alarmSirenAlarmOff();
        alarmFlasherAlarmOff();
      };
      // One-time siren signal when switching the arming mode
      alarmSirenChangeMode();
      alarmFlasherChangeMode();
      alarmBuzzerChangeMode();
    };
    
    switch (_alarmMode) {
      // Security mode is on
      case ASM_ARMED:
        rlog_w(logTAG, "Full security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_ARMED, nullptr, 0, portMAX_DELAY);
        #if CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
          tgSend(CONFIG_ALARM_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_ARMED);
        #endif // CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
        break;

      // Perimeter security mode
      case ASM_PERIMETER:
        rlog_w(logTAG, "Perimeter security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_PERIMETER, nullptr, 0, portMAX_DELAY);
        #if CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
          tgSend(CONFIG_ALARM_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_PERIMETER);
        #endif // CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
        break;

      // Outbuilding security regime
      case ASM_OUTBUILDINGS:
        rlog_w(logTAG, "Outbuildings security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_OUTBUILDINGS, nullptr, 0, portMAX_DELAY);
        #if CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
          tgSend(CONFIG_ALARM_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_OUTBUILDINGS);
        #endif // CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
        break;

      // Security mode disabled
      default:
        _alarmCount = 0;
        rlog_w(logTAG, "Security mode disabled");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_DISABLED, nullptr, 0, portMAX_DELAY);
        #if CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
          tgSend(CONFIG_ALARM_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_DISABLED);
        #endif // CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE
        break;
    };
    
    // Callback
    if (_alarmOnChangeMode) {
      _alarmOnChangeMode(_alarmMode);
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Siren ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

ledQueue_t _siren = nullptr;
static uint32_t _sirenDuration = CONFIG_ALARM_DURATION_SIREN;
static esp_timer_handle_t _sirenTimer = nullptr;
static bool _sirenActive = false;

static void alarmSirenTimerEnd(void* arg)
{
  alarmSirenAlarmOff();
}

static bool alarmSirenTimerCreate()
{
  if (_siren) {
    esp_timer_create_args_t siren_timer_args;
    memset(&siren_timer_args, 0, sizeof(esp_timer_create_args_t));
    siren_timer_args.callback = &alarmSirenTimerEnd;
    siren_timer_args.name = "timer_siren";
    ERR_CHECK(esp_timer_create(&siren_timer_args, &_sirenTimer), "Failed to create siren timer");
    return true;
  };
  return false;
}

static bool alarmSirenTimerStart()
{
  if (_sirenTimer) {
    if (_sirenDuration > 0) {
      ERR_CHECK(esp_timer_start_once(_sirenTimer, _sirenDuration * 1000000), "Failed to start siren timer");
    } else {
      if (esp_timer_is_active(_sirenTimer)) {
        ERR_CHECK(esp_timer_stop(_sirenTimer), "Failed to stop siren timer");
      };
    };
    return true;
  };
  return false;
}

static bool alarmSirenTimerStop()
{
  if (_sirenTimer && esp_timer_is_active(_sirenTimer)) {
    ERR_CHECK(esp_timer_stop(_sirenTimer), "Failed to stop siren timer");
  };
  return true;
}

static void alarmSirenSwitch()
{
  if (_siren) {
    if (_sirenActive) {
      rlog_d(logTAG, "Siren activated");
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIREN_ON, nullptr, 0, portMAX_DELAY);
      ledTaskSend(_siren, lmOn, 1, 0, 0);
    } else {
      rlog_d(logTAG, "Siren disabled");
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIREN_OFF, nullptr, 0, portMAX_DELAY);
      ledTaskSend(_siren, lmOff, 1, 0, 0);
    };
  };
}

static void alarmSirenAlarmOn() 
{
  if (!_sirenActive) {
    _sirenActive = true;
    alarmSirenTimerStart();
    alarmSirenSwitch();
  };
}

static void alarmSirenAlarmOff() 
{
  if (_sirenActive) {
    _sirenActive = false;
    alarmSirenTimerStop();
    alarmSirenSwitch();
  };
}

static void alarmSirenChangeMode()
{
  if (_siren) {
    ledTaskSend(_siren, lmOff, 1, 0, 0);
    if (_alarmMode == ASM_DISABLED) {
      if (_alarmCount > 0) {
        #if CONFIG_ALARM_SIREN_DISABLED_WARNING_QUANTITY > 0
          ledTaskSend(_siren, lmFlash, 
            CONFIG_ALARM_SIREN_DISABLED_WARNING_QUANTITY, 
            CONFIG_ALARM_SIREN_DISABLED_WARNING_DURATION, 
            CONFIG_ALARM_SIREN_DISABLED_WARNING_INTERVAL);
        #else
          #if CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY > 0
            ledTaskSend(_siren, lmFlash, 
              CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY, 
              CONFIG_ALARM_SIREN_DISABLED_NORMAL_DURATION, 
              CONFIG_ALARM_SIREN_DISABLED_NORMAL_INTERVAL);
          #endif // CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY
        #endif // CONFIG_ALARM_SIREN_DISABLED_WARNING_QUANTITY
      } else {
        #if CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY > 0
          ledTaskSend(_siren, lmFlash, 
            CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY, 
            CONFIG_ALARM_SIREN_DISABLED_NORMAL_DURATION, 
            CONFIG_ALARM_SIREN_DISABLED_NORMAL_INTERVAL);
        #endif // CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY
      };
    } else if (_alarmMode == ASM_ARMED) {
      #if CONFIG_ALARM_SIREN_ARMED_QUANTITY > 0
        ledTaskSend(_siren, lmFlash, 
          CONFIG_ALARM_SIREN_ARMED_QUANTITY, 
          CONFIG_ALARM_SIREN_ARMED_DURATION, 
          CONFIG_ALARM_SIREN_ARMED_INTERVAL);
      #endif // CONFIG_ALARM_SIREN_ARMED_QUANTITY
    } else {
      #if CONFIG_ALARM_SIREN_PARTIAL_QUANTITY > 0
        ledTaskSend(_siren, lmFlash, 
          CONFIG_ALARM_SIREN_PARTIAL_QUANTITY, 
          CONFIG_ALARM_SIREN_PARTIAL_DURATION, 
          CONFIG_ALARM_SIREN_PARTIAL_INTERVAL);
      #endif // CONFIG_ALARM_SIREN_PARTIAL_QUANTITY
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Buzzer ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void alarmBuzzerChangeMode()
{
  if (_alarmMode == ASM_DISABLED) {
    if (_alarmCount > 0) {
      beepTaskSend(CONFIG_ALARM_BUZZER_DISABLED_WARNING_FREQUENCY, 
        CONFIG_ALARM_BUZZER_DISABLED_WARNING_DURATION, 
        CONFIG_ALARM_BUZZER_DISABLED_WARNING_QUANTITY);
    } else {
      beepTaskSend(CONFIG_ALARM_BUZZER_DISABLED_NORMAL_FREQUENCY, 
        CONFIG_ALARM_BUZZER_DISABLED_NORMAL_DURATION, 
        CONFIG_ALARM_BUZZER_DISABLED_NORMAL_QUANTITY);
    };
  } else if (_alarmMode == ASM_ARMED) {
    beepTaskSend(CONFIG_ALARM_BUZZER_ARMED_FREQUENCY, 
      CONFIG_ALARM_BUZZER_ARMED_DURATION, 
      CONFIG_ALARM_BUZZER_ARMED_QUANTITY);
  } else {
    beepTaskSend(CONFIG_ALARM_BUZZER_PARTIAL_FREQUENCY, 
      CONFIG_ALARM_BUZZER_PARTIAL_DURATION, 
      CONFIG_ALARM_BUZZER_PARTIAL_QUANTITY);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Flasher ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

ledQueue_t _flasher  = nullptr;
static uint32_t _flasherDuration = CONFIG_ALARM_DURATION_FLASH;
static esp_timer_handle_t _flasherTimer = nullptr;
static bool _flasherActive = false;

static void alarmFlasherTimerEnd(void* arg)
{
  alarmFlasherAlarmOff();
}

static bool alarmFlasherTimerCreate()
{
  if (_flasher) {
    esp_timer_create_args_t flasher_timer_args;
    memset(&flasher_timer_args, 0, sizeof(esp_timer_create_args_t));
    flasher_timer_args.callback = &alarmFlasherTimerEnd;
    flasher_timer_args.name = "timer_flasher";
    ERR_CHECK(esp_timer_create(&flasher_timer_args, &_flasherTimer), "Failed to create flasher timer");
    return true;
  };
  return false;
}

static bool alarmFlasherTimerStart()
{
  if (_flasherTimer) {
    if (_flasherDuration > 0) {
      ERR_CHECK(esp_timer_start_once(_flasherTimer, _flasherDuration * 1000000), "Failed to start flasher timer");
    } else {
      if (esp_timer_is_active(_flasherTimer)) {
        ERR_CHECK(esp_timer_stop(_flasherTimer), "Failed to stop flasher timer");
      };
    };
    return true;
  };
  return false;
}

static bool alarmFlasherTimerStop()
{
  if (_flasherTimer && esp_timer_is_active(_flasherTimer)) {
    ERR_CHECK(esp_timer_stop(_flasherTimer), "Failed to stop flasher timer");
  };
  return true;
}

static void alarmFlasherBlinkOn(uint16_t quantity, uint16_t duration, uint16_t interval)
{
  if (quantity > 0) {
    if (_flasher) {
      ledTaskSend(_flasher, lmBlinkOn, quantity, duration, interval);
    };
    if (_ledAlarm) {
      ledTaskSend(_ledAlarm, lmBlinkOn, quantity, duration, interval);
    };
  } else {
    if (_flasher) {
      ledTaskSend(_flasher, lmBlinkOff, 0, 0, 0);
    };
    if (_ledAlarm) {
      ledTaskSend(_ledAlarm, lmBlinkOff, 0, 0, 0);
    };
  };
}

static void alarmFlasherChangeMode()
{
  if (_flasherActive) {
    rlog_d(logTAG, "Flasher activated");
    // Alarm now
    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_ON, nullptr, 0, portMAX_DELAY);
    alarmFlasherBlinkOn(CONFIG_ALARM_ALARM_QUANTITY, CONFIG_ALARM_ALARM_DURATION, CONFIG_ALARM_ALARM_INTERVAL);
  } else {
    if (_alarmMode == ASM_DISABLED) {
      rlog_d(logTAG, "Flasher disabled");
      // Security is fully enabled
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_OFF, nullptr, 0, portMAX_DELAY);
      alarmFlasherBlinkOn(0, 0, 0);
    } else if (_alarmMode == ASM_ARMED) {
      rlog_d(logTAG, "Flasher set fully armed");
      // Security is active
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_BLINK, nullptr, 0, portMAX_DELAY);
      if (_alarmCount == 0) {
        // All is calm, all is well
        alarmFlasherBlinkOn(CONFIG_ALARM_ARMED_QUANTITY, CONFIG_ALARM_ARMED_DURATION, CONFIG_ALARM_ARMED_INTERVAL);
      } else {
        // Something happened since the last reset
        alarmFlasherBlinkOn(CONFIG_ALARM_WARNING_QUANTITY, CONFIG_ALARM_WARNING_DURATION, CONFIG_ALARM_WARNING_INTERVAL);
      };
    } else {
      rlog_d(logTAG, "Flasher set partially armed");
      // Security partially enabled (perimeter only)
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_BLINK, nullptr, 0, portMAX_DELAY);
      alarmFlasherBlinkOn(CONFIG_ALARM_PARTIAL_QUANTITY, CONFIG_ALARM_PARTIAL_DURATION, CONFIG_ALARM_PARTIAL_INTERVAL);
    };
  };
}

static void alarmFlasherAlarmOn() 
{
  if (!_flasherActive) {
    _flasherActive = true;
    alarmFlasherTimerStart();
    alarmFlasherChangeMode();
  };
}

static void alarmFlasherAlarmOff() 
{
  if (_flasherActive) {
    _flasherActive = false;
    alarmFlasherTimerStop();
    alarmFlasherChangeMode();
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Initialization ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void alarmParamsEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if (*(uint32_t*)event_data == (uint32_t)&_alarmMode) {
    if ((event_id == RE_PARAMS_CHANGED) || (event_id == RE_PARAMS_RESTORED))  {
      alarmModeChange(_alarmMode, true);
    };
  };
}

static bool alarmParamsRegister()
{
  paramsGroupHandle_t pgSecurity = paramsRegisterGroup(nullptr, 
    CONFIG_ALARM_PARAMS_ROOT_KEY, CONFIG_ALARM_PARAMS_ROOT_TOPIC, CONFIG_ALARM_PARAMS_ROOT_FRIENDLY);
  
  _alarmParamMode = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_MODE_KEY, CONFIG_ALARM_PARAMS_MODE_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_alarmMode);
  paramsSetLimitsU8(_alarmParamMode, (uint8_t)ASM_DISABLED, (uint8_t)ASM_MAX-1);
  _alarmParamMode->notify = (CONFIG_ALARM_NOTIFY_TELEGRAM_MODE_CHANGE == 0);

  paramsSetLimitsU32(
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_SIREN_DUR_KEY, CONFIG_ALARM_PARAMS_SIREN_DUR_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_sirenDuration),
    CONFIG_ALARM_PARAMS_MIN_DURATION, CONFIG_ALARM_PARAMS_MAX_DURATION);
  paramsSetLimitsU32(
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_FLASHER_DUR_KEY, CONFIG_ALARM_PARAMS_FLASHER_DUR_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_flasherDuration),
    CONFIG_ALARM_PARAMS_MIN_DURATION, CONFIG_ALARM_PARAMS_MAX_DURATION);

  return eventHandlerRegister(RE_PARAMS_EVENTS, ESP_EVENT_ANY_ID, &alarmParamsEventHandler, nullptr);
}

extern bool alarmSystemInit(cb_alarm_change_mode_t cb_mode)
{
  _alarmOnChangeMode = cb_mode;

  return alarmSirenTimerCreate() 
      && alarmFlasherTimerCreate() 
      && alarmParamsRegister();
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Zones --------------------------------------------------------
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

alarmZoneHandle_t alarmZoneAdd(const char* name, 
  char* topic_local, char* topic_public, cb_mqtt_publish_t cb_mqtt_publish, 
  cb_relay_control_t cb_relay_ctrl)
{
  if (!alarmZones) {
    alarmZonesInit();
  };
  if (alarmZones) {
    alarmZoneHandle_t item = (alarmZoneHandle_t)calloc(1, sizeof(alarmZone_t));
    if (item) {
      item->name = name;
      item->mqtt_topic_local = topic_local;
      item->mqtt_topic_public = topic_public;
      item->mqtt_publish = cb_mqtt_publish;
      item->relay_ctrl = cb_relay_ctrl;
      item->relay_state = false;
      for (size_t i = 0; i < ASM_MAX; i++) {
        item->resp_set[i] = ASRS_NONE;
        item->resp_clr[i] = ASRS_NONE;
      };
      STAILQ_INSERT_TAIL(alarmZones, item, next);
      return item;
    };
  };
  return nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Notifications -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Responses ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmResponsesSet(alarmZoneHandle_t zone, alarm_mode_t mode, uint16_t resp_set, uint16_t resp_clr)
{
  if (zone) {
    zone->resp_set[mode] = resp_set;
    zone->resp_clr[mode] = resp_clr;
  }
}

void alarmResponsesProcess(bool state, alarmEventData_t event_data)
{
  uint16_t responses = 0;
  if (state) {
    rlog_d(logTAG, "Alarm signal for sensor: [ %s ], zone: [ %s ], type: [ %d ]", 
      event_data.sensor->name, event_data.event->zone->name, event_data.event->type);
 
    responses = event_data.event->zone->resp_set[_alarmMode];
    event_data.event->event_last = time(nullptr);
    event_data.event->events_count++;
    event_data.event->state = true;

    if ((_alarmMode != ASM_DISABLED) && (responses & ASR_COUNT_INC) && (_alarmCount < UINT32_MAX)) {
      _alarmCount++;
    };

    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIGNAL_SET, &event_data, sizeof(alarmEventData_t), portMAX_DELAY);
  } else {
    rlog_d(logTAG, "Clear signal for sensor: [ %s ], zone: [ %s ], type: [ %d ]", 
      event_data.sensor->name, event_data.event->zone->name, event_data.event->type);

    responses = event_data.event->zone->resp_clr[_alarmMode];
    event_data.event->state = false;

    if ((_alarmMode != ASM_DISABLED) && (responses & ASR_COUNT_DEC) && (_alarmCount > 0)) {
      _alarmCount--;
    };

    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIGNAL_CLEAR, &event_data, sizeof(alarmEventData_t), portMAX_DELAY);
  };

  // Handling arming switch events
  if (event_data.event->type == ASE_RCTRL_OFF) {
    // If the alarm is currently on, then first we just reset the alarm
    if (_sirenActive || _flasherActive) {
      alarmSirenAlarmOff();
      alarmFlasherAlarmOff();
    } else {
      alarmModeChange(ASM_DISABLED, false);
    };
  } else if (event_data.event->type == ASE_RCTRL_ON) {
    alarmModeChange(ASM_ARMED, false);
  } else if (event_data.event->type == ASE_RCTRL_PERIMETER) {
    alarmModeChange(ASM_PERIMETER, false);
  } else if (event_data.event->type == ASE_RCTRL_OUTBUILDINGS) {
    alarmModeChange(ASM_OUTBUILDINGS, false);
  };

  // Posting status on MQTT
  if (responses & ASR_MQTT_LOCAL) {
  };
  if (responses & ASR_MQTT_PUBLIC) {
  };

  // Sound and visual notification
  if (responses & ASR_SIREN) {
    if (state) {
      alarmSirenAlarmOn();
    };
  };
  if (responses & ASR_FLASHER) {
    if (state) {
      alarmFlasherAlarmOn();
    };
  };
  if (responses & ASR_BUZZER) {
    if (state) {
      // alarmSirenAlarmOn();
    };
  };

  // Relay control
  if (responses & ASR_RELAY_ON) {
    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_RELAY_ON, nullptr, 0, portMAX_DELAY);
    if (event_data.event->zone->relay_ctrl) {
      event_data.event->zone->relay_state = event_data.event->zone->relay_ctrl(true);
    };
  };
  if (responses & ASR_RELAY_OFF) {
    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_RELAY_OFF, nullptr, 0, portMAX_DELAY);
    if (event_data.event->zone->relay_ctrl) {
      event_data.event->zone->relay_state = event_data.event->zone->relay_ctrl(false);
    };
  };
  if (responses & ASR_RELAY_SWITCH) {
    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_RELAY_TOGGLE, nullptr, 0, portMAX_DELAY);
    if (event_data.event->zone->relay_ctrl) {
      event_data.event->zone->relay_state = event_data.event->zone->relay_ctrl(!event_data.event->zone->relay_state);
    };
  };

  // Sending notifications
  if (responses & ASR_EMAIL) {
  };
  if (responses & ASR_TELEGRAM) {
  };
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
    sensor->events[index].zone = zone;
    sensor->events[index].state = false;
    sensor->events[index].value_set = value_set;
    sensor->events[index].value_clr = value_clear;
    sensor->events[index].threshold = threshold;
    sensor->events[index].timeout = timeout;
    sensor->events[index].events_count = 0;
    sensor->events[index].event_last = 0;
  };
}

bool alarmEventCheckAddress(const reciever_data_t data, alarmSensorHandle_t sensor)
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

bool alarmEventCheckValueSet(const reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
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

bool alarmEventCheckValueClr(const reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
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

bool alarmProcessIncomingData(const reciever_data_t data, bool end_of_packet)
{
  // Log
  rlog_w(logTAG, "Incoming message:: source: %d, protocol: %d, count: %d, value: 0x%.8X / %d, address: %.8X, command: %02X", 
    data.source, data.address, data.count, data.value, data.value, data.value >> 4, data.value & 0x0f);

  // Scanning the entire list of sensors
  alarmSensorHandle_t item, sensor = nullptr;
  STAILQ_FOREACH(item, alarmSensors, next) {
    // Check sensor type and address
    if (alarmEventCheckAddress(data, item)) {
      sensor = item;
      // Check values
      for (uint8_t i = 0; i < CONFIG_ALARM_MAX_EVENTS; i++) {
        if (sensor->events[i].type != ASE_EMPTY) {
          if (alarmEventCheckValueSet(data, sensor->type, &sensor->events[i])) {
            if (data.count >= sensor->events[i].threshold) {
              alarmEventData_t event_data = {sensor, &sensor->events[i]};  
              alarmResponsesProcess(true, event_data);
              return true;
            } else {
              return false;
            };
          } else if (alarmEventCheckValueClr(data, sensor->type, &sensor->events[i])) {
            if (data.count >= sensor->events[i].threshold) {
              alarmEventData_t event_data = {sensor, &sensor->events[i]};  
              alarmResponsesProcess(false, event_data);
              return true;
            } else {
              return false;
            };
          };
        };
      };
    };
  };

  if (end_of_packet && (data.value > 0xffff)) {
    if (sensor) {
      // Sensor found, but no command defined
      rlog_w(logTAG, "Failed to identify command [0x%.8X] for sensor [ %s ]!", data.value, sensor->name);
      #if CONFIG_TELEGRAM_ENABLE && defined(CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED) && CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED
        tgSend(CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED_STATE, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED_TEMPLATE, sensor->name, data.value, data.value >> 4, data.value & 0x0f);
      #endif // CONFIG_TELEGRAM_ENABLE
    } else {
      // Sensor not found
      rlog_w(logTAG, "Failed to identify event [%d : 0x%.8X]!", data.source, data.value);
      #if CONFIG_TELEGRAM_ENABLE && defined(CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED) && CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED
        tgSend(CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED_TEMPLATE, data.address, data.value, data.value >> 4, data.value & 0x0f);
      #endif // CONFIG_TELEGRAM_ENABLE
    };
  };

  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Task function ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmTaskExec(void *pvParameters)
{
  static reciever_data_t data;
  static reciever_data_t last = {RTM_NONE, 0, 0, 0};
  static bool signal_processed = false;
  static TickType_t queueWait = portMAX_DELAY;
  while (1) {
    if (xQueueReceive(_alarmQueue, &data, queueWait) == pdPASS) {
      // Send signal to LED
      if (_ledRx433) {
        ledTaskSend(_ledRx433, lmFlash, CONFIG_ALARM_INCOMING_QUANTITY, CONFIG_ALARM_INCOMING_DURATION, CONFIG_ALARM_INCOMING_INTERVAL);
      };

      // Process received signal RX433
      if (data.source == RTM_RX433) {
        if ((data.source == last.source) && (data.address == last.address) && (data.value == last.value)) {
          last.count++;
          if (!signal_processed && (last.count == CONFIG_ALARM_THRESHOLD_RF)) {
            signal_processed = alarmProcessIncomingData(last, false);
          };
        } else {
          // Push the previous signal for further processing
          if ((last.source != RTM_NONE) && (last.address > 0) && (last.count > 0) && !signal_processed) {
            alarmProcessIncomingData(last, true);
          };
          // Set new data to last and reset the counter (we don't really need it), instead we will count the number of packets
          last = data;
          last.count = 1;
          signal_processed = false;
        };
      };

      // Waiting for the next signal %CONFIG_ALARM_TIMEOUT_RF% ms
      queueWait = pdMS_TO_TICKS(CONFIG_ALARM_TIMEOUT_RF);
    } else {
      // End of transmission, push the previous signal for further processing
      if ((last.source != RTM_NONE) && (last.address > 0) && (last.count > 0) && !signal_processed) {
        alarmProcessIncomingData(last, true);
      };

      // Clear buffer
      memset(&last, 0, sizeof(reciever_data_t));
      signal_processed = false;

      // Wait for the next signal forever
      queueWait = portMAX_DELAY;
    };
  };
  alarmTaskDelete();
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Task routines ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool alarmTaskCreate(ledQueue_t siren, ledQueue_t flasher, ledQueue_t ledAlarm, ledQueue_t ledRx433, cb_alarm_change_mode_t cb_mode) 
{
  if (!_alarmTask) {
    _siren = siren;
    _flasher = flasher;
    _ledAlarm = ledAlarm;
    _ledRx433 = ledRx433;
    
    alarmZonesInit();
    alarmSensorsInit();
    if (alarmSystemInit(cb_mode)) {
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


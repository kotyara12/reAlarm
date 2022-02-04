#include "reAlarm.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "esp_err.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rLog.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reRx433.h"
#include "reParams.h"
#include "reEvents.h"
#include "reBeep.h"
#include "reMqtt.h"
#include "reStates.h"
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#include "project_config.h"
#include "def_consts.h"
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
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"
#define ERR_GPIO_SET_ISR  "Failed to set GPIO ISR handler"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Modes ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static alarm_mode_t _alarmMode = ASM_DISABLED;
static paramsEntryHandle_t _alarmParamMode = nullptr;
static cb_alarm_change_mode_t _alarmOnChangeMode = nullptr;
static uint32_t _alarmCount = 0;
static time_t _alarmLastEvent = 0;
static time_t _alarmLastAlarm = 0;
static alarmEventData_t _alarmLastEventData = {nullptr, nullptr};
static alarmEventData_t _alarmLastAlarmData = {nullptr, nullptr};

static void alarmSensorsReset();
static void alarmSirenAlarmOff();
static void alarmFlasherAlarmOff();
static void alarmSirenChangeMode();
static void alarmFlasherChangeMode();
static void alarmBuzzerChangeMode();
static void alarmMqttPublishEvent(alarmEventData_t event_data);
static void alarmMqttPublishStatus();

static const char* alarmModeText(alarm_mode_t mode) 
{
  switch (mode) {
    case ASM_ARMED: 
      return CONFIG_ALARM_MODE_ARMED;
    case ASM_PERIMETER:
      return CONFIG_ALARM_MODE_PERIMETER;
    case ASM_OUTBUILDINGS:
      return CONFIG_ALARM_MODE_OUTBUILDINGS;
    default:
      return CONFIG_ALARM_MODE_DISABLED;
  };
}

static const char* alarmSourceText(alarm_control_t source, const char* sensor) 
{
  switch (source) {
    case ACC_STORED: 
      return CONFIG_ALARM_SOURCE_STORED;
    case ACC_BUTTONS:
      return CONFIG_ALARM_SOURCE_BUTTONS;
    case ACC_RCONTROL:
      if (sensor) {
        return sensor;
      } else {
        return CONFIG_ALARM_SOURCE_RCONTROL;
      };
    default:
      return CONFIG_ALARM_SOURCE_MQTT;
  };
}

static void alarmModeChange(alarm_mode_t newMode, alarm_control_t source, const char* sensor, bool forced, bool publish_status)
{
  bool alarmModeChanged = newMode != _alarmMode;
  if (forced || alarmModeChanged) {
    // Store and publish new value
    if (_alarmParamMode) {
      if (alarmModeChanged) {
        _alarmMode = newMode;
        paramsValueStore(_alarmParamMode, false);
      } else {
        paramsMqttPublish(_alarmParamMode, true);
      };
    };

    // Reset counters
    if (newMode != ASM_DISABLED) {
      _alarmCount = 0;
      _alarmLastAlarm = 0;
      _alarmLastAlarmData = {nullptr, nullptr};
      alarmSensorsReset();
    };

    // Disable siren if ASM_DISABLED mode is set
    if (newMode == ASM_DISABLED) {
      alarmSirenAlarmOff();
      alarmFlasherAlarmOff();
    };

    // One-time siren signal when switching the arming mode
    alarmFlasherChangeMode();
    if ((source == ACC_BUTTONS) || (source == ACC_RCONTROL)) {
      alarmSirenChangeMode();
      alarmBuzzerChangeMode();
    };
    
    // Publish current mode and status on MQTT broker
    if (publish_status) {
      alarmMqttPublishStatus();
    };

    // Notifications
    switch (_alarmMode) {
      // Security mode is on
      case ASM_ARMED:
        rlog_w(logTAG, "Full security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_ARMED, &source, sizeof(alarm_control_t), portMAX_DELAY);
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_ARMED, alarmSourceText(source, sensor));
        #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
        break;

      // Perimeter security mode
      case ASM_PERIMETER:
        rlog_w(logTAG, "Perimeter security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_PERIMETER, &source, sizeof(alarm_control_t), portMAX_DELAY);
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_PERIMETER, alarmSourceText(source, sensor));
        #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
        break;

      // Outbuilding security regime
      case ASM_OUTBUILDINGS:
        rlog_w(logTAG, "Outbuildings security mode activated");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_OUTBUILDINGS, &source, sizeof(alarm_control_t), portMAX_DELAY);
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_OUTBUILDINGS, alarmSourceText(source, sensor));
        #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
        break;

      // Security mode disabled
      default:
        rlog_w(logTAG, "Security mode disabled");
        eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_MODE_DISABLED, &source, sizeof(alarm_control_t), portMAX_DELAY);
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_DISABLED, alarmSourceText(source, sensor));
        #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
        break;
    };
    
    // Callback
    if (_alarmOnChangeMode) {
      _alarmOnChangeMode(_alarmMode, source);
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Buzzer ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static bool _alarmBuzzerEnabled = true;

static void alarmBuzzerChangeMode()
{
  if (_alarmBuzzerEnabled) {
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
  };
}

static void alarmBuzzerAlarmOn()
{
  if (_alarmBuzzerEnabled) {
    beepTaskSend(CONFIG_ALARM_BUZZER_ALARM_FREQUENCY, 
      CONFIG_ALARM_BUZZER_ALARM_DURATION, 
      CONFIG_ALARM_BUZZER_ALARM_QUANTITY);
  };
}

static void alarmBuzzerAlarmOff()
{
  if (_alarmBuzzerEnabled) {
    beepTaskSend(CONFIG_ALARM_BUZZER_ALARM_CLEAR_FREQUENCY, 
      CONFIG_ALARM_BUZZER_ALARM_CLEAR_DURATION, 
      CONFIG_ALARM_BUZZER_ALARM_CLEAR_QUANTITY);
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
  alarmMqttPublishStatus();
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
  if (_flasherTimer && (_flasherDuration > 0)) {
    if (!esp_timer_is_active(_flasherTimer)) {
      ERR_CHECK(esp_timer_start_once(_flasherTimer, _flasherDuration * 1000000), "Failed to start flasher timer");
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

static void alarmFlasherBlinkOn(uint16_t blink_quantity, uint16_t blink_duration, uint16_t blink_interval)
{
  if (blink_quantity > 0) {
    if (_flasher) {
      ledTaskSend(_flasher, lmBlinkOn, blink_quantity, blink_duration, blink_interval);
    };
    if (_ledAlarm) {
      ledTaskSend(_ledAlarm, lmBlinkOn, blink_quantity, blink_duration, blink_interval);
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

static void alarmFlasherFlashOn(uint16_t flash_quantity, uint16_t flash_duration, uint16_t flash_interval)
{
  if (_flasher && (flash_quantity > 0)) {
    ledTaskSend(_flasher, lmFlash, flash_quantity, flash_duration, flash_interval);
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
      vTaskDelay(10);
      if (_alarmCount > 0) {
        alarmFlasherFlashOn(CONFIG_ALARM_SIREN_DISABLED_WARNING_QUANTITY, CONFIG_ALARM_SIREN_DISABLED_WARNING_DURATION, CONFIG_ALARM_SIREN_DISABLED_WARNING_INTERVAL);
      } else {
        alarmFlasherFlashOn(CONFIG_ALARM_SIREN_DISABLED_NORMAL_QUANTITY, CONFIG_ALARM_SIREN_DISABLED_NORMAL_DURATION, CONFIG_ALARM_SIREN_DISABLED_NORMAL_INTERVAL);
      };
    } else if (_alarmMode == ASM_ARMED) {
      rlog_d(logTAG, "Flasher set fully armed");
      // Security is active
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_BLINK, nullptr, 0, portMAX_DELAY);
      if (_alarmCount == 0) {
        // All is calm, all is well
        alarmFlasherBlinkOn(CONFIG_ALARM_ARMED_QUANTITY, CONFIG_ALARM_ARMED_DURATION, CONFIG_ALARM_ARMED_INTERVAL);
        vTaskDelay(10);
        alarmFlasherFlashOn(CONFIG_ALARM_SIREN_ARMED_QUANTITY, CONFIG_ALARM_SIREN_ARMED_DURATION, CONFIG_ALARM_SIREN_ARMED_INTERVAL);
      } else {
        // Something happened since the last reset
        alarmFlasherBlinkOn(CONFIG_ALARM_WARNING_QUANTITY, CONFIG_ALARM_WARNING_DURATION, CONFIG_ALARM_WARNING_INTERVAL);
        vTaskDelay(10);
        alarmFlasherFlashOn(CONFIG_ALARM_SIREN_ARMED_QUANTITY, CONFIG_ALARM_SIREN_ARMED_DURATION, CONFIG_ALARM_SIREN_ARMED_INTERVAL);
      };
    } else {
      rlog_d(logTAG, "Flasher set partially armed");
      // Security partially enabled (perimeter only)
      eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_FLASHER_BLINK, nullptr, 0, portMAX_DELAY);
      alarmFlasherBlinkOn(CONFIG_ALARM_PARTIAL_QUANTITY, CONFIG_ALARM_PARTIAL_DURATION, CONFIG_ALARM_PARTIAL_INTERVAL);
      vTaskDelay(10);
      alarmFlasherFlashOn(CONFIG_ALARM_SIREN_PARTIAL_QUANTITY, CONFIG_ALARM_SIREN_PARTIAL_DURATION, CONFIG_ALARM_SIREN_PARTIAL_INTERVAL);
    };
  };
}

static void alarmFlasherAlarmOn() 
{
  if (!_flasherActive && alarmFlasherTimerStart()) {
    _flasherActive = true;
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
// ------------------------------------------------------ Siren ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

ledQueue_t _siren = nullptr;
static uint32_t _sirenDuration = CONFIG_ALARM_DURATION_SIREN;
static esp_timer_handle_t _sirenTimer = nullptr;
static bool _sirenActive = false;

static void alarmSirenTimerEnd(void* arg)
{
  alarmSirenAlarmOff();
  alarmMqttPublishStatus();
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
  if (_sirenTimer && (_sirenDuration > 0)) {
    if (!esp_timer_is_active(_sirenTimer)) {
      ERR_CHECK(esp_timer_start_once(_sirenTimer, _sirenDuration * 1000000), "Failed to start siren timer");
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
  if (!_sirenActive && alarmSirenTimerStart()) {
    _sirenActive = true;
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
// --------------------------------------------------- Initialization ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void alarmMqttEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  rlog_v(logTAG, "Restore security mode...");
  alarmModeChange(_alarmMode, ACC_STORED, nullptr, true, true);
  eventHandlerUnregister(RE_MQTT_EVENTS, RE_MQTT_CONNECTED, alarmMqttEventHandler);
}

static void alarmParamsEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if (*(uint32_t*)event_data == (uint32_t)&_alarmMode) {
    rlog_v(logTAG, "Security mode changed via MQTT, event_id=%d", event_id);
    if (event_id == RE_PARAMS_CHANGED)  {
      alarmModeChange(_alarmMode, ACC_MQTT, nullptr, true, true);
    };
  };
}

static void alarmOtaEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if ((event_id == RE_SYS_OTA) && (event_data)) {
    re_system_event_data_t* data = (re_system_event_data_t*)event_data;
    if (data->type == RE_SYS_SET) {
      alarmTaskSuspend();
    } else {
      alarmTaskResume();
    };
  };
}

static bool alarmParamsRegister()
{
  paramsGroupHandle_t pgSecurity = paramsRegisterGroup(nullptr, 
    CONFIG_ALARM_PARAMS_ROOT_KEY, CONFIG_ALARM_PARAMS_ROOT_TOPIC, CONFIG_ALARM_PARAMS_ROOT_FRIENDLY);
  RE_MEM_CHECK(logTAG, pgSecurity, return false);
  
  #if CONFIG_ALARM_MQTT_DEVICE_MODE
    _alarmParamMode = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_MODE_KEY, CONFIG_ALARM_PARAMS_MODE_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_alarmMode);
  #else
    _alarmParamMode = paramsRegisterValue(OPT_KIND_PARAMETER_LOCATION, OPT_TYPE_U8, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_MODE_KEY, CONFIG_ALARM_PARAMS_MODE_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_alarmMode);
  #endif // CONFIG_ALARM_MQTT_DEVICE_MODE
  RE_MEM_CHECK(logTAG, _alarmParamMode, return false);
  _alarmParamMode->notify = false;
  paramsSetLimitsU8(_alarmParamMode, (uint8_t)ASM_DISABLED, (uint8_t)ASM_MAX-1);
  eventHandlerRegister(RE_MQTT_EVENTS, RE_MQTT_CONNECTED, alarmMqttEventHandler, nullptr);

  paramsSetLimitsU32(
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_SIREN_DUR_KEY, CONFIG_ALARM_PARAMS_SIREN_DUR_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_sirenDuration),
    CONFIG_ALARM_PARAMS_MIN_DURATION, CONFIG_ALARM_PARAMS_MAX_DURATION);
  paramsSetLimitsU32(
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgSecurity, 
      CONFIG_ALARM_PARAMS_FLASHER_DUR_KEY, CONFIG_ALARM_PARAMS_FLASHER_DUR_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_flasherDuration),
    CONFIG_ALARM_PARAMS_MIN_DURATION, CONFIG_ALARM_PARAMS_MAX_DURATION);
  paramsSetLimitsU8(
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_I8, nullptr, pgSecurity, 
        CONFIG_ALARM_PARAMS_BUZZER_KEY, CONFIG_ALARM_PARAMS_BUZZER_FRIENDLY, CONFIG_ALARM_PARAMS_QOS, &_alarmBuzzerEnabled),
    0, 1);

  return eventHandlerRegister(RE_PARAMS_EVENTS, ESP_EVENT_ANY_ID, &alarmParamsEventHandler, nullptr);
}

extern bool alarmSystemInit(cb_alarm_change_mode_t cb_mode)
{
  _alarmOnChangeMode = cb_mode;

  gpio_install_isr_service(0);

  return alarmSirenTimerCreate() 
      && alarmFlasherTimerCreate() 
      && alarmParamsRegister()
      && eventHandlerRegister(RE_SYSTEM_EVENTS, RE_SYS_OTA, &alarmOtaEventHandler, nullptr);
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Zones --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

STAILQ_HEAD(alarmZoneHead_t, alarmZone_t);
typedef struct alarmZoneHead_t *alarmZoneHeadHandle_t;

static alarmZoneHeadHandle_t alarmZones = nullptr;

bool alarmZonesInit()
{
  if (!alarmZones) {
    alarmZones = (alarmZoneHeadHandle_t)esp_calloc(1, sizeof(alarmZoneHead_t));
    RE_MEM_CHECK(logTAG, alarmZones, return false);
    STAILQ_INIT(alarmZones);
  };
  return true;
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
    alarmZones = nullptr;
  };
}

alarmZoneHandle_t alarmZoneAdd(const char* name, const char* topic, cb_relay_control_t cb_relay_ctrl)
{
  if (!alarmZones) {
    alarmZonesInit();
  };
  if (alarmZones) {
    alarmZoneHandle_t item = (alarmZoneHandle_t)esp_calloc(1, sizeof(alarmZone_t));
    RE_MEM_CHECK(logTAG, item, return nullptr);
    item->name = name;
    item->topic = topic;
    item->status = 0;
    item->last_set = 0;
    item->last_clr = 0;
    item->relay_ctrl = cb_relay_ctrl;
    item->relay_state = false;
    for (size_t i = 0; i < ASM_MAX; i++) {
      item->resp_set[i] = ASRS_NONE;
      item->resp_clr[i] = ASRS_NONE;
    };
    STAILQ_INSERT_TAIL(alarmZones, item, next);
    return item;
  };
  return nullptr;
}

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

static alarm_control_t alarmResponsesSource(alarmEventData_t event_data)
{
  if (event_data.sensor->type == AST_WIRED) {
    return ACC_BUTTONS;
  } else if (event_data.sensor->type == AST_MQTT) {
    return ACC_MQTT;
  } else {
    return ACC_RCONTROL;
  };
}

static void alarmResponsesProcess(bool state, alarmEventData_t event_data);
static void alarmResponsesClrTimerEnd(void* arg)
{
  alarmEventData_t* event_data = (alarmEventData_t*)arg;
  if (event_data) {
    alarmResponsesProcess(false, *event_data);
    free(event_data);
  };
}

static bool alarmResponsesClrTimerCreate(alarmEventData_t event_data)
{
  esp_err_t err = ESP_OK;
  if (event_data.event->timer_clr) {
    if (esp_timer_is_active(event_data.event->timer_clr)) {
      err = esp_timer_stop(event_data.event->timer_clr);
      if (err != ESP_OK) {
        rlog_e(logTAG, "Failed to stop event timer!");
        return false;
      };
    };
  } else {
    void* timer_data = (alarmEventData_t*)esp_malloc(sizeof(alarmEventData_t));
    RE_MEM_CHECK(logTAG, timer_data, return false);
    memcpy(timer_data, &event_data, sizeof(alarmEventData_t));

    esp_timer_create_args_t timer_args;
    memset(&timer_args, 0, sizeof(esp_timer_create_args_t));
    timer_args.callback = &alarmResponsesClrTimerEnd;
    timer_args.name = "timer_event";
    timer_args.arg = timer_data;

    err = esp_timer_create(&timer_args, &event_data.event->timer_clr);
    if (err != ESP_OK) {
      rlog_e(logTAG, "Failed to create event timer!");
      return false;
    };
  };
  
  if (event_data.event->timer_clr) {
    err = esp_timer_start_once(event_data.event->timer_clr, 1000 * event_data.event->timeout_clr);
    if (err != ESP_OK) {
      rlog_e(logTAG, "Failed to start event timer");
      return false;
    };
    return true;
  };
  return false;
}

static void alarmResponsesProcess(bool state, alarmEventData_t event_data)
{
  uint16_t responses = 0;
  if (state) {
    rlog_d(logTAG, "Alarm signal for sensor: [ %s ], zone: [ %s ], type: [ %d ]", 
      event_data.sensor->name, event_data.event->zone->name, event_data.event->type);
 
    // Fix event status
    responses = event_data.event->zone->resp_set[_alarmMode];
    event_data.event->event_last = time(nullptr);
    event_data.event->events_count++;
    event_data.event->state = true;

    // Fix zone status
    event_data.event->zone->last_set = event_data.event->event_last;
    if (event_data.event->zone->status < UINT16_MAX) {
      event_data.event->zone->status++;
    };

    // Fix total status
    _alarmLastEvent = event_data.event->event_last;
    _alarmLastEventData = event_data;
    if (responses & ASR_ALARM_INC) {
      if (_alarmCount < UINT32_MAX) {
        _alarmCount++;
      };
      _alarmLastAlarm = event_data.event->event_last;
      _alarmLastAlarmData = event_data;
    };
    if ((responses & ASR_ALARM_DEC) && (_alarmCount > 0)) {
      _alarmCount--;
    };

    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIGNAL_SET, &event_data, sizeof(alarmEventData_t), portMAX_DELAY);

    if (event_data.event->timeout_clr > 0) {
      alarmResponsesClrTimerCreate(event_data);
    };
  } else {
    rlog_d(logTAG, "Clear signal for sensor: [ %s ], zone: [ %s ], type: [ %d ]", 
      event_data.sensor->name, event_data.event->zone->name, event_data.event->type);

    // Fix event status
    responses = event_data.event->zone->resp_clr[_alarmMode];
    event_data.event->state = false;

    // Fix zone status
    if (event_data.event->zone->status > 0) {
      event_data.event->zone->status--;
    };
    if (event_data.event->zone->status == 0) {
      event_data.event->zone->last_clr = time(nullptr);
    };

    // Fix total status
    if (responses & ASR_ALARM_INC) {
      if (_alarmCount < UINT32_MAX) {
        _alarmCount++;
      };
      _alarmLastAlarm = event_data.event->event_last;
      _alarmLastAlarmData = event_data;
    };
    if ((responses & ASR_ALARM_DEC) && (_alarmCount > 0)) {
      _alarmCount--;
    };

    eventLoopPost(RE_ALARM_EVENTS, RE_ALARM_SIGNAL_CLEAR, &event_data, sizeof(alarmEventData_t), portMAX_DELAY);

    if (event_data.event->timer_clr) {
      esp_timer_delete(event_data.event->timer_clr);
      event_data.event->timer_clr = nullptr;
    };
  };

  // Handling arming switch events
  if (state) {
    if (event_data.event->type == ASE_CTRL_OFF) {
      // If the alarm is currently on, then first we just reset the alarm
      if (_sirenActive || _flasherActive) {
        alarmSirenAlarmOff();
        alarmFlasherAlarmOff();
        alarmBuzzerAlarmOff();
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_NOTIFY_TELEGRAM_ALARM_CANCELED, 
            alarmSourceText(alarmResponsesSource(event_data), event_data.sensor->name));
        #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE
      } else {
        alarmModeChange(ASM_DISABLED, alarmResponsesSource(event_data), event_data.sensor->name, false, false);
      };
    } else if (event_data.event->type == ASE_CTRL_ON) {
      alarmModeChange(ASM_ARMED, alarmResponsesSource(event_data), event_data.sensor->name, false, false);
    } else if (event_data.event->type == ASE_CTRL_PERIMETER) {
      alarmModeChange(ASM_PERIMETER, alarmResponsesSource(event_data), event_data.sensor->name, false, false);
    } else if (event_data.event->type == ASE_CTRL_OUTBUILDINGS) {
      alarmModeChange(ASM_OUTBUILDINGS, alarmResponsesSource(event_data), event_data.sensor->name, false, false);
    };
  };

  // Posting event on MQTT
  if (responses & ASR_MQTT_EVENT) {
    alarmMqttPublishEvent(event_data);
  };
  
  // Sound and visual notification
  if (state) {
    if (responses & ASR_BUZZER) {
      alarmBuzzerAlarmOn();
    };
    if (responses & ASR_SIREN) {
      alarmSirenAlarmOn();
    };
    if (responses & ASR_FLASHER) {
      alarmFlasherAlarmOn();
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
  if (responses & ASR_TELEGRAM) {
    #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_ALARM_ALARM
      const char* msg_header = state ? event_data.event->msg_set : event_data.event->msg_clr;
      if (msg_header) {
        char* msg_ts = malloc_timestr_empty(CONFIG_FORMAT_DTS, event_data.event->event_last);
        if (msg_ts) {
          tgSend(TG_SECURITY, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_ALARM, CONFIG_TELEGRAM_DEVICE,
            CONFIG_NOTIFY_TELEGRAM_ALARM_TEMPLATE, 
              msg_header, 
              event_data.sensor->name, event_data.event->zone->name,
              alarmModeText(_alarmMode), 
              _sirenActive ? CONFIG_ALARM_SIREN_ENABLED : CONFIG_ALARM_SIREN_DISABLED,
              msg_ts, event_data.event->events_count);
          free(msg_ts);
        };
      };
    #endif // CONFIG_NOTIFY_TELEGRAM_ALARM_ALARM
  };

  // Publish status on MQTT broker
  alarmMqttPublishStatus();
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Sensors ----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

STAILQ_HEAD(alarmSensorHead_t, alarmSensor_t);
typedef struct alarmSensorHead_t *alarmSensorHeadHandle_t;

static alarmSensorHeadHandle_t alarmSensors = nullptr;

bool alarmSensorsInit()
{
  if (!alarmSensors) {
    alarmSensors = (alarmSensorHeadHandle_t)esp_calloc(1, sizeof(alarmSensorHead_t));
    RE_MEM_CHECK(logTAG, alarmSensors, return false);
    STAILQ_INIT(alarmSensors);
  };
  return true;
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

alarmSensorHandle_t alarmSensorAdd(alarm_sensor_type_t type, const char* name, const char* topic, uint32_t address)
{
  if (!alarmSensors) {
    alarmSensorsInit();
  };
  if (alarmSensors) {
    alarmSensorHandle_t item = (alarmSensorHandle_t)esp_calloc(1, sizeof(alarmSensor_t));
    RE_MEM_CHECK(logTAG, item, return nullptr);
    item->name = name;
    item->topic = topic;
    item->type = type;
    item->address = address;
    for (uint8_t i = 0; i < CONFIG_ALARM_MAX_EVENTS; i++) {
      item->events[i].zone = nullptr;
      item->events[i].type = ASE_EMPTY;
      item->events[i].value_set = 0;
      item->events[i].msg_set = nullptr;
      item->events[i].value_clr = 0;
      item->events[i].msg_clr = nullptr;
      item->events[i].threshold = 0;
      item->events[i].timeout_clr = 0;
      item->events[i].events_count = 0;
      item->events[i].event_last = 0;
    };
    STAILQ_INSERT_TAIL(alarmSensors, item, next);
    return item;
  };
  return nullptr;
}

static void alarmSensorsReset()
{
  if (alarmSensors) {
    alarmSensorHandle_t itemS;
    STAILQ_FOREACH(itemS, alarmSensors, next) {
      for (uint8_t i = 0; i < CONFIG_ALARM_MAX_EVENTS; i++) {
        if (itemS->events[i].type == ASE_ALARM) {
          itemS->events[i].events_count = 0;
        };
      };
    };
  };
}

static void IRAM_ATTR alarmSensorsIsrHandler(void* arg)
{
  alarmSensorHandle_t sensor = (alarmSensorHandle_t)arg;
  gpio_num_t gpio = (gpio_num_t)sensor->address;

  reciever_data_t data;
  data.source = RTM_WIRED;
  data.address = sensor->address;
  // data.value = gpio_get_level(gpio); 
  data.value = 0;
  data.count = 0;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(_alarmQueue, &data, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  };
}

bool alarmSensorsWiredInit(alarmSensorHandle_t sensor, gpio_pull_mode_t pull)
{
  if (sensor && sensor->type == AST_WIRED) {
    gpio_num_t gpio = (gpio_num_t)sensor->address;
    gpio_pad_select_gpio(gpio);
    ERR_CHECK(gpio_set_direction(gpio, GPIO_MODE_INPUT), ERR_GPIO_SET_MODE);
    ERR_CHECK(gpio_set_pull_mode(gpio, pull), ERR_GPIO_SET_MODE);
    ERR_CHECK(gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE), ERR_GPIO_SET_ISR);
    ERR_CHECK(gpio_isr_handler_add(gpio, alarmSensorsIsrHandler, sensor), ERR_GPIO_SET_ISR);
    return true;
  };
  return false;
}

bool alarmSensorsWiredStart(alarmSensorHandle_t sensor)
{
  if (sensor && _alarmQueue && sensor->type == AST_WIRED) {
    gpio_num_t gpio = (gpio_num_t)sensor->address;
    esp_err_t err = gpio_intr_enable(gpio);
    if (err == ESP_OK) {
      rlog_i(logTAG, "Interrupt handler for GPIO %d started", gpio);
      return true;
    } else {
      rlog_e(logTAG, "Failed to start interrupt handler for GPIO %d", gpio);
    };
  };
  return false;
}

bool alarmSensorsWiredStop(alarmSensorHandle_t sensor)
{
  if (sensor && sensor->type == AST_WIRED) {
    gpio_num_t gpio = (gpio_num_t)sensor->address;
    esp_err_t err = gpio_intr_disable(gpio);
    if (err == ESP_OK) {
      rlog_i(logTAG, "Interrupt handler for GPIO %d stopped", gpio);
      return true;
    } else {
      rlog_e(logTAG, "Failed to stop interrupt handler for GPIO %d", gpio);
    };
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Sensor events -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void alarmEventSet(alarmSensorHandle_t sensor, alarmZoneHandle_t zone, uint8_t index, alarm_event_t type,  
  uint32_t value_set, const char* message_set, uint32_t value_clear, const char* message_clr, 
  uint16_t threshold, uint32_t timeout_clr)
{
  if ((sensor) && (zone) && (index<CONFIG_ALARM_MAX_EVENTS)) {
    sensor->events[index].zone = zone;
    sensor->events[index].type = type;
    sensor->events[index].state = false;
    sensor->events[index].value_set = value_set;
    sensor->events[index].msg_set = message_set;
    sensor->events[index].value_clr = value_clear;
    sensor->events[index].msg_clr = message_clr;
    sensor->events[index].threshold = threshold;
    sensor->events[index].timeout_clr = timeout_clr;
    sensor->events[index].events_count = 0;
    sensor->events[index].event_last = 0;
    sensor->events[index].timer_clr = nullptr;
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

static bool alarmEventCheckValueSet(const reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
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

static bool alarmEventCheckValueClr(const reciever_data_t data, alarm_sensor_type_t type, alarmEventHandle_t event)
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

static bool alarmProcessIncomingData(const reciever_data_t data, bool end_of_packet)
{
  // Log
  rlog_w(logTAG, "Incoming message:: end of packet: %d, source: %d, gpio: %d, count: %d, value: 0x%.8X / %d, address: %.8X, command: %02X", 
    end_of_packet, data.source, data.address, data.count, data.value, data.value, data.value >> 4, data.value & 0x0f);

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
              // Discard short noise on wire lines if the logic level has not changed
              if (!sensor->events[i].state || (data.source != RTM_WIRED)) {
                alarmEventData_t event_data = {sensor, &sensor->events[i]};
                alarmResponsesProcess(true, event_data);
              };
              return true;
            } else {
              return false;
            };
          } else if (alarmEventCheckValueClr(data, sensor->type, &sensor->events[i])) {
            if (data.count >= sensor->events[i].threshold) {
              // Discard short noise on wire lines if the logic level has not changed
              if (sensor->events[i].state || (data.source != RTM_WIRED)) {
                alarmEventData_t event_data = {sensor, &sensor->events[i]};
                alarmResponsesProcess(false, event_data);
              };
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
        tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_COMMAND_UNDEFINED, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED_TEMPLATE, sensor->name, data.value, data.value >> 4, data.value & 0x0f);
      #endif // CONFIG_TELEGRAM_ENABLE
    } else {
      // Sensor not found
      rlog_w(logTAG, "Failed to identify event [%d : 0x%.8X]!", data.source, data.value);
      #if CONFIG_TELEGRAM_ENABLE && defined(CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED) && CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED
        tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_SENSOR_UNDEFINED, CONFIG_TELEGRAM_DEVICE, 
          CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED_TEMPLATE, data.address, data.value, data.value >> 4, data.value & 0x0f);
      #endif // CONFIG_TELEGRAM_ENABLE
    };
  };

  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ MQTT -----------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static const char* alarmMqttEventTopic(alarm_event_t type)
{
  switch (type) {
    case ASE_TAMPER:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_TAMPER;
    case ASE_POWER:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_POWER;
    case ASE_LOW_BATTERY:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_BATTERY;
    case ASE_CTRL_OFF:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_CONTROL_OFF;
    case ASE_CTRL_ON:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_CONTROL_ON;
    case ASE_CTRL_PERIMETER:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_CONTROL_PERIMETER;
    case ASE_CTRL_OUTBUILDINGS:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_CONTROL_OUTBUILDINGS;
    default:
      return CONFIG_ALARM_MQTT_EVENTS_ASE_ALARM;
  };
}

static char _alarmTimestampL[CONFIG_ALARM_TIMESTAMP_LONG_BUF_SIZE];
static char _alarmTimestampS[CONFIG_ALARM_TIMESTAMP_SHORT_BUF_SIZE];

static void alarmFormatTimestamps(time_t value)
{
  static struct tm timeinfo;

  memset(&_alarmTimestampL, 0, sizeof(_alarmTimestampL));
  memset(&_alarmTimestampS, 0, sizeof(_alarmTimestampS));
  if (value > 0) {
    localtime_r(&value, &timeinfo);
    strftime(_alarmTimestampL, sizeof(_alarmTimestampL), CONFIG_ALARM_TIMESTAMP_LONG, &timeinfo);
    strftime(_alarmTimestampS, sizeof(_alarmTimestampS), CONFIG_ALARM_TIMESTAMP_SHORT, &timeinfo);
  } else {
    strcpy(_alarmTimestampL, CONFIG_FORMAT_EMPTY_DATETIME);
    strcpy(_alarmTimestampS, CONFIG_FORMAT_EMPTY_DATETIME);
  };
}

static void alarmMqttPublishEvent(alarmEventData_t event_data)
{
  if (event_data.event->zone->topic && event_data.sensor->topic && statesMqttIsEnabled()) {
    char* topicSensor = nullptr;
    #if CONFIG_ALARM_MQTT_DEVICE_EVENTS
      topicSensor = mqttGetTopicDevice5(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_EVENTS_LOCAL,
        CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_EVENTS_TOPIC, event_data.event->zone->topic, event_data.sensor->topic, 
        alarmMqttEventTopic(event_data.event->type)); 
    #else
      topicSensor = mqttGetTopicSpecial4(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_EVENTS_LOCAL,
        CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_EVENTS_TOPIC, event_data.event->zone->topic, event_data.sensor->topic, 
        alarmMqttEventTopic(event_data.event->type)); 
    #endif // CONFIG_ALARM_MQTT_DEVICE_EVENTS

    if (topicSensor) {
      mqttPublish(mqttGetSubTopic(topicSensor, CONFIG_ALARM_MQTT_EVENTS_STATUS), 
        malloc_stringf("%d", event_data.event->state), 
        CONFIG_ALARM_MQTT_EVENTS_QOS, CONFIG_ALARM_MQTT_EVENTS_RETAINED, true, true, true);

      alarmFormatTimestamps(event_data.event->event_last);
      mqttPublish(mqttGetSubTopic(topicSensor, CONFIG_ALARM_MQTT_EVENTS_JSON), 
        malloc_stringf(CONFIG_ALARM_MQTT_EVENTS_JSON_TEMPLATE, 
          event_data.event->state, _alarmTimestampL, _alarmTimestampS, event_data.event->event_last, event_data.event->events_count), 
        CONFIG_ALARM_MQTT_EVENTS_QOS, CONFIG_ALARM_MQTT_EVENTS_RETAINED, false, true, true);
      
      free(topicSensor);
    } else {
      rlog_e(logTAG, "Failed to generate a topic for publishing an event \"%s\"", event_data.event->msg_set);
    }
  };
}

static char* alarmMqttJsonZone(alarmZoneHandle_t zone)
{
  char* ret = nullptr;
  char* lstSet = nullptr;
  char* lstClr = nullptr;
  lstSet = malloc_timestr_empty(CONFIG_FORMAT_DTS, zone->last_set);
  RE_MEM_CHECK(logTAG, lstSet, goto exit);
  lstClr = malloc_timestr_empty(CONFIG_FORMAT_DTS, zone->last_clr);
  RE_MEM_CHECK(logTAG, lstClr, goto exit);
  ret = malloc_stringf("\"%s\":{\"name\":\"%s\",\"status\":%d,\"last_alarm\":\"%s\",\"last_clear\":\"%s\",\"relay\":%d}",
    zone->topic, zone->name, zone->status, lstSet, lstClr, zone->relay_state);
exit:
  if (lstSet) free(lstSet);
  if (lstClr) free(lstClr);
  return ret;
}

static void alarmMqttPublishStatus()
{
  if (statesMqttIsEnabled()) {
    char * topicStatus = nullptr;

    #if CONFIG_ALARM_MQTT_DEVICE_STATUS
      #ifdef CONFIG_ALARM_MQTT_DEVICE_TOPIC
        topicStatus = mqttGetTopicSpecial2(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_STATUS_LOCAL,
          CONFIG_ALARM_MQTT_DEVICE_TOPIC, CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_STATUS_TOPIC); 
      #else 
        topicStatus = mqttGetTopicSpecial1(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_STATUS_LOCAL,
          CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_STATUS_TOPIC); 
      #endif // CONFIG_ALARM_MQTT_DEVICE
    #else
      #ifdef CONFIG_ALARM_MQTT_DEVICE_TOPIC
        topicStatus = mqttGetTopicSpecial2(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_STATUS_LOCAL,
          CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_STATUS_TOPIC, CONFIG_ALARM_MQTT_DEVICE_TOPIC); 
      #else 
        topicStatus = mqttGetTopicSpecial1(statesMqttIsPrimary(), CONFIG_ALARM_MQTT_STATUS_LOCAL,
          CONFIG_ALARM_MQTT_SECURITY_TOPIC, CONFIG_ALARM_MQTT_STATUS_TOPIC); 
      #endif // CONFIG_ALARM_MQTT_DEVICE_TOPIC
    #endif // CONFIG_ALARM_MQTT_DEVICE_STATUS
    RE_MEM_CHECK(logTAG, topicStatus, return);

    char * jsonStatus = nullptr;
    char * jsonZones = nullptr;
    char * jsonZone = nullptr;
    char * jsonTemp = nullptr;
    char * statusSummary = nullptr;
    char * statusAnnunciator = nullptr;
    char * jsonLastAlarm = nullptr;
    char * jsonLastEvent = nullptr;

    // Getting names of sensors
    const char* sensorLastAlarm = nullptr;
    const char* sensorLastEvent = nullptr;
    if (_alarmLastAlarmData.sensor) {
      sensorLastAlarm = _alarmLastAlarmData.sensor->name;
    } else {
      sensorLastAlarm = CONFIG_ALARM_MQTT_STATUS_DEVICE_EMPTY;
    };
    if (_alarmLastEventData.sensor) {
      sensorLastEvent = _alarmLastEventData.sensor->name;
    } else {
      sensorLastEvent = CONFIG_ALARM_MQTT_STATUS_DEVICE_EMPTY;
    };

    // Forming an array with zones
    alarmZoneHandle_t zone;
    STAILQ_FOREACH(zone, alarmZones, next) {
      jsonZone = alarmMqttJsonZone(zone);
      if (jsonZone) {
        if (jsonZones) {
          jsonTemp = jsonZones;
          jsonZones = malloc_stringf("%s,%s", jsonTemp, jsonZone);
          free(jsonTemp);
        } else {
          jsonZones = malloc_string(jsonZone);
        };
        free(jsonZone);
      };
    };

    // Select mode labels
    const char* sMode = CONFIG_ALARM_MODE_CHAR_DISABLED;
    if (_alarmMode == ASM_ARMED) {
      sMode = CONFIG_ALARM_MODE_CHAR_ARMED;        
    } else if (_alarmMode == ASM_PERIMETER) {
      sMode = CONFIG_ALARM_MODE_CHAR_PERIMETER;    
    } else if (_alarmMode == ASM_OUTBUILDINGS) {
      sMode = CONFIG_ALARM_MODE_CHAR_OUTBUILDINGS;
    };

    // Select annunciator labels
    const char* sAnnunciator = CONFIG_ALARM_ANNUNCIATOR_OFF;
    if (_sirenActive) {
      if (_flasherActive) {
        sAnnunciator = CONFIG_ALARM_ANNUNCIATOR_TOTAL;
      } else {
        sAnnunciator = CONFIG_ALARM_ANNUNCIATOR_SIREN;
      };
    } else {
      if (_flasherActive) {
        sAnnunciator = CONFIG_ALARM_ANNUNCIATOR_FLASHER;
      };
    };

    // Generate status line
    statusSummary = malloc_stringf(CONFIG_ALARM_MQTT_STATUS_SUMMARY, sMode, _alarmCount, sAnnunciator);
    RE_MEM_CHECK(logTAG, statusSummary, goto free_strings_error);

    // Generate annunciator status
    statusAnnunciator = malloc_stringf(CONFIG_ALARM_MQTT_STATUS_JSON_ANNUNCIATOR, _sirenActive, _flasherActive, _sirenActive << 1 | _flasherActive);
    RE_MEM_CHECK(logTAG, statusAnnunciator, goto free_strings_error);

    // Generate last event data
    alarmFormatTimestamps(_alarmLastEvent);
    jsonLastEvent = malloc_stringf(CONFIG_ALARM_MQTT_STATUS_JSON_ALARM, sensorLastEvent, _alarmTimestampL, _alarmTimestampS, _alarmLastEvent);
    RE_MEM_CHECK(logTAG, jsonLastEvent, goto free_strings_error);

    // Generate last alarm data
    alarmFormatTimestamps(_alarmLastAlarm);
    jsonLastAlarm = malloc_stringf(CONFIG_ALARM_MQTT_STATUS_JSON_ALARM, sensorLastAlarm, _alarmTimestampL, _alarmTimestampS, _alarmLastAlarm);
    RE_MEM_CHECK(logTAG, jsonLastAlarm, goto free_strings_error);

    // Generate full JSON string
    #if CONFIG_ALARM_MQTT_STATUS_DISPLAY
      if (jsonZones) {
        jsonStatus = malloc_stringf("{\"mode\":%d,\"alarms\":%d,\"status\":\"%s\",\"annunciator\":%s,\"alarm\":%s,\"event\":%s,\"display\":\"%s\n%s\n%s\",\"zones\":{%s}}", 
          _alarmMode, _alarmCount, 
          statusSummary, statusAnnunciator, 
          jsonLastAlarm, jsonLastEvent, 
          statusSummary, sensorLastAlarm, _alarmTimestampS,          
          jsonZones);
      } else {
        jsonStatus = malloc_stringf("{\"mode\":%d,\"alarms\":%d,\"status\":\"%s\",\"annunciator\":%s,\"alarm\":%s,\"event\":%s,\"display\":\"%s\n%s\n%s\",\"zones\":{}}", 
          _alarmMode, _alarmCount, 
          statusSummary, statusAnnunciator, 
          jsonLastAlarm, jsonLastEvent, 
          statusSummary, sensorLastAlarm, _alarmTimestampS);
      };
    #else
      if (jsonZones) {
        jsonStatus = malloc_stringf("{\"mode\":%d,\"alarms\":%d,\"status\":\"%s\",\"annunciator\":%s,\"alarm\":%s,\"event\":%s,\"zones\":{%s}}", 
          _alarmMode, _alarmCount, 
          statusSummary, statusAnnunciator, 
          jsonLastAlarm, jsonLastEvent, 
          jsonZones);
      } else {
        jsonStatus = malloc_stringf("{\"mode\":%d,\"alarms\":%d,\"status\":\"%s\",\"annunciator\":%s,\"alarm\":%s,\"event\":%s,\"zones\":{}}", 
          _alarmMode, _alarmCount, 
          statusSummary, statusAnnunciator, 
          jsonLastAlarm, jsonLastEvent);
      };
    #endif // CONFIG_ALARM_MQTT_STATUS_DISPLAY
    RE_MEM_CHECK(logTAG, jsonLastAlarm, goto free_strings_error);
    
    mqttPublish(topicStatus, jsonStatus, 
      CONFIG_ALARM_MQTT_STATUS_QOS, CONFIG_ALARM_MQTT_STATUS_RETAINED, false, true, true);
    goto free_strings_ok;

    // Free resources
    free_strings_error:
      if (jsonStatus) free(jsonStatus);
      goto free_strings_ok;

    free_strings_ok:
      if (jsonZones) free(jsonZones);
      if (statusSummary) free(statusSummary);
      if (statusAnnunciator) free(statusAnnunciator);
      if (jsonLastAlarm) free(jsonLastAlarm);
      if (jsonLastEvent) free(jsonLastEvent);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Task function ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void alarmTaskExec(void *pvParameters)
{
  static reciever_data_t data, last;
  static bool signal_processed = false;
  static TickType_t queueWait = portMAX_DELAY;

  memset(&last, 0, sizeof(reciever_data_t));
  while (1) {
    if (xQueueReceive(_alarmQueue, &data, queueWait) == pdPASS) {
      // Send signal to LED
      if (_ledRx433) {
        ledTaskSend(_ledRx433, lmFlash, CONFIG_ALARM_INCOMING_QUANTITY, CONFIG_ALARM_INCOMING_DURATION, CONFIG_ALARM_INCOMING_INTERVAL);
      };

      // Handling signals from ISR
      if (data.source == RTM_WIRED) {
        // Due to contact bounce, hundreds of switches can come in, you must definitely wait for the package to complete
        if ((data.source == last.source) && (data.address == last.address)) {
          last.count++;
        } else {
          // Push the previous signal for further processing
          if ((last.source > RTM_NONE) && (last.address > 0) && (last.count > 0) && !signal_processed) {
            rlog_d(logTAG, "Process signal (last not processed): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
            alarmProcessIncomingData(last, true);
          };
          // Set new data to last and reset the counter (we don't really need it), instead we will count the number of packets
          memcpy(&last, &data, sizeof(reciever_data_t));
          last.count = 1;
          signal_processed = false;
          rlog_d(logTAG, "Init last signal: source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
        };

        // Waiting for the next signal
        queueWait = pdMS_TO_TICKS(CONFIG_ALARM_TIMEOUT_ISR);
      }
      
      // Handling packets from RX433
      else if (data.source == RTM_RX433) {
        if ((data.source == last.source) && (data.address == last.address) && (data.value == last.value)) {
          // One more signal from the packet
          last.count++;
          // If the number of signals has exceeded the threshold, process the signal
          if (!signal_processed && (last.count == CONFIG_ALARM_THRESHOLD_RF)) {
            rlog_d(logTAG, "Process signal (exceeded threshold): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
            signal_processed = alarmProcessIncomingData(last, false);
          };
        } else {
          // Push the previous signal for further processing
          if ((last.source > RTM_NONE) && (last.address > 0) && (last.count > 0) && !signal_processed) {
            rlog_d(logTAG, "Process signal (last not processed): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
            alarmProcessIncomingData(last, true);
          };
          // Set new data to last and reset the counter (we don't really need it), instead we will count the number of packets
          memcpy(&last, &data, sizeof(reciever_data_t));
          last.count = 1;
          signal_processed = false;
          rlog_d(logTAG, "Init last signal: source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
          // If the threshold is not set, process the signal immediately
          if (!signal_processed && (last.count == CONFIG_ALARM_THRESHOLD_RF)) {
            rlog_d(logTAG, "Process signal (no threshold): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
            signal_processed = alarmProcessIncomingData(last, false);
          };
        };
        
        // Waiting for the next signal
        queueWait = pdMS_TO_TICKS(CONFIG_ALARM_TIMEOUT_RF);

      // Handling non-repeating signals
      } else if (data.source > RTM_NONE) {
        memcpy(&last, &data, sizeof(reciever_data_t));
        last.count = 1;
        rlog_d(logTAG, "Process signal (non-repeating): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
        alarmProcessIncomingData(last, true);
        
        // Waiting for the next signal forever
        signal_processed = false;
        memset(&last, 0, sizeof(reciever_data_t));
        queueWait = portMAX_DELAY;

      // What was it?
      } else {
        rlog_e(logTAG, "Signal received from RTM_NONE!");
        signal_processed = false;
        memset(&last, 0, sizeof(reciever_data_t));
        queueWait = portMAX_DELAY;
      };
    } else {
      // End of transmission, push the previous signal for further processing
      if ((last.source > RTM_NONE) && (last.address > 0) && (last.count > 0) && !signal_processed) {
        // For GPIO, you need to update the current value, since the data received from the ISR is very unreliable
        if (last.source == RTM_WIRED) {
          last.value = gpio_get_level((gpio_num_t)last.address);
        };
        rlog_d(logTAG, "Process signal (end of transmission): source=%d, address=%d, value=%d, count=%d", last.source, last.address, last.value, last.count);
        alarmProcessIncomingData(last, true);
      };

      // Waiting for the next signal forever
      signal_processed = false;
      memset(&last, 0, sizeof(reciever_data_t));
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
    vTaskResume(_alarmTask);
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


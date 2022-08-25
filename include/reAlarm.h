/* 
   EN: Security and fire alarm module controlled via MQTT and Telegram
   RU: Модуль охранно-пожарной сигнализации с управлением через MQTT и Telegram
   --------------------------
   (с) 2021-2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/consts/reAlarm
*/

#ifndef __RE_ALARM_H__
#define __RE_ALARM_H__

#pragma GCC diagnostic ignored "-Wunused-variable"

#include <stdbool.h>
#include "reLed.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/queue.h"
#include "rTypes.h"
#include "def_alarm.h"

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Типы данных --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/**
 * РЕАКЦИЯ НА СОБЫТИЯ
 * 
 * Реакция на события задается битовыми флагами для каждого режима работы
 * */
static const uint16_t ASR_ALARM_INC    = BIT0;   // Увеличить счетчик тревог
static const uint16_t ASR_ALARM_DEC    = BIT1;   // Уменьшить счетчик тревог
static const uint16_t ASR_MQTT_EVENT   = BIT2;   // Публикация события на MQTT
static const uint16_t ASR_MQTT_STATUS  = BIT3;   // Публикация состояния охраны на MQTT
static const uint16_t ASR_TELEGRAM     = BIT4;   // Уведомление в Telegram
static const uint16_t ASR_SIREN        = BIT5;   // Включить сирену
static const uint16_t ASR_FLASHER      = BIT6;   // Включить маячок
static const uint16_t ASR_BUZZER       = BIT7;   // Звуковой сигнал на пульте
static const uint16_t ASR_RELAY_ON     = BIT8;   // Включить реле (нагрузку)
static const uint16_t ASR_RELAY_OFF    = BIT9;   // Выключить реле (нагрузку)
static const uint16_t ASR_RELAY_SWITCH = BIT10;  // Переключить реле (нагрузку)

// "Стандартные" наборы реакций
static const uint16_t ASRS_NONE         = 0x0000; // Никакой реакции (по умолчанию)
static const uint16_t ASRS_CONTROL      = ASR_MQTT_EVENT | ASR_MQTT_STATUS;
static const uint16_t ASRS_REGISTER     = ASR_MQTT_EVENT | ASR_MQTT_STATUS;
static const uint16_t ASRS_ONLY_NOTIFY  = ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM;
static const uint16_t ASRS_FLASH_NOTIFY = ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_FLASHER;
static const uint16_t ASRS_ALARM_NOTIFY = ASR_ALARM_INC | ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_BUZZER;
static const uint16_t ASRS_ALARM_SILENT = ASR_ALARM_INC | ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_BUZZER | ASR_FLASHER;
static const uint16_t ASRS_ALARM_SIREN  = ASR_ALARM_INC | ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_BUZZER | ASR_SIREN | ASR_FLASHER;
static const uint16_t ASRS_POWER_ON     = ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_FLASHER;
static const uint16_t ASRS_POWER_OFF    = ASR_ALARM_INC | ASR_MQTT_EVENT | ASR_MQTT_STATUS | ASR_TELEGRAM | ASR_BUZZER | ASR_FLASHER;

/**
 * ТИП ДАТЧИКА
 * 
 * Тип датчика используется для установки прерываний или подписки на MQTT топики
 * */
typedef enum {
  AST_WIRED = 0,          // Проводная зона
  AST_RX433_GENERIC,      // Беспроводной сенсор, без выделения команд
  AST_RX433_20A4C,        // Беспроводной сенсор, общая длина кода 24 бит: 20 бит - адрес, последние 4 бита - команда
  AST_MQTT                // Виртуальный сенсор, получение данных с других устройств через локальный MQTT брокер
} alarm_sensor_type_t;

/**
 * ИСТОЧНИК СИГНАЛА УПРАВЛЕНИЯ
 * 
 * Через какой из каналов управления поступил сигнал на переключение режима работы
 * */
typedef enum {
  ACC_STORED = 0,         // Режим считан из памяти
  ACC_BUTTONS,            // Переключение с помощью кнопок на устройстве
  ACC_RCONTROL,           // Переключение с помощью дистанционного пульта управления
  ACC_MQTT,               // Переключение через MQTT
  ACC_COMMANDS            // Переключение через команды
} alarm_control_t;  

/**
 * РЕЖИМ РАБОТЫ
 * 
 * Режим работы определяет реакцию на события в зависимости от типа зоны
 * */
typedef enum {
  ASM_DISABLED = 0,      // Security mode disabled
  ASM_ARMED,             // Security mode is on
  ASM_PERIMETER,         // Perimeter security mode
  ASM_OUTBUILDINGS,      // Outbuilding security regime
  ASM_MAX                // Not used, it's just a counter
} alarm_mode_t;

typedef void (*cb_alarm_change_mode_t) (alarm_mode_t mode, alarm_control_t source);

/**
 * ТИП СОБЫТИЯ
 * 
 * Тип события определяет его обработку
 * */
typedef enum {
  ASE_EMPTY = 0,          // Не обрабатывается
  ASE_ALARM,              // Сигнал тревоги
  ASE_TAMPER,             // Попытка вскрытия датчика
  ASE_POWER,              // Состояние электропитания
  ASE_BATTERY_LOW,        // Низкий уровень заряда батареи
  ASE_CTRL_OFF,           // Пульт: режим охраны отключен
  ASE_CTRL_ON,            // Пульт: режим охраны включен
  ASE_CTRL_PERIMETER,     // Пульт: режим охраны периметра
  ASE_CTRL_OUTBUILDINGS   // Пульт: режим охраны внешних помещений
} alarm_event_t;

/**
 * СИСТЕМНЫЕ СОБЫТИЯ 
 * 
 * Отправка уведомлений в системный цикл событий
 * */
static const char* RE_ALARM_EVENTS = "REVT_ALARM";

typedef enum {
  RE_ALARM_MODE_DISABLED = 0,
  RE_ALARM_MODE_ARMED,
  RE_ALARM_MODE_PERIMETER,
  RE_ALARM_MODE_OUTBUILDINGS,
  RE_ALARM_SIGNAL_SET,
  RE_ALARM_SIGNAL_CLEAR,
  RE_ALARM_SIREN_ON,
  RE_ALARM_SIREN_OFF,
  RE_ALARM_FLASHER_ON,
  RE_ALARM_FLASHER_OFF,
  RE_ALARM_FLASHER_BLINK,
  RE_ALARM_RELAY_ON,
  RE_ALARM_RELAY_OFF,
  RE_ALARM_RELAY_TOGGLE
} re_alarm_event_id_t;

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Структуры ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Параметры зоны
typedef struct alarmZone_t {
  const char* name;
  const char* topic;
  cb_relay_control_t relay_ctrl = nullptr;
  uint16_t status;
  time_t   last_set;
  time_t   last_clr;
  bool relay_state = false;
  uint16_t resp_set[ASM_MAX];
  uint16_t resp_clr[ASM_MAX];
  STAILQ_ENTRY(alarmZone_t) next;
} alarmZone_t;
// Ссылка-указатель на параметры зоны
typedef alarmZone_t *alarmZoneHandle_t;

static const uint32_t ALARM_VALUE_NONE = 0xFFFFFFFF;

// Параметры события (сигнала с датчика)
typedef struct alarmEvent_t {
  alarmZoneHandle_t zone;
  alarm_event_t type;
  bool state;
  bool confirm;
  uint32_t value_set;
  const char* msg_set;
  uint32_t value_clr;
  const char* msg_clr;
  uint16_t threshold;
  uint32_t timeout_clr;
  uint32_t events_count;
  time_t   event_last;
  uint16_t mqtt_interval;
  time_t   mqtt_next;
  esp_timer_handle_t timer_clr = nullptr;
} alarmEvent_t;
// Ссылка-указатель на параметры события
typedef alarmEvent_t *alarmEventHandle_t;

// Параметры датчика
typedef struct alarmSensor_t {
  alarm_sensor_type_t type;
  const char* name;
  const char* topic;
  uint32_t address;
  alarmEvent_t events[CONFIG_ALARM_MAX_EVENTS];
  STAILQ_ENTRY(alarmSensor_t) next;
} alarmSensor_t;
// Ссылка-указатель на параметры датчика
typedef alarmSensor_t *alarmSensorHandle_t;

// Данные для обаботки события
typedef struct {
  alarmSensorHandle_t sensor;
  alarmEventHandle_t event;
} alarmEventData_t;

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Управление задачей --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/**
 * Инициализация сигнализации 
 * @brief Запуск таймеров сирены, флешера, регистрация параметров
 * */
bool alarmSystemInit(cb_alarm_change_mode_t cb_mode);

/**
 * Создать задачу
 * @brief Создать и запустить задачу ОПС
 * @param siren Ссылка-указатель на виртуальный "светодиод", отвечающий за включение сирены
 * @param flasher Ссылка-указатель на виртуальный "светодиод", отвечающий за включение светового маяка
 * @param buzzer Ссылка-указатель на виртуальный "светодиод", отвечающий за зуммер
 * @param ledAlarm Ссылка-указатель на светодиод, индицирующий режим работы
 * @param ledRx433 Ссылка-указатель на светодиод, индицирующий получение события с приемника RX433. Можно использовать тот же, что и ledAlarm
 * @param cb_mode Функция обратного вызова, вызываемая при изменении режима охраны
 * @return Успех или неуспех
 * */
bool alarmTaskCreate(ledQueue_t siren, ledQueue_t flasher, ledQueue_t buzzer, ledQueue_t ledAlarm, ledQueue_t ledRx433, cb_alarm_change_mode_t cb_mode);

/**
 * Приостановать задачу
 * @brief Приостановать задачу ОПС
 * */
bool alarmTaskSuspend();

/**
 * Восстановить задачу
 * @brief Восстановить задачу ОПС
 * */
bool alarmTaskResume();

/**
 * Удалить задачу
 * @brief Удалить задачу ОПС и освободить ресурсы
 * */
void alarmTaskDelete();

/**
 * Указатель на очередь сообщений
 * @brief Получить указатель на очередь сообщений задачи ОПС
 * @return Ссылка-указатель на очередь сообщений задачи ОПС
 * */
QueueHandle_t alarmTaskQueue();

/**
 * Добавить зону
 * @brief Добавить зону в список зон ОПС
 * @param name Понятное наименование зоны
 * @param topic Субтопик для публикации данных с сенсоров зоны
 * @param cb_relay_ctrl Функция обратного вызова для реакции на события ASR_RELAY_xxx
 * @return Ссылка-указатель на созданную зону
 * */
alarmZoneHandle_t alarmZoneAdd(const char* name, const char* topic, cb_relay_control_t cb_relay_ctrl);

/**
 * Добавить реакции на события
 * @brief Добавить реакции на события (битовые флаги) для выбранной зоны и режима. 
 *        Необходимо повторить это опрделение реакций для всех режимов.
 * @param zone Ссылка-указатель на зону
 * @param mode Выбранный режим
 * @param resp_set Битовая маска, указывающая как регировать на активацию события в данной зоне для данного режима
 * @param resp_clr Битовая маска, указывающая как регировать на сброс события в данной зоне для данного режима
 * */
void alarmResponsesSet(alarmZoneHandle_t zone, alarm_mode_t mode, uint16_t resp_set, uint16_t resp_clr);

/**
 * Добавить датчик
 * @brief Добавить датчик в список ОПС. Датчик не привязан к зоне, к зонам привязаны события датчика
 * @param type Тип датчика
 * @param name Понятное наименование датчика
 * @param topic Субтопик для публикации данных с датчика
 * @param address Адрес датчика для беспроводных датчиков или номер вывода GPIO для проводных зон
 * @return Ссылка-указатель на созданную зону
 * */
alarmSensorHandle_t alarmSensorAdd(alarm_sensor_type_t type, const char* name, const char* topic, uint32_t address);

/**
 * Добавить событие датчика
 * @brief Установить команду датчика в заданную зону
 * @param sensor Ссылка-указатель на датчик
 * @param zone Ссылка-указатель на зону
 * @param index Порядковый индекс команды от 0 до CONFIG_ALARM_MAX_EVENTS-1 в порядке приоритета
 * @param type Тип события
 * @param value_set Значение для установки статуса тревоги. Это может быть команда для беспроводного датчика или логический уровень на входе GPIO. Если 0xFFFFFFFF, то не используется.
 * @param message_set Сообщение для события установки статуса тревоги
 * @param value_clear Значение для сброса статуса тревоги. Это может быть команда для беспроводного датчика или логический уровень на входе GPIO. Если 0xFFFFFFFF, то не используется.
 * @param message_clr Сообщение для события сброса статуса тревоги
 * @param threshold Пороговое значение. Должно придти не менее заданного значения команд подряд в течение timeout. Имеет смысл для беспроводных датчиков, чтобы исключить ложные срабатывания
 * @param timeout_clr Таймаут в миллисекундах. Используется для сброса статуса тревоги после получения последней команды value_set
 * @param mqtt_interval Интервал публикации на MQTT брокере в секундах
 * @param alarm_confirm Тревога будет вызвана, только если есть подтверждение с этого же или другого датчика в течение заданного времени
 * */
void alarmEventSet(alarmSensorHandle_t sensor, alarmZoneHandle_t zone, uint8_t index, alarm_event_t type,  
  uint32_t value_set, const char* message_set, uint32_t value_clear, const char* message_clr, 
  uint16_t threshold, uint32_t timeout_clr, uint16_t mqtt_interval, bool alarm_confirm);

/**
 * Отправить внешнее событие в очередь обработки
 * @brief Отправить внешнее событие в очередь обработки ОПС
 * @param id Идентификатор события (должен соответствовать адресу виртуального сенсора)
 * @param value Логический уровень: 1 - тревога, 0 - сброс тревоги
 * @return true в случае успеха, false в случае отказа
 * */
bool alarmPostQueueExtId(source_type_t source, uint32_t id, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // __RE_ALARM_H__
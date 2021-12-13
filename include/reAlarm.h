/* 
   Модуль охранно-пожарной сигнализации с управлением через MQTT и Telegram
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/consts/reAlarm
*/

#ifndef __RE_ALARM_H__
#define __RE_ALARM_H__

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
static const uint16_t ASR_MQTT_LOCAL   = BIT0;  // Публикация события в локальном топике MQTT
static const uint16_t ASR_MQTT_PUBLIC  = BIT1;  // Публикация события в публичном топике MQTT
static const uint16_t ASR_EMAIL        = BIT2;  // Уведомление на электронную почту
static const uint16_t ASR_TELEGRAM     = BIT3;  // Уведомление в Telegram
static const uint16_t ASR_SIREN        = BIT4;  // Включить сирену
static const uint16_t ASR_FLASHER      = BIT5;  // Включить маячок
static const uint16_t ASR_RELAY_ON     = BIT6;  // Включить реле (нагрузку)
static const uint16_t ASR_RELAY_OFF    = BIT7;  // Выключить реле (нагрузку)
static const uint16_t ASR_RELAY_SWITCH = BIT8;  // Переключить реле (нагрузку)
static const uint16_t ASR_RELAY_TIMER  = BIT9;  // Включить реле (нагрузку) на заданное время

// "Стандартные" наборы реакций
static const uint16_t ASRS_NONE        = 0x0000; // Никаой реакции (по умолчанию)
static const uint16_t ASRS_REGISTER    = ASR_MQTT_LOCAL | ASR_MQTT_PUBLIC;
static const uint16_t ASRS_NOTIFY      = ASR_MQTT_LOCAL | ASR_MQTT_PUBLIC | ASR_EMAIL | ASR_TELEGRAM;
static const uint16_t ASRS_SILENT      = ASR_MQTT_LOCAL | ASR_MQTT_PUBLIC | ASR_EMAIL | ASR_TELEGRAM | ASR_FLASHER;
static const uint16_t ASRS_ALARM       = ASR_MQTT_LOCAL | ASR_MQTT_PUBLIC | ASR_EMAIL | ASR_TELEGRAM | ASR_SIREN | ASR_FLASHER;

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
 * ТИП СОБЫТИЯ
 * 
 * Тип события определяет текст уведомления, которое будет отправлено пользователю
 * */
typedef enum {
  ASE_EMPTY = 0,          // Не обрабатывается
  ASE_TAMPER,             // Попытка взлома
  ASE_POWER_OFF,          // Основное питание отсутствует
  ASE_POWER_ON,           // Основное питание восстановлено
  ASE_LOW_BATTERY,        // Низкий уровень заряда батареи
  ASE_DOOR,               // Дверь открыта
  ASE_WINDOW,             // Окно открыто
  ASE_MOTION,             // Обнаружено движение
  ASE_SMOKE,              // Обнаружено задымление
  ASE_FIRE,               // Обнаружено пламя
  ASE_WATER_LEAK,         // Протечка воды
  ASE_GAS_LEAK,           // Утечка газа
  ASE_MONOXIDE,           // Угарный газ
  ASE_SHOCK,              // Удар
  ASE_BUTTON,             // Нажата кнопка
  ASE_RCONTROL_OFF,       // Пульт: режим охраны отключен
  ASE_RCONTROL_ON,        // Пульт: режим охраны включен
  ASE_RCONTROL_INHOME,    // Пульт: режим охраны периметра
  ASE_RCONTROL_ALARM      // Пульт: тревога с пульта
} alarm_event_t;

/**
 * ТИП ЗОНЫ
 * 
 * Тип зоны определяет реакцию на событие в текущем режиме работы
 * Примечание: тип зоны "перекликаеся" с событиями, но это не одно и то же
 * */
typedef enum {
  ASZ_IGNORE = 0,         // Не обрабатывается
  ASZ_PERIMETER,          // Периметр (внешние окна и двери)
  ASZ_INDOOR,             // Внутри, в режиме охраны периметра игнорируются
  ASZ_OUTDOOR,            // Снаружи, в любом режиме только уведомления (без сирены)
  ASZ_OUTBUILDINGS,       // Хозяйственные постройки
  ASZ_FIRE,               // Пожарные датчики
  ASZ_WATER,              // Датчики протечки воды
  ASZ_GAS,                // Датчики утечки газа
  ASZ_CRASH,              // Сбои и поломки оборудования
  ASZ_ALARM,              // Тревога с пульта
  ASZ_EMERGENCY,          // Кнопки вызова экстренной помощи
  ASZ_DOORBELL,           // Дверной звонок
  ASZ_AUTOMATIC,          // Сенсор использутся для автоматики или управления освещением, но его данные регистрируются
  ASZ_POWER,              // Контроль напряжения питания
  ASZ_MAX                 // Не используется, это просто счетчик
} alarm_zone_t;

/**
 * РЕЖИМ РАБОТЫ
 * 
 * Режим работы определяет реакцию на события в зависимости от типа зоны
 * */
typedef enum {
  ASM_ALARM_OFF = 0,      // Режим охраны отключен
  ASM_ALARM_FULL,         // Режим охраны включен
  ASM_ALARM_PERIMETER,    // Режим охраны периметра
  ASM_ALARM_OUTBUILDINGS, // Режим охраны хозпостроек
  ASM_ALARM_MAX           // Не используется, это просто счетчик
} alarm_mode_t;

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Структуры ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Параметры зоны
typedef struct alarmZone_t {
  alarm_zone_t type;
  const char* name;
  uint16_t responses[ASM_ALARM_MAX];
  STAILQ_ENTRY(alarmZone_t) next;
} alarmZone_t;
// Ссылка-указатель на параметры зоны
typedef alarmZone_t *alarmZoneHandle_t;

static const uint32_t ALARM_VALUE_NONE = 0xFFFFFFFF;

// Параметры события (сигнала с датчика)
typedef struct alarmEvent_t {
  alarm_event_t type;
  alarmZoneHandle_t* zone;
  uint32_t value_set;
  uint32_t value_clr;
  uint16_t threshold;
  uint32_t timeout;
  uint16_t value_set_count;
  uint16_t value_clr_count;
  uint32_t events_count;
  time_t   event_last;
} alarmEvent_t;
// Ссылка-указатель на параметры события
typedef alarmEvent_t *alarmEventHandle_t;

// Параметры датчика
typedef struct alarmSensor_t {
  alarm_sensor_type_t type;
  const char* name;
  uint32_t address;
  alarmEvent_t events[CONFIG_ALARM_MAX_EVENTS];
  STAILQ_ENTRY(alarmSensor_t) next;
} alarmSensor_t;
// Ссылка-указатель на параметры датчика
typedef alarmSensor_t *alarmSensorHandle_t;

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Управление задачей --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/**
 * Создать задачу
 * @brief Создать и запустить задачу ОПС
 * @param siren Ссылка-указатель на виртуальный "светодиод", отвечающий за включение сирены
 * @param flasher Ссылка-указатель на виртуальный "светодиод", отвечающий за включение светового маяка
 * @param ledAlarm Ссылка-указатель на светодиод, индицирующий режим работы
 * @param ledRx433 Ссылка-указатель на светодиод, индицирующий получение события с приемника RX433. Можно использовать тот же, что и ledAlarm
 * @return Успех или неуспех
 * */
bool alarmTaskCreate(ledQueue_t siren, ledQueue_t flasher, ledQueue_t ledAlarm, ledQueue_t ledRx433);

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
 * @param type Тип зоны
 * @param name Понятное наименование зоны
 * @return Ссылка-указатель на созданную зону
 * */
alarmZoneHandle_t alarmZoneAdd(alarm_zone_t type, const char* name);

/**
 * Добавить реакции на события
 * @brief Добавить реакции на события (битовые флаги) для выбранной зоны и режима. 
 *        Необходимо повторить это опрделение реакций для всех режимов.
 * @param zone Ссылка-указатель на зону
 * @param mode Выбранный режим
 * @param resonses Битовая маска, указывающая как регировать на событие в данной зоне для данного режима
 * */
void alarmResponsesSet(alarmZoneHandle_t zone, alarm_mode_t mode, uint16_t responses);

/**
 * Добавить датчик
 * @brief Добавить датчик в список ОПС. Датчик не привязан к зоне, к зонам привязаны события датчика
 * @param type Тип датчика
 * @param name Понятное наименование датчика
 * @param address Адрес датчика для беспроводных датчиков или номер вывода GPIO для проводных зон
 * @return Ссылка-указатель на созданную зону
 * */
alarmSensorHandle_t alarmSensorAdd(alarm_sensor_type_t type, const char* name, uint32_t address);

/**
 * Добавить событие датчика
 * @brief Установить команду датчика в заданную зону
 * @param sensor Ссылка-указатель на датчик
 * @param zone Ссылка-указатель на зону
 * @param index Порядковый индекс команды от 0 до CONFIG_ALARM_MAX_EVENTS-1 в порядке приоритета
 * @param type Тип события
 * @param value_set Значение для установки статуса тревоги. Это может быть команда для беспроводного датчика или логический уровень на входе GPIO. Если 0xFFFFFFFF, то не используется.
 * @param value_clear Значение для сброса статуса тревоги. Это может быть команда для беспроводного датчика или логический уровень на входе GPIO. Если 0xFFFFFFFF, то не используется.
 * @param threshold Пороговое значение. Должно придти не менее заданного значения команд подряд в течение timeout. Имеет смысл для беспроводных датчиков, чтобы исключить ложные срабатывания
 * @param timeout Таймаут в миллисекундах. Используется для сброса статуса тревоги после получения последней команды value_set и при threshold больше 1
 * */
void alarmEventSet(alarmSensorHandle_t sensor, alarmZoneHandle_t zone, uint8_t index, alarm_event_t type, 
  uint32_t value_set, uint32_t value_clear, uint16_t threshold, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif // __RE_ALARM_H__
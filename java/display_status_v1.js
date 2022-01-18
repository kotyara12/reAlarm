var sJSON = event.payload;
if (sJSON != '') {
  event.data = JSON.parse(sJSON);
};

======

function twoDigits(value) {
  if(value < 10) {
    return '0' + value;
  };
  return value;
}

var sMode = 'Offline ⁉️';
var sAlarms = '-';
var evtSensor = '-';
var evtTimeS = '---';

if (event.data) {
  // Конвертируем режим работы в условный символ
  var iMode = event.data['mode'];
  if (iMode == 0) { sMode = '🔓'; } 
  else if (iMode == 1) { sMode = '🔒'; }
  else if (iMode == 2) { sMode = '🔳'; }
  else if (iMode == 3) { sMode = '🏘️'; };

  // Извлекаем количество тревог
  sAlarms = event.data['alarms'];

  // Извлекаем название сенсора
  evtSensor = event.data['event_sensor'];
  if (evtSensor == '') { evtSensor = '---'; };

  // Конвертируем время
  var evtTimeI = event.data['event_unixtime'];
  if (evtTimeI > 0) {
    var evtTimeD = new Date(evtTimeI*1000);
    evtTimeS = twoDigits(evtTimeD.getDate())
       + '.' + twoDigits(evtTimeD.getMonth()+1)
       + '.' + evtTimeD.getFullYear().toString().substring(2, 4)
       + ' ' + twoDigits(evtTimeD.getHours())
       + ':' + twoDigits(evtTimeD.getMinutes());
  };
};

// Собираем сообщение из 3 строк
event.text = sMode + ' ( ' + sAlarms + ' )' + '\n' + evtSensor + '\n' + evtTimeS;
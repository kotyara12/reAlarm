if (event.data) {
  // Формируем текст состояния из трех строк: статус, 
  // последний активный сенсор (или пульт) и время его сработки
  event.text = event.data['status'] + '\n';
  if (event.data['event']) {
    event.text = event.text + event.data['event']['sensor'] + '\n' + event.data['event']['time_short'];
  } else {
    event.text = event.text + 'Нет событий\n---';
  };

  var iMode = event.data['mode'];
  if (iMode == 0) {
    // Охрана отключена
    if (event.data['annunciator']['summary'] > 0) {
      // Тревога по зонам 24 в настоящий момент
      event.textColor = '#FF0000';
      event.blink = true;
    } else {
      // Тревоги нет, всё тихо
      event.textColor = '#9ACD32';
      event.blink = false;
    };
  } else {
    // Мигание, если были зафиксированы тревоги с момента последнего включения
    if ((event.data['alarms'] > 0) || (event.data['annunciator']['summary'] > 0)) {
      event.textColor = '#FF0000';
      event.blink = true;
    } else {
      event.textColor = '#FFFF00';
      event.blink = false;
    };
  };
} else {
  // Нет данных
  event.text = 'Устройство выключено или не доступно';
  event.textColor = '#696969';
  event.blink = false;
}
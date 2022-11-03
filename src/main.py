import atexit
import logging
import traceback
from datetime import datetime
from drone import Drone

import requests
from flask import Flask, request
from apscheduler.schedulers.background import BackgroundScheduler


app = Flask(__name__)

# Создание объекта класса Drone
copter = Drone('drone', 'DEBUG', 1)

# -----------------------------------------------------------------------------------------------------------------------
# Блок функций:

def set_flight_task():
    """
    Устанавливает полётное задание в контроллер.
    :return:
    Возвращает True в случае успешного прохождения операции, иначе возвращает False)
    """
    try:
        return copter.navigate_to_target()
    except:
        logging.error(traceback.format_exc())
        logging.error(f'[set_flight_task]: {datetime.now()} Error setting flight task!')
        return False


# Конец блока функций.
# -----------------------------------------------------------------------------------------------------------------------


# -----------------------------------------------------------------------------------------------------------------------
# Отправка локации с интервалом раз в секунду

def send_location():
    """
    Отправка текущих координат и скорости дрона.
    :return:
    Возвращает id дрона, его координаты и скорость
    """
    # Получение данных от дрона
    position = copter.get_gps_data()

    #id = position['id']  # TODO сделать нормальное получение id дрона
    #lat = position['lat']
    #lon = position['lon']
    #vel = position['vel']
    # TODO тут напиши строчки с получением локации в две переменные
    try:
        r = requests.post('http://192.168.1.184:5000/torpedo/system-json/add-location-history-info', data=position)
        if r.status_code == 200:
            logging.log(f'[send_location][{datetime.now()}] Successfully sent location')
        else:
            logging.error(f'[send_location][{datetime.now()}] Server error {r.status_code} on sending location')
    except:
        logging.error(f'[send_location][{datetime.now()}] Error while sending location')
        logging.error(traceback.format_exc())


scheduler = BackgroundScheduler()
scheduler.add_job(func=send_location, trigger='interval', seconds=1)
scheduler.start()
atexit.register(lambda: scheduler.shutdown())

# Конец блока отправки
# -----------------------------------------------------------------------------------------------------------------------


# -----------------------------------------------------------------------------------------------------------------------
# Начало блока хэндлеров:

@app.route('/post-flight-task')
def post_flight_task():
    """
    Получение полётного задания.
    :return:
    Возвращает id дрона и результат обработки полётного задания.
    В случае успешной обработки также возвращает время начала полёта.
    """
    task = request.get_json()
    target_lat = task['lat']
    target_lon = task['lon']
    copter.get_mission_target(task)
    flight_set = set_flight_task()
    if(flight_set):
        return "Success setting flight task", 200
    else:
        return "Error setting flight data", 500


# Конец блока хендлеров.
# -----------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
#    app.run(host='0.0.0.0', port=5000)
	send_location()

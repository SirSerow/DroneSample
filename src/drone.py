import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import logging
import traceback
import datetime

class Drone:

    def __init__(self, node_name, log_level, drone_id):
        '''
        Инициализация
        '''
        self.node_name = node_name
        self.drone_id = drone_id
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        rospy.init_node(node_name, log_level)

    #def __del__(self):

    def fly_square(self, square_side, alt):
        '''
        Функция с примером полёта
        '''
        # Takeoff and hover 1 m above the ground
        self.navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

        # Wait for 3 seconds
        rospy.sleep(3)

        # Fly forward 2 m
        self.navigate(x=2, y=0, z=0, frame_id='body')

        # Wait for 3 seconds
        rospy.sleep(3)

        # Fly forward 2 m
        self.navigate(x=0, y=2, z=0, frame_id='body')

        # Wait for 3 seconds
        rospy.sleep(3)

        # Fly forward 2 m
        self.navigate(x=-2, y=0, z=0, frame_id='body')

        # Wait for 3 seconds
        rospy.sleep(3)

        # Fly forward 2 m
        self.navigate(x=0, y=-2, z=0, frame_id='body')

        # Wait for 3 seconds
        rospy.sleep(3)

        # Perform landing
        self.land()

    def get_gps_data(self):
        '''
        Функция считывания геолокации и скорости
        :return:
        Возвращает id дрона, его координаты и скорость
        '''
        try:
            self.get_telemetry()
        except:           
            logging.error(traceback.format_exc())
            logging.error(f'[set_flight_task]: {datetime.now()} Flight controller is not available')
        else:
            data = {
                'id': 1,
                'lat': self.get_telemetry().lat,
                'lon': self.get_telemetry().lat,
                'vel': math.sqrt(
                        pow(self.get_telemetry().vx, 2) +
                        pow(self.get_telemetry().vy, 2) + 
                        pow(self.get_telemetry().vz, 2)
                        )
            }
            return data 

    def get_mission_target(self, task):
        '''
        Функция обработки поступающего полётного
        задания
        :return:
        Возвращает id дрона и результат обработки полётного задания.
        В случае успешной обработки также возвращает время начала полёта.
        '''
        if len(task) == 2:
            if 'lat' in task and 'lon' in task:
                self.target = task
                processing_result = 'SUCCESS'
            else:
                logging.error(f'[get_mission_target]: {datetime.now()} Wrong keys')
                processing_result = 'FAILED TO PROCESS TASK'
        else:
            logging.error(f'[get_mission_target]: {datetime.now()} Wrong mission parameters')
            processing_result = 'FAILED TO PROCESS TASK'    
        
        return self.drone_id, processing_result

if __name__ == '__main__':
    test = Drone('drone', 'DEBUG', 1)
    print(test.get_gps_data())




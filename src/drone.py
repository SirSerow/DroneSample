import rospy
from clover import srv
from std_srvs.srv import Trigger
import requests

class Drone:

    def __init__(self, node_name, log_level):
        '''
        Инициализация
        '''
        self.node_name = node_name
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





# Взлет на высоту 1 м
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Ожидание 3 секунды
rospy.sleep(3)

# Пролет вперед 1 метр
navigate(x=1, y=0, z=0, frame_id='body')

# Ожидание 3 секунды
rospy.sleep(3)

# Посадка
land()
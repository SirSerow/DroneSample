import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Takeoff and hover 1 m above the ground
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 3 seconds
rospy.sleep(3)

# Fly forward 1 m
navigate(x=1, y=0, z=0, frame_id='body')

# Wait for 3 seconds
rospy.sleep(3)

# Perform landing
land()
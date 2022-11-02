import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Takeoff and hover 1 m above the ground
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
print(get_telemetry())
# Wait for 3 seconds
rospy.sleep(3)

# Fly forward 2 m
navigate(x=2, y=0, z=0, frame_id='body')
print(get_telemetry())
# Wait for 3 seconds
rospy.sleep(3)

# Fly forward 2 m
navigate(x=0, y=2, z=0, frame_id='body')
print(get_telemetry())
# Wait for 3 seconds
rospy.sleep(3)

# Fly forward 2 m
navigate(x=-2, y=0, z=0, frame_id='body')
print(get_telemetry())
# Wait for 3 seconds
rospy.sleep(3)

# Fly forward 2 m
navigate(x=0, y=-2, z=0, frame_id='body')
print(get_telemetry())
# Wait for 3 seconds
rospy.sleep(3)

# Perform landing
land()
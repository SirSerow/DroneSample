import rospy
from clover import srv

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Print drone's state
print(get_telemetry())
print()
print(get_telemetry().x, get_telemetry().y, get_telemetry().z)
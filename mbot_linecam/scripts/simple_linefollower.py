import rospy

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


node_name = "simple_linefollwer"
rospy.init_node(node_name, anonymous=True)


line_sensor_val = 50
def lineSensorCallback(sensor_data):
    global line_sensor_val
    line_sensor_val = sensor_data.data

rospy.Subscriber("/linecam/line_sensor_reading", Int8, lineSensorCallback)


sonar_sensor_val = 1.0
def sonarSensorCallback(sensor):
    global sonar_sensor_val
    sonar_sensor_val = round(sensor.range,2)
    

rospy.Subscriber("/sonar_scan", Range, sonarSensorCallback)



pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

def drive_robot(v, w):
    global pub_cmd
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    pub_cmd.publish(cmd)
    # rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")

lin_vel = 0.3
ang_vel = 0.6
def simple_line_follower():
    global lin_vel, ang_vel
    if line_sensor_val == 50:
        drive_robot(lin_vel,0)
    elif line_sensor_val < 50:
        drive_robot(lin_vel,ang_vel)
    elif line_sensor_val > 50:
        drive_robot(lin_vel,-ang_vel)



def follow_line():
    if line_sensor_val==100 or line_sensor_val==0:
        drive_robot(0,0)
    
    elif line_sensor_val==50:
        drive_robot(0.3,0)

    elif line_sensor_val==60:
        drive_robot(0.3,-0.2)
    elif line_sensor_val==70:
        drive_robot(0.2,-0.6)
    elif line_sensor_val==80:
        drive_robot(0.2,-1.0)
    elif line_sensor_val==90:
        drive_robot(0.1,-1.4)

    elif line_sensor_val==40:
        drive_robot(0.3,0.2)
    elif line_sensor_val==30:
        drive_robot(0.2,0.6)
    elif line_sensor_val==20:
        drive_robot(0.2,1.0)
    elif line_sensor_val==10:
        drive_robot(0.1,1.4)


def follow_line_with_obstacle_detection():
    global sonar_sensor_val

    if sonar_sensor_val<0.2:
        drive_robot(0,0)
    else:
        follow_line()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            follow_line_with_obstacle_detection()
        except rospy.ROSInterruptException:
            break


rospy.spin() # spin() simply keeps python from exiting until this node is stopped
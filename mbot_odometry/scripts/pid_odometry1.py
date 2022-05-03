import rospy
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from mbot_pymodule.simplePID import PID
# from mbot_message.msg import OdomData

rospy.init_node("pid_odometry1", anonymous=True)


# subscriber to the /new_odom topic
x_pos=0; y_pos=0; theta=0; v_rob=0; w_rob=0
dist_buffer=0; angle_buffer=0

def readNewOdomData(odom_data):
    global x_pos, y_pos, theta, v_rob, w_rob
    global dist_buffer, angle_buffer

    msg = odom_data.data

    x_pos = round(msg[0],4)
    y_pos = round(msg[1],4)
    theta = round(msg[2],4)
    v_rob = round(msg[3],4)
    w_rob = round(msg[4],4)
    dist_buffer = round(msg[5],4)
    angle_buffer = round(msg[6],4)

    # rospy.loginfo(f"({x_pos}, {y_pos}, {theta}, {v_rob}, {w_rob}, {dist_buffer}, {angle_buffer})")
     
rospy.Subscriber("/new_odom", Float32MultiArray, readNewOdomData)




exit_cmd = ''

def updateExitCmd(cmd):
    global exit_cmd
    exit_cmd = cmd.data
     
rospy.Subscriber("/exit_cmd_topic", String, updateExitCmd)




# publisher to the /cmd_vel topic
pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

def drive_robot(v, w):
    global pub_cmd
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    pub_cmd.publish(cmd)
    # rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")




def translateControl(dist):
    global exit_cmd

    setpoint = dist

    dist_init = dist_buffer
    actual = dist_buffer-dist_init
    error = setpoint-actual

    kp = 2; ki = 0.2; kd = 0; output_upper_limit = 0.5; output_lower_limit = -0.5
    translationPIDController = PID()
    translationPIDController.set_parameters(kp, ki, kd, output_upper_limit, output_lower_limit)

    # t0 = rospy.get_time(); t1 = rospy.get_time()
    while True:
        if exit_cmd == 's':
            break
        elif round(error,1) == 0.000:
            break
        # t1 = rospy.get_time()
        # print(t1-t0)
        output = translationPIDController.update_PID(error)
        drive_robot(output,0)
        actual = dist_buffer-dist_init
        error = setpoint-actual

        # t0 = t1
        

    drive_robot(0,0)
    exit_cmd = ''   





def deg2rad(deg):
    return round(deg*0.017460317,4)

def rotateControl(theta):
    global exit_cmd

    setpoint = deg2rad(theta)

    angle_init = angle_buffer
    actual = angle_buffer-angle_init
    error = setpoint-actual

    kp = 2; ki = 0.2; kd = 0; output_upper_limit = 0.8; output_lower_limit = -0.8
    rotationPIDController = PID()
    rotationPIDController.set_parameters(kp, ki, kd, output_upper_limit, output_lower_limit)

    # t0 = rospy.get_time(); t1 = rospy.get_time()
    while True:
        if exit_cmd == 's':
            break
        elif round(error,1) == 0.000:
            break
        # t1 = rospy.get_time()
        # print(t1-t0)
        output = rotationPIDController.update_PID(error)
        drive_robot(0, output)
        actual = angle_buffer-angle_init
        error = setpoint-actual
        # t0 = t1
        

    drive_robot(0,0)
    exit_cmd = ''   



def main(): 
    while rospy.is_shutdown:
        inputcmd = input("enter command: ")
        if inputcmd == 's':
            drive_robot(0,0)
            break
        else:
            motioncmd = inputcmd.split(',')
            if motioncmd[0] == 'r':
                rotateControl(float(motioncmd[1]))
            elif motioncmd[0] == 't':
                translateControl(float(motioncmd[1]))
            else:
                print("invalid motion command")
          


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInitException:
        pass



rospy.spin() # simply keeps python from exiting until this node is stopped
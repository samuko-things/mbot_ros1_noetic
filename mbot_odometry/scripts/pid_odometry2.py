import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist
# from mbot_message.msg import OdomData


rospy.init_node("pid_odometry2", anonymous=True)



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




pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

def drive_robot(v, w):
    global pub_cmd
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    pub_cmd.publish(cmd)
    # rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")




exit_cmd = ''

def updateExitCmd(cmd):
    global exit_cmd
    exit_cmd = cmd.data
     
rospy.Subscriber("/exit_cmd_topic", String, updateExitCmd)





pub_error = rospy.Publisher("/pid_error_data", Float32MultiArray, queue_size=100)

def sendErrorVal(error_data0, error_data1):
    global pub_error

    error=Float32MultiArray()
    error.data = [error_data0, error_data1]

    pub_error.publish(error)
    # rospy.loginfo(f"sent_error_val = {error}")




v_cmd = 0
w_cmd = 0

def readPIDOutput(output):
    global v_cmd, w_cmd
    output_cmd = output.data
    v_cmd = output_cmd[0]
    w_cmd = output_cmd[1]
     
rospy.Subscriber("/pid_output_data", Float32MultiArray, readPIDOutput)




def translateControl(dist):
    global exit_cmd, v_cmd

    setpoint = dist

    dist_init = dist_buffer
    actual = dist_buffer-dist_init
    error = setpoint-actual
    sendErrorVal(error,0)

    while True:
        if exit_cmd == 's':
            break
        elif round(error,2) == 0.000:
            break
        drive_robot(v_cmd,0)
        actual = dist_buffer-dist_init
        error = setpoint-actual
        sendErrorVal(error,0)
        

    drive_robot(0,0)
    exit_cmd = ''   



def deg2rad(deg):
    return round(deg*0.017460317,4)


def rotateControl(theta):
    global exit_cmd, w_cmd

    setpoint = deg2rad(theta)

    angle_init = angle_buffer
    actual = angle_buffer-angle_init
    error = setpoint-actual
    sendErrorVal(0,error)

    while True:
        if exit_cmd == 's':
            break
        elif round(error,2) == 0.000:
            break
        drive_robot(0,w_cmd)
        actual = angle_buffer-angle_init
        error = setpoint-actual
        sendErrorVal(0,error)
        

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
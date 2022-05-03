import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
# from mbot_message.msg import OdomData


# initialize the node
rospy.init_node("simple_odometry", anonymous=True)



default_lin_vel = 0.3
default_ang_vel = 0.6



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



# publisher to the /cmd_vel topic
pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

def drive_robot(v, w):
    global pub_cmd
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    pub_cmd.publish(cmd)
    # rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")




######## odom motions ##################
def translate(dist):
    dist_init = dist_buffer
    if dist<0:
        while (dist_buffer-dist_init) > dist:
            drive_robot(-default_lin_vel, 0) # reverse motion
    else:
        while (dist_buffer-dist_init) < dist:
            drive_robot(default_lin_vel, 0) # forward motion
    drive_robot(0,0) # stop motion




def deg2rad(deg):
    return round(deg*0.017460317,4)


def rotate(theta):
    angle = deg2rad(theta)
    angle_init = angle_buffer
    if angle<0:
        while (angle_buffer-angle_init) > angle:
            drive_robot(0, -default_ang_vel) # right turn
    else:
        while (angle_buffer-angle_init) < angle:
            drive_robot(0, default_ang_vel) # left turn
    drive_robot(0,0) # stop motion






def main():
    while True:
        inputcmd = input("enter command: ")
        if inputcmd == 's':
            drive_robot(0,0)
            break
        else:
            motioncmd = inputcmd.split(',')
            if motioncmd[0] == 'r':
                rotate(float(motioncmd[1]))
            elif motioncmd[0] == 't':
                translate(float(motioncmd[1]))
            else:
                print("invalid motion command")


        


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInitException:
        pass

rospy.spin() # simply keeps python from exiting until this node is stopped
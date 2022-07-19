import rospy
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist




speed_command = 0
move_command = 0
direction_command = 0

def get_motion_cmd(cmd):
    global speed_command, move_command, direction_command

    motion_cmd = cmd.data

    speed_command = motion_cmd[0]
    move_command = motion_cmd[1]
    direction_command= motion_cmd[2]
     
rospy.Subscriber("/pynput_cmd", Int8MultiArray, get_motion_cmd)


def compute_vel_from_cmd(speed_cmd, move_cmd, direction_cmd):
    lin_vel = 0.6
    ang_vel = 1.0

    v = lin_vel*move_cmd
    w = ang_vel*direction_cmd

    return v, w



# pub_cmd = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=100)      # for turtlesim simulation drive
pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

def drive_robot(v, w):
    global pub_cmd

    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    # rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")
    pub_cmd.publish(cmd)





def main():
    global speed_command, move_command, direction_command

    rospy.init_node("keyboard_teleop", anonymous=False) #initialize and startup the ros node
    drive_robot(0,0)
    rospy.loginfo("use direction keys to control the robot")

    while not rospy.is_shutdown():
        try:
                v,w = compute_vel_from_cmd(speed_command, move_command, direction_command)
                drive_robot(v,w)

        except Exception as e:
            print(e)



if __name__=="__main__":
    main()
rospy.spin()

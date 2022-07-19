import rospy
from geometry_msgs.msg import Twist


lin_vel = 0.3
ang_vel = 0.6


pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100) # define the publisher object (topic, message_type, queue_size)
cmd_type = ['s', 'f', 'b', 'l', 'r', 'd', 'set']


def drive_robot(v, w):
    global pub_cmd
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    pub_cmd.publish(cmd)
    rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")


def settings(v,w):
    global lin_vel, ang_vel
    lin_vel = v
    ang_vel = w


def Stop():
    drive_robot(0,0)
    rospy.loginfo(f"stopped")

def forward():
    global lin_vel
    drive_robot(lin_vel, 0)
    rospy.loginfo(f"moving forward ...")

def backward():
    global lin_vel
    drive_robot(-1*lin_vel,0)
    rospy.loginfo(f"moving backward ...")

def left_turn():
    global ang_vel
    drive_robot(0,ang_vel)
    rospy.loginfo(f"turing left ...")

def right_turn():
    global ang_vel
    drive_robot(0,-1*ang_vel)
    rospy.loginfo(f"turing right ...")







def main():
    global cmd_type
    while not rospy.is_shutdown():
        cmd_str = input("enter command: ")
        cmd_list = cmd_str.split(',')

        if cmd_list[0] not in cmd_type:
            rospy.loginfo("invalid command")
            continue
        
        if cmd_list[0] == 'd':
            if len(cmd_list)!=3:
                Stop()
                rospy.loginfo("invalid command")
            else:
                drive_robot(float(cmd_list[1]), float(cmd_list[2]))
                rospy.loginfo(f"driving robot: v = {cmd_list[1]}, w = {cmd_list[2]}")
        
        elif cmd_list[0] == 'set':
            if len(cmd_list)!=3:
                Stop()
                rospy.loginfo("invalid command")
            else:
                settings(float(cmd_list[1]), float(cmd_list[2]))
                rospy.loginfo(f"settings changed: v = {cmd_list[1]}, w = {cmd_list[2]}")

        elif cmd_list[0] == 's':
            Stop()
        elif cmd_list[0] == 'f':
            forward()
        elif cmd_list[0] == 'b':
            backward()
        elif cmd_list[0] == 'l':
            left_turn()
        elif cmd_list[0] == 'r':
            right_turn()
        


if __name__=="__main__":
    rospy.init_node("command_teleop", anonymous=False) #initialize and startup the ros node

    try:
        main()
    except rospy.ROSInitException:
        pass
import rospy
from std_msgs.msg import String



pub_cmd = rospy.Publisher("/exit_cmd_topic", String, queue_size=100)

def main():
    while True:
        exit_cmd=String()
        exit_cmd.data = input("enter exit cmd: ")
        if exit_cmd.data != 's':
            break
        else:
            pub_cmd.publish(exit_cmd)
                
        

if __name__=="__main__":
    rospy.init_node("exit_cmd_node", anonymous=True)

    try:
        main()
    except rospy.ROSInitException:
        pass


# rosservice call /gazebo/reset_simulation
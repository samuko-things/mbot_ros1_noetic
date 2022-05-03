import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
# from mbot_message.msg import OdomData



rospy.init_node("new_odometry_reading", anonymous=True)



x_odom = 0; y_odom = 0; yaw_odom = 0
v_odom = 0; w_odom = 0
dist_traveled = 0; angle_turned = 0
sample_time = 0; startTime = rospy.get_time(); stopTime = rospy.get_time()



pub_msg = rospy.Publisher("/new_odom", Float32MultiArray, queue_size=100)

def publishNewOdom(x_odom, y_odom, yaw_odom, v_odom, w_odom, dist_traveled, angle_turned):
    msg = Float32MultiArray()
    msg_list = []

    msg_list.append(x_odom)
    msg_list.append(y_odom)
    msg_list.append(yaw_odom)

    msg_list.append(v_odom)
    msg_list.append(w_odom)

    msg_list.append(dist_traveled)
    msg_list.append(angle_turned)

    msg.data = msg_list

    pub_msg.publish(msg)



def odomCallback(odom_data):
    global x_odom, y_odom, yaw_odom, v_odom, w_odom, dist_traveled, angle_turned
    global startTime, stopTime, sample_time
    
    x_odom = odom_data.pose.pose.position.x
    y_odom = odom_data.pose.pose.position.y

    orientation_q = odom_data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw_odom) = euler_from_quaternion(orientation_list) 

    v_odom = odom_data.twist.twist.linear.x
    w_odom = odom_data.twist.twist.angular.z
  
    stopTime=rospy.get_time()
    sample_time = stopTime-startTime
    dist_traveled += v_odom*sample_time    
    angle_turned += w_odom*sample_time

    publishNewOdom(x_odom, y_odom, yaw_odom, v_odom, w_odom, dist_traveled, angle_turned)

    # rospy.loginfo(f"pose=({round(x_odom,4)}, {round(y_odom,4)}, {round(yaw_odom,4)})")
    # rospy.loginfo(f"twist=({round(v_odom,4)}, {round(w_odom,4)})")
    # rospy.loginfo(f"dist=({round(dist_traveled,4)}")

    startTime = stopTime

rospy.Subscriber("/odom", Odometry, odomCallback)
    
    


# if __name__ == '__main__':
    

    # try:
    #     subscribing()
    # except rospy.ROSInterruptException:
    #     pass

rospy.spin() # spin() simply keeps python from exiting until this node is stopped
import rospy
import cv2
from std_msgs.msg import Int16MultiArray


node_name = "hsv_trackbar"
rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(20)




def callback(value):
    pass

def setup_trackbars(range_filter):
    cv2.namedWindow("HSV_Trackbar", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "HSV_Trackbar", v, 255, callback)

def get_trackbar_values(range_filter):
    values = []
    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "HSV_Trackbar")
            values.append(v)

    return values








pub_cmd = rospy.Publisher("/hsv_caliberation", Int16MultiArray, queue_size=100)

def pub_hsv_caliberation():
    global pub_cmd
    cmd = Int16MultiArray()
    cmd.data = get_trackbar_values(range_filter)

    pub_cmd.publish(cmd)
    # rospy.loginfo(cmd.data)
    



if __name__ == '__main__':
    range_filter = "HSV"
    print(range_filter)
    setup_trackbars(range_filter)
    
    while not rospy.is_shutdown():
        try:
            pub_hsv_caliberation()
            cv2.waitKey(30)
        except rospy.ROSInterruptException:
            break

    cv2.destroyAllWindows()


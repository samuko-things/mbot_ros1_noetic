import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Int16MultiArray


node_name = "hsv_color_filter"
rospy.init_node(node_name, anonymous=True)

v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = 0,0,0,255,255,255

def get_caliberation_vals(data):
    global v1_min, v2_min, v3_min, v1_max, v2_max, v3_max
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = data.data

rospy.Subscriber("/hsv_caliberation", Int16MultiArray, get_caliberation_vals)



def camCallback(cam_data):
    global display_res, range_filter, v1_min, v2_min, v3_min, v1_max, v2_max, v3_max

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(cam_data, "bgr8")
    cv_hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(cv_hsv_image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
    # kernel = np.ones((5,5),'int')
    # eroded = cv2.erode(mask, kernel, iterations=2)
    # dilated = cv2.dilate(eroded,kernel, iterations=4)
    # res = cv2.bitwise_and(image,image,mask=dilated)
    # ret,thresh = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    # contours,hier = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow("Original", cv_image)
    cv2.imshow("Thresh", mask)

    cv2.waitKey(30)

rospy.Subscriber("/linecam/linecam_raw", Image, camCallback)



if __name__ == '__main__':
    
    
    while not rospy.is_shutdown():
        try:        
            pass
        except rospy.ROSInterruptException:
            break

    cv2.destroyAllWindows()

rospy.spin() # spin() simply keeps python from exiting until this node is stopped

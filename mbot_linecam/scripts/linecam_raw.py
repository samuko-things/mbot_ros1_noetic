import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


node_name = "linecam_raw"
rospy.init_node(node_name, anonymous=True)


def camCallback(cam_data):
    bridge = CvBridge()

    cv_img = bridge.imgmsg_to_cv2(cam_data, "bgr8")

    cv2.imshow("Camera output normal", cv_img)


    # hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # lower = np.array([0,0,0])
    # upper = np.array([15,15,15])
    # masked_img = cv2.inRange(hsv, lower, upper)  

    # cv2.imshow("Camera output normal", masked_img)

    cv2.waitKey(1)
rospy.Subscriber("/linecam/linecam_raw", Image, camCallback)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
          pass
        except rospy.ROSInterruptException:
            break

    cv2.destroyAllWindows()

rospy.spin() # spin() simply keeps python from exiting until this node is stopped
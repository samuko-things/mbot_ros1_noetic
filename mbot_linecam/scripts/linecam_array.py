import rospy
from std_msgs.msg import String, Int8
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


node_name = "linecam_array"
rospy.init_node(node_name, anonymous=True)


no_of_sensor = 5
sensor_size = 10 #pixels
dist_btw_sensor = 20


prev_reading = ""
print_cnt = False
prev_cam_array = "xxxxx"
cam_array_buffer = "xxxxx"

def convert_to_cam_array(count1, count2, count3, count4, count5):
    global prev_cam_array

    threshold = 10
    cam_array = ""

    if count1 > threshold:
        cam_array +="1"
    else:
        cam_array +="0"

    if count2 > threshold:
        cam_array +="1"
    else:
        cam_array +="0"

    if count3 > threshold:
        cam_array +="1"
    else:
        cam_array +="0"

    if count4 > threshold:
        cam_array +="1"
    else:
        cam_array +="0"

    if count5 > threshold:
        cam_array +="1"
    else:
        cam_array +="0"

    new_cam_array = ""
    if (cam_array == "00000") or (cam_array == "11111") or (cam_array == "00001") or (cam_array == "00011") or (cam_array == "00010") or (cam_array == "00110") or (cam_array == "00100") or (cam_array == "01100") or (cam_array == "01000") or (cam_array == "11000") or (cam_array == "10000") :
        new_cam_array = cam_array
        prev_cam_array = cam_array
    else:
        new_cam_array = prev_cam_array

    return new_cam_array



def convert_to_num(cam_array):
    if cam_array == "00000":
        return 0
    elif cam_array == "10000":
        return 10
    elif cam_array == "11000":
        return 20
    elif cam_array == "01000":
        return 30
    elif cam_array == "01100":
        return 40
    elif cam_array == "00100":
        return 50
    elif cam_array == "00110":
        return 60
    elif cam_array == "00010":
        return 70
    elif cam_array == "00011":
        return 80
    elif cam_array == "00001":
        return 90
    elif cam_array == "11111":
        return 100
    else:
        return 200




pub = rospy.Publisher("/linecam/line_sensor_reading", Int8, queue_size=1000)
sensor_val = Int8()

def camCallback(cam_data):
    global sensor_val, prev_reading

    bridge = CvBridge()

    raw_img = bridge.imgmsg_to_cv2(cam_data, "bgr8")
    # cv2.imshow("Camera output normal", cv_img)

    new_width = 80; new_height = 60
    resized_img = cv2.resize(raw_img, (new_width, new_height)) 
    # cv2.imshow("Camera output resized", resized_image)
    
    hsv = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)

    lower = np.array([0,0,0])
    upper = np.array([15,15,15])
    masked_img = cv2.inRange(hsv, lower, upper)    

    # kernel = np.ones((5,5),'int')
    # eroded = cv2.erode(mask, kernel, iterations=2)
    # dilated = cv2.dilate(eroded,kernel, iterations=4)
    # res = cv2.bitwise_and(img,img,mask=dilated)
    # ret,threshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    # contours,hier = cv2.findContours(threshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    rect_h = 8
    rect_w = 8
    h = 34
    w = 4
    d = 16

    # rect_point = [(w,h), (w+d,h), (w+(d*2),h), (w+(d*3),h), (w+(d*3),h)]
    rect1_point = (w,h)
    rect2_point = (w+d,h)
    rect3_point = (w+(d*2),h)
    rect4_point = (w+(d*3),h)
    rect5_point = (w+(d*4),h)
    cv2.rectangle(resized_img, rect1_point, (rect1_point[0]+rect_w, rect1_point[1]+rect_h), (255,0,0), 2)
    cv2.rectangle(resized_img, rect2_point, (rect2_point[0]+rect_w, rect2_point[1]+rect_h), (255,0,0), 2)
    cv2.rectangle(resized_img, rect3_point, (rect3_point[0]+rect_w, rect3_point[1]+rect_h), (255,0,0), 2)
    cv2.rectangle(resized_img, rect4_point, (rect4_point[0]+rect_w, rect4_point[1]+rect_h), (255,0,0), 2)
    cv2.rectangle(resized_img, rect5_point, (rect5_point[0]+rect_w, rect5_point[1]+rect_h), (255,0,0), 2)

    count1 = 0
    for i in range(h, h+rect_h):
        for j in range(rect1_point[0], rect1_point[0]+rect_w):
            if masked_img[i][j] == 255:
                count1+=1

    count2 = 0
    for i in range(h, h+rect_h):
        for j in range(rect2_point[0], rect2_point[0]+rect_w):
            if masked_img[i][j] == 255:
                count2+=1

    count3 = 0
    for i in range(h, h+rect_h):
        for j in range(rect3_point[0], rect3_point[0]+rect_w):
            if masked_img[i][j] == 255:
                count3+=1

    count4 = 0
    for i in range(h, h+rect_h):
        for j in range(rect4_point[0], rect4_point[0]+rect_w):
            if masked_img[i][j] == 255:
                count4+=1

    count5 = 0
    for i in range(h, h+rect_h):
        for j in range(rect5_point[0], rect5_point[0]+rect_w):
            if masked_img[i][j] == 255:
                count5+=1
    
    cam_array = convert_to_cam_array(count1, count2, count3, count4, count5)
    sensor_val.data = convert_to_num(cam_array)
    pub.publish(sensor_val)

    if cam_array != prev_reading:
        print(cam_array)
        # print(count1, count2, count3, count4, count5)
        print(sensor_val.data)
        prev_reading = ""
        prev_reading = cam_array

    cv2.imshow("img", resized_img)
    cv2.waitKey(1)
    # pass
rospy.Subscriber("/linecam/linecam_raw", Image, camCallback)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
          pass
        except rospy.ROSInterruptException:
            break

    cv2.destroyAllWindows()

rospy.spin() # spin() simply keeps python from exiting until this node is stopped
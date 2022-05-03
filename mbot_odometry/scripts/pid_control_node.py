import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from mbot_pymodule.simplePID import PID
# from mbot_message.msg import OdomData



rospy.init_node("pid_control_node", anonymous=True)
sample_freq=50
rate=rospy.Rate(sample_freq)


error0 = 0; error1 = 0

def getErrorVal(error_val):
    global error0, error1
    error = error_val.data
    error0 = error[0]
    error1 = error[1]
    # rospy.loginfo(f"received_error_val = {error_val.data}")
     
rospy.Subscriber("/pid_error_data", Float32MultiArray, getErrorVal)




kp0 = 1; ki0 = 0.1; kd0 = 0; output_upper_limit0 = 0.5; output_lower_limit0 = -0.5
translationPIDController = PID()
translationPIDController.set_parameters(kp0, ki0, kd0, output_upper_limit0, output_lower_limit0)


kp1 = 1; ki1 = 0.1; kd1 = 0; output_upper_limit1 = 0.5; output_lower_limit1 = -0.5
rotationPIDController = PID()
rotationPIDController.set_parameters(kp1, ki1, kd1, output_upper_limit1, output_lower_limit1)


pub_cmd = rospy.Publisher("/pid_output_data", Float32MultiArray, queue_size=100)

def computePID():
    global error0, error1, translationPIDController, rotationPIDController

    output = Float32MultiArray()
    output0 = translationPIDController.update_PID(error0, 1/sample_freq)
    output1 = rotationPIDController.update_PID(error1, 1/sample_freq)
    output.data = [output0, output1]
    pub_cmd.publish(output)
    # rospy.loginfo(f"output_val = {output.data}")
    rate.sleep()






if __name__=="__main__":
    try:
        while rospy.is_shutdown:
            computePID()
    except rospy.ROSInitException:
        pass



rospy.spin() # simply keeps python from exiting until this node is stopped
import rospy
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, KeyCode, Listener




pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

in_motion = False

lin_vel = 0.3
ang_vel = 0.6


def drive_robot(v, w):
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w

    rospy.loginfo(f"v = {cmd.linear.x}, w = {cmd.angular.z}")
    pub_cmd.publish(cmd)





   
def Stop():
    drive_robot(0,0)

def forward():
    global lin_vel
    drive_robot(lin_vel, 0)

def backward():
    global lin_vel
    drive_robot(-1*lin_vel,0)

def left_turn():
    global ang_vel
    drive_robot(0,ang_vel)

def right_turn():
    global ang_vel
    drive_robot(0,-1*ang_vel)




def press(key):
    
    global in_motion
    
    if key == Key.up:
        if not in_motion:
            in_motion = True
            forward()    

    elif key == Key.down:
        if not in_motion:
            in_motion = True
            backward() 

    elif key == Key.left:
        if not in_motion:
            in_motion = True
            left_turn() 

    elif key == Key.right:
        if not in_motion:
            in_motion = True
            right_turn() 
   


def release(key):
    global in_motion

    if key == Key.up:
        in_motion= False
        Stop()
        
    elif key == Key.down:
        in_motion= False
        Stop()

    elif key == Key.left:
        in_motion= False
        Stop()

    elif key == Key.right:
        in_motion= False
        Stop()

    elif key == Key.esc:
        # Stop listener
        Stop()
        return False




def main():
    rospy.init_node("keyboard_teleop", anonymous=False) #initialize and startup the ros node
    Stop()
    rospy.loginfo("use direction keays to control the robot")

    # ...or, in a non-blocking fashion:
    listener = Listener(on_press=press, on_release=release)
    listener.start()
    listener.join()


if __name__=="__main__":
    main()

    
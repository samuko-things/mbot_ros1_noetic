import rospy
from std_msgs.msg import Int8MultiArray
from pynput.keyboard import Key, KeyCode, Listener


speed_command = 0            # 0 is None,           1 is accelerate,            -1 is deccelerate to stop
move_command = 0             # 0 is None,           1 is forward,                -1 is reverse
direction_command = 0      # 0 is None,          1 is left_turn,               -1 is right turn



pub_cmd = rospy.Publisher("/pynput_cmd", Int8MultiArray, queue_size=100)

def publishMotionCommand(speed_cmd,  move_cmd, direction_cmd):
    global pub_cmd

    motion_code = Int8MultiArray()
    motion_code.data = [speed_cmd,move_cmd, direction_cmd]

    pub_cmd.publish(motion_code)
    # rospy.loginfo(motion_code.data)


    


def press(key):
    global speed_command,  move_command, direction_command
    
    if key == Key.up:
        if move_command == 0:
            move_command = 1
            publishMotionCommand(speed_command, move_command, direction_command)
            
    elif key == Key.down:
        if move_command == 0:
            move_command = -1
            publishMotionCommand(speed_command, move_command, direction_command)
            
    if key == Key.left:
        if direction_command == 0:
            direction_command = 1
            publishMotionCommand(speed_command, move_command, direction_command)
            
    elif key == Key.right:
         if direction_command == 0:
            direction_command = -1
            publishMotionCommand(speed_command, move_command, direction_command)
            
    if key == Key.alt:
         if speed_command == 0:
            speed_command = 1
            publishMotionCommand(speed_command, move_command, direction_command)
                
    elif key == Key.space:
         if speed_command == 0:
            speed_command = -1
            publishMotionCommand(speed_command, move_command, direction_command)

               


def release(key):
    global speed_command,  move_command, direction_command

    if key == Key.up:
        if move_command == 1:
            move_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)
            
    elif key == Key.down:
        if move_command == -1:
            move_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)
            
    if key == Key.left:
        if direction_command == 1:
            direction_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)
            
    elif key == Key.right:
        if direction_command == -1:
            direction_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)
                
    if key == Key.alt:
         if speed_command == 1:
            speed_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)
                
    elif key == Key.space:
         if speed_command == -1:
            speed_command = 0
            publishMotionCommand(speed_command, move_command, direction_command)

    elif key == Key.esc:
        # Stop listener
        return False

    # publishMotionCommand(speed_command, move_command, direction_command)


def main():
    rospy.init_node("keyboard_pynput", anonymous=False) #initialize and startup the ros node

    # ...or, in a non-blocking fashion:
    listener = Listener(on_press=press, on_release=release)
    listener.start()
    listener.join()


if __name__=="__main__":
    main()

    
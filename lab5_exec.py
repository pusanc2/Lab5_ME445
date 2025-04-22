#!/usr/bin/env python3

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *

# ========================= Student's code starts here =========================

go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

xw_yw_G = []
xw_yw_Y = []

Dest = [[0.20, 0.25], [0.20, 0.30],[0.15, 0.25],[0.15, 0.30]]

# ========================= Student's code ends here ===========================

SPIN_RATE = 20

home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

current_position = copy.deepcopy(home)
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False

def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1

def position_callback(msg):
    global thetas
    global current_position
    global current_position_set

    for i in range(6):
        thetas[i] = msg.position[i]
        current_position[i] = thetas[i]

    current_position_set = True

def gripper(pub_cmd, loop_rate, io_0):
    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):
        if all(abs(thetas[i] - driver_msg.destination[i]) < 0.0005 for i in range(6)):
            at_goal = 1

        loop_rate.sleep()

        if(spin_count > SPIN_RATE*5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count += 1

    return error

def move_arm(pub_cmd, loop_rate, dest, vel, accel):
    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):
        if all(abs(thetas[i] - driver_msg.destination[i]) < 0.0005 for i in range(6)):
            at_goal = 1

        loop_rate.sleep()

        if(spin_count > SPIN_RATE*5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count += 1

    return error

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, Dest, vel, accel):

    # ========================= Student's code ends here ===========================


    print("Moving block from", start_xw_yw_zw, "to", Dest)
    
    start_angles_safe = lab_invk(start_xw_yw_zw[0] + 0.002, start_xw_yw_zw[1] + 0.006, start_xw_yw_zw[2] + 0.05, 0)
    start_angles = lab_invk(start_xw_yw_zw[0] + 0.002, start_xw_yw_zw[1] + 0.006, start_xw_yw_zw[2], 0)
    target_angles = lab_invk(Dest[0], Dest[1], 0.04, 0)
    

    gripper(pub_cmd, loop_rate, suction_on)
    move_arm(pub_cmd, loop_rate, start_angles_safe, vel, accel)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, start_angles, vel, accel)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, start_angles_safe, vel, accel)

    if 1.2 < analog_in_0 < 2:
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, go_away, vel, accel)
        rospy.loginfo("Block not found. Exiting.")
        
        return -2

    move_arm(pub_cmd, loop_rate, go_away, vel, accel)
    move_arm(pub_cmd, loop_rate, target_angles, vel, accel)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, go_away, vel, accel)

    return 0

class ImageConverter:
    def __init__(self, SPIN_RATE):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        while(rospy.is_shutdown()):
            print("ROS is shutdown!")

    def image_callback(self, data):
        global xw_yw_G
        global xw_yw_Y

        try:
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)
        cv2.imshow("Raw Feed Debug", cv_image)
        cv2.waitKey(1)

        xw_yw_G = blob_search(cv_image, "green")
        print("Green block coordinates:", xw_yw_G)
        xw_yw_Y = blob_search(cv_image, "yellow")
        print("Yellow block coordinates:", xw_yw_Y)

def main():
    global go_away
    global xw_yw_R
    global xw_yw_G

    rospy.init_node('lab5node')

    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)
    
    def is_close(p1, p2, threshold=0.1):
        return np.linalg.norm(np.array(p1) - np.array(p2)) < threshold

    placed_blocks = set()
    
    while not rospy.is_shutdown():
        for i, coords in enumerate(xw_yw_G):
            if not any(is_close(coords, placed) for placed in placed_blocks):
                move_block(pub_command, loop_rate, [coords[0], coords[1], 0.03], Dest[i % len(Dest)], vel, accel)
                placed_blocks.add(tuple(Dest[i % len(Dest)]))

        for i, coords in enumerate(xw_yw_Y):
            if not any(is_close(coords, placed) for placed in placed_blocks):
                move_block(pub_command, loop_rate, [coords[0], coords[1], 0.025], Dest[(i + 2) % len(Dest)], vel, accel)
                placed_blocks.add(tuple(Dest[(i + 2) % len(Dest)]))

        rospy.sleep(1.0)


    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed.")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

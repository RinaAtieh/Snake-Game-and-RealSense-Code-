#!/usr/bin/env python3
from collections import namedtuple
import util as cm
import cv2
import time
import pyrealsense2 as rs
import math
import numpy as np
from skeletontracker import skeletontracker


import serial
import serial.tools.list_ports


ser = serial.Serial("COM4",9600, timeout = 1)

DIRECTION_UP = 'U'
DIRECTION_DOWN = 'D'
DIRECTION_LEFT = 'L'
DIRECTION_RIGHT = 'R'
DIRECTION_UKNOWN = -1
flagHand = 0
MAX_NUMBER_OF_FRAMES_TO_CHEAT = 10

last_left_hand_direction = DIRECTION_RIGHT
current_times_left_hand_is_in_left_direction = 0

last_sent_data = DIRECTION_UKNOWN

def send_to_arduino(data):
    global last_sent_data
    if not data == last_sent_data:
        last_sent_data = data
        print(data)
        ser.write(str.encode(data))



def HandInTheCenter(skeletons_2d):
    global flagHand
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints
        if (joints_2D[7].x > 200 and joints_2D[7].x < 400 and joints_2D[7].y < 300 and joints_2D[7].y > 150):
            flagHand = 1
            print("Hand in center")
        else:
            flagHand = 0
            MoveUpOrDown(skeletons_2d)


def MoveUpOrDown(skeletons_2d):
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints

        if (joints_2D[7].y < 150):
            ser.write(str.encode('N'))
            print("go up")

        elif (joints_2D[7].y > 300):
            ser.write(str.encode('W'))
            print("go down")

        elif (joints_2D[7].y < 300 and joints_2D[7].y > 150):
            MoveRightOrLeft(skeletons_2d)

def MoveRightOrLeft(skeletons_2d):
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints

        if (joints_2D[7].x < 200):
            ser.write(str.encode('A'))
            print("go left")


        elif (joints_2D[7].x > 400):
            ser.write(str.encode('B'))
            print("go right")



def render_ids_3d_start_game(
    render_image, skeletons_2d, depth_map, depth_intrinsic, joint_confidence
):
    global last_left_hand_direction

    # calculate 3D keypoints and display them
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints

        if (joints_2D[4].x < 200 and joints_2D[4].y < 150 and joints_2D[7].x > 400 and joints_2D[7].y < 150):
            send_to_arduino('S')


def getHandRegion(joints_2D, index):
    retVal = DIRECTION_UKNOWN
    if (joints_2D[index].x < 200 and joints_2D[index].y > 150 and joints_2D[index].y < 300):
        #print("Right")
        retVal = DIRECTION_RIGHT

    elif (joints_2D[index].x > 400 and joints_2D[index].y > 150 and joints_2D[index].y < 300):
        #print("Left")
        retVal = DIRECTION_LEFT

    if (joints_2D[index].x > 200 and joints_2D[index].x < 400 and joints_2D[index].y > 300):
        #print("Down")
        retVal = DIRECTION_DOWN

    elif (joints_2D[index].x > 200 and joints_2D[index].x < 400 and joints_2D[index].y < 150):
        #print("Up")
        retVal = DIRECTION_UP

    return retVal

def getLeftHandRegion(joints_2D):
    return getHandRegion(joints_2D, 7)

def getRightHandRegion(joints_2D):
    return getHandRegion(joints_2D, 4)




def render_ids_3d(
    render_image, skeletons_2d, depth_map, depth_intrinsic, joint_confidence
):
    global current_times_left_hand_is_in_left_direction


    # calculate 3D keypoints and display them
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints
        did_once = False


        current_left_hand_direction = getLeftHandRegion(joints_2D)

        if (current_left_hand_direction == DIRECTION_UKNOWN):
            return

        if current_left_hand_direction != DIRECTION_UKNOWN and current_left_hand_direction != DIRECTION_LEFT :
            send_to_arduino(current_left_hand_direction)

        if current_left_hand_direction != DIRECTION_LEFT:
            current_times_left_hand_is_in_left_direction = 0
        else:
            current_times_left_hand_is_in_left_direction += 1
            if current_times_left_hand_is_in_left_direction >= MAX_NUMBER_OF_FRAMES_TO_CHEAT:
                send_to_arduino(current_left_hand_direction)
                current_times_left_hand_is_in_left_direction = 0
            else:
                current_right_hand_direction = getRightHandRegion(joints_2D)
                if current_right_hand_direction == DIRECTION_RIGHT:
                    current_times_left_hand_is_in_left_direction = 0
                    send_to_arduino('C')


        #print("RIGHT HAND LOCATION = {}     LEFT HAND LOCATION = {}".format(current_right_hand_direction, current_left_hand_direction))



# Main content begins
if __name__ == "__main__":
    try:
        # Configure depth and color streams of the intel realsense
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start the realsense pipeline
        pipeline = rs.pipeline()
        pipeline.start()

        # Create align object to align depth frames to color frames
        align = rs.align(rs.stream.color)

        # Get the intrinsics information for calculation of 3D point
        unaligned_frames = pipeline.wait_for_frames()
        frames = align.process(unaligned_frames)
        depth = frames.get_depth_frame()
        depth_intrinsic = depth.profile.as_video_stream_profile().intrinsics

        # Initialize the cubemos api with a valid license key in default_license_dir()
        skeletrack = skeletontracker(cloud_tracking_api_key="")
        joint_confidence = 0.2

        # Create window for initialisation
        window_name = "cubemos skeleton tracking with realsense D400 series"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL + cv2.WINDOW_KEEPRATIO)

        while True:
            # Create a pipeline object. This object configures the streaming camera and owns it's handle
            unaligned_frames = pipeline.wait_for_frames()
            frames = align.process(unaligned_frames)
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth.get_data())
            color_image = np.asanyarray(color.get_data())

         #   print(color_image[1])
         #   print(len(depth_image))
          #  print(len(depth_image))

            # perform inference and update the tracking id
            skeletons = skeletrack.track_skeletons(color_image)

            # render the skeletons on top of the acquired image and display it
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            cv2.line(color_image, (200, 0), (200, 720), (0, 0, 0))
            cv2.line(color_image, (400, 0), (400, 720), (0, 0, 0))

            cv2.line(color_image, (0, 150), (1280, 150), (0, 0, 0))
            cv2.line(color_image, (0, 300), (640, 300), (0, 0, 0))

            cm.render_result(skeletons, color_image, joint_confidence)
            #data1=ser.read().decode()
            #print(len(skeletons))

            if not len(skeletons) == 1:
                flagHand=0

            if(flagHand == 0):
                HandInTheCenter(skeletons)

            if(flagHand == 1):
                render_ids_3d(
                  color_image, skeletons, depth, depth_intrinsic, joint_confidence
                )

                render_ids_3d_start_game(
                  color_image, skeletons, depth, depth_intrinsic, joint_confidence
                 )

            cv2.imshow(window_name, color_image)
            if cv2.waitKey(1) == 27:
                break

        pipeline.stop()
        cv2.destroyAllWindows()

    except Exception as ex:
        print('Exception occured: "{}"'.format(ex))

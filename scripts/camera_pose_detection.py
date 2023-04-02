#!/usr/bin/env python


import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import mediapipe as mp
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import threading

import cProfile 


class tracker:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        rospy.loginfo("[mediapipe_camer]:",self.cap.isOpened())
        self.start = True
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters =  aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.pr = cProfile.Profile()
        rospy.init_node('body_tracker')
        self.rate = 100 # desired 100 hz actual time per loop 0.13 second
        self._pub_angle = rospy.Publisher('/mediapipe/angle', PointStamped, queue_size=1)
        self._pub_runtime = rospy.Publisher('/mediapipe/runtime', PointStamped, queue_size=1)
        #self.pub_img = rospy.Publisher('/mediapipe/processed_img', Image, queue_size=1)
        
        self.sub = rospy.Subscriber('/start', Bool, callback=self.callback)
        self.processing_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.process_img)
        
        self.bridge = CvBridge()

    def mid_point(self, l_x, l_y, r_x, r_y):
        return (int((l_x + r_x)/2), int((l_y + r_y)/2))

    def flip_corners (self, w, corners):
        for corner in corners:
            corner[0][0][0] = w - corner[0][0][0]
            corner[0][1][0] = w - corner[0][1][0]
            corner[0][2][0] = w - corner[0][2][0]
            corner[0][3][0] = w - corner[0][3][0]

        return corners

    def callback(self, msg):
        self.start = msg.data
    
    def publish_angle(self,angle, publisher: rospy.Publisher):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.point.x = angle
        publisher.publish(msg)

    def publish_runtime(self, float_, publisher: rospy.Publisher):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.point.x = float_
        publisher.publish(msg)

    def process_img (self, image, critical_angle): 
        cv2.setUseOptimized(True)
        original_img = image
        angle = -1
        
        with self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image) # slow function
            # Draw the pose annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            
            # extract specific landmarks (mirrored view)
            lm = results.pose_landmarks
            lmPose = self.mp_pose.PoseLandmark
            h, w = image.shape[:2]

            output_text = None
            output_color = None
            
            if lm is not None:
                # Left shoulder.
                l_shldr_x = int(lm.landmark[lmPose.LEFT_SHOULDER].x * w)
                l_shldr_y = int(lm.landmark[lmPose.LEFT_SHOULDER].y * h)
                
                # Right shoulder.
                r_shldr_x = int(lm.landmark[lmPose.RIGHT_SHOULDER].x * w)
                r_shldr_y = int(lm.landmark[lmPose.RIGHT_SHOULDER].y * h)

                # Left hip.
                l_hip_x = int(lm.landmark[lmPose.LEFT_HIP].x * w)
                l_hip_y = int(lm.landmark[lmPose.LEFT_HIP].y * h)

                # Left hip.
                r_hip_x = int(lm.landmark[lmPose.RIGHT_HIP].x * w)
                r_hip_y = int(lm.landmark[lmPose.RIGHT_HIP].y * h)

                # print("left_shoulder ({},{}) right_shoulder ({},{})".format(l_shldr_x, l_shldr_y, r_shldr_x, r_shldr_y))
            
                c_shldr = self.mid_point(l_shldr_x, l_shldr_y, r_shldr_x, r_shldr_y)
                c_hip = self.mid_point(l_hip_x, l_hip_y, r_hip_x, r_hip_y)
                cv2.circle(image, c_hip, 5, (255, 0, 0), 3)
                cv2.line(image, c_shldr, c_hip, (255, 255, 255), 3)

                diff_x = c_shldr[0] - c_hip[0]
                diff_y = c_shldr[1] - c_hip[1]
                if (diff_x == 0):
                    diff_x = 0.01
                
                angle = (np.arctan([diff_y/diff_x])/np.pi * 180)[0]
                angle = round(90 - abs(angle),3)
                if (angle > critical_angle):
                    output_text = 'angle: {} unstable'.format(angle)
                    output_color = (0,0,255)
                else:
                    output_text = 'angle: {} stable'.format(angle)
                    output_color = (0,255,0)
            else:
                output_text = 'pose not detected'
                output_color = (0,255,255)

            corners, ids, rejectedImgPoints = self.detector.detectMarkers(original_img)  
            
            dists = []
            if len(corners)!=0 and lm is not None:
                for corner in corners:
                    centerY = int((corner[0][0][1] + corner[0][2][1]) / 2)
                    centerX = int((corner[0][0][0] + corner[0][2][0]) / 2)
                    Center = (centerX, centerY)
                    cv2.circle(image, Center, 5, (255, 0, 0), 3)
                    cv2.line(image, c_shldr, Center, (255, 255, 255), 3)
                    dist = np.sqrt((centerY - c_shldr[1]) **2 + (centerX - c_shldr[0]) **2) 
                    dists.append((dist, Center))
                
            image = cv2.flip(image, 1)
            cv2.putText(image, output_text, (50,50),  cv2.FONT_HERSHEY_SIMPLEX, 1, output_color, 3, cv2.LINE_AA)

            if (len(corners) !=0):
                aruco.drawDetectedMarkers(image, self.flip_corners(w, corners), ids, (0, 0, 255))
                for dist in dists:
                    Center = dist[1]
                    Center = (w-Center[0], Center[1])        
                    cv2.putText(image, ('%d px' % dist[0]), (Center[0], Center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 2,
                                cv2.LINE_AA)
        self.publish_angle(angle,self._pub_angle)
        return image

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            start_time = rospy.get_time()
            start = rospy.get_param("/mediapipe/start", True)
            if start == True:
                critical_angle = rospy.get_param("/stiffness_manager/offset_angle", default=10) #limit to instability in degree
                ret, frame = self.cap.read(critical_angle)
                if not ret: 
                    continue
                image = self.process_img(frame, critical_angle)
                cv2.imshow("pose",image)
                cv2.waitKey(1)
            endtime = rospy.get_time()
            runtime = endtime - start_time
            self.publish_runtime(runtime, self._pub_runtime)
            r.sleep()


if __name__ == '__main__':
    human_robot_tracker = tracker()
    human_robot_tracker.run()
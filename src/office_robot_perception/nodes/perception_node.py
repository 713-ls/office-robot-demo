#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2

def find_contours(mask):
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return contours

class PerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        rospy.loginfo("Perception node started,waiting for images...")
        self.detect_pub = rospy.Publisher("/detected_objects",String,queue_size=10)
        
    def callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        
        lower_red1 = (0,100,100)
        upper_red1 = (10,255,255)
        lower_red2 = (170,100,100)
        upper_red2 = (180,255,255)
        mask_red1 = cv2.inRange(hsv,lower_red1,upper_red1)
        mask_red2 = cv2.inRange(hsv,lower_red2,upper_red2)
        mask_red = cv2.bitwise_or(mask_red1,mask_red2)
        
        lower_brown = (10,50,50)
        upper_brown = (25,255,200)
        mask_brown = cv2.inRange(hsv,lower_brown,upper_brown)
        
        red_contours = find_contours(mask_red)
        brown_contours = find_contours(mask_brown)
        
        detections = []
        
        for c in red_contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area < 500:
                continue
            center_x = x + w // 2
            center_y = y + h // 2
            label = "coke_can"
            detections.append((label, x, y, w, h))
            
        for c in brown_contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area < 500:
                continue
            aspect_ratio = float(w) / float(h)
            center_x = x + w // 2
            center_y = y + h // 2
            if aspect_ratio > 1.4:
                label = "wooden_board"
            else:
                label = "cardboard_box"
            detections.append((label, x, y, w, h))
        
        det_strs = []
        for label, x, y, w, h in detections:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            det_strs.append("%s at (%d,%d)" % (label, x + w // 2, y + h // 2))

        if det_strs:
            self.detect_pub.publish(", ".join(det_strs))
        
        cv2.imshow("Camera",cv_image)
        cv2.waitKey(1)
        
if __name__ == "__main__":
    rospy.init_node("perception_node")
    node = PerceptionNode()
    rospy.spin()        

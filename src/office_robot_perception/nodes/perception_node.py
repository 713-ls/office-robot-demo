#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


RED1_L = np.array([0, 90, 60])
RED1_U = np.array([10, 255, 255])
RED2_L = np.array([170, 90, 60])
RED2_U = np.array([180, 255, 255])

# The blue and green marks are attached only to the wooden board and box in
# office.world.  They make the targets visible when placed on a wood table.
BLUE_L = np.array([95, 100, 60])
BLUE_U = np.array([130, 255, 255])
GREEN_L = np.array([45, 80, 60])
GREEN_U = np.array([85, 255, 255])

# At the initial pose the can is about 8 m away and only occupies a few
# pixels.  Shape checks below keep this lower threshold from accepting noise.
MIN_COKE_AREA = 18
MIN_TAG_AREA = 40
COKE_MIN_ASPECT = 1.2
COKE_MAX_ASPECT = 5.5
COKE_MIN_FILL_RATIO = 0.65


def find_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def clean_mask(mask, close_size=5):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    close_kernel = np.ones((close_size, close_size), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)


def iou(first, second):
    _, ax, ay, aw, ah, _ = first
    _, bx, by, bw, bh, _ = second
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    union = aw * ah + bw * bh - intersection
    return float(intersection) / union if union > 0 else 0.0


class PerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.process_hz = float(rospy.get_param("~process_hz", 6.0))
        show_window = rospy.get_param("~show_window", True)
        self.show_window = str(show_window).lower() not in ("false", "0", "no")
        self.last_process_time = 0.0

        image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.sub = rospy.Subscriber(
            image_topic, Image, self.callback, queue_size=1, buff_size=2 ** 24
        )
        self.detect_pub = rospy.Publisher("/detected_objects", String, queue_size=1)
        self.debug_pub = rospy.Publisher("/perception/debug_image", Image, queue_size=1)

        if self.show_window:
            cv2.namedWindow("Office Robot Perception", cv2.WINDOW_NORMAL)
            rospy.on_shutdown(cv2.destroyAllWindows)

        # OpenCV defaults to multiple worker threads.  A single lightweight
        # worker prevents image processing from starving Gazebo/TF on a laptop.
        cv2.setNumThreads(1)
        rospy.loginfo(
            "Perception started: %s at %.1f Hz; this node does not publish TF",
            image_topic,
            self.process_hz,
        )

    def callback(self, msg):
        now = time.monotonic()
        if self.process_hz > 0 and now - self.last_process_time < 1.0 / self.process_hz:
            return
        self.last_process_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "camera conversion failed: %s", error)
            return

        detections = self.detect(image)
        debug = self.draw_detections(image, detections)
        self.publish_debug(debug, msg.header)
        if self.show_window:
            cv2.imshow("Office Robot Perception", debug)
            cv2.waitKey(1)

        if detections:
            lines = []
            for label, x, y, w, h, confidence in detections:
                lines.append(
                    "%s,%.2f,%d,%d,%d,%d"
                    % (label, confidence, x + w // 2, y + h // 2, w, h)
                )
            self.detect_pub.publish("\n".join(lines))

    def detect(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []

        red_mask = clean_mask(
            cv2.bitwise_or(
                cv2.inRange(hsv, RED1_L, RED1_U),
                cv2.inRange(hsv, RED2_L, RED2_U),
            ),
            close_size=7,
        )
        detections.extend(self.find_coke(red_mask))

        # Do not use the brown table surface as a target.  The target marks are
        # on the visible faces, so detection remains stable above ground level.
        detections.extend(self.find_tag(clean_mask(cv2.inRange(hsv, BLUE_L, BLUE_U)), "wooden_board"))
        detections.extend(self.find_tag(clean_mask(cv2.inRange(hsv, GREEN_L, GREEN_U)), "cardboard_box"))
        return self.nms(detections)

    def find_coke(self, mask):
        detections = []
        for contour in find_contours(mask):
            area = cv2.contourArea(contour)
            if area < MIN_COKE_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 3 or h < 5:
                continue

            aspect = float(h) / w
            fill_ratio = float(area) / (w * h)
            # A Coke can is a compact vertical cylinder.  The red cone has a
            # triangular/trapezoidal red region, so its fill ratio is lower.
            if not (COKE_MIN_ASPECT <= aspect <= COKE_MAX_ASPECT):
                continue
            if fill_ratio < COKE_MIN_FILL_RATIO:
                continue

            confidence = min(1.0, 0.55 + 0.45 * min(area / 900.0, 1.0))
            detections.append(("coke_can", x, y, w, h, round(confidence, 2)))
        return detections

    def find_tag(self, mask, label):
        detections = []
        for contour in find_contours(mask):
            area = cv2.contourArea(contour)
            if area < MIN_TAG_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 4 or h < 4:
                continue
            fill_ratio = float(area) / (w * h)
            if fill_ratio < 0.55:
                continue

            confidence = min(1.0, 0.65 + 0.35 * min(area / 500.0, 1.0))
            detections.append((label, x, y, w, h, round(confidence, 2)))
        return detections

    @staticmethod
    def nms(detections, threshold=0.5):
        by_label = {}
        for detection in detections:
            by_label.setdefault(detection[0], []).append(detection)

        output = []
        for candidates in by_label.values():
            candidates.sort(key=lambda item: item[5], reverse=True)
            while candidates:
                best = candidates.pop(0)
                output.append(best)
                candidates = [
                    candidate for candidate in candidates if iou(best, candidate) < threshold
                ]
        return output

    @staticmethod
    def draw_detections(image, detections):
        debug = image.copy()
        colors = {
            "coke_can": (0, 0, 255),
            "wooden_board": (255, 0, 0),
            "cardboard_box": (0, 255, 0),
        }
        for label, x, y, w, h, confidence in detections:
            color = colors[label]
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            cv2.putText(
                debug,
                "%s %.2f" % (label, confidence),
                (x, max(y - 5, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        return debug

    def publish_debug(self, debug, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "debug image conversion failed: %s", error)


if __name__ == "__main__":
    rospy.init_node("perception_node")
    PerceptionNode()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class VisualizationNode:
    """发布 RViz 标记：取货点、配送点、任务状态文字、携带物、检测高亮。"""

    def __init__(self):
        self.pickup_points = {
            label: tuple(point)
            for label, point in rospy.get_param(
                "~pickup_points",
                {
                    "cardboard_box": [-3.0, 3.0, -1.5708],
                    "wooden_board": [0.0, 3.0, -1.5708],
                    "coke_can": [3.0, 3.0, -1.5708],
                },
            ).items()
        }
        self.delivery_point = tuple(
            rospy.get_param("~delivery_point", [-1.5, -3.0, 0.0])
        )

        self.state = "IDLE"
        self.detected = set()

        self.marker_pub = rospy.Publisher(
            "/visualization_marker_array", MarkerArray, queue_size=5
        )
        rospy.Subscriber("/task_state", String, self.on_task_state, queue_size=5)
        rospy.Subscriber(
            "/detected_objects", String, self.on_detected, queue_size=5
        )

        rospy.Timer(rospy.Duration(0.2), self.publish_markers)
        rospy.loginfo("Visualization started (frame: map)")

    def on_task_state(self, msg):
        self.state = msg.data.strip()

    def on_detected(self, msg):
        labels = set()
        for line in msg.data.splitlines():
            parts = line.split(",")
            if parts:
                labels.add(parts[0].strip())
        self.detected = labels

    # ---------- 标记生成 ----------

    def _base_marker(self, ns, mid, mtype, x, y, z, scale, r, g, b, text=""):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = mid
        marker.type = mtype
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        if text:
            marker.text = text
        return marker

    def _state_color(self):
        palette = {
            "IDLE": (0.6, 0.6, 0.6),
            "DETECTED": (1.0, 1.0, 0.0),
            "NAVIGATING_TO_OBJECT": (0.2, 0.6, 1.0),
            "PICKING": (1.0, 0.6, 0.0),
            "NAVIGATING_TO_DELIVERY": (0.2, 0.6, 1.0),
            "PLACING": (1.0, 0.6, 0.0),
            "RETURNING_HOME": (0.2, 0.6, 1.0),
            "DONE": (0.1, 0.9, 0.2),
            "FAILED": (1.0, 0.0, 0.0),
        }
        return palette.get(self.state, (0.8, 0.8, 0.8))

    def publish_markers(self, _event):
        markers = []
        mid = 0

        # 取货点：三张桌子前方的球体
        for label, (x, y, _yaw) in sorted(self.pickup_points.items()):
            color = (1.0, 1.0, 0.0) if label in self.detected else (0.2, 0.6, 1.0)
            scale = (0.55, 0.55, 0.55) if label in self.detected else (0.35, 0.35, 0.35)
            markers.append(
                self._base_marker(
                    "pickup_points", mid, Marker.SPHERE,
                    x, y, 0.35, scale, color[0], color[1], color[2],
                )
            )
            mid += 1

        # 配送点：绿色圆柱
        dx, dy, _ = self.delivery_point
        markers.append(
            self._base_marker(
                "delivery_point", mid, Marker.CYLINDER,
                dx, dy, 0.04, (0.35, 0.35, 0.08),
                0.1, 0.9, 0.2,
            )
        )
        mid += 1

        # 携带物：PICKING / NAVIGATING_TO_DELIVERY 时在配送点上方，PLACING 时落地
        if self.state in ("PICKING", "NAVIGATING_TO_DELIVERY"):
            markers.append(
                self._base_marker(
                    "carried", mid, Marker.SPHERE,
                    dx, dy, 1.5, (0.25, 0.25, 0.25),
                    1.0, 0.5, 0.0,
                )
            )
            mid += 1
        elif self.state == "PLACING":
            markers.append(
                self._base_marker(
                    "carried", mid, Marker.SPHERE,
                    dx, dy, 0.35, (0.25, 0.25, 0.25),
                    1.0, 0.5, 0.0,
                )
            )
            mid += 1

        # 任务状态文字：显示在配送点上方
        r, g, b = self._state_color()
        markers.append(
            self._base_marker(
                "task_state", mid, Marker.TEXT_VIEW_FACING,
                dx, dy, 2.0, (0.0, 0.0, 0.3),
                r, g, b, text=self.state,
            )
        )

        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rospy.init_node("visualization_node")
    VisualizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()

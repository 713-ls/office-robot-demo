#!/usr/bin/env python3
import threading

import rospy
from std_msgs.msg import String


class TaskManager:
    """任务状态机：巡逻 -> 感知触发 -> 取货 -> 配送 -> 回家。

    导航目标不直接发给 move_base，而是发给 /task_command，
    由 navigation_manager_node 串行执行（保留其重试/清代价地图逻辑）。
    """

    def __init__(self):
        self.home = tuple(rospy.get_param("~home", [-3.0, -4.0, 0.0]))
        self.delivery_point = tuple(
            rospy.get_param("~delivery_point", [-1.5, -3.0, 0.0])
        )
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
        self.patrol_points = [
            tuple(point)
            for point in rospy.get_param(
                "~patrol_points",
                [
                    [-3.0, 3.0, -1.5708],
                    [0.0, 3.0, -1.5708],
                    [3.0, 3.0, -1.5708],
                ],
            )
        ]
        self.pick_duration = float(rospy.get_param("~pick_duration", 2.0))
        self.place_duration = float(rospy.get_param("~place_duration", 2.0))
        self.patrol_wait = float(rospy.get_param("~patrol_wait", 4.0))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 150.0))
        self.auto_start_delay = float(
            rospy.get_param("~auto_start_delay", 3.0)
        )
        self.detect_confirm_frames = int(
            rospy.get_param("~detect_confirm_frames", 3)
        )

        self.cmd_pub = rospy.Publisher("/task_command", String, queue_size=5)
        self.state_pub = rospy.Publisher(
            "/task_state", String, queue_size=5, latch=True
        )

        self._nav_event = threading.Event()
        self._nav_ok = False
        rospy.Subscriber("/navigation_state", String, self.on_nav_state, queue_size=5)
        rospy.Subscriber("/task_control", String, self.on_task_control, queue_size=5)

        self._busy = False
        self._patrol_active = False
        self._task_active = False

        self.set_state("IDLE")
        rospy.Timer(
            rospy.Duration(self.auto_start_delay), self._auto_start, oneshot=True
        )

    # ---------- 对外接口 ----------

    def set_state(self, state, note=""):
        self.state_pub.publish(state)
        rospy.loginfo(
            "task_state -> %s%s", state, (" (%s)" % note) if note else ""
        )

    def on_task_control(self, msg):
        if msg.data.strip() == "start":
            self._start_patrol()
        else:
            rospy.logwarn("unknown task_control command: %s", msg.data)

    def on_nav_state(self, msg):
        state = msg.data.strip()
        if state in ("SUCCEEDED", "FAILED"):
            self._nav_ok = state == "SUCCEEDED"
            self._nav_event.set()

    # ---------- 巡逻与任务执行 ----------

    def _auto_start(self, _event):
        rospy.loginfo("auto start patrol")
        self._start_patrol()

    def _start_patrol(self):
        if self._busy:
            rospy.logwarn("task already running, ignore start")
            return
        self._busy = True
        threading.Thread(target=self._patrol_loop, daemon=True).start()

    def _patrol_loop(self):
        self._patrol_active = True
        try:
            for index, point in enumerate(self.patrol_points):
                rospy.loginfo(
                    "patrol %d/%d -> (%.2f, %.2f)",
                    index + 1, len(self.patrol_points), point[0], point[1],
                )
                ok = self._execute_nav(point)
                if not ok:
                    self._fail_and_home(
                        "patrol navigation failed at point %d" % (index + 1)
                    )
                    return
                # 第一阶段：仅停留等待；第二阶段接入感知确认
                rospy.sleep(self.patrol_wait)
            self._fail_and_home("no target detected after full patrol")
        finally:
            self._patrol_active = False
            self._busy = False

    def _run_task(self, label):
        self._task_active = True
        try:
            self.set_state("DETECTED", label)
            pickup = self.pickup_points.get(label)
            if pickup is None:
                self._fail_and_home("unknown target label: %s" % label)
                return

            self.set_state("NAVIGATING_TO_OBJECT", label)
            if not self._execute_nav(pickup):
                self._fail_and_home("navigation to pickup failed")
                return

            self.set_state("PICKING", label)
            rospy.sleep(self.pick_duration)

            self.set_state("NAVIGATING_TO_DELIVERY", label)
            if not self._execute_nav(self.delivery_point):
                self._fail_and_home("navigation to delivery failed")
                return

            self.set_state("PLACING", label)
            rospy.sleep(self.place_duration)

            self.set_state("RETURNING_HOME", label)
            if not self._execute_home():
                self._fail_and_home("return home failed")
                return

            self.set_state("DONE", label)
            rospy.sleep(2.0)
            self.set_state("IDLE")
        finally:
            self._task_active = False

    def _fail_and_home(self, reason):
        self.set_state("FAILED", reason)
        rospy.sleep(1.0)
        ok = self._execute_home()
        if not ok:
            rospy.logerr("return home after failure also failed: %s", reason)
        self.set_state("IDLE", "returned home after failure")

    # ---------- 导航命令 ----------

    def _execute_nav(self, pose):
        self._nav_event.clear()
        self.cmd_pub.publish("goto:%.3f,%.3f,%.3f" % tuple(pose))
        if self._nav_event.wait(self.nav_timeout):
            return self._nav_ok
        rospy.logerr("navigation timed out for (%.2f, %.2f)", pose[0], pose[1])
        return False

    def _execute_home(self):
        self._nav_event.clear()
        self.cmd_pub.publish("return_home")
        if self._nav_event.wait(self.nav_timeout):
            return self._nav_ok
        rospy.logerr("return home timed out")
        return False


def main():
    rospy.init_node("task_manager_node")
    TaskManager()
    rospy.spin()


if __name__ == "__main__":
    main()

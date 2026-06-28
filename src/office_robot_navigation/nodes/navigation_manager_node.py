#!/usr/bin/env python3
import math
import rospy
import tf
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty


TERMINAL_STATES = {
    GoalStatus.SUCCEEDED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.PREEMPTED,
    GoalStatus.LOST,
}


def make_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def clear_costmaps():
    try:
        rospy.wait_for_service("/move_base/clear_costmaps", timeout=3.0)
        srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        srv()
        rospy.sleep(0.5)
        rospy.logwarn("costmaps cleared")
    except Exception as e:
        rospy.logwarn("clear costmap failed: %s", str(e))


def get_distance(tf_listener, gx, gy):
    for base in ["base_footprint", "base_link"]:
        try:
            trans, _ = tf_listener.lookupTransform("map", base, rospy.Time(0))
            return math.hypot(trans[0] - gx, trans[1] - gy)
        except:
            continue
    return None


def wait_stable_goal(client, tf_listener, gx, gy, tolerance):
    """
    防止 TF 抖动导致误判：
    连续 1 秒稳定小于 tolerance 才算到达
    """
    stable_time = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        dist = get_distance(tf_listener, gx, gy)
        state = client.get_state()

        if dist is None:
            rate.sleep()
            continue

        if dist < tolerance:
            stable_time += 0.1
        else:
            stable_time = 0.0

        # 必须 move_base 也处于完成状态才算成功
        if stable_time >= 1.0 and state in TERMINAL_STATES:
            return True

        # 如果还在 active/oscillation，不提前结束
        if state in TERMINAL_STATES and dist < tolerance:
            return True

        rate.sleep()

    return False


def send_goal(client, tf_listener, gx, gy, yaw=0.0,
              timeout=90.0, retries=1, tolerance=0.35):

    for attempt in range(1, retries + 2):
        if rospy.is_shutdown():
            return False

        rospy.loginfo("Go to (%.2f, %.2f) attempt %d", gx, gy, attempt)

        goal = make_goal(gx, gy, yaw)
        client.send_goal(goal)

        start = rospy.Time.now()
        rate = rospy.Rate(5)

        last_state = None

        while not rospy.is_shutdown():
            state = client.get_state()
            dist = get_distance(tf_listener, gx, gy)

            if state != last_state:
                rospy.loginfo("state change: %s", state)
                last_state = state

            # timeout
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("timeout goal")
                client.cancel_goal()
                rospy.sleep(0.5)
                break

            # 真成功
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("move_base succeeded")
                return True

            # 失败状态
            if state in TERMINAL_STATES:
                break

            rate.sleep()

        # 二次验证（关键：防 TF 抖动误判）
        if wait_stable_goal(client, tf_listener, gx, gy, tolerance):
            rospy.loginfo("goal confirmed stable reached")
            return True

        rospy.logwarn("goal failed (state=%s dist=%s)", state, dist)

        clear_costmaps()

    return False


def main():
    rospy.init_node("navigation_manager_node")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    tf_listener = tf.TransformListener()
    rospy.sleep(1.0)

    goals = [
        (-3.0, 3.0, 0.0),
        (0.0, 3.0, 0.0),
        (3.0, 3.0, 0.0),
    ]

    home = (-3.0, -4.0, 0.0)

    failed = []

    try:
        for g in goals:
            ok = send_goal(client, tf_listener, *g)
            if not ok:
                failed.append(g)

    finally:
        rospy.loginfo("return home")
        send_goal(client, tf_listener, *home, retries=2)

    rospy.loginfo("done. failed goals: %s", failed)


if __name__ == "__main__":
    main()

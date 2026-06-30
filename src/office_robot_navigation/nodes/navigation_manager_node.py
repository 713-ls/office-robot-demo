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


def wait_stable_goal(client, tf_listener, gx, gy, tolerance, timeout=15.0):
    """
    二次验证：防止 TF 抖动导致误判。
    连续 1 秒稳定小于 tolerance 且 move_base 已终止 才算到达。
    超时返回 False。
    """
    stable_time = 0.0
    rate = rospy.Rate(10)
    start = rospy.Time.now()

    while not rospy.is_shutdown():
        dist = get_distance(tf_listener, gx, gy)
        state = client.get_state()

        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("wait_stable_goal timeout after %.1fs", timeout)
            return False

        if dist is None:
            rate.sleep()
            continue

        if dist < tolerance:
            stable_time += 0.1
        else:
            stable_time = 0.0

        # 必须 move_base 也处于完成状态才算成功
        if stable_time >= 1.0 and state in TERMINAL_STATES:
            rospy.loginfo("stable for 1.0s at dist=%.3f, state=%s", dist, state)
            return True

        # 如果 move_base 终止且已经在容差内，直接算到达
        if state in TERMINAL_STATES and dist < tolerance:
            rospy.loginfo("terminal + within tolerance (dist=%.3f, state=%s)", dist, state)
            return True

        rate.sleep()

    return False


def send_goal(client, tf_listener, gx, gy, yaw=0.0,
              timeout=90.0, retries=1, tolerance=0.35):

    # 0. 如果机器人已经在目标点附近（0.3m），直接跳过
    pre_dist = get_distance(tf_listener, gx, gy)
    if pre_dist is not None and pre_dist < 0.30:
        rospy.loginfo("Already at (%.2f, %.2f), dist=%.3f, skipping", gx, gy, pre_dist)
        return True

    # 1. 仅当上一目标仍在活跃时才取消；否则等状态自然沉淀
    current_state = client.get_state()
    if current_state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        client.cancel_goal()
    try:
        client.wait_for_result(rospy.Duration(1.0))  # 让 SimpleActionClient 内部 DONE 落定
    except:
        pass

    # 2. 清除代价地图
    clear_costmaps()

    for attempt in range(1, retries + 2):
        if rospy.is_shutdown():
            return False

        rospy.loginfo("Go to (%.2f, %.2f) attempt %d/%d", gx, gy, attempt, retries + 1)

        goal = make_goal(gx, gy, yaw)
        client.send_goal(goal)
        rospy.sleep(0.3)  # 等 move_base 接收新目标

        start = rospy.Time.now()
        rate = rospy.Rate(5)

        last_state = None

        while not rospy.is_shutdown():
            state = client.get_state()
            dist = get_distance(tf_listener, gx, gy)

            if state != last_state:
                rospy.loginfo("  state: %s (dist=%.3f)", state, dist if dist else -1)
                last_state = state

            # 超时（此时 goal 一定 ACTIVE，取消安全）
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("  timeout after %.1fs", timeout)
                client.cancel_goal()
                client.wait_for_result(rospy.Duration(2.0))
                break

            # move_base 报告 SUCCEEDED
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("  -> move_base SUCCEEDED")
                client.wait_for_result(rospy.Duration(0.5))
                return True

            # 到达终止状态
            if state in TERMINAL_STATES:
                rospy.logwarn("  -> terminal: %s (dist=%.3f)", state, dist if dist else -1)
                client.wait_for_result(rospy.Duration(0.5))
                break

            rate.sleep()

        # 3. 二次验证：move_base 虽然终止了，但机器人可能已经到了
        if wait_stable_goal(client, tf_listener, gx, gy, tolerance, timeout=15.0):
            rospy.loginfo("  -> goal confirmed (secondary check passed)")
            try:
                client.wait_for_result(rospy.Duration(0.5))
            except:
                pass
            rospy.sleep(0.3)
            return True

        # 4. 确实失败：报告原因
        final_dist = get_distance(tf_listener, gx, gy)
        rospy.logwarn("  -> goal failed: state=%s dist=%.3f", state, final_dist if final_dist else -1)

        # 5. 重试前清理（此时 goal 已终止，只清代价地图，不 cancel）
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
        # === 两段式回家：先沿已知通道回 (-3,3)，再直下到家 ===
        # 这样可以避免在 (3,3) 死胡同里硬转弯
        rospy.loginfo("return phase 1/2: follow corridor back to (-3.0, 3.0)")
        clear_costmaps()
        rospy.sleep(1.0)
        ok = send_goal(client, tf_listener, -3.0, 3.0, 0.0, retries=2, timeout=90.0)
        if not ok:
            rospy.logwarn("waypoint (-3,3) failed, will try direct home anyway")

        rospy.loginfo("return phase 2/2: go home (-3.0, -4.0)")
        clear_costmaps()
        rospy.sleep(1.0)
        try:
            client.cancel_all_goals()
        except:
            pass
        rospy.sleep(0.5)
        ok = send_goal(client, tf_listener, *home, retries=3, timeout=180.0)
        if ok:
            rospy.loginfo("HOME reached!")
        else:
            rospy.logerr("HOME failed!")

    rospy.loginfo("done. failed goals: %s", failed)


if __name__ == "__main__":
    main()

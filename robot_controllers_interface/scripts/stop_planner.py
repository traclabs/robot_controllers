#!/usr/bin/env python

# Author: Stephen Hart

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_controllers_msgs.msg import QueryPlannerStatesAction, \
                                       QueryPlannerStatesGoal, \
                                       PlannerState

ACTION_NAME = "/query_planner_states"

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage: stop_planner.py <planner_name> [optional_planner_type]")
        exit(-1)

    rospy.init_node("stop_robot_planners")

    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client = actionlib.SimpleActionClient(ACTION_NAME, QueryPlannerStatesAction)
    client.wait_for_server()
    rospy.loginfo("Done.")

    state = ControllerState()
    state.name = sys.argv[1]
    if len(sys.argv) > 2:
        state.type = sys.argv[2]
    state.state = state.STOPPED

    goal = QueryPlannerStatesGoal()
    goal.updates.append(state)

    rospy.loginfo("Requesting that %s be stopped..." % state.name)
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Done.")
    elif client.get_state() == GoalStatus.ABORTED:
        rospy.logerr(client.get_goal_status_text())

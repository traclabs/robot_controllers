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
    rospy.init_node("get_robot_planners_state")

    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client = actionlib.SimpleActionClient(ACTION_NAME, QueryPlannerStatesAction)
    client.wait_for_server()

    rospy.loginfo("Requesting state of planners...")

    goal = QueryPlannerStatesGoal()
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == GoalStatus.SUCCEEDED:
        result = client.get_result()
        for state in result.state:
            if state.state == state.RUNNING:
                print("%s[%s]: RUNNING" % (state.name, state.type))
            elif state.state == state.STOPPED:
                print("%s[%s]: STOPPED" % (state.name, state.type))
            elif state.state == state.ERROR:
                print("%s[%s]: ERROR!!" % (state.name, state.type))
    elif client.get_state() == GoalStatus.ABORTED:
        rospy.logerr(client.get_goal_status_text())

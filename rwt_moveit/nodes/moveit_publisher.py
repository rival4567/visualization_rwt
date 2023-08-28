#!/usr/bin/python3

import roslib
import rospy
import time
import actionlib
roslib.load_manifest("rwt_moveit")

from rwt_moveit.msg import MoveGroupPlan
from moveit_msgs.srv import *
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import WorkspaceParameters
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MoveGroupAction
from moveit_msgs.msg import MoveGroupGoal
from moveit_msgs.msg import MoveGroupActionGoal
from moveit_msgs.msg import MotionPlanRequest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveGroupActionResult
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionFeedback

def joint_position_callback(joints):

    global plan_only
    fixed_frame = rospy.get_param("/fixed_frame")

    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    client.wait_for_server()
    move_group_goal = MoveGroupGoal()

    try:

        msg = MotionPlanRequest()
        workspace_parameters = WorkspaceParameters()
        workspace_parameters.header.stamp = rospy.Time.now()
        workspace_parameters.header.frame_id = fixed_frame
        workspace_parameters.min_corner = Vector3(-1.0, -1.0, -1.0)
        workspace_parameters.max_corner = Vector3(1.0, 1.0, 1.0)

        start_state = RobotState()
#        start_state.joint_state.header.stamp = rospy.Time.now()
        start_state.joint_state.header.frame_id = fixed_frame
        start_state.joint_state.name =  []
        start_state.joint_state.position = []

        cons = Constraints()
        cons.name = ""
        i = 0
        for dim in joints.start_joint.layout.dim:
            start_state.joint_state.name.append(dim.label)
            start_state.joint_state.position.append(joints.start_joint.data[i])

            jc = JointConstraint()
            jc.joint_name = dim.label
            jc.position = joints.goal_joint.data[i]
            jc.tolerance_above = 0.0001
            jc.tolerance_below = 0.0001
            jc.weight = 1.0
            i = i + 1
            cons.joint_constraints.append(jc)


        msg.workspace_parameters = workspace_parameters
        msg.start_state = start_state
        msg.goal_constraints.append(cons)

        msg.num_planning_attempts = 1
        msg.allowed_planning_time = 5.0

        msg.group_name = joints.group_name
        move_group_goal.request = msg


        if joints.plan_only:
            plan_only = True
            move_group_goal.planning_options.plan_only = True
        else:
            plan_only = False
        rospy.logwarn("Sim Mode: " + str(rospy.get_param('/sim_mode')))
        if rospy.get_param('/sim_mode'):
            move_group_goal.planning_options.plan_only = True

        client.send_goal(move_group_goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))


    except rospy.ROSInterruptException as e:
        print("failed: %s"%e)


def moveit_callback(msg):    
    stock_pub = rospy.Publisher('/stock_joint_position', Float64MultiArray, queue_size=60)
    pub = rospy.Publisher('/update_joint_position', Float64MultiArray, queue_size=60)

    global plan_only

    if 'moveit_publisher' in msg.status.goal_id.id:
        r = rospy.Rate(10)
        names = msg.result.planned_trajectory.joint_trajectory.joint_names
        points = msg.result.planned_trajectory.joint_trajectory.points

        for point in points:
            pos_msg = Float64MultiArray()
            for name in names:
                dim = MultiArrayDimension()
                dim.label = name
                pos_msg.layout.dim.append(dim)
            pos_msg.data = point.positions
            if plan_only:
                stock_pub.publish(pos_msg)
            if rospy.get_param('/sim_mode') and not plan_only:
                pub.publish(pos_msg)
            r.sleep()

        if plan_only:
            global robot_trajectory
            robot_trajectory = msg.result.planned_trajectory

def update_joint_position(msg):
    if not rospy.get_param('/sim_mode'):
        pub = rospy.Publisher('/update_joint_position', Float64MultiArray, queue_size=60)
        names = msg.feedback.joint_names
        positions = msg.feedback.actual.positions
        pos_msg = Float64MultiArray()
        for name in names:
            dim = MultiArrayDimension()
            dim.label = name
            pos_msg.layout.dim.append(dim)
        pos_msg.data = positions
        pub.publish(pos_msg)    

def execute_callback(msg):
    if not rospy.get_param('/sim_mode'):
        global robot_trajectory
        robot_goal = FollowJointTrajectoryActionGoal()  
        manipulator_execute_pub = rospy.Publisher(
            "/kr1410/manipulator_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=60
        )
        end_effector_execute_pub = rospy.Publisher(
            "/kr1410/end_effector_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=60
        )
        robot_goal.goal.trajectory = robot_trajectory.joint_trajectory
        manipulator_execute_pub.publish(robot_goal)
        end_effector_execute_pub.publish(robot_goal)


if __name__ == '__main__':
    rospy.init_node('moveit_publisher')
    rospy.Subscriber("/execute_trajectory", Empty, execute_callback)
    rospy.Subscriber("moveit_joint", MoveGroupPlan, joint_position_callback)
    rospy.Subscriber("/move_group/result", MoveGroupActionResult, moveit_callback)
    rospy.Subscriber('/kr1410/manipulator_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, update_joint_position)
    rospy.Subscriber('/kr1410/end_effector_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, update_joint_position)

    rospy.spin()

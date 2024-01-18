#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Vector3
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler


from ur3_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles
    print("============Initial Joint Configuration:", start_joint_angles)


    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)

"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def unity_to_ros(pose):
    position = Vector3(pose.position.z, -pose.position.x, pose.position.y)

    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    #orientation = Quaternion(qz, -qx, qy, qw)  # Rearrange components as needed'''
    orientation = Quaternion(qx, qy, qz, qw)
    return Pose(position, orientation)

def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ####### Blue Ball 1 #########
    current_robot_joint_configuration = req.joints_input.joints
    #unity_home.position.x += 0.5    # ANTES DE unity_to_ros y=up, x= to me and z= rigth (if we are in the camera of unity) 
    unity_home = unity_to_ros(req.pick_pose_blueball_1)
    unity_home.position.z += 0.2     # DESPUES DE unity_to_ros y=to inside, x= to right and z= up (if we are in the camera of unity)
    #create_ball(unity_home)
    # Move to home position of safety
    home_pose = plan_trajectory(move_group,unity_home, current_robot_joint_configuration)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not home_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = home_pose.joint_trajectory.points[-1].positions
    
    # Pre grasp - position gripper directly above target object
    unity_home.position.z -= 0.07
    grasp_pose = plan_trajectory(move_group,unity_home, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not grasp_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    
    # Pick Up - raise gripper back to thepre-grasp position
    unity_home.position.z += 0.2
    pick_up_pose = plan_trajectory(move_group, unity_home, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    
    # Place - move gripper to the desired placement position
    basket_1 = unity_to_ros(req.Place_basket_1)
    basket_1.position.z += 0.3
    place_pose_basket_1 = plan_trajectory(move_group, basket_1, previous_ending_joint_angles)
        
    if not place_pose_basket_1.joint_trajectory.points:
        return response    
    previous_ending_joint_angles = place_pose_basket_1.joint_trajectory.points[-1].positions 

    ####### Blue Ball 2 #########
    #unity_home.position.x += 0.5    # ANTES DE unity_to_ros y=up, x= to me and z= rigth (if we are in the camera of unity) 
    blue_ball_2 = unity_to_ros(req.pick_pose_blueball_2)
    blue_ball_2.position.z += 0.27     # DESPUES DE unity_to_ros y=to inside, x= to right and z= up (if we are in the camera of unity)
    
    # Move to home position of safety
    blue_ball_2_home_pose = plan_trajectory(move_group,blue_ball_2, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not blue_ball_2_home_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = blue_ball_2_home_pose.joint_trajectory.points[-1].positions
    
    # Pre grasp - position gripper directly above target object
    blue_ball_2.position.z -= 0.14
    blue_ball_2_grasp_pose = plan_trajectory(move_group,blue_ball_2, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not blue_ball_2_grasp_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = blue_ball_2_grasp_pose.joint_trajectory.points[-1].positions
    
    # Pick Up - raise gripper back to thepre-grasp position
    blue_ball_2.position.z += 0.2
    blue_ball_2_pick_up_pose = plan_trajectory(move_group, blue_ball_2, previous_ending_joint_angles)
    
    if not blue_ball_2_pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = blue_ball_2_pick_up_pose.joint_trajectory.points[-1].positions
    
    # Place - move gripper to the desired placement position
    place_pose_basket_1 = plan_trajectory(move_group, basket_1, previous_ending_joint_angles)
        
    if not place_pose_basket_1.joint_trajectory.points:
        return response    
    previous_ending_joint_angles = place_pose_basket_1.joint_trajectory.points[-1].positions
    
    ####### Red Ball 1 #########
    #unity_home.position.x += 0.5    # ANTES DE unity_to_ros y=up, x= to me and z= rigth (if we are in the camera of unity) 
    red_ball_1 = unity_to_ros(req.pick_pose_redball_1)
    red_ball_1.position.z += 0.25     # DESPUES DE unity_to_ros y=to inside, x= to right and z= up (if we are in the camera of unity)

    # Move to home position of safety
    red_ball_1_home_pose = plan_trajectory(move_group,red_ball_1, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not red_ball_1_home_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = red_ball_1_home_pose.joint_trajectory.points[-1].positions
    
    # Pre grasp - position gripper directly above target object
    red_ball_1.position.z -= 0.11
    red_ball_1_grasp_pose = plan_trajectory(move_group,red_ball_1, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not red_ball_1_grasp_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = red_ball_1_grasp_pose.joint_trajectory.points[-1].positions
    
    # Pick Up - raise gripper back to thepre-grasp position
    red_ball_1.position.z += 0.2
    red_ball_1_pick_up_pose = plan_trajectory(move_group, red_ball_1, previous_ending_joint_angles)
    
    if not red_ball_1_pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = red_ball_1_pick_up_pose.joint_trajectory.points[-1].positions
    
    # Place - move gripper to the desired placement position
    basket_2 = unity_to_ros(req.Place_basket_2)
    basket_2.position.z += 0.3
    place_pose_basket_2 = plan_trajectory(move_group, basket_2, previous_ending_joint_angles)
        
    if not place_pose_basket_2.joint_trajectory.points:
        return response    
    previous_ending_joint_angles = place_pose_basket_2.joint_trajectory.points[-1].positions

    ####### Red Ball 2 #########
    red_ball_2 = unity_to_ros(req.pick_pose_redball_2)
    red_ball_2.position.z += 0.25     # DESPUES DE unity_to_ros y=to inside, x= to right and z= up (if we are in the camera of unity)

    # Move to home position of safety
    red_ball_2_home_pose = plan_trajectory(move_group,red_ball_2, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not red_ball_2_home_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = red_ball_2_home_pose.joint_trajectory.points[-1].positions
    
    # Pre grasp - position gripper directly above target object
    red_ball_2.position.z -= 0.11
    red_ball_2_grasp_pose = plan_trajectory(move_group,red_ball_2, previous_ending_joint_angles)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not red_ball_2_grasp_pose.joint_trajectory.points:
        return response
    previous_ending_joint_angles = red_ball_2_grasp_pose.joint_trajectory.points[-1].positions
    
    # Pick Up - raise gripper back to thepre-grasp position
    red_ball_2.position.z += 0.2
    red_ball_2_pick_up_pose = plan_trajectory(move_group, red_ball_2, previous_ending_joint_angles)
    
    if not red_ball_2_pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = red_ball_2_pick_up_pose.joint_trajectory.points[-1].positions
    
    # Place - move gripper to the desired placement position
    place_pose_basket_2 = plan_trajectory(move_group, basket_2, previous_ending_joint_angles)
        
    if not place_pose_basket_2.joint_trajectory.points:
        return response    
    previous_ending_joint_angles = place_pose_basket_2.joint_trajectory.points[-1].positions

    # If trajectory planning worked for all pick and place stages, add plan to response
    ####### Blue Ball 1 #########
    response.trajectories.append(home_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose_basket_1)
    ###### Blue Ball 2 #########
    response.trajectories.append(blue_ball_2_home_pose)
    response.trajectories.append(blue_ball_2_grasp_pose)
    response.trajectories.append(blue_ball_2_pick_up_pose)
    response.trajectories.append(place_pose_basket_1)
    ####### Red Ball 1 #########
    response.trajectories.append(red_ball_1_home_pose)
    response.trajectories.append(red_ball_1_grasp_pose)
    response.trajectories.append(red_ball_1_pick_up_pose)
    response.trajectories.append(place_pose_basket_2)
    ####### Red Ball 2 #########
    response.trajectories.append(red_ball_2_home_pose)
    response.trajectories.append(red_ball_2_grasp_pose)
    response.trajectories.append(red_ball_2_pick_up_pose)
    response.trajectories.append(place_pose_basket_2)

    move_group.clear_pose_targets()

    return response

def create_table():
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = -0.75  # above the ur3
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(1.5, 1.5, 1.5))

def create_ball(ball):
    scene = moveit_commander.PlanningSceneInterface()
    ball_pose = geometry_msgs.msg.PoseStamped()
    ball_pose.header.frame_id = "base_link"
    ball_pose.pose.orientation.w = 1.0
    ball_pose.pose.position.x = ball.position.x
    ball_pose.pose.position.y = ball.position.y
    ball_pose.pose.position.z = ball.position.z
    ball_name = "ball_1"
    scene.add_box(ball_name, ball_pose, size=(0.05, 0.05, 0.05))


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_moveit_server')

    s = rospy.Service('ur3_moveit', MoverService, plan_pick_and_place)

    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    create_table()
    moveit_server()
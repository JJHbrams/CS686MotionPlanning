/***************************************************************************************
| * Author: Jinhyeok Jang                                                              |
| * Based on OMPL SST                                                                  |
| * Date : 2019 11 15 Fri                                                              |
| * SGVR lab : https://sgvr.kaist.ac.kr/                                               |
| * E-mail : antion001@gmail.com                                                       |
****************************************************************************************/
/***************************************************************************************
| * Author: Jinhyeok Jang                                                              |
| * Based on OMPL SST                                                                  |
| * Date : 2019 11 15 Fri                                                              |
| * SGVR lab : https://sgvr.kaist.ac.kr/                                               |
| * E-mail : antion001@gmail.com                                                       |
****************************************************************************************/

#define _CRT_SECURE_NO_WARNINGS    // strcat 보안 경고로 인한 컴파일 에러 방지

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <boost/scope_exit.hpp>
#include <boost/date_time.hpp>
#include <cstdlib>

// ompl
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include "ompl/control/PathControl.h"
#include "ompl/geometric/PathGeometric.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/config.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

// uncertain_kinodynamic
#include <uncertain_kinodynamic/StateSpaces.h>
#include <uncertain_kinodynamic/Scene.h>
#include <uncertain_kinodynamic/UncertainKinodynamicPlanner.h>

#define NUM_BASE_DOF 3
#define MAX_COlUMN 1000
#define MAX_ROW 6
#define MAX_WORDS 20

void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result)
{
    const auto *se2state = start->as<ompl::base::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    result->as<ompl::base::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * duration * cos(rot),
        pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
        rot    + ctrl[1] * duration);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "UncertainKinodynamic");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    //// Please set your group in moveit!.
    const std::string PLANNING_GROUP = "vehicle";
    const std::string DISPLAY_GROUP = "vehicle_dis";
    //const std::string PLANNING_GROUP = "husky";
    // const std::string BASE_GROUP = "base";
    // const std::string MANI_COLL_CHECK_GROUP = "without_right_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    //// Please set your start configuration and predefined configuration.
    std::vector<double> s_conf, pre_conf;
    joint_model_group->getVariableDefaultPositions(s_conf);

    joint_model_group->getVariableDefaultPositions(pre_conf);
    pre_conf.erase(pre_conf.begin(), pre_conf.begin()+NUM_BASE_DOF);

    // moveit planning scene
    ScenePtr scene(new Scene(planning_scene, "world"));
    scene->addCollisionObjects();
    scene->updateCollisionScene();

    // state space
    std::string collisionCheckGroup;
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    collisionCheckGroup = PLANNING_GROUP;

    ROS_INFO("num_dof = %d", num_dof);

    // State space, SE(2)
    auto state_space(std::make_shared<ompl::base::SE2StateSpace>());
    // State space bounds
    ompl::base::RealVectorBounds bounds(2);
    // x
    bounds.setLow(0, -10.0);
    bounds.setHigh(0, 10.0);
    // y
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    // yaw
    bounds.setLow(2, -M_PI);
    bounds.setHigh(2, M_PI);
    state_space->setBounds(bounds);

    // Control space
    auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 2));

    // Control space bounds
    ompl::base::RealVectorBounds control_bounds(2);
    control_bounds.setLow(-0.05);
    control_bounds.setHigh(0.05);
    control_bounds.setLow(2,-M_PI);
    control_bounds.setHigh(2, M_PI);
    control_space->setBounds(control_bounds);

    // Simpl setup
    ompl::control::SimpleSetupPtr simple_setup;
    simple_setup.reset(new ompl::control::SimpleSetup(control_space));
    // State propagation routine
    simple_setup->setStatePropagator(propagate);
    // State validity checking
    StateValidityCheckerPtr validity_checker;
    validity_checker.reset(new StateValidityChecker(simple_setup->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
    simple_setup->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

    typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> ScopedState;

    // Start state
    ScopedState Start(state_space);
    Start->setX(4.3);// x
    Start->setY(1.3);// y
    Start->setYaw(0.0);// yaw

    // Goal state
    // ; use the hard way to set the elements
    ScopedState Goal(state_space);
    (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -4.3;
    (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -4.3;
    (*Goal)[1]->as<ompl::base::SO2StateSpace::StateType>()->value            = M_PI;

    // Start and goal states
    simple_setup->setStartAndGoalStates(Start, Goal, 0.05);// setStartAndGoalStates(start, goal, threshold)

    ROS_INFO("Start : %f , %f , %f   Goal : %f , %f , %f", Start[0], Start[1], Start[2], Goal[0], Goal[1], Goal[2]);

    // Planning
    simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::control::UncertainKinodynamicPlanner(simple_setup->getSpaceInformation())));
    simple_setup->solve(ompl::base::timedPlannerTerminationCondition(10.0));

    if (simple_setup->haveSolutionPath()){
      std::cout << "Found solution:" << std::endl;

      // Store path
      ompl::control::PathControl &path = simple_setup->getSolutionPath();
      //path.printAsMatrix(std::cout);

      std::fstream fileout("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path.txt", std::ios::out);
      path.printAsMatrix(fileout);
      fileout.close();

      std::fstream filein("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path.txt", std::ios::in);

      char word;
      char data[MAX_COlUMN][MAX_ROW][MAX_WORDS]={0};
      int i=0, j=0, k;
      //Read text
      while(filein.get(word)){
          //filein.get(word);
          //std::cout << "Word : " << word << std::endl;
          if((word == ' ') || (word == '\n') || (k>=MAX_WORDS)){
            //Next column
            k=0;
            j++;
            if(j>=MAX_ROW){
            //Next row
                j=0;
                i++;
            }
          }
          else{
            data[i][j][k] = word;
            k++;
          }
      }
      filein.close();

      ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      ros::Publisher point_pub = node_handle.advertise<visualization_msgs::Marker>("StartGoalPoints",0);
      ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path_vis",1);
      //Robot trajectory
      moveit_msgs::DisplayTrajectory display_trajectory;
      moveit_msgs::RobotTrajectory robot_traj;
      //Path visualize
      nav_msgs::Path path_vis;
      geometry_msgs::PoseStamped pose;
      path_vis.poses.clear();
      path_vis.header.stamp = ros::Time::now();
      path_vis.header.frame_id = "world";

      const moveit::core::JointModelGroup* model_group = planning_scene->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
      const std::vector<std::string>& active_joint_names = model_group->getActiveJointModelNames();
      uint NoPathPoints = path.getStateCount();//State
      //uint NoPathPoints = path.getControlCount();//Control
      robot_traj.joint_trajectory.joint_names = active_joint_names;
      robot_traj.joint_trajectory.points.resize(NoPathPoints);
/*
      //TEST
      for(int i=0; i<NoPathPoints; i++){
          std::cout << "Data["<<i<<"] : ";
          for(int j=0; j<MAX_ROW; j++){
              double num = std::stod(static_cast<const std::string>(data[i][j]));
              std::cout<< num <<" ";
          }
          std::cout << std::endl;
      }
      //std::string str = data[0][0];
      //std::cout<<"data[0][0] : "<<str<<std::endl;
      //std::cout<<"data[0][0] : "<< std::stod(std::string(data[0][0])) <<std::endl;
*/
      ROS_INFO("\nNo. of States = %d\n", NoPathPoints);
      //ROS_INFO("\nNo. of Controls = %d\n", NoPathPoints);

      //States
      typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
      typedef ompl::control::RealVectorControlSpace::ControlType* ControlTypePtr;

      for(uint i = 0; i < NoPathPoints; i++){
          //StateTypePtr rstate = static_cast<StateTypePtr>(path.getState(i));
          robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
          for (uint j = 0; j < num_dof; j++){
              double traj_value = std::stod(static_cast<const std::string>(data[i][j]));
              //robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
              robot_traj.joint_trajectory.points[i].positions[j] = traj_value;
          }
          pose.pose.position.x = robot_traj.joint_trajectory.points[i].positions[0];
          pose.pose.position.y = robot_traj.joint_trajectory.points[i].positions[1];
          path_vis.poses.push_back(pose);
          /*
          std::cout << "Points [x]:" << robot_traj.joint_trajectory.points[i].positions[0];
          std::cout << "  Points [y]:" << robot_traj.joint_trajectory.points[i].positions[1];
          std::cout << "  Points [yaw]:" << robot_traj.joint_trajectory.points[i].positions[2];
          std::cout << std::endl;
           */

          robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
      }
/*
      //Controls
      for(uint i = 0; i < NoPathPoints; i++){
          ControlTypePtr rcontrol = static_cast<ControlTypePtr>(path.getControl(i));
          robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
          for (uint j = 0; j < num_dof; j++){
              robot_traj.joint_trajectory.points[i].positions[j] = rcontrol->values[j];
          }
          std::cout << "Points [dx]:" << robot_traj.joint_trajectory.points[i].positions[0];
          std::cout << "  Points [dy]:" << robot_traj.joint_trajectory.points[i].positions[1];
          std::cout << "  Points [dyaw]:" << robot_traj.joint_trajectory.points[i].positions[2];
          //std::cout << "  Points [dx]:" << robot_traj.joint_trajectory.points[i].positions[3];
          //std::cout << "  Points [dy]:" << robot_traj.joint_trajectory.points[i].positions[4];
          //std::cout << "  Points [dyaw]:" << robot_traj.joint_trajectory.points[i].positions[5];
          std::cout << std::endl;

          robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
      }
*/
      display_trajectory.trajectory.push_back(robot_traj);
      display_pub.publish(display_trajectory);

      while(ros::ok()){
        // Start, Goal marker
        visualization_msgs::Marker points;
        geometry_msgs::Point pt;
        points.header.frame_id ="world";
        points.header.stamp= ros::Time();

        points.ns="Start_Pt";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.3;
        points.scale.y = 0.3;
        points.color.a = 1.0; // Don't forget to set the alpha!
        points.color.r = 0.0f;
        points.color.g = 1.0f;
        points.color.b = 0.0f;
        pt.x = Start[0];
        pt.y = Start[1];
        points.points.push_back(pt);
        point_pub.publish(points);//Pulish Start

        points.points.clear();
        points.ns="Goal_Pt";
        points.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.3;
        points.scale.y = 0.3;
        points.color.r = 1.0f;
        points.color.b = 0.0f;
        points.color.g = 0.0f;
        pt.x = Goal[0];
        pt.y = Goal[1];
        points.points.push_back(pt);
        point_pub.publish(points);// Publish Goal

        //Path visulaization
        path_pub.publish(path_vis);
      }

      ros::Duration(1.0).sleep();
    }

    else
      std::cout << "No solution found" << std::endl;

    return 0;
}

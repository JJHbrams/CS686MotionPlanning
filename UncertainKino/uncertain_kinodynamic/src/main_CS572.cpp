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
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalStates.h>

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
#define MAX_COLUMN 1000
#define MAX_ROW 4
#define MAX_WORDS 20

int main(int argc, char** argv) {
    ros::init(argc, argv, "UncertainKinodynamic");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    //// Please set your group in moveit!.
    const std::string PLANNING_GROUP = "vehicle";
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
    s_conf[0] = 5.0;
    s_conf[1] = 5.0;
    s_conf[2] = -M_PI/2;

    joint_model_group->getVariableDefaultPositions(pre_conf);
    pre_conf.erase(pre_conf.begin(), pre_conf.begin()+NUM_BASE_DOF);

    // moveit planning scene
    ScenePtr scene(new Scene(planning_scene, "world"));
    scene->addCollisionObjects();
    scene->updateCollisionScene();

    // state space
    ompl::base::StateSpacePtr state_space;
    std::string collisionCheckGroup;
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    collisionCheckGroup = PLANNING_GROUP;

    ompl::base::RealVectorBounds bounds(num_dof);

    ROS_INFO("num_dof = %d", num_dof);

    for (std::size_t i = 0; i < joint_bounds.size(); ++i) {
        const robot_model::JointModel::Bounds& b = *joint_bounds[i];
        bounds.setLow(i, b[0].min_position_);
        bounds.setHigh(i, b[0].max_position_);
    }

    //// Please set your boundaries for base space.
    // x
    bounds.setLow(0, -10.0);
    bounds.setHigh(0, 10.0);
    // y
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    // yaw
    bounds.setLow(2, -M_PI);
    bounds.setHigh(2, M_PI);

    //// ompl
    state_space.reset(new ompl::base::RealVectorStateSpace(num_dof));
    state_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    state_space->setup();

    // simple setup
    ompl::geometric::SimpleSetupPtr simple_setup;
    simple_setup.reset(new ompl::geometric::SimpleSetup(state_space));

    // state validity checker
    StateValidityCheckerPtr validity_checker;
    validity_checker.reset(new StateValidityChecker(simple_setup->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
    simple_setup->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    // set initial state
    ScopedState q_start(state_space);
    q_start[0] = 4.3;
    q_start[1] = 4.3;
    q_start[2] = 0.0;
    simple_setup->addStartState(q_start);

    // set goal state
    ScopedState q_goal(state_space);
    q_goal[0] = -3.3;
    q_goal[1] = -3.3;
    q_goal[2] = -M_PI;

    ROS_INFO("Start : %f , %f , %f   Goal : %f , %f , %f", q_start[0], q_start[1], q_start[2], q_goal[0], q_goal[1], q_goal[2]);

    simple_setup->setGoalState(q_goal);
    simple_setup->setStartAndGoalStates(q_start, q_goal);

    // set planner
    simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::geometric::UncertainKinodynamicPlanner(simple_setup->getSpaceInformation())));
    simple_setup->solve(ompl::base::timedPlannerTerminationCondition(10.0));

    if (simple_setup->haveSolutionPath()) {
//        simple_setup->simplifySolution();
        ompl::geometric::PathGeometric &p = simple_setup->getSolutionPath();
//        simple_setup->getPathSimplifier()->simplifyMax(p);
        simple_setup->getPathSimplifier()->smoothBSpline(p);

        //Save path information as text
        std::fstream fileout("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path_geo.txt", std::ios::out);
        p.printAsMatrix(fileout);
        //p.printAsMatrix(std::cout);
        fileout.close();

        ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        //ros::Publisher display_pub2 = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_actual_path", 1, true);
        ros::Publisher point_pub = node_handle.advertise<visualization_msgs::Marker>("StartGoalPoints",0);
        ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path_vis",1);
        ros::Publisher path_pub2 = node_handle.advertise<nav_msgs::Path>("/actual_vis",1);

        //Robot messeges
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::DisplayTrajectory display_actual;
        moveit_msgs::RobotTrajectory robot_traj;
        moveit_msgs::RobotTrajectory Actual_traj;

        //Path visualize
        nav_msgs::Path path_vis;
        nav_msgs::Path path_vis2;
        geometry_msgs::PoseStamped pose;
        path_vis.poses.clear();
        path_vis.header.stamp = ros::Time::now();
        path_vis.header.frame_id = "world";

        const moveit::core::JointModelGroup* model_group = planning_scene->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
        const std::vector<std::string>& active_joint_names = model_group->getActiveJointModelNames();
        uint NoPathPoints = p.getStateCount();

        robot_traj.joint_trajectory.joint_names = active_joint_names;
        robot_traj.joint_trajectory.points.resize(p.getStateCount());
        Actual_traj.joint_trajectory.joint_names = active_joint_names;
        Actual_traj.joint_trajectory.points.resize(p.getStateCount());

        ROS_INFO("No. of Waypoints = %d", p.getStateCount());

        typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
        for(uint i = 0; i < p.getStateCount(); i++){
          // Primitive path
          StateTypePtr rstate = static_cast<StateTypePtr>(p.getState(i));
          robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
          for (uint j = 0; j < num_dof; j++){
              robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
          }
          pose.pose.position.x = robot_traj.joint_trajectory.points[i].positions[0];
          pose.pose.position.y = robot_traj.joint_trajectory.points[i].positions[1];
          path_vis.poses.push_back(pose);

          robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        display_trajectory.trajectory.push_back(robot_traj);
        display_pub.publish(display_trajectory);



        char word;
        char data[MAX_COLUMN][MAX_ROW][MAX_WORDS]={0};
        int i=0, j=0, k;
        //Read text
        std::fstream filein("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path_geo0.txt", std::ios::in);//should be control output
        while(filein.get(word)){
            //filein.get(word);
            if((word == ' ') || (word == '\n') || (k>=MAX_WORDS)){
              //Next column       Next row           Next value
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
        std::cout << std::stod(static_cast<const std::string>(data[0][0])) << std::endl;
/*
        for(int col=0; col<NoPathPoints; col++){
            std::cout << "Data["<<col<<"] : ";
            for(int row=0; row<(MAX_ROW-1); row++){
                double num = std::stod(static_cast<const std::string>(data[col][row]));
                std::cout<< num <<" ";
            }
            std::cout << std::endl;
        }
*/

        path_vis2.poses.clear();
        path_vis2.header.stamp = ros::Time::now();
        path_vis2.header.frame_id = "world";

        display_trajectory.trajectory.clear();

        //Text to path
        for(uint i = 0; i < p.getStateCount(); i++){
          // Corrected path
          Actual_traj.joint_trajectory.points[i].positions.resize(num_dof);
          for (uint j = 0; j < num_dof; j++){
              Actual_traj.joint_trajectory.points[i].positions[j] = std::stod(static_cast<const std::string>(data[i][j]));
          }
          pose.pose.position.x = Actual_traj.joint_trajectory.points[i].positions[0];
          pose.pose.position.y = Actual_traj.joint_trajectory.points[i].positions[1];
          path_vis2.poses.push_back(pose);

          //Actual_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        display_trajectory.trajectory.push_back(Actual_traj);
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
          pt.x = q_start[0];
          pt.y = q_start[1];
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
          pt.x = q_goal[0];
          pt.y = q_goal[1];
          points.points.push_back(pt);
          point_pub.publish(points);// Publish Goal

          //Path visulaization
          path_pub.publish(path_vis);
          path_pub2.publish(path_vis2);
        }

        ros::Duration(1.0).sleep();
    }


    return 0;
}

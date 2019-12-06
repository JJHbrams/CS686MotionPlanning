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
#include<random>
#include <stdio.h>
#include <string.h>
#include <algorithm>
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
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
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
#include <uncertain_kinodynamic/UncertainKinodynamicPlanner_RRTstar.h>

// Vehicle degree of freedom (x,y,yaw)
#define NUM_BASE_DOF 3
// Local planner's path data : Control path는 Geometry path처럼 path.getState() 가 먹히지 않아서 text로 저장한 후 읽어오는 방식으로 진행합니다.
#define MAX_COlUMN 1000// Maximum number of local planner's path points
#define MAX_ROW 6// Number of each path point's elements, (x,y,yaw,dx,dy,d_yaw)
#define MAX_WORDS 20// Each data's length, ex)x=4.33221...2 : Total 20 words
// Planner setting
#define G_PLANTIME  5.0      // Global planner planning time(sec)
#define L_PLANTIME  5.0      // Local planner planning time(sec)
// Local planner's local search setting
#define SAMPLING_R  5      //해당 범위 이내의 waypoint sampling(현재 scene과 설정 상 Global planner의 전체 point수가 대체로 97~129 정도)
#define S_RADIUS    5       //Local planner 탐색 범위 (속도 bound에 따라 적절한 웨이포인트 샘플링 거리 및 로컬 플래닝 범위값이 있는듯)

void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result){
    // Propagation function local planner
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
    ros::NodeHandle node_handle("~");// ROS메시지 담을 변

    //// Please set your group in moveit!. (Calling configuration from "vehicle" folder)
    const std::string PLANNING_GROUP = "vehicle";
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

    // state space, check if given state is validate for planning
    std::string collisionCheckGroup;
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    collisionCheckGroup = PLANNING_GROUP;

    ROS_INFO("num_dof = %d", num_dof);

    /*************************************************** Geometry based planner ******************************************************/
    // RRT star
    ompl::base::StateSpacePtr state_space_geo;//Geometric planning space, configuration : x,y,yaw
    ompl::base::RealVectorBounds bounds(num_dof);//Geometric planning boundary, upper and lower for each configuration

    ROS_INFO("num_dof = %d", num_dof);

    for (std::size_t i = 0; i < joint_bounds.size(); ++i) {
        const robot_model::JointModel::Bounds& b = *joint_bounds[i];
        bounds.setLow(i, b[0].min_position_);
        bounds.setHigh(i, b[0].max_position_);
    }

    //// Please set your boundaries for base space.
    // x boundary
    bounds.setLow(0, -10.0);
    bounds.setHigh(0, 10.0);
    // y boundary
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    // yaw boundary
    bounds.setLow(2, -M_PI);
    bounds.setHigh(2, M_PI);

    //// ompl
    state_space_geo.reset(new ompl::base::RealVectorStateSpace(num_dof));
    state_space_geo->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    state_space_geo->setup();

    // simple setup - help us making the planning environment easy
    ompl::geometric::SimpleSetupPtr simple_setup_geo;
    simple_setup_geo.reset(new ompl::geometric::SimpleSetup(state_space_geo));

    // state validity checker
    StateValidityCheckerPtr validity_checker_geo;
    validity_checker_geo.reset(new StateValidityChecker(simple_setup_geo->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
    simple_setup_geo->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker_geo));

    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    // set initial state
    ScopedState q_start(state_space_geo);
    q_start[0] = 4.3;//x_init
    q_start[1] = 4.3;//y_init
    q_start[2] = -M_PI/2;//yaw_init
    simple_setup_geo->addStartState(q_start);

    // set goal state
    ScopedState q_goal(state_space_geo);
    q_goal[0] = -3.3;//x_goal
    q_goal[1] = -3.3;//y_goal
    q_goal[2] = -M_PI;//yaw_goal
    simple_setup_geo->setGoalState(q_goal);
    simple_setup_geo->setStartAndGoalStates(q_start, q_goal);
    ROS_INFO("Start : %f , %f , %f   \nGoal : %f , %f , %f", q_start[0], q_start[1], q_start[2], q_goal[0], q_goal[1], q_goal[2]);

    // set planner - Run RRT* for given time, G_PLANTIME
    simple_setup_geo->setPlanner(ompl::base::PlannerPtr(new ompl::geometric::UncertainKinodynamicPlanner_RRTstar(simple_setup_geo->getSpaceInformation())));
    simple_setup_geo->solve(ompl::base::timedPlannerTerminationCondition(G_PLANTIME));

    if (simple_setup_geo->haveSolutionPath()) {
//        simple_setup->simplifySolution();
        ompl::geometric::PathGeometric &p = simple_setup_geo->getSolutionPath();//Found path's reference, p
//        simple_setup->getPathSimplifier()->simplifyMax(p);
        simple_setup_geo->getPathSimplifier()->smoothBSpline(p);//Apply B-spline for smoothing global path

        //Save path information as text
        std::fstream fileout("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path_geo.txt", std::ios::out);//Export geometric path
        p.printAsMatrix(fileout);
        //p.printAsMatrix(std::cout);
        fileout.close();

        // ROS messege publishers
        ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        ros::Publisher display_pub2 = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_local_path", 1, true);
        ros::Publisher point_pub = node_handle.advertise<visualization_msgs::Marker>("StartGoalPoints",0);
        ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path_vis",1);
        ros::Publisher path_pub2 = node_handle.advertise<nav_msgs::Path>("/local_vis",1);

        //Robot messeges
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::DisplayTrajectory display_trajectory2;
        moveit_msgs::RobotTrajectory robot_traj;
        moveit_msgs::RobotTrajectory robot_traj2;

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

        ROS_INFO("No. of Waypoints = %d", NoPathPoints);

        // Hand over path data to the visualizer
        typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
        for(uint i = 0; i < NoPathPoints; i++){
          // Primitive path
          StateTypePtr rstate = dynamic_cast<StateTypePtr>(p.getState(i));
          robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
          for (uint j = 0; j < num_dof; j++){
              robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
          }
          pose.pose.position.x = robot_traj.joint_trajectory.points[i].positions[0];
          pose.pose.position.y = robot_traj.joint_trajectory.points[i].positions[1];
          path_vis.poses.push_back(pose);

          robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        //display_trajectory.trajectory.push_back(robot_traj);
        //display_pub.publish(display_trajectory);


        /*************************************************** Control based planner ******************************************************/
        // SST, unlike global planner this planner worls on "Control space"
        // Uses global path's points as a temporal goal point - waypoint
        path_vis2.poses.clear();
        path_vis2.header.stamp = ros::Time::now();
        path_vis2.header.frame_id = "world";

        display_trajectory.trajectory.clear();

        #define MAX_WPT_IDX (NoPathPoints-1)
        uint wpt_idx = 0;// index of waypoint for local planner
        uint wpt_idx_next = 0;

        //Generate random device
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(0.0, SAMPLING_R);

        uint NoPathPoints2 = 0;
        double local_s[3], local_g[3];

        //Initial start and goal point
        local_s[0] = robot_traj.joint_trajectory.points[0].positions[0];
        local_s[1] = robot_traj.joint_trajectory.points[0].positions[1];
        local_s[2] = robot_traj.joint_trajectory.points[0].positions[2];
        //determine First waypoint
        wpt_idx_next = wpt_idx + dist(mt);
        local_g[0] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[0];
        local_g[1] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[1];
        local_g[2] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[2];

        while(wpt_idx<MAX_WPT_IDX){
            // Iterative planning until the vehicle reaches to the goal
            wpt_idx_next = wpt_idx + dist(mt);//Update waypoint index
            while(wpt_idx_next == wpt_idx || wpt_idx_next > MAX_WPT_IDX)    wpt_idx_next = wpt_idx + dist(mt);//In case wpt index doesn't change or exceeds maximum index
            std::cout << "Wpt index : " << wpt_idx << ", Wpt index next : " << wpt_idx_next << ", Wpt index MAX : " << NoPathPoints << std::endl;

            // State space, SE(2)
            auto state_space(std::make_shared<ompl::base::SE2StateSpace>());
            // State space bounds
            ompl::base::RealVectorBounds bounds(2);//Local planning boundary
            // x
            bounds.setLow(0, std::min(local_g[0]-S_RADIUS,local_s[0]-S_RADIUS));
            bounds.setHigh(0, std::max(local_g[0]+S_RADIUS,local_s[0]+S_RADIUS));
            // y
            bounds.setLow(1, std::min(local_g[1]-S_RADIUS,local_s[1]-S_RADIUS));
            bounds.setHigh(1, std::max(local_g[1]+S_RADIUS,local_s[1]+S_RADIUS));
            // yaw
            bounds.setLow(2, -M_PI);
            bounds.setHigh(2, M_PI);
            state_space->setBounds(bounds);

            // Control space
            auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 2));

            // Control space bounds
            ompl::base::RealVectorBounds control_bounds(2);//Speed boundary
            control_bounds.setLow(-0.07);//vx
            control_bounds.setHigh(0.07);//vy
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

            typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> ScopedSE2State;

            // Start state
            ScopedSE2State Start(state_space);
            Start->setX(local_s[0]);// x
            Start->setY(local_s[1]);// y
            Start->setYaw(local_s[2]);// yaw

            // Goal state
            // ; use the hard way to set the elements
            ScopedSE2State Goal(state_space);
            (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = local_g[0];
            (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = local_g[1];
            (*Goal)[1]->as<ompl::base::SO2StateSpace::StateType>()->value            = local_g[2];

            // Start and goal states
            simple_setup->setStartAndGoalStates(Start, Goal, 0.05);// setStartAndGoalStates(start, goal, threshold)

            ROS_INFO("Local Start : %f , %f , %f   \nLocal Goal : %f , %f , %f", Start[0], Start[1], Start[2], Goal[0], Goal[1], Goal[2]);

            // Planning
            simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::control::UncertainKinodynamicPlanner(simple_setup->getSpaceInformation())));
            simple_setup->solve(ompl::base::timedPlannerTerminationCondition(L_PLANTIME));

            if (simple_setup->haveSolutionPath()){
              std::cout << "Found solution:" << std::endl;

              // Store path
              ompl::control::PathControl &path = simple_setup->getSolutionPath();
              //path.printAsMatrix(std::cout);

              std::fstream fileout("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path.txt", std::ios::out);
              path.printAsMatrix(fileout);
              fileout.close();

              // Import local planner's path from text
              std::fstream filein("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path.txt", std::ios::in);

              char word;
              char data[MAX_COlUMN][MAX_ROW][MAX_WORDS]={0};
              int i=0, j=0, k;
              //Read text
              while(filein.get(word)){
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

              NoPathPoints2 = path.getStateCount();//State
              ROS_INFO("\nNo. of States2 = %d\n", NoPathPoints2);

              robot_traj2.joint_trajectory.joint_names = active_joint_names;
              robot_traj2.joint_trajectory.points.resize(path.getStateCount());

              // Hand over local path's point to the visualizer
              for(uint i = 0; i < NoPathPoints2; i++){
                  robot_traj2.joint_trajectory.points[i].positions.resize(num_dof);
                  for (uint j = 0; j < num_dof; j++){
                      double traj_value = std::stod(static_cast<const std::string>(data[i][j]));
                      //std::cout<<traj_value<<std::endl;
                      robot_traj2.joint_trajectory.points[i].positions[j] = traj_value;
                  }
                  pose.pose.position.x = robot_traj2.joint_trajectory.points[i].positions[0];
                  pose.pose.position.y = robot_traj2.joint_trajectory.points[i].positions[1];
                  path_vis2.poses.push_back(pose);

                  robot_traj2.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
              }
              display_trajectory.trajectory.push_back(robot_traj);
              display_pub.publish(display_trajectory);

              display_trajectory2.trajectory.push_back(robot_traj2);
              display_pub2.publish(display_trajectory2);

              wpt_idx = wpt_idx_next;//Store next waypoint

              // Local start : Previous local planner's last position
              local_s[0] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[0];
              local_s[1] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[1];
              local_s[2] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[2];
            }
            else{
              std::cout << "No local solution found" << std::endl;
              // Local start : Previous local planner's last position
              if(wpt_idx == 0){
                  // Couldn't find path at the first time
                  local_s[0] = robot_traj.joint_trajectory.points[0].positions[0];
                  local_s[1] = robot_traj.joint_trajectory.points[0].positions[1];
                  local_s[2] = robot_traj.joint_trajectory.points[0].positions[2];
              }
              else{
                  local_s[0] = robot_traj2.joint_trajectory.points[wpt_idx].positions[0];
                  local_s[1] = robot_traj2.joint_trajectory.points[wpt_idx].positions[1];
                  local_s[2] = robot_traj2.joint_trajectory.points[wpt_idx].positions[2];
              }

            }

            // Next planning's start and goal setting
            // Local goal : One of the global planner's point (in a restricted area, random)
            local_g[0] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[0];
            local_g[1] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[1];
            local_g[2] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[2];


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
            points.ns="local Goal";
            points.id = 1;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.3;
            points.scale.y = 0.3;
            points.color.r = 1.0f;
            points.color.b = 0.0f;
            points.color.g = 1.0f;
            pt.x = local_g[0];
            pt.y = local_g[1];
            points.points.push_back(pt);
            point_pub.publish(points);// Publish localGoal

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
        std::cout << "Done!" << std::endl;

        ros::Duration(1.0).sleep();

    }

    return 0;
}

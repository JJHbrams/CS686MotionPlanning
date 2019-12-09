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
#include <math.h>
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


#include <cmath>
#include <vector>
using namespace std;


// Vehicle degree of freedom (x,y,yaw)
#define NUM_BASE_DOF 3
// Local planner's path data : Control path는 Geometry path처럼 path.getState() 가 먹히지 않아서 text로 저장한 후 읽어오는 방식으로 진행합니다.
#define MAX_COlUMN 1000// Maximum number of local planner's path points
#define MAX_ROW 6// Number of each path point's elements, (x,y,yaw,dx,dy,d_yaw)
#define MAX_WORDS 20// Each data's length, ex)x=4.33221...2 : Total 20 words
// Planner setting
#define G_PLANTIME  10.0      // Global planner planning time(sec)
#define L_PLANTIME  1.0      // Local planner planning time(sec)
// Local planner's local search setting
#define DIST_DENOM  0.08
#define SAMPLING_R  20//해당 범위 이내의 waypoint sampling
#define S_RADIUS    5       //Local planner 탐색 범위 (속도 bound에 따라 적절한 웨이포인트 샘플링 거리 및 로컬 플래닝 범위값이 있는듯)

#define thres_Pearson 0.99 // Allowed Pearson value

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

///*

vector<double> operator-(vector<double> a, double b)
{
    vector<double> retvect;
    for (int i = 0; i < a.size(); i++)
    {
        retvect.push_back(a[i] - b);
    }
    return retvect;
}

vector<double> operator*(vector<double> a, vector<double> b)
{
    vector<double> retvect;
    for (int i = 0; i < a.size() ; i++)
    {
        retvect.push_back(a[i] * b[i]);
    }
    return retvect;
}
double suma(vector<double> a)
{
    double s = 0;
    for (int i = 0; i < a.size(); i++)
    {
        s = s + a[i];
    }
    return s;
}

double mean(vector<double> a)
{
    return suma(a) / a.size();
}



double sqsum(vector<double> a)
{
    double s = 0;
    for (int i = 0; i < a.size(); i++)
    {
        s = s + pow(a[i], 2);
    }
    return s;
}

double stdev(vector<double> nums)
{
    double N = nums.size();
    return pow(sqsum(nums) / N - pow(suma(nums) / N, 2), 0.5);
}



double pearsoncoeff(vector<double> X, vector<double> Y)
{
    return suma((X - mean(X))*(Y - mean(Y))) / (X.size()*stdev(X)* stdev(Y));
}
//*/

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

    /*************************************************** Geometry based planner ******************************************************/
    // RRT star
    ompl::base::StateSpacePtr state_space_geo;
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
    state_space_geo.reset(new ompl::base::RealVectorStateSpace(num_dof));
    state_space_geo->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    state_space_geo->setup();

    // simple setup
    ompl::geometric::SimpleSetupPtr simple_setup_geo;
    simple_setup_geo.reset(new ompl::geometric::SimpleSetup(state_space_geo));

    // state validity checker
    StateValidityCheckerPtr validity_checker_geo;
    validity_checker_geo.reset(new StateValidityChecker(simple_setup_geo->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
    simple_setup_geo->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker_geo));

    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    // set initial state
    ScopedState q_start(state_space_geo);
    q_start[0] = 4.3;
    q_start[1] = 4.3;
    q_start[2] = -M_PI/2;
    simple_setup_geo->addStartState(q_start);

    // set goal state
    ScopedState q_goal(state_space_geo);
    q_goal[0] = -6.3;
    q_goal[1] = -7.3;
    q_goal[2] = -M_PI/2;
    simple_setup_geo->setGoalState(q_goal);
    simple_setup_geo->setStartAndGoalStates(q_start, q_goal);
    ROS_INFO("Start : %f , %f , %f   \nGoal : %f , %f , %f", q_start[0], q_start[1], q_start[2], q_goal[0], q_goal[1], q_goal[2]);

    // set planner
    ros::Time Start_time = ros::Time::now();
    simple_setup_geo->setPlanner(ompl::base::PlannerPtr(new ompl::geometric::UncertainKinodynamicPlanner_RRTstar(simple_setup_geo->getSpaceInformation())));
    simple_setup_geo->solve(ompl::base::timedPlannerTerminationCondition(G_PLANTIME));

    if (simple_setup_geo->haveSolutionPath()) {
//        simple_setup->simplifySolution();
        ompl::geometric::PathGeometric &p = simple_setup_geo->getSolutionPath();
//        simple_setup->getPathSimplifier()->simplifyMax(p);
//        simple_setup_geo->getPathSimplifier()->smoothBSpline(p);
        uint NoPathPoints = p.getStateCount();
        ROS_INFO("No. of Waypoints = %d", NoPathPoints);

        //Save leadpath information as text
        std::fstream fileout("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path_geo.txt", std::ios::out);
        p.printAsMatrix(fileout);
        //p.printAsMatrix(std::cout);
        fileout.close();

        //Read leadpath and divide it into constatnt distance
        std::fstream filein("/home/mrjohd/MotionPlanning_ws/src/UncertainKino/uncertain_kinodynamic/path_geo.txt", std::ios::in);

        char word_geo;
        char data_geo[MAX_COlUMN][4][MAX_WORDS]={0};
        int i=0, j=0, k;
        //Read text
        while(filein.get(word_geo)){
            //filein.get(word);
            if((word_geo == ' ') || (word_geo == '\n') || (k>=MAX_WORDS)){
                //Next column       Next row           Next value
                k=0;
                j++;
                if(j>=4){
                    //Next row
                    j=0;
                    i++;
                }
            }
            else{
                data_geo[i][j][k] = word_geo;
                k++;
            }
        }
        filein.close();

        double x,y,yaw,xn,yn,yawn,dx,dy,dyaw;
        double dist=0, head=0;
        std::vector<std::vector<double> > wpt_data;
        int NoNewPathPoints=0;
        for(uint i = 0; i < NoPathPoints-1; i++){
            x = std::stod(static_cast<const std::string>(data_geo[i][0]));
            y = std::stod(static_cast<const std::string>(data_geo[i][1]));
            yaw = std::stod(static_cast<const std::string>(data_geo[i][2]));

            xn = std::stod(static_cast<const std::string>(data_geo[i+1][0]));
            yn = std::stod(static_cast<const std::string>(data_geo[i+1][1]));
            yawn = std::stod(static_cast<const std::string>(data_geo[i+1][2]));

            dx = xn-x;
            dy = yn-y;
            dyaw = yawn-yaw;

            dist = sqrt(pow(dx,2)+pow(dy,2));
            //std::cout<<"Dist : "<<dist<<std::endl;
            int MAX_local_wpt_idx = dist/DIST_DENOM;
            //std::cout<<"MAX_local_wpt_idx : "<<MAX_local_wpt_idx<<std::endl;
            for(uint k=0;k<MAX_local_wpt_idx;k++){
                std::vector<double> temp;
                temp.push_back(x+k*(dx/MAX_local_wpt_idx));
                temp.push_back(y+k*(dy/MAX_local_wpt_idx));
                temp.push_back(yaw+k*(dyaw/MAX_local_wpt_idx));
                //std::cout<<"temp[0] : "<<temp[0]<<"     temp[1] : "<<temp[1]<<std::endl;
                wpt_data.push_back(temp);
                //std::cout<<"wpt_data : "<<wpt_data[NoNewPathPoints][0]<<", "<<wpt_data[NoNewPathPoints][1]<<", "<<wpt_data[NoNewPathPoints][2]<<std::endl;
                NoNewPathPoints++;
            }
        }
        std::cout<<"No. of New Waypoints = "<< NoNewPathPoints << std::endl;

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

        robot_traj.joint_trajectory.joint_names = active_joint_names;
        //robot_traj.joint_trajectory.points.resize(p.getStateCount());
        robot_traj.joint_trajectory.points.resize(NoNewPathPoints);

        typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
        for(uint i = 0; i < NoNewPathPoints; i++){
            // Primitive path
            StateTypePtr rstate = static_cast<StateTypePtr>(p.getState(i));
            robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
            for (uint j = 0; j < num_dof; j++){
                //robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
                robot_traj.joint_trajectory.points[i].positions[j] = wpt_data[i][j];
            }
            pose.pose.position.x = robot_traj.joint_trajectory.points[i].positions[0];
            pose.pose.position.y = robot_traj.joint_trajectory.points[i].positions[1];
            path_vis.poses.push_back(pose);

            robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        //display_trajectory.trajectory.push_back(robot_traj);
        //display_pub.publish(display_trajectory);


        /*************************************************** Control based planner ******************************************************/
        // SST
        path_vis2.poses.clear();
        path_vis2.header.stamp = ros::Time::now();
        path_vis2.header.frame_id = "world";

        display_trajectory.trajectory.clear();

        uint wpt_idx = 0;// index of waypoint for local planner
        uint wpt_idx_pre = 0;
        uint wpt_idx_next = 0;

#define MAX_WPT_IDX (NoNewPathPoints-1)
        uint NoPathPoints2;
        double local_s[3], local_g[3];
        //Initial start and goal point
        vector<double> points_x;
        vector<double> points_y;
        double pearson_value;
        int num_wpt;

        for(num_wpt = 1; num_wpt <= SAMPLING_R; num_wpt++){
            points_x.clear();
            points_y.clear();
            for(int ind = wpt_idx; ind <= wpt_idx + num_wpt; ind++){
                points_x.push_back(robot_traj.joint_trajectory.points[ind].positions[0]);
                points_y.push_back(robot_traj.joint_trajectory.points[ind].positions[1]);
            }
            pearson_value = pearsoncoeff(points_x,points_y);
            if(pearson_value < thres_Pearson)
                break;
        }

        std::cout<<"pearson_value "<< pearson_value << std::endl;

        local_s[0] = robot_traj.joint_trajectory.points[wpt_idx].positions[0];
        local_s[1] = robot_traj.joint_trajectory.points[wpt_idx].positions[1];
        local_s[2] = robot_traj.joint_trajectory.points[wpt_idx].positions[2];
        wpt_idx_next = wpt_idx + num_wpt;
        local_g[0] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[0];
        local_g[1] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[1];
        local_g[2] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[2];



        while(wpt_idx<=MAX_WPT_IDX){
            std::cout << "\nStart Local planning" << std::endl;
            std::cout << "Wpt index pre: " << wpt_idx_pre << "  Wpt index : " << wpt_idx << ", Wpt index MAX : " << MAX_WPT_IDX << std::endl;

            // State space, SE(2)
            auto state_space(std::make_shared<ompl::base::SE2StateSpace>());
            // State space bounds
            ompl::base::RealVectorBounds bounds(2);
            // x
            bounds.setLow(0, std::min(local_g[0]-S_RADIUS, local_s[0]-S_RADIUS));
            bounds.setHigh(0, std::max(local_g[0]+S_RADIUS, local_s[0]+S_RADIUS));
            // y
            bounds.setLow(1, std::min(local_g[1]-S_RADIUS, local_s[1]-S_RADIUS));
            bounds.setHigh(1, std::max(local_g[1]+S_RADIUS, local_s[1]+S_RADIUS));
            // yaw
            bounds.setLow(2, -M_PI);
            bounds.setHigh(2, M_PI);
            state_space->setBounds(bounds);

            // Control space
            auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 2));

            // Control space bounds
            ompl::base::RealVectorBounds control_bounds(2);
            control_bounds.setLow(-0.065);
            control_bounds.setHigh(0.065);
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

                //std::cout << "Wpt index : " << wpt_idx << " wpt_idx + SAMPLING_R : " << wpt_idx + SAMPLING_R << std::endl;
                //for(num_wpt = 1; num_wpt <= SAMPLING_R; num_wpt++){

                num_wpt = 1;
                int wpt_inx_temp = wpt_idx_pre + num_wpt;
                while(num_wpt <= SAMPLING_R){
                    points_x.clear();
                    points_y.clear();

                    for(int ind = wpt_idx_pre; ind <= wpt_inx_temp; ind++){
                        points_x.push_back(robot_traj.joint_trajectory.points[ind].positions[0]);
                        points_y.push_back(robot_traj.joint_trajectory.points[ind].positions[1]);
                    }
                    pearson_value = pearsoncoeff(points_x,points_y);
                    if(pearson_value < thres_Pearson)
                        break;
                    num_wpt++;
                    wpt_inx_temp = wpt_idx_pre + num_wpt;
                    if(wpt_inx_temp > MAX_WPT_IDX)  break;
                }
                //std::cout << "Wpt index : " << wpt_idx << " wpt_idx + num_wpt : " << wpt_idx + num_wpt << std::endl;
                std::cout<<"pearson_value "<< pearson_value << std::endl;

                wpt_idx_next = wpt_idx + num_wpt;
                while(wpt_idx_next > MAX_WPT_IDX)    wpt_idx_next = wpt_idx_next - 1;//In case next wpt index is out of range
                std::cout << "Wpt index : " << wpt_idx << ", Wpt index next : " << wpt_idx_next << ", Wpt index MAX : " << MAX_WPT_IDX << std::endl;
                wpt_idx_pre = wpt_idx;
                wpt_idx = wpt_idx_next;//Store next waypoint

                // Next planning's start and goal setting
                local_s[0] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[0];
                local_s[1] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[1];
                local_s[2] = robot_traj2.joint_trajectory.points[NoPathPoints2-1].positions[2];

                local_g[0] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[0];
                local_g[1] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[1];
                local_g[2] = robot_traj.joint_trajectory.points[wpt_idx_next].positions[2];
            }
            else
                std::cout << "No local solution found" << std::endl;



            pearson_value = pearsoncoeff(points_x,points_y);
            std::cout<<"pearson_value "<< pearson_value << std::endl;

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

            if(wpt_idx_pre >= MAX_WPT_IDX)  break;

        }
        std::cout << "Done!" << std::endl;
        ros::Time End_time = ros::Time::now();
        //ros::Time planning_time = End_time - Start_time;
        //ROS_INFO("Total Path Planning time :  %f", planning_time.toSec());
        ROS_INFO("Total Path Planning time :  %f sec", (End_time - Start_time).toSec());

        ros::Duration(1.0).sleep();

    }

    return 0;
}

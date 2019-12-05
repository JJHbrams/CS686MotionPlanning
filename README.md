# CS686MotionPlanning
2019 Fall semester - CS686 Motion Planning Project source code
# How to run
1. Open the terminal and
```bash
cd ~/MotionPlanning_ws
```
2. After get to MotionPlanning_ws, enter
```bash
source devel/setup.sh
```    
  This will make your shell as a ROS workspace.  
  Should do this once you open new shell.  
  To check if your workspace setting is correct...  
```bash
echo $ROS_PACKAGE_PATH        
```
  If the setting is not correct you would see  
```bash
$/opt/ros/$(ROS_VERSION)/share
```
   ex)ROS melodic use, /opt/ros/melodic/share  
   Unless you would see  
```bash
$/home/mrjohd/MotionPlanning_ws/src:/opt/ros/$(ROS_VERSION)/share
```  
3. If you are ...  
  A. using CLion  
    Enter following command (Should set the current shell as workspace!!)  
```bash    
sh  /home/mrjohd/clion/bin/clion.sh
```  
    And open "/UncertainKino/uncertain_kinodynamic/CMakeList.txt" as a project.   
    Then run main.  

  B. Not using CLion, just using terminal  
    Go to MotionPlanning_ws  
```bash
cd ~MotionPlanning_ws
```
Enter following command  
```bash
catin_make
```  
After compile is done without any error messeges, enter  
```bash
roslaunch vehicle demo.launch
```  
and  
```bash
rosrun uncertain_kinodynamic main
```  

##Folder description
* UncertainKino  
This is a actual source code directory.  
How to access to the source code   
Go to "/uncertain_kinodynamic/src" there exist following three files.  
 main.cpp   Scene.cpp   StateSpaces.cpp   UncertainKinodynamicPlanner.cpp   UncertainKinodynamicPlanner_RRTstar.cpp  
First, main.cpp runs total procedure, global planning, local planning and displaying.  
Second, Scene.cpp consist the synthetic scene with obstacles.  
Third, StateSpaces.cpp checks the state validity.  
Fourth, UncertainKinodynamicPlanner.cpp and UncertainKinodynamicPlanner_RRTstar.cpp are those who contains each planner's implementation.  
              SST with control                      RRT* with geometric    
Plus under "/uncertain_kinodynamic/include/uncertain_kinodynamic" there are header files such as  
Scene.h   StateSpaces.h   UncertainKinodynamicPlanner.h   UncertainKinodynamicPlanner_RRTstar.h  
They just contains each source codes' information which has same name.  

* vehicle  
This is a collision checking configuration for robot model,vehicle, two wheeled differential drive.  
Moveit based collision checker.  
to run the demo.  
```bash
roslaunch vehicle demo.launch
```  

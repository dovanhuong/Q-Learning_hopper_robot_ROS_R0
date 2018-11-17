# Q-Learning_hopper_robot_ROS_R0
This robot used Q-Learning algorithm for Hopper robot can learn how to reach the desired point in meters. 
<br>Thanks to ROS platform, Gazebo 7, the construct tutorial, OpenAI Gym to make this simulation. <br>
In order to run this training algorithm, you should follows steps as below:<br>
 1. Run catkin_make in ROS workspace folder. <br>
    <ROS_workspace_location> $ catkin_make
 2. Run simulation 3D model in gazebo. <br>
    $ roslaunch my_legged_robots_sims main.launch
 3. Launch Q-Training for hopper robot.<br>
    $ roslaunch my_hopper_training main.launch
If you have any issue or problem, contact to me via: vanhuong.robotics@gmail.com


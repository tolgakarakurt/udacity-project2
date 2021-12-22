# udacity-project2
### Udacity's nano degree program: Robotics Software Engineering  
**Project Title**: Go Chase It!  
**Project Goals**: 
- Design and build a 4 wheeled skid-steering robot equipped with a camera and lidar,
- Use the world environment created in project1,
- Create a white ball detector algorithm,
- Drive the robot toward the ball and stop once nothing appears on the cam.
- Full video: [ball chaser](https://drive.google.com/file/d/1KChEJ0094AI-eKJTh-2Iie8hSe548AJE/view?usp=sharing)  

**Setup and pull**
```
mkdir -p $HOME/<user_workspace>/src
cd $HOME/<user_workspace>/src
git clone git@github.com:tolgakarakurt/udacity-project2.git
catkin_make
roslaunch my_robot world.launch
roslaunch ball_chaser ball_chaser.launch
```

**Package Directory**
```
TKProject2                          # Go Chase It Project
├── my_robot                       # my_robot package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── worlds                      # world folder for world files
│   │   ├── myoffice.world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_image.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info                  
└──   
```
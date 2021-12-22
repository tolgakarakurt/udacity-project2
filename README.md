# udacity-project2
### Udacity's nano degree program: Robotics Software Engineering  
**Project Title**: Go Chase It!  
**Project Goals**: 
- Design and build a 4 wheeled skid-steering robot equipped with a camera and lidar,
- Use the world environment created in project1,
- Create a white ball detector algorithm,
- Drive the robot toward the ball and stop once nothing appears on the cam.
 
### Setup and pull
```
mkdir -p $HOME/<user_workspace>/src
cd $HOME/<user_workspace>/src
git clone git@github.com:tolgakarakurt/udacity-project2.git
catkin_make
roslaunch my_robot world.launch
roslaunch ball_chaser ball_chaser.launch
```
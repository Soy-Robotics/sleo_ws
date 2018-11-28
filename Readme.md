#在个人工作目录下执行如下命令

  mkdir ~/catkin_ws/src
  cd ~/catkin_ws
  git clone https://github.com/Soy-Robotics/sleo_ws.git
  rosdep install --from-paths src --rosdistro kinetic -y
  catkin_make
  
#编译完成后，运行如下命令
  echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc
  source ~/.bashrc

#Sleo Simulator

Here under commands is for test python GUI with Sleo in gazebo simulator.

## Gazebo 
Typing under command for launch gazebo and load Sleo mode:
```
roslaunch sleo_gazebo sleo_playpen.launch
```

## Testsuite
Under command is for testsuite which is a GUI for test Sleo:
```
rosrun sleo_testsuite sleo_testuiste
```
Just click "开始" for test linear error.

### Expect the result
Sleo will run and stop close 1 meter.

### View odom
Under command you can view current odometry:
```
rostopic echo /odom
```


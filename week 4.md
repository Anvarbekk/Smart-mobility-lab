# week-4-lab
### This is the installation of Turtlesim
- Step 1 we have to make our system always up-to-date
```
sudo apt update
```
![image](https://user-images.githubusercontent.com/95737530/195253041-824edf0c-1e0c-4c04-8eb9-d8f899296c1e.png)
- Step 2 Installing the turtlesim library
```
sudo apt install ros-rolling-turtlesim
```
![image](https://user-images.githubusercontent.com/95737530/195253482-f99daf75-2344-4296-97b0-1483f82bb005.png)
- Step 3 checking package is installed or not 
```
ros2 pkg executables turtlesim
```
![image](https://user-images.githubusercontent.com/95737530/195253945-021295d2-2850-4fa6-8026-44fdbd870162.png)
- commanding by terminal 
```
ros2 run turtlesim turtlesim_node
```
![image](https://user-images.githubusercontent.com/95737530/195254474-6eb4c9f6-ab09-44b6-9893-22875c1f9201.png)
- turtle appears in the center of window   
![image](https://user-images.githubusercontent.com/95737530/195254624-47fe5235-7227-4794-b8e2-ffa3f816764d.png)
- this is a command which used for moving turtle 
![image](https://user-images.githubusercontent.com/95737530/195255087-cef5a9d1-ce33-4c81-be8b-08a129f033b9.png)
- We can control turtle with arrow keys
```
ros2 run turtlesim turtle_teleop_key
```

[Screencast from 10-12-2022 02:12:36 PM.webm](https://user-images.githubusercontent.com/95737530/195261460-1fb2205d-f27c-47c1-839c-b945ef078106.webm)
### Installation of RQT
- Step 1 Always make it up to date 
```
sudo apt update
```
![image](https://user-images.githubusercontent.com/95737530/195263029-580599aa-08d6-4094-b109-2e5d4928e2e0.png)
- Step 2 Install the rqt library and its plugins
```
sudo apt install ~nros-rolling-rqt*
```
![image](https://user-images.githubusercontent.com/95737530/195263370-ab619f4d-5c22-430d-81d5-31a6f5b24b65.png)
- Step 3 Using rqt
```
rqt
```
![image](https://user-images.githubusercontent.com/95737530/195264887-71516f39-bb9b-4cb9-8f52-dedc30aaa59a.png)
- To start rqt_console in a new terminal with the following command
```
ros2 run rqt_console rqt_console
```

![image](https://user-images.githubusercontent.com/95737530/195265221-8ee66513-cb92-42e0-9958-e53433fb6fb4.png)
# ROS 2 Calcon
- Step 1 Installing calcon extensions
```
sudo apt install python3-colcon-common-extensions
```

![image](https://user-images.githubusercontent.com/95737530/195266583-6d871c25-3d74-4af4-bacc-ccbc8cec94ae.png)
- Step 2 Creating workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
![image](https://user-images.githubusercontent.com/95737530/195267230-459851a6-23c9-4698-b5c9-499cbd266f4f.png)
- Add some sources
```
git clone https://github.com/ros2/examples src/examples -b humble
```
- build the workspace 
```
colcon build --symlink-install
```
![image](https://user-images.githubusercontent.com/95737530/196668869-b923eaf4-efb2-458b-8327-e7d4243b25a9.png)
- Source the environment
```
. install/setup.bash
```
- Setup
  -The command colcon_cd allows you to quickly change the current working d
irectory of your shell to the directory of a package.
```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/rolling/" >> ~/.bashrc
```
## Creating workspace
- Step 1 Source ROS 2 environment
```
source /opt/ros/rolling/setup.bash
```
- Step 2 Create a new Directory
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src
 ```
 - Step 3 Clone the Github repo
 ```
 git clone https://github.com/ros/ros_tutorials.git -b rolling-devel
 ```
 - Step Resolve dependencies
 ```
 rosdep install -i --from-path src --rosdistro rolling -y
 ```

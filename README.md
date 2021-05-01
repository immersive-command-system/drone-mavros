# drone-mavros
File that runs onboard the drone, connects to and communicates with the ISAACS server

## Setup
### Drone and Simulation Computer Instructions

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation). We recommend installing the option with Gazebo at this stage (desktop full).

2. Install Dependencies: geographiclib, mavlink, roslibpy
    ```
    sudo apt-get install geographiclib-* ros-melodic-geographic-*  
    sudo apt-get install ros-melodic-mavlink
    sudo apt-get install libgeographic-dev ros-melodic-geographic-msgs
    pip install roslibpy
    ```
   
3. Clone this repository by running: `git clone https://github.com/immersive-command-system/drone-mavros.git --recursive`.

4. Once in the directory of the repository (drone-mavros), run `catkin build`

### Simulation Installation Instructions
Any repositories downloaded can be placed independently of the rest.
1. Install the desired firmware SITL (Software-In-The-Loop). The two available options are PX4 or ArduPilot.
    1. For ArduPilot, run the following: `git clone https://github.com/ArduPilot/ardupilot.git`
    Change directory into ardupilot via `cd ardupilot`. Run `git submodule update --init --recursive`. 
    From inside the ardupilot directory, run `Tools/environment_install/install-prereqs-ubuntu.sh -y`. Finally,
    run `. ~/.profile`. For more information, visit https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux.
    
    2. For PX4, run the following: `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`.
    Change directory into PX4-Ardupilot via `cd PX4-Ardupilot`. Run `bash ./Tools/setup/ubuntu.sh`.
    For more information, visit https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html.
    
2. If you have not yet installed Gazebo, please install Gazebo by following [this link](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

3. Download https://github.com/SwiftGust/ardupilot_gazebo and follow installation instructions on that README.

### Multi-Drone Simulation with Depth Camera Installation Instructions
1. Go to drone-mavros/src/hexacopter_launch/scripts/start_ardupilot.sh and edit it so that the quotation marks after cd on line 2 has the absolute file path of the ardupilot/Tools/autotest folder that was downloaded earlier. An example of line 2 is cd '/home/Documents/ardupilot/Tools/autotest'.
2. Repeat the above step, but for drone-mavros/src/hexacopter_launch/scripts/start_second_ardupilot.sh

## Usage
### Instructions for Running Simulation
These steps must be completed before running ROS if the simulator is to be used. Use these steps for a generic world.
1. Launch Gazebo with ArduPilot Plugin.
    1. Run gazebo and the corresponding .world file. Ex: `gazebo --verbose typhoon_ardupilot.world`.
    The .world file is found in the worlds folder of the ardupilot_gazebo repo that was downloaded previously.
    
2. Launch the desired firmware SITL.  
    1. For ArduPilot, change directory into `ardupilot/Tools/autotest`. 
    To start the firmware SITL, run `python sim_vehicle.py -v ArduCopter -f gazebo-iris`. 
    This should provide information about the possible arguments when starting the SITL.
    See https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html for more information.
    Keep in mind, the default parameters make the drone in a quadcopter configuration in SITL (but may be hexacopter in Gazebo).
    In order to change the parameter to a hexacopter configuration, `param set FRAME_CLASS 2`.
    To see a map of the flight area use: `python sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0`
    
### Instructions for Running Drone Code
1. After building the ROS project, `source devel/setup.bash` in the `drone-mavros` directory.
2. Run `roslaunch server_connector start_connection.launch server_ip:={ip}` to launch the drone where {ip} should be replaced by the ip of the ISAACS Server.

### Running Multi-Drone Simulation with Depth Cameras
1. After building the ROS project, `source devel/setup.bash` in the `drone-mavros` directory.

2. Launch the custom created Gazebo world with ROS by running `roslaunch hexacopter_launch drone_sim.launch`. 
   This should open up RVIZ to visualize depth camera point clouds, Gazebo with two drones in the world, and ardupilot for the first drone.
   
3. Launch the second drone's ardupilot by running in a separate terminal, `roslaunch hexacopter_launch drone_sim2.launch`

4. 

## Common Errors
`Resource not found: mavros` - Probably a faulty git submodule.
1. `cd drone-mavros`
2. `git submodule update --init --recursive`
3. `catkin build`
4. `source devel/setup.bash`

`catkin: command not found`
1. Try `sudo apt-get install ros-$ROS_DISTRO-catkin python-catkin-tools`
    
When `libmavconn` package, `CMake Error` finding package configuration file provided by `mavlink`.
1. `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`

`Pymavlink not found`
1. Pymavlink may not be installed. To fix this issue, run `pip install pymavlink`

`roslibpy not found`
1. roslibpy may not be installed. To fix this issue, run `pip install roslibpy`
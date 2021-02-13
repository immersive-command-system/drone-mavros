# drone-mavros
File that runs onboard the drone, connects to and communicates with the ISAACS server

## Setup
### Simulation Installation Instructions
1. Install the desired firmware SITL (Software-In-The-Loop). The two available options are PX4 or ArduPilot.
    1. For ArduPilot, run the following: `git clone https://github.com/ArduPilot/ardupilot.git`
    Change directory into ardupilot via `cd ardupilot`. Run `git submodule update --init --recursive`. 
    From inside the ardupilot directory, run `Tools/environment_install/install-prereqs-ubuntu.sh -y`. Finally,
    run `. ~/.profile`. For more information, visit https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux.
    
    2. For PX4, run the following: `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`.
    Change directory into PX4-Ardupilot via `cd PX4-Ardupilot`. Run `bash ./Tools/setup/ubuntu.sh`.
    For more information, visit https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html.
    
2. Please install Gazebo by following [this link](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

3. Download https://github.com/SwiftGust/ardupilot_gazebo and follow installation instructions on that README.

### Drone and Simulation Computer Instructions

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation)

2. Install geographiclib
    ```
    sudo apt-get install geographiclib-* ros-melodic-geographic-*    
    ```
   
3. Clone this repository by running: `git clone https://github.com/immersive-command-system/drone-mavros.git --recursive`.

4. Once in the directory of the repository, run `catkin build`

## Usage
### Instructions for Running Simulation
These steps must be completed before running ROS if the simulator is to be used.
1. Launch the desired firmware SITL.  
    1. For ArduPilot, change directory into `ardupilot/Tools/autotest`. 
    To start the firmware SITL, run `sim_vehicle.py -v ArduCopter -f hexa`. 
    This should provide information about the possible arguments when starting the SITL.
    See https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html for more information.
    
2. Launch Gazebo with ArduPilot Plugin.
    1. Run gazebo and the corresponding .world file. Ex: `gazebo --verbose typhoon_ardupilot.world`.
    The .world file is found in the worlds folder of the ardupilot_gazebo repo that was downloaded previously.
    
### Instructions for Running Drone Code
1. After building the ROS project, `source devel/setup.bash` in the directory.
2. Run `roslaunch drone.launch` to launch the drone.

    
    
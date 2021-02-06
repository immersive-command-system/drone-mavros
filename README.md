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

### Drone and Simulation Computer Instructions

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation)

2. Install MAVROS and dependency by running
    ```sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras        
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod a+x install_geographiclib_datasets.sh                
    ./install_geographiclib_datasets.sh
    sudo apt-get install python-catkin-tools
    ```
   More information can be found here: https://ardupilot.org/dev/docs/ros-install.html
   
3. Clone this repository by running: `git clone https://github.com/immersive-command-system/drone-mavros.git --recursive`.

4. Once in the directory of the repository, run `catkin build`



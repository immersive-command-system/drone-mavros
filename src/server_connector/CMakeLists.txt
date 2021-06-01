cmake_minimum_required(VERSION 3.0.2)
project(server_connector)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
find_package(catkin REQUIRED COMPONENTS message_generation)
add_service_files(FILES AdvertiseService.srv PublishTopic.srv)
catkin_python_setup()
generate_messages(DEPENDENCIES )
catkin_package(CATKIN_DEPENDS rospy)

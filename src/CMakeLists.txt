cmake_minimum_required(VERSION 3.20.0)

# Set the board you're targeting
set(BOARD Nucleo_F767ZI)

# Find Zephyr package, this will set up the Zephyr environment
find_package(Zephyr)

# Project name for this application
project(micro_ros_zephyr_app)

# Add your source files here
target_sources(app PRIVATE src/main.cpp)

# Include directories for your application
target_include_directories(app PRIVATE include)

# Enable MicroROS module
# Assuming the module is added as a Zephyr module in west.yml or similar
# If not, you might need to explicitly include the path to the module

# Configuration for MicroROS
set(MICRO_ROS_TRANSPORT "udp") # or "udp", "serial-usb" etc., based on your setup
set(MICRO_ROS_FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/modules/zephyr/micro_ros)

# Add MicroROS to the build
zephyr_include_directories(
        ${MICRO_ROS_FIRMWARE_DIR}/include
)

# MicroROS specific configurations
if(${MICRO_ROS_TRANSPORT} STREQUAL "serial")
    add_compile_definitions(MICRO_ROS_TRANSPORT_SERIAL)
elseif(${MICRO_ROS_TRANSPORT} STREQUAL "udp")
    add_compile_definitions(MICRO_ROS_TRANSPORT_UDP)
elseif(${MICRO_ROS_TRANSPORT} STREQUAL "serial-usb")
    add_compile_definitions(MICRO_ROS_TRANSPORT_SERIALUSB)
endif()

# Include MicroROS library
# Note: This assumes that micro_ros_zephyr_module has been set up correctly in your environment
target_link_libraries(app PUBLIC micro_ros_zephyr_module)

# If you have additional configuration files or need to adjust memory settings, etc., you might do it here
# Example:
# zephyr_linker_script(zephyr.ld)

# You might need to include additional MicroROS specific source files or configurations
# Example:
# FILE(GLOB microros_src ${MICRO_ROS_FIRMWARE_DIR}/src/*.c)
# target_sources(app PRIVATE ${microros_src})

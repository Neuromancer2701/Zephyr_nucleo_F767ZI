cmake_minimum_required(VERSION 3.20.0)

# Set the board you're targeting
set(BOARD nucleo_f767zi) # Corrected case

# Find Zephyr package, this will set up the Zephyr environment
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name for this application
project(micro_ros_zephyr_app)

# Add your source files here
target_sources(app PRIVATE src/main.cpp)

# Include directories for your application (if you have a local 'include' folder)
# target_include_directories(app PRIVATE include) # Uncomment if you create an 'include' dir

# Micro-ROS module is typically handled by Kconfig (CONFIG_MICRO_ROS=y in prj.conf)
# and its own CMake files within the Zephyr build system.
# No need for MICRO_ROS_FIRMWARE_DIR or manual include directories here if the module
# is correctly located in $ZEPHYR_BASE/modules/optional/micro_ros

# The compile definitions for transport should also be handled by Kconfig options
# e.g. CONFIG_MICRO_ROS_TRANSPORT_UDP=y from prj.conf

# Link against the Micro-ROS module library.
# The name 'micro_ros_zephyr_module' should be the target name provided by the module's CMake files.
# This might also be 'microros' or similar, depending on the module's definition.
# Often, Zephyr modules are linked automatically when enabled via Kconfig.
# If direct linking is needed and 'micro_ros_zephyr_module' is the correct target:
target_link_libraries(app PUBLIC micro_ros_zephyr_module)
# Alternatively, sometimes modules use zephyr_library_link_libraries(app micro_ros_zephyr_module)
# or are linked transitively. For now, explicit public linking is a safe bet.

# If you have project-specific configurations or need to adjust memory settings, etc., you might do it here
# Example:
# zephyr_linker_script(zephyr.ld)

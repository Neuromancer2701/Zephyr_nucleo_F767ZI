# Micro-ROS on Zephyr for Nucleo-F767ZI

This repository provides a complete scaffolding for running Micro-ROS on Zephyr OS, specifically targeting the Nucleo-144 STM32F767ZIT6 development board.

## Prerequisites

- Docker
- An ST-LINK programmer (on-board the Nucleo board)

## Development Environment

This project uses a Docker container to provide a consistent and reproducible development environment.

### 1. Build the Docker Image

Open a terminal in the root of this repository and run the following command to build the Docker image:

```bash
docker build -t micro-ros-zephyr-dev ./docker
```

### 2. Run the Docker Container

After building the image, run the following command to start a container. This command will also mount the current directory into the container at `/home/dev/workspace` and pass the USB device for the ST-LINK programmer.

**Note:** You may need to adjust the `--device` flag depending on your system. You can use `lsusb` to find the correct device.

```bash
docker run -it --rm \
    -v $(pwd):/home/dev/workspace \
    --device /dev/bus/usb \
    micro-ros-zephyr-dev
```

Alternatively, you can use the `--privileged` flag, which is less secure but often more convenient:

```bash
docker run -it --rm \
    -v $(pwd):/home/dev/workspace \
    --privileged \
    micro-ros-zephyr-dev
```

## Building and Flashing the Firmware

Once you are inside the Docker container, you can build and flash the firmware.

### 1. Initialize the West Workspace

Before building, you need to initialize the `west` workspace and update the dependencies:

```bash
cd /home/dev/workspace
west init -l
west update
```

### 2. Build the Firmware

Use the following command to build the firmware for the `nucleo_f767zi` board:

```bash
west build -b nucleo_f767zi app
```

### 3. Flash the Firmware

Finally, use the following command to flash the firmware to the board:

```bash
west flash
```

## Micro-ROS Agent

To communicate with the Micro-ROS application running on the Nucleo board, you will need to run a Micro-ROS agent on your host machine. You can find instructions on how to do this in the official [Micro-ROS documentation](https://micro.ros.org/docs/tutorials/core/first_application_rtos/zephyr/).

Once the agent is running, you should be able to see the `micro_ros_counter` topic and listen to the incrementing counter.

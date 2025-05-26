# micro-ROS on Zephyr for ST Nucleo STM32F767 via Docker

This project provides a Dockerized environment for developing and building micro-ROS applications for the ST Nucleo STM32F767 board using Zephyr RTOS.

## Prerequisites

- Docker installed on your host machine.
- ST Nucleo STM32F767 development board (for flashing and testing).
- Basic understanding of ROS 2 and Zephyr.

## Directory Structure

- `docker/`: Contains the Dockerfile for building the development environment.
- `src/`: Contains the sample micro-ROS application source code (`main.cpp`, `CMakeLists.txt`, `prj.conf`).

## Setup and Build Instructions

### 1. Build the Docker Image

Navigate to the root directory of this repository and run:

```bash
docker build -t micro-ros-zephyr-stm32f767 docker/
```

This command builds the Docker image named `micro-ros-zephyr-stm32f767` using the `docker/dockerfile`. The image will contain Zephyr, the necessary toolchain, west, and micro-ROS components.

### 2. Run the Docker Container

Once the image is built, run a container with the `src` directory mounted as a volume. This allows you to edit the application code on your host machine and build it inside the container.

Open a terminal and run:

```bash
docker run -it --rm \
  -v $(pwd)/src:/app \
  micro-ros-zephyr-stm32f767 \
  bash
```

This command:
- Starts an interactive terminal (`-it`) in the Docker container.
- Removes the container when it exits (`--rm`).
- Mounts the current directory's `src` folder to `/app` inside the container (`-v $(pwd)/src:/app`).
- Uses the image `micro-ros-zephyr-stm32f767` we built earlier.
- Runs `bash` as the entry command, giving you a shell inside the container.

You will now be inside the container's shell, in the `/app` directory (which is your `src` directory). The Zephyr environment (`ZEPHYR_BASE`, SDK, `west` in PATH) is already set up.

### 3. Build the micro-ROS Application

Inside the Docker container's shell (in the `/app` directory):

First, create a build directory and navigate into it:
```bash
mkdir build && cd build
```

Then, run `west build` to compile the application:
```bash
# To ensure a clean build the first time or after configuration changes:
# west build -p auto -b nucleo_f767zi ..

# Standard build command:
west build -b nucleo_f767zi ..
```
- `-b nucleo_f767zi` specifies the target board.
- `..` tells `west` to look for the application source in the parent directory (`/app`).

If the build is successful, the firmware will be located at `build/zephyr/zephyr.elf` (and `zephyr.bin`, `zephyr.hex`).

### 4. Flash the Application

Flashing the firmware to the ST Nucleo STM32F767 board typically requires tools like `STM32_Programmer_CLI` or `openocd`, which are usually run from the host machine or need specific USB device access.

**Option A: Using `west flash` (if host tools are configured and accessible via Docker)**

If your Docker container has access to the USB device (e.g., via `--device=/dev/ttyACM0` or similar, and udev rules are set up on the host), and `west` is configured with the correct flasher (e.g., `openocd`, `stlink`), you might be able to use:

```bash
# Inside the container, in the build directory
west flash
```
*Note: This often requires additional Docker container privileges and host-side configuration (udev rules for device access, OpenOCD/STLink installation on the host if west calls out to them).*

**Option B: Copy firmware to host and flash manually**

1.  From **another terminal on your host machine** (not inside the container), copy the firmware from the container to your host. First, find your container ID:
    ```bash
    docker ps 
    ```
    Look for the container running `micro-ros-zephyr-stm32f767`. Let's say its ID is `your_container_id`.

2.  Copy the firmware (e.g., `zephyr.bin`):
    ```bash
    docker cp your_container_id:/app/build/zephyr/zephyr.bin .
    ```

3.  Then, use your preferred flashing tool on the host to flash `zephyr.bin` to the Nucleo board. For example, using `STM32_Programmer_CLI` (ensure it's installed and in your PATH):
    ```bash
    STM32_Programmer_CLI -c port=SWD -d zephyr.bin 0x08000000 -v
    ```
    Or using `openocd`:
    ```bash
    # Example openocd command (may vary based on your setup)
    # openocd -f board/st_nucleo_f7.cfg -c "program zephyr.elf verify reset exit"
    ```

### 5. Run the micro-ROS Agent

To communicate with your micro-ROS application running on the STM32, you need a micro-ROS agent running on a machine that can reach the STM32 via its configured transport (UDP/IP in this case).

If your STM32 board is connected to your local network (e.g., via Ethernet), and your host machine is on the same network:

1.  **Install the micro-ROS agent** on your host machine (or another Docker container with network access):
    ```bash
    # If you have a ROS 2 Humble/Iron/Jazzy environment sourced:
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 
    # Or using the Docker image for the agent:
    # docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v 6
    ```
    Ensure the agent's IP address and port match what the firmware expects (see `prj.conf`). The sample uses UDP on port 8888.

Once the agent is running and the STM32 application starts, you should see the `zephyr_int32_publisher` node and its `/zephyr_int_topic` topic appear in your ROS 2 network:

```bash
# On your host machine with ROS 2 environment sourced
ros2 node list
ros2 topic list
ros2 topic echo /zephyr_int_topic
```

## Sample Application

The sample application in `src/main.cpp`:
- Initializes a micro-ROS node named `zephyr_int32_publisher`.
- Creates a publisher on the topic `zephyr_int_topic`.
- Periodically publishes an incrementing `std_msgs/msg/Int32` message to this topic.
- Prints messages to the Zephyr console (e.g., "Sent: X").

This demonstrates a basic working micro-ROS setup on Zephyr for the Nucleo F767ZI.

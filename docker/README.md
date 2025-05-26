

docker build -t zephyr-dev .

docker run -v /home/seegrid.local/sking/repos/Zephyr_nucleo_F767ZI:/checkout/src/zephyr_ws -td --name micro_ros_env zephyr-dev

docker exec -it micro_ros_env bash


--progress=plain --no-cache  # helpful for build debugging

```
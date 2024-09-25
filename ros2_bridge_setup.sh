#!/bin/bash

# Run the Docker container with the specified options
docker run -it --rm \
    --network host \
    ghcr.io/screamlab/pros_base_image:latest \
    /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"  
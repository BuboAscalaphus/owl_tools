# --- User config ---
WS_DIR_NAME=${WS_DIR_NAME:-ros2_ws} 
IMAGE=${IMAGE:-my-ros-image:latest}   # your image tag (e.g. owl:desktop or owl:desktop-itp)
NAME=${NAME:-owl-dev}             # container name
WS=${WS:-$HOME/$WS_DIR_NAME}            # host workspace folder
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} # DDS domain

# Set NV=1 if you need Jetson NV GStreamer plugins (Argus / nv*).
NV=${NV:-0}
BAGS=${BAGS:-$WS/src/owl_bags}   # host folder where bags are saved
CONTAINER_USER=dev
USERNAME=dev

# --- User config ---
IMAGE=${IMAGE:-owl:desktop-itp}   # your image tag (e.g. owl:desktop or owl:desktop-itp)
NAME=${NAME:-owl-dev}             # container name
WS=${WS:-$HOME/owl_ws}            # host workspace folder
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} # DDS domain

# Set NV=1 if you need Jetson NV GStreamer plugins (Argus / nv*).
NV=${NV:-0}
BAGS=${BAGS:-$HOME/owl_bags}   # host folder where bags are saved


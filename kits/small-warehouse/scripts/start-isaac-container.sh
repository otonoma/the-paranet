#/bin/bash
xhost +local:docker

docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" \
  -e "ROS_DISTRO=foxy" -e "ROS_PYTHON_VERSION=3" -e "ROS_VERSION=2" \
  -e "LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/foxy/lib" \
  -e "PARAFLOW_HOST=http://localhost:3030" \
  -e DISPLAY=$DISPLAY \
  -e "QT_X11_NO_MITSHM=1" \
  -e "XDG_RUNTIME_DIR=/tmp/runtime-root" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/docker/scripts:/isaac-sim/scripts \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v ~/uw-capstone/:/uw-capstone:rw \
  isaac-sim-warm 
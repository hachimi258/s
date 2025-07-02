SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../")

echo $PROJECT_DIR

mkdir -p "$PROJECT_DIR/.cppad"

xhost +
docker run --name kuavo-tv -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    --ulimit rtprio=99 \
    -e "PRIVACY_CONSENT=Y" \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v "$PROJECT_DIR/.cppad:/var/ocs2" \
    -e DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all,display \
    -e CARB_GRAPHICS_API=vulkan \
    -e GDK_SYNCHRONIZE=1 \
    -e ROBOT_VERSION=45 \
    -v $PROJECT_DIR:/TongVerse/biped_challenge:rw \
    -v $PROJECT_DIR/score.yaml:/TongVerse/tongverse/score.yaml \
    kuavo_tv_release:latest bash
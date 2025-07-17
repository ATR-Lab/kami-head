# ros-source.sh

ros-source() {
    # Determine the ROS version by checking what exists in /opt/ros
    local ros_version=""
    for dir in /opt/ros/*/; do
        if [[ -f "$dir/setup.bash" ]]; then
            ros_version=$(basename "$dir")
            break
        fi
    done

    if [[ -z "$ros_version" ]]; then
        echo "ROS installation not found in /opt/ros/"
        return 1
    fi

    # Source global ROS setup
    local ros_setup="/opt/ros/$ros_version/setup.bash"
    if [[ -f "$ros_setup" ]]; then
        echo "Sourcing: $ros_setup"
        source "$ros_setup"
    else
        echo "Could not find: $ros_setup"
        return 1
    fi

    # Search upward for a workspace with install/setup.bash
    local dir="$PWD"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/install/setup.bash" ]]; then
            echo "Sourcing workspace: $dir/install/setup.bash"
            source "$dir/install/setup.bash"
            return 0
        fi
        dir=$(dirname "$dir")
    done

    echo "No ROS workspace with 'install/setup.bash' found in parent directories."
    return 1
}

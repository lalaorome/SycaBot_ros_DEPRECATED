#!/bin/bash

show_help() {
    echo " "
    echo "  usage: Source ROS2 and different local setup + eventually set fast dds server ip"
    echo "          ./source_ros.sh --pkg PKG_NAME"
    echo "                          --srv IP_ID"
    echo ""
    echo ""
    echo "  args :"
    echo ""
    echo "      --help          Show this help"
    echo ""
    echo "      --pkg Package you want to source. keys : sycabot"
    echo ""
    echo "      --srv Id of the ip adress of the fast dds server."
    echo "            the id is the x in 192.168.1.x"
    echo ""
    echo ""

}
die() {
    printf '%s\n' "$1"
    show_help
}

#Source ROS2 foxy :
source /opt/ros/foxy/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

while :; do
    case $1 in
        -h|-\?|--help)
            show_help    # Display a usage synopsis.
            ;;
        --pkg)
            if [ "$2" == "sycabot" ];then
                echo "sourcing sycabot package..."
                source ~/syca_ws/install/local_setup.bash
                shift
            else
                die 'ERROR: "--pkg" wrong input argument.'
            fi
            ;;
        --srv)
            if [ "$2" ];then
                echo "setting discovery server ip..."
                export ROS_DISCOVERY_SERVER="192.168.1.$2:11811"
                shift
            else
                die 'ERROR: "--srv" requires a non-empty option argument.'
            fi
            ;;
        --)              # End of all options.
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)               # Default case: No more options, so break out of the loop.
            break
    esac
    shift
done




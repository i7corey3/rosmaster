source /opt/ros/$ROS_DISTRO/setup.bash
DIR="$( cd "$( dirname "$0" )" && pwd )"
WS="$(dirname "$DIR")"
cd $DIR

if [ "$3" == "test" ]; 
then
    printf "The node name test is reserved and not allowed\n"
    exit 1
fi
if [ "$2" == "test" ]; 
then
    printf "The package name test is reserved and not allowed\n"
    exit 1
fi

case $1 in
    python)
        ros2 pkg create --build-type ament_python $2
        mv $2 $WS/src
        python3 $PWD/buildNode.py $2 python $3
    ;;
    c++)
        ros2 pkg create --build-type ament_cmake $2
        mv $2 $WS/src
        python3 $PWD/buildNode.py $2 c++ $3
    ;;
    main)
        ros2 pkg create --build-type ament_cmake $2
        mv $2 $WS/src
        python3 $PWD/buildNode.py $2 main pass
    ;;
    messages)
        ros2 pkg create --build-type ament_cmake $2
        mv $2 $WS/src
        python3 $PWD/buildNode.py $2 messages none
    ;;
    systemctl)
        mkdir -p $WS/systemctl/{scripts/placeholder,services/placeholder}
        cp $PWD/files/setup.sh $WS/systemctl
        cat > "$WS/systemctl/services/placeholder/template.service" <<EOF
[Unit]
Description=Description Goes Here
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/path/to/your/executable
Restart=on-failure
RestartSec=5s
User=$(whoami)

[Install]
WantedBy=multi-user.target
EOF

    ;;
    *)
        printf "Type python, c++, messages, or main (for urdf main package) then the package name and node name\n\nexample:\nFor Python\n\t./createPackage.sh python <package_name> <node_name>\n\nFor C++\n\t./createPackage.sh c++ <package_name> <node_name>\n\nFor main urdf package\n\t./createPackage.sh main <package_name>\n\nFor custom messages\n\t./createPackage.sh messages <package_name>\n\nFor creating systemctl folder\n\t./createPackage.sh systemctl\n\n"
    ;;
esac
cd ../
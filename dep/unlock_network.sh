
## Removes all REJECT from iptables
IFS=$' ' num=( $(sudo iptables -L --line-number | grep REJECT | cut -f 1 -d " ") )
readarray -t arr <<<$num

for (( i=${#arr[@]}-1; i>=0; i-- )); do
    if [[ "${arr[i]}" =~ ^[0-9]+$ ]]; then
        sudo iptables -D FORWARD "${arr[i]}";
    fi
done 

## Allows ROS2 topics over wifi and ethernet

if [ "$CYCLONEDDS_URI" != "<CycloneDDS><Domain><General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General></Domain></CycloneDDS>" ]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
    ros2 daemon stop
    ros2 daemon start
fi
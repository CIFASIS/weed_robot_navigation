#!/bin/bash

#./field_test.sh -n -t "[3, 4]" -v -p "-x -14.56 -y 1.0 -Y 1.570796327" -l "-10.0"
#./field_test.sh -n -t "[3, 8]" -v -p "-x -14.56 -y 1.0 -Y 1.570796327" -l "-10.0"

#./field_test.sh -n -t "[3, 4]" -v -p "-x -14.56 -y -10.5 -Y 1.570796327"
#./field_test.sh -n -t "[4, 3]" -v -p "-x -12.48 -y -10.5 -Y 1.570796327"
#./field_test.sh -n -t "[17, 14]" -v -p "-x 14.56 -y -10.5 -Y 1.570796327"
#./field_test.sh -n -t "[17, 14]" -v -b -p "-x 14.56 -y 10.5 -Y -1.570796327"
#./field_test.sh -n -t "[8, 5]" -v -p "-x -4.16 -y -10.5 -Y 1.570796327"
#./field_test.sh -n -t "[11, 8]" -v -b -p "-x 2.08 -y -1 -Y -1.570796327"
#./field_test.sh -n -t "[15, 11]" -v -b -p "-x 10.4 -y -1 -Y -1.570796327"

#./field_test.sh -n -t "[8, 9]" -g -v -p "-x -4.16 -y 1.0 -Y 1.570796327" -l "-10.0"

# defaults
world="empty"
map="field"
navigation=false
pose="-x -14.56 -y -11.0 -Y 1.570796327"
gui=false
visualization=false
tracks="[1, 2]"
omega=false
bottom=false
last=4.0
sensor=false

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -w|--world)
    world="$2"
    shift # past argument
    shift # past value
    ;;
    -m|--map)
    map="$2"
    shift # past argument
    shift # past value
    ;;
    -n|--navigation)
    navigation=true
    shift # past argument
    ;;
    -s|--sensor)
    sensor=true
    shift # past argument
    ;;
    -p|--pose)
    pose="$2"
    shift # past argument
    shift # past value
    ;;
    -l|--last)
    last="$2"
    shift # past argument
    shift # past value
    ;;
    -g|--gui)
    gui=true
    shift # past argument
    ;;
    -t|--tracks)
    tracks="$2"
    shift # past argument
    shift # past value
    ;;
    -v|--visualization)
    visualization=true
    shift # past argument
    ;;
    -o|--omega)
    omega=true
    shift # past argument
    ;;
    -b|--bottom)
    bottom=true
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

echo "Ejecutando simulación del robot desmalezador"
echo "World Gazebo: ${world}"
echo "Mapa global: ${map}"
echo "Navigation: ${navigation}"
echo "Pose: ${pose}"
echo "Gui: ${gui}"

sleep 1s
roslaunch weed_robot_gazebo gazebo.launch world:="${world}" pose:="${pose}" gui:="${gui}" paused:=true &

if [ "${sensor}" = true ];
then
  local_costmap="local_costmap_params"
else
  local_costmap="local_costmap_static_params"
fi

sleep 4s
roslaunch weed_robot_navigation navigation.launch global_map:="${map}" visualization:="${visualization}" \
  rviz:=teb local_costmap:="${local_costmap}" &

sleep 2s
if [ "${sensor}" = true ];
then
  echo "Ejecutando generación de nube de puntos"
  roslaunch weed_robot_navigation sensor.launch common:=navigation_common_params &
else
  echo "No se ejecutará la generación de nube de puntos"
fi

sleep 2s
if [ "${navigation}" = true ];
then
  echo "Ejecutando navegación"
  roslaunch weed_robot_navigation waypoint.launch common:=navigation_common_params \
    travel_last:=true tracks:="${tracks}" use_omega:="${omega}" bottom:="${bottom}" last_offset:="${last}"
else
  echo "No se ejecutará navegación"
fi

wait

# ps -aux | less | grep gazebo
# kill pid


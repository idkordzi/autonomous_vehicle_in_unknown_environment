#!/bin/bash
if [ -z "$PROJECT_PATH" ];
then
    PROJECT_PATH="$HOME/workspace/autonomous_vehicle_in_unknown_environment"

    export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJECT_PATH/build/simulation_tools/modules/husarion_gz_worlds:$GZ_SIM_SYSTEM_PLUGIN_PATH

    export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/install/simulation_tools/share/husarion_gz_worlds/worlds:$GZ_SIM_RESOURCE_PATH
    export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/install/simulation_tools/share/husarion_gz_worlds/models:$GZ_SIM_RESOURCE_PATH
else
    echo "Gazebo variables already set!"
fi

source $PROJECT_PATH/install/setup.bash

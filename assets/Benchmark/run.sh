#! /bin/bash

# rosnode kill --all & sleep 2s

source /home/pjlab/anaconda3/etc/profile.d/conda.sh
conda deactivate
source ~/.bashrc  # load ~/.bashrc setup
source /home/pjlab/liukehui/COHERENT-OmniGibson/Benchmark/ros_hademo_ws/devel/setup.bash
{
	gnome-terminal --tab "roscore_node" -- bash -c "roscore;exec bash"
}&
sleep 2s



BENCHMARK_ROOT=`pwd`

# Scenes
	# house_double_floor_lower
	# Merom_1_int
	# Pomaria_1_int
	# grocery_store_cafe
	# restaurant_brunch

# _Task1, _Task2, ....

# Scene_name_Taskid
#demo1
task_name="Merom_1_int_Task1"
#demo2
# task_name='house_double_floor_lower_Task1'


conda activate omnigibson

# echo $BENCHMARK_ROOT

# {
# 	 gnome-terminal --tab "SIM" -- python3 $BENCHMARK_ROOT/sim.py --task_name $task_name
# }&
# sleep 2s

{
	 gnome-terminal --tab "LLM" -- python3 $BENCHMARK_ROOT/ros_hademo_ws/src/hademo/src/action_publisher.py --task_name $task_name
}&
sleep 2s


python3 $BENCHMARK_ROOT/sim.py --task_name $task_name




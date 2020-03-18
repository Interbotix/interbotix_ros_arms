# interbotix_pid_control

## Overview
This package can be used as a way to test 'pwm' or 'current' PID gains when operating the arm in either 'pwm' or 'current' mode. PID gains are read into the controller node from the [gains.yaml](config/gains.yaml) file. The node then commands the arm to its 'home' pose and waits ten seconds for it to settle. Then it commands the arm to its 'sleep' pose and waits another ten seconds for it to settle. Finally, the node commands all the motors to either zero pwm or zero current (effectively torquing them off) before shutting itself down.

## Structure
![pid_control_flowchart](images/pid_control_flowchart.png)
As shown above, the *interbotix_pid_control* package builds on top of the *interbotix_sdk* package. To get familiar with the nodes in the *interbotix_sdk* package, please look at its README. The node specific to this package is described below.
- **pid_control_node** - contains a simple PID controller to command the arm joints (excluding gripper) first to their 'home' positions and then to their 'sleep' positions; it receives the desired PID gains from [gains.yaml](config/gains.yaml) and the `control_mode` as a parameter.

## Usage
To run this package, first create a 'gains.yaml' file similar to the one [here](config/gains.yaml) with your desired 'pwm' or 'current' PID gains. Then run the command below (assuming the ReactorX 150 is being used in 'pwm' mode).
```
roslaunch interbotix_pid_control pid_control.launch robot_name:=rx150 control_mode:=pwm
```
To further customize the launch of the robot, take a look at the table below.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_name | name of a robot (ex. 'wx200') | "" |
| control_mode | the outputted command from the control loop - can be either 'pwm' or 'current' | pwm |
| gains_filepath | location of your 'gains.yaml' file | reference [pid_control.launch](launch/pid_control.launch) |
| arm_run | 'true' if the *arm_run.launch* file should be launched - set to 'false' if you would like to run your own version of this file separately | true |

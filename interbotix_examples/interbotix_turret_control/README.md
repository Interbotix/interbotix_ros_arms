# interbotix_turret_control

## Overview
This package can be used to control the motion of any of the five [X-series turret](https://www.trossenrobotics.com/c/robot-turrets.aspx) platforms. Essentially pan-and-tilt mechanisms, the turrets can be manipulated either using a SONY PS3 or PS4 controller via Bluetooth and/or a GUI. The target audience for this package are users who prefer to operate the turrets without having to directly program them.

Regarding naming conventions, there are three parts to a turret's name. The first two letters correspond to the robot model name. For example, the 'WidowX' platform is signified by 'wx'. The next two letters describe the type of motor used in the turret. There are only two choices available: 'xl' and 'xm'. Finally, the last letter signifies if the 'tilt' joint is composed of a single motor ('s') or dual motors ('d'). Thus, the 'PhantomX XL430 Robot Turret' can be abbreviated as 'pxxls'.

## Structure
![turret_control_flowchart](images/turret_control_flowchart.png)
As shown above, the *interbotix_turret_control* package builds on top of the *interbotix_sdk* package. To get familiar with the nodes in the *interbotix_sdk* package, please look at its README. The other nodes are described below:
- **joy** - a ROS driver for a generic Linux joystick; it reads data from a SONY PS3 or PS4 controller joystick over Bluetooth and publishes  [sensor_msgs/Joy](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html) messages to the `joy` topic
- **turret_control_joy_node** - responsible for reading in raw [sensor_msgs/Joy](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html) messages from the `joy` topic and converting them into [TurretJoyControl](msg/TurretJoyControl.msg) messages; this makes the code more readable and allows users to remap buttons very easily later.
- **turret_control** - responsible for reading in [TurretJoyControl](msg/TurretJoyControl.msg) messages and sending 'pan' and 'tilt' commands to the *interbotix_sdk* node; besides for allowing external joystick control, it also displays a GUI to the user that can be used to manipulate the turret.

## Bluetooth Setup
#### Sony PS4 Controller (Recommended)
Getting a PS4 controller connected via Bluetooth to a Linux laptop is pretty straightforward. Click the *Bluetooth* icon on the top right of your screen, followed by *Bluetooth Settings...*. Next, press and hold the *Share* button on the PS4 controller (see image below for reference). While holding the *Share* button, press and hold the *PS* button. After a few seconds, the triangular shaped LED located between the *L2* and *R2* buttons should start rapidly flashing white (about twice a second) at which point you can let go.

On the computer, click the '+' icon in the *Bluetooth* settings window. Wait until you see 'Wireless Controller' pop up, select it, and click *Next* on the bottom right of the window. A message should display saying 'successfully set up new device 'Wireless Controller'' and the LED should turn blue. This means the controller is connected to the computer. To disconnect, hold down the *PS* button for about 10 seconds until the LED turns off. To reconnect, just press the *PS* button (no need to hold it down). After blinking white a few times, the LED should turn blue.

![ps4](images/ps4.jpg)

#### Sony PS3 Controller
Getting a PS3 controller connected via Bluetooth to a Linux laptop can be a bit finicky at times. However, the commands below should do the trick. Get an original SONY PS3 controller, it's accompanying USB cable, and open up a terminal. Type:
```
$ sudo bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# scan on
```
Now, plug the PS3 controller into the Linux Laptop. At this point, a message should pop up in the terminal that looks something like the following (with a different MAC address):
```
[NEW] Device FC:62:B9:3F:79:E7 PLAYSTATION(R)3 Controller
```
When it appears, type:
```
[bluetooth]# trust <MAC-address>
```
Now unplug the PS3 controller and press the PS button. The four red LEDs at the front of the controller should flash a few times, eventually leaving just one LED on by the '1'. This means that the joystick paired successfully.

Sometimes, the joystick might cause the cursor of the computer mouse to go crazy. To fix this, add the following line to the `.bashrc` file:
```
alias joy_stop='xinput set-prop "PLAYSTATION(R)3 Controller" "Device Enabled" 0'
```
Now whenver the PS3 joystick is paired to the computer, just type `joy_stop` in the terminal to stop it messing with the mouse (you're welcome).

## Usage
After pairing the joystick, type the following in a terminal (let's say to control the ViperX Dual XM540 Turret with a PS4 controller):
```
roslaunch interbotix_turret_control turret_control.launch robot_name:=vxxmd
```
A red error message might appear in the screen saying `Couldn't open joystick force feedback!`. This is normal and will not affect the joystick operation. To further customize the launch file at run-time, look at the table below:

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_name | five character name of a turret | "" |
| use_default_rviz | 'true' if Rviz should be displayed; 'false' otherwise | true |
| threshold | value from 0 to 1 defining joystick sensitivity; a larger number means the joystick should be less sensitive | 0.75 |
| controller | type of Playstation controller ('ps3' or 'ps4') | ps4 |
| start_non_gui_nodes | 'true' if the 'non GUI nodes' should be launched; 'false' otherwise | true |
| turret_run | 'true' if the *turret_run.launch* file should be launched - set to 'false' if you would like to run your own version of this file separately | true |

After launching, a GUI should pop up similar to the one below. To become more familiar with the GUI and external joystick controls, please refer to the tutorial located [here](Turret_Control_Tutorial.pdf).

![turret_control_gui](images/turret_control_gui.png)

If only controlling the turret with a joystick, look at the diagram and table below (make sure that the 'External Joystick' check-box is checked in the GUI or your controller will not do anything):

#### PS3 & PS4 Button Mapping

![ps3](images/ps3.jpg)

| Button | Action |
| ------ | ------ |
| START/OPTIONS | move the turret to its center position |
| R2 | rotate the 'pan' joint clockwise |
| L2 | rotate the 'pan' joint counterclockwise |
| D-pad Up | increase motor angular velocity in steps |
| D-pad Down | decrease motor angular velocity in steps|
| D-pad Left | 'Coarse' control - sets motor angular velocity to a user-preset 'fast' speed |
| D-pad Right | 'Fine' control - set motor angular velocity to a user-preset 'slow' speed |
| Right stick Left/Right | pan the Turret CCW/CW |
| R3 | reverses the Right stick Left/Right control and R2/L2 buttons |
| Left stick Up/Down | tilt the Turret forward/backward |
| L3 | reverses the Left stick Up/Down control|

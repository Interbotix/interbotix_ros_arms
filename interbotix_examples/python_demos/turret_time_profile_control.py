from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script commands arbitrary positions to the PhantomX XL430 Turret when using a Time-Based-Profile for its Drive Mode
# When operating a robot in 'position' control mode, Time-Based-Profile allows you to easily set the duration of a particular movement
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk turret_run.launch robot_name:=pxxls use_time_based_profile:=true'
# Then change to this directory and type 'python turret_time_profile_control.py'
# A moving_time of '2' means the joints will complete the desired motion in 2 seconds. An accel_time of '0.3' means that each joint will
# spend 0.3 seconds accelerating/decelerating to a constant velocity.

def main():
    joint_positions = [0, 0]
    turret = InterbotixRobot(robot_name="pxxls", moving_time=2, accel_time=0.3)
    turret.set_joint_commands(joint_positions, delay=2.0)
    turret.set_single_joint_command("pan", 1.57, delay=2.0)
    turret.set_joint_commands([3.14, 1.5], delay=2.0)
    turret.set_joint_commands([1,1], delay=2.0)

if __name__=='__main__':
    main()

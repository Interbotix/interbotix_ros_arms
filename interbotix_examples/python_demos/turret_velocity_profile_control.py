from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script commands arbitrary positions to the PhantomX XL430 Turret when using a Velocity-Based-Profile for its Drive Mode
# When operating a robot in 'position' control mode, Velocity-Based-Profile allows you to easily set the max velocity and acceleration for a particular movement
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk turret_run.launch robot_name:=pxxls'
# Then change to this directory and type 'python turret_velocity_profile_control.py'
# A moving_time of '131' corresponds to a max allowable velocity of 3.14 rad/s while an accel_time of '15' corresponds to a max acceleration of 5.6 rad/s^2
# However, as shown below, the max allowable velocity and acceleration can be changed for each motion. If the moving_time or accel_time arguments
# are not specified, the last specified moving_time and accel_time are used.

def main():
    joint_positions = [0, 0]
    turret = InterbotixRobot(robot_name="pxxls", moving_time=131, accel_time=15, use_time=False)
    turret.set_joint_commands(joint_positions, delay=2.0)
    turret.set_single_joint_command("pan", 1.57, delay=1.0)
    turret.set_joint_commands([3.14, 1.5], moving_time=20, accel_time=5, delay=5)
    turret.set_joint_commands([1,1], delay=5)

if __name__=='__main__':
    main()

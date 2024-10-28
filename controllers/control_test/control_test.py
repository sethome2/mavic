import math
import sys

from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Keyboard, LED, Motor

def clamp(value, low, high):
    return max(low, min(value, high))

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    camera = robot.getDevice("camera")
    camera.enable(timestep)
    front_left_led = robot.getDevice("front left led")
    front_right_led = robot.getDevice("front right led")
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")

    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(1.0)

    print("Start the drone...")

    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

    print("You can control the drone with your computer keyboard:")
    print("- 'up': move forward.")
    print("- 'down': move backward.")
    print("- 'right': turn right.")
    print("- 'left': turn left.")
    print("- 'shift + up': increase the target altitude.")
    print("- 'shift + down': decrease the target altitude.")
    print("- 'shift + right': strafe right.")
    print("- 'shift + left': strafe left.")

    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0

    target_altitude = 1.0

    while robot.step(timestep) != -1:
        time = robot.getTime()

        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        altitude = gps.getValues()[2]
        roll_velocity = gyro.getValues()[0]
        pitch_velocity = gyro.getValues()[1]

        led_state = int(time) % 2
        front_left_led.set(led_state)
        front_right_led.set(not led_state)

        camera_roll_motor.setPosition(-0.115 * roll_velocity)
        camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                pitch_disturbance = -2.0
            elif key == Keyboard.DOWN:
                pitch_disturbance = 2.0
            elif key == Keyboard.RIGHT:
                yaw_disturbance = -1.3
            elif key == Keyboard.LEFT:
                yaw_disturbance = 1.3
            elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
                roll_disturbance = -1.0
            elif key == (Keyboard.SHIFT + Keyboard.LEFT):
                roll_disturbance = 1.0
            elif key == (Keyboard.SHIFT + Keyboard.UP):
                target_altitude += 0.05
                print("target altitude: {} [m]".format(target_altitude))
            elif key == (Keyboard.SHIFT + Keyboard.DOWN):
                target_altitude -= 0.05
                print("target altitude: {} [m]".format(target_altitude))
            key = keyboard.getKey()

        roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
        pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = clamp(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
        vertical_input = k_vertical_p * clamped_difference_altitude ** 3.0

        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)

if __name__ == "__main__":
    main()

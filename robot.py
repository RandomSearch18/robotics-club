import robot
import time

R = robot.Robot()

# while True:

#     markers = R.see()
#     time.sleep(2)

# while True:
#   markers = R.see()
#   print(len(markers))
#   time.sleep(2)

# TIME_FOR_90DEG =
# TIME_FOR_1M =

R.motors[0] = -100
R.motors[1] = 100

MOTOR_POWER_1 = 100
MOTOR_POWER_2 = 100


HOME_WALL_MARKERS = [i + R.zone + 100 for i in range(0, 5)]
CUBES_TO_FIND = [i + R.zone for i in range(0, 5)]
marker = HOME_WALL_MARKER, CUBES_TO_FIND


def forward():
    R.motors[0] = MOTOR_POWER_1
    R.motors[1] = MOTOR_POWER_2


def left():
    R.motors[0] = MOTOR_POWER_1
    R.motors[1] = -MOTOR_POWER_2


def right():
    R.motors[0] = -MOTOR_POWER_1
    R.motors[1] = MOTOR_POWER_2


def backwards():
    R.motors[0] = -MOTOR_POWER_1
    R.motors[1] = -MOTOR_POWER_2


def stop():
    R.motors[0] = 0
    R.motors[1] = 0


def drive_forward():
    forward()
    time.sleep(distance * TIME_FOR_1M)
    stop()


def turn_clockwise(angle):
    right()
    time.sleep(angle * TIME_FOR_90DEG)
    stop()


def align_with_marker(code):
    """`code` - the code of the marker to which the robot should align
    Returns:
     - True if the robot is aligned with the marker
     - False otherwise
    """
    for i in range(0, ALIGN_ATTEMPTS):
        markers = R.see(look_for=code)
        if len(markers) == 0:
            return False
        marker = markers[0]
        if -ALIGN_TOLERANCE < marker.bearing.y < ALIGN_TOLERANCE:
            return True
        # Saw marker but we need to turn to be aligned
        turn_clockwise(marker.bearing.y)
    return False


def find_and_face_marker(marker_type):
    """Turn the robot twice round to find a marker of a given marker_type.
    `marker_type` - The marker_type of the marker to look for
    Returns:
     - True: The robot is facing the marker marker_type
     - False: The robot couldn't find the marker
    """
    for i in range(0, 720, SEARCH_ANGLE):
        markers = R.see(look_for=marker_type)
        if len(markers) == 0:
            # No markers found so turn and we might see one next time
            turn_clockwise(SEARCH_ANGLE)
        else:
            marker = markers[0]  # The closest marker
            if align_with_marker(marker.code) is True:
                # The robot is now facing the marker
                return marker
            # We couldn't see the marker this time that we looked
            print("Lost tracking on marker")
    return False


def find_and_face_sheep():
    for i in range(0, 720, SEARCH_ANGLE):
        markers = R.see()
        sheep = []
        for marker in markers:
            print(marker.info.target_type)
            if marker.info.target_type == TARGET_TYPE.SHEEP:
                sheep.append(marker)

        if len(sheep) == 0:
            # No sheep found so turn and we might see one next time
            turn_clockwise(SEARCH_ANGLE)
        else:
            marker = sheep[0]  # The closest marker
            if align_with_marker(marker.code) is True:
                # The robot is now facing the marker
                return marker
            # We couldn't see the marker this time that we looked
            print("Lost tracking on marker")
    return False


def drive_to_marker(marker, overshoot):
    """Drive in steps realigning each step towards a given marker
    `marker` - The marker to drive towards
    `overshoot` - The distance to drive further than the marker. This makes sure
                  that we get there. Negative values will drive short of the
                  marker.
    Returns:
    - True: The robot has reached the marker
    - False: The robot lost sight of the marker
    """
    while marker.dist > STEP_SIZE_M:
        drive_forward(STEP_SIZE_M)
        markers = R.see(look_for=id)
        if len(markers) == 0:
            # We drove forward and can no longer see the marker
            # Return False to show we failed to reach the marker
            return False
        marker = markers[0]
        align_with_marker(marker.code)
    drive_forward(marker.dist + overshoot)
    return True


def go_to_marker(marker_type, overshoot=0.0):
    """Will turn and drive until the robot is at a marker of the given marker_type.
    `overshoot` - The distance to drive further than the marker. This makes sure
                  that we get there. Negative values will drive short of the
                  marker.
    Returns:
    - True: The robot has reached the marker
    - False: The robot lost sight of the marker
    """
    while True:
        marker = find_and_face_marker(marker_type)
        if marker is False:
            print(f"{marker_type} not found, driving forward to improve the view")
            drive_forward(STEP_SIZE_M)
        else:
            if drive_to_marker(marker, overshoot) is False:
                print("Lost marker whilst driving to it")
            else:
                return True  # At the marker

        # Maybe we have hit something, so back up and try again
        drive_forward(-STEP_SIZE_M)


def main():
    """Forever drive to markers, then drive home. The reverse is included so
    as not to interfere with any other marker which we might have just carried
    home with us."""
    while True:
        go_to_marker(CUBES_TO_FIND, overshoot=0.1)
        go_to_marker(HOME_WALL_MARKERS, overshoot=-0.1)
        drive_forward(-0.2)
        turn_clockwise(180)


main()
# forward()

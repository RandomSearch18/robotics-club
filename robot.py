import robot
import time

R = robot.Robot()

# Commented-out code is for testing the motors

# while True:

#     markers = R.see()
#     time.sleep(2)

# while True:
#   markers = R.see()
#   print(len(markers))
#   time.sleep(2)

# TIME_FOR_90DEG =
# TIME_FOR_1M =

# R.motors[0] = -100
# R.motors[1] = 100

MOTOR_POWER_1 = 200
MOTOR_POWER_2 = 200


# Number of seconds it take the robot to drive 1 metre
TIME_FOR_1M = 2.0
# Number of seconds it take the robot to turn 90 degrees
TIME_FOR_90DEG = 3.0
# Number of times the robot should try turning towards a marker before it gives up
ALIGN_ATTEMPTS = 5
# The robot will consider itself aligned to a marker if it's facing it head-on (0°) or within ± this number
ALIGN_TOLERANCE = 10  # degrees
# Angle that the robot will turn each time when it's scanning for markers
SEARCH_ANGLE = 45  # degrees
# How far the robot should drive forward until it stops to check that it's still aligned with the marker
STEP_SIZE_M = 3


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


def drive_forward(distance):
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
    """Turn the robot twice round to find a marker of a given marker type.
    `marker_type` - "sheep" to look for sheep, "home_wall" to look for a wall marker in our area
    Returns:
     - True: The robot is facing the marker marker_type
     - False: The robot couldn't find the marker
    """
    for i in range(0, 720, SEARCH_ANGLE):
        all_markers = R.see()
        # Find the markers that match our given type
        correct_markers = []
        for marker in all_markers:
            ## SHEEP MARKERS
            if marker_type == "sheep":
                if marker.info.target_type == TARGET_TYPE.SHEEP:
                    correct_markers.append(marker)
            ## HOME WALL MARKERS
            elif marker_type == "home_wall":
                if (
                    # Check that it's a wall marker
                    marker.info.type == robot.ARENA
                    # and that it's owned by our team (R.zone is our team)
                    and marker.info.owning_team == R.zone
                ):
                    correct_markers.append(marker)
            else:
                print(f"ERROR: {marker_type} is an invalid marker type")
                return False

        if len(correct_markers) == 0:
            # No markers found so turn and we might see one next time
            turn_clockwise(SEARCH_ANGLE)
        else:
            marker = correct_markers[0]  # The closest marker
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
    """Will turn and drive until the robot is at a marker of the given marker type.
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
        go_to_marker("sheep", overshoot=0.1)
        go_to_marker("home_wall", overshoot=-0.1)
        drive_forward(-0.2)
        turn_clockwise(180)


main()
# forward()

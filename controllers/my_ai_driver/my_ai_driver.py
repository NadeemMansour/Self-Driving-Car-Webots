from vehicle import Driver
from controller import Keyboard
import math

# =========================================================
# CONSTANTS
# =========================================================

TIME_STEP = 50
UNKNOWN = 99999.99

KP = 0.25
KI = 0.006
KD = 2

FILTER_SIZE = 3


# GLOBAL VARIABLES


PID_need_reset = False

speed = 0.0
steering_angle = 0.0
manual_steering = 0
autodrive = True


# DRIVER

driver = Driver()


# DEVICES


camera = driver.getDevice("camera")
camera.enable(TIME_STEP)

camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

sick = driver.getDevice("Sick LMS 291")
sick.enable(TIME_STEP)

sick_width = sick.getHorizontalResolution()
sick_fov = sick.getFov()

gps = driver.getDevice("gps")
gps.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# =========================================================
# HELP
# =========================================================

def print_help():

    print("You can drive this car!")
    print("Select the 3D window and use:")
    print("[LEFT]/[RIGHT] - steer")
    print("[UP]/[DOWN] - accelerate")
    print("[A] - auto drive")


# =========================================================
# AUTO DRIVE
# =========================================================

def set_autodrive(onoff):

    global autodrive

    if autodrive == onoff:
        return

    autodrive = onoff

    if autodrive:
        print("switching to auto-drive...")
    else:
        print("switching to manual drive...")


# =========================================================
# SPEED
# =========================================================

def set_speed(kmh):

    global speed

    if kmh > 250.0:
        kmh = 250.0

    if kmh < 0:
        kmh = 0

    speed = kmh

    print(f"setting speed to {kmh} km/h")

    driver.setCruisingSpeed(kmh)


# =========================================================
# STEERING
# =========================================================

def set_steering_angle(wheel_angle):

    global steering_angle

    # smooth steering

    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle + 0.1

    if wheel_angle - steering_angle < -0.1:
        wheel_angle = steering_angle - 0.1

    steering_angle = wheel_angle

    # clamp steering

    if wheel_angle > 0.5:
        wheel_angle = 0.5

    elif wheel_angle < -0.5:
        wheel_angle = -0.5

    driver.setSteeringAngle(wheel_angle)


# =========================================================
# MANUAL STEERING
# =========================================================

def change_manual_steer_angle(inc):

    global manual_steering

    set_autodrive(False)

    new_manual_steering = manual_steering + inc

    if -25 <= new_manual_steering <= 25:

        manual_steering = new_manual_steering

        set_steering_angle(manual_steering * 0.02)

    if manual_steering == 0:
        print("going straight")

    else:

        direction = "left" if steering_angle < 0 else "right"

        print(
            f"turning {steering_angle:.2f} rad ({direction})"
        )


# =========================================================
# KEYBOARD
# =========================================================

def check_keyboard():

    key = keyboard.getKey()

    if key == Keyboard.UP:

        set_speed(speed + 5.0)

    elif key == Keyboard.DOWN:

        set_speed(speed - 5.0)

    elif key == Keyboard.RIGHT:

        change_manual_steer_angle(+1)

    elif key == Keyboard.LEFT:

        change_manual_steer_angle(-1)

    elif key == ord('A'):

        set_autodrive(True)


# =========================================================
# COLOR DIFFERENCE
# =========================================================

def color_diff(a, b):

    diff = 0

    for i in range(3):

        d = a[i] - b[i]

        diff += abs(d)

    return diff


# =========================================================
# CAMERA PROCESSING
# =========================================================

def process_camera_image(image):

    REF = [95, 187, 203]

    sumx = 0
    pixel_count = 0

    pixel_index = 0

    for y in range(camera_height):

        for x in range(camera_width):

            r = camera.imageGetRed(
                image,
                camera_width,
                x,
                y
            )

            g = camera.imageGetGreen(
                image,
                camera_width,
                x,
                y
            )

            b = camera.imageGetBlue(
                image,
                camera_width,
                x,
                y
            )

            pixel = [b, g, r]

            if color_diff(pixel, REF) < 30:

                sumx += x

                pixel_count += 1

            pixel_index += 1

    if pixel_count == 0:
        return UNKNOWN

    return (
        (
            (sumx / pixel_count)
            / camera_width
        ) - 0.5
    ) * camera_fov


# =========================================================
# FILTER
# =========================================================

def filter_angle(new_value):

    global old_values

    if new_value == UNKNOWN:

        old_values = [0.0] * FILTER_SIZE

        return UNKNOWN

    old_values.pop(0)

    old_values.append(new_value)

    total = 0.0

    for value in old_values:
        total += value

    return total / FILTER_SIZE


old_values = [0.0] * FILTER_SIZE

# =========================================================
# LIDAR PROCESSING
# =========================================================

def process_sick_data(sick_data):

    HALF_AREA = 20

    sumx = 0
    collision_count = 0
    obstacle_dist = 0.0

    start = sick_width // 2 - HALF_AREA
    end = sick_width // 2 + HALF_AREA

    for x in range(start, end):

        distance = sick_data[x]

        # IMPORTANT FIX
        if 0.01 < distance < 20.0:

            sumx += x

            collision_count += 1

            obstacle_dist += distance

    if collision_count == 0:
        return UNKNOWN, 0.0

    obstacle_dist = obstacle_dist / collision_count

    angle = (
        (
            (sumx / collision_count)
            / sick_width
        ) - 0.5
    ) * sick_fov

    return angle, obstacle_dist


# =========================================================
# GPS
# =========================================================

gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0


def compute_gps_speed():

    global gps_coords
    global gps_speed

    gps_coords = gps.getValues()

    speed_ms = gps.getSpeed()

    gps_speed = speed_ms * 3.6


# =========================================================
# PID
# =========================================================

oldValue = 0.0
integral = 0.0


def applyPID(yellow_line_angle):

    global oldValue
    global integral
    global PID_need_reset

    if PID_need_reset:

        oldValue = yellow_line_angle

        integral = 0.0

        PID_need_reset = False

    # anti-windup

    if math.copysign(1, yellow_line_angle) != math.copysign(1, oldValue):

        integral = 0.0

    diff = yellow_line_angle - oldValue

    if -30 < integral < 30:

        integral += yellow_line_angle

    oldValue = yellow_line_angle

    return (
        KP * yellow_line_angle
        + KI * integral
        + KD * diff
    )


# =========================================================
# INITIALIZATION
# =========================================================

set_speed(50.0)

driver.setHazardFlashers(True)
driver.setDippedBeams(True)
driver.setAntifogLights(True)

print_help()

# =========================================================
# MAIN LOOP
# =========================================================

i = 0

while driver.step() != -1:

    check_keyboard()

    if i % int(TIME_STEP / driver.getBasicTimeStep()) == 0:

        camera_image = camera.getImage()

        sick_data = sick.getRangeImage()

        if autodrive:

            yellow_line_angle = filter_angle(
                process_camera_image(camera_image)
            )

            obstacle_angle, obstacle_dist = process_sick_data(
                sick_data
            )

            # =================================================
            # OBSTACLE AVOIDANCE
            # =================================================

            if obstacle_angle != UNKNOWN:

                driver.setBrakeIntensity(0.0)

                obstacle_steering = steering_angle

                # IMPORTANT FIX
                if obstacle_dist > 0.01:

                    if 0.0 < obstacle_angle < 0.4:

                        obstacle_steering = (
                            steering_angle
                            + (
                                obstacle_angle - 0.25
                            ) / obstacle_dist
                        )

                    elif obstacle_angle > -0.4:

                        obstacle_steering = (
                            steering_angle
                            + (
                                obstacle_angle + 0.25
                            ) / obstacle_dist
                        )

                steer = steering_angle

                if yellow_line_angle != UNKNOWN:

                    line_following_steering = applyPID(
                        yellow_line_angle
                    )

                    if (
                        obstacle_steering > 0
                        and line_following_steering > 0
                    ):

                        steer = max(
                            obstacle_steering,
                            line_following_steering
                        )

                    elif (
                        obstacle_steering < 0
                        and line_following_steering < 0
                    ):

                        steer = min(
                            obstacle_steering,
                            line_following_steering
                        )

                else:

                    PID_need_reset = True

                set_steering_angle(steer)

            # =================================================
            # LINE FOLLOWING
            # =================================================

            elif yellow_line_angle != UNKNOWN:

                driver.setBrakeIntensity(0.0)

                set_steering_angle(
                    applyPID(yellow_line_angle)
                )

            # =================================================
            # LOST LINE
            # =================================================

            else:

                driver.setBrakeIntensity(0.4)

                PID_need_reset = True

        # =====================================================
        # GPS
        # =====================================================

        compute_gps_speed()

    i += 1
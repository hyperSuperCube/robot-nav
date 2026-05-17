#!/usr/bin/env python3
import argparse
import csv
import math
import select
import signal
import sys
import time
from enum import Enum, auto

import pigpio

from constants import param
from drivesystem import DriveAction, DriveSystem
from proximitysensor import ProximitySensor


def shutdown_resources(resources):
    seen = set()
    for resource in resources:
        if resource is None:
            continue

        resource_id = id(resource)
        if resource_id in seen:
            continue
        seen.add(resource_id)

        if hasattr(resource, "shutdown"):
            resource.shutdown()
            continue
        if hasattr(resource, "brake"):
            resource.brake()
        if hasattr(resource, "stop"):
            resource.stop()
        if hasattr(resource, "cancel"):
            resource.cancel()


def install_ctrl_c_shutdown(get_resources):
    def handle_ctrl_c(signum, frame):
        print("\nCtrl+C received. Stopping robot and exiting...")
        shutdown_resources(get_resources())
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_ctrl_c)


class TestMode(Enum):
    INTERACTIVE = auto()
    READ = auto()
    HERDING_AVOIDANCE = auto()
    WALL_FOLLOWING = auto()
    WALL_PWM = auto()
    QUIT = auto()


class WallSide(Enum):
    LEFT = auto()
    RIGHT = auto()


CLI_MODE_ALIASES = {
    "interactive": TestMode.INTERACTIVE,
    "read": TestMode.READ,
    "herd": TestMode.HERDING_AVOIDANCE,
    "avoid": TestMode.HERDING_AVOIDANCE,
    "wall": TestMode.WALL_FOLLOWING,
    "wall-pwm": TestMode.WALL_PWM,
}

MAIN_MODE_ALIASES = {
    "r": TestMode.READ,
    "read": TestMode.READ,
    "h": TestMode.HERDING_AVOIDANCE,
    "herd": TestMode.HERDING_AVOIDANCE,
    "herding": TestMode.HERDING_AVOIDANCE,
    "a": TestMode.HERDING_AVOIDANCE,
    "avoid": TestMode.HERDING_AVOIDANCE,
    "avoidance": TestMode.HERDING_AVOIDANCE,
    "w": TestMode.WALL_FOLLOWING,
    "wall": TestMode.WALL_FOLLOWING,
    "q": TestMode.QUIT,
    "quit": TestMode.QUIT,
}

WALL_SIDE_ALIASES = {
    "l": WallSide.LEFT,
    "left": WallSide.LEFT,
    "r": WallSide.RIGHT,
    "right": WallSide.RIGHT,
}

WALL_DISTANCE_INDEX = {
    WallSide.LEFT: 0,
    WallSide.RIGHT: 2,
}

WALL_ERROR_SIGN = {
    WallSide.LEFT: 1.0,
    WallSide.RIGHT: -1.0,
}


def _connected_io():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        sys.exit(1)
    print("GPIO ready...")
    return io


def _typed_quit():
    """
    Return True when the operator typed q/quit and pressed Enter.
    """
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return False

    command = sys.stdin.readline().strip().lower()
    return command in ("q", "quit")


def _prompt_choice(prompt, aliases):
    while True:
        raw_command = input(prompt).strip().lower()
        choice = aliases.get(raw_command)
        if choice is not None:
            return choice
        print("Unknown option.")


def _open_csv(filename):
    if filename is None:
        return None, None

    csv_file = open(filename, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "time_s",
            "left_time_of_flight_s",
            "middle_time_of_flight_s",
            "right_time_of_flight_s",
            "left_distance_m",
            "middle_distance_m",
            "right_distance_m",
        ]
    )
    return csv_file, writer


def run_read_mode(sensor, interval, duration, csv_filename):
    """
    Trigger the ultrasonic sensors and print/log distances for characterization.
    """
    start_time = time.time()
    csv_file, writer = _open_csv(csv_filename)
    waiting_for_echo = False
    read_time = start_time

    while duration is None or time.time() - start_time < duration:
        now = time.time()
        if not waiting_for_echo:
            sensor.trigger()
            waiting_for_echo = True
            read_time = now + interval
            continue

        if now < read_time:
            continue

        distances = sensor.read()
        times = sensor.read_time_of_flight()
        print("TOF", times, "dist", distances)

        if writer is not None:
            writer.writerow(
                [
                    now - start_time,
                    times[0],
                    times[1],
                    times[2],
                    distances[0],
                    distances[1],
                    distances[2],
                ]
            )
        waiting_for_echo = False

    if csv_file is not None:
        csv_file.close()


def select_herding_command(
    distances,
    clear_distance=param.proximity_clear_distance,
    close_distance=param.proximity_close_distance,
):
    """
    Return the drive command for the 12-case herding table from the handout.

    Returns:
        (action, reverse, label), where action is None for stop.
    """
    left, middle, right = distances
    left_blocked = left is not None and math.isfinite(left) and left < clear_distance
    right_blocked = right is not None and math.isfinite(right) and right < clear_distance
    middle_clear = middle is not None and middle > clear_distance
    middle_close = middle is not None and math.isfinite(middle) and middle < close_distance

    if middle_clear:
        if right_blocked and not left_blocked:
            return DriveAction.TURN_LEFT, False, "advance: obstacle right"
        if left_blocked and not right_blocked:
            return DriveAction.TURN_RIGHT, False, "advance: obstacle left"
        return DriveAction.STRAIGHT, False, "advance"

    if middle_close:
        if right_blocked and not left_blocked:
            return DriveAction.TURN_RIGHT, True, "back ccw"
        if left_blocked and not right_blocked:
            return DriveAction.TURN_LEFT, True, "back cw"
        return DriveAction.STRAIGHT, True, "back up"

    if right_blocked and not left_blocked:
        return DriveAction.SPIN_LEFT, False, "spin left"
    if left_blocked and not right_blocked:
        return DriveAction.SPIN_RIGHT, False, "spin right"
    return None, False, "stop"


def run_herding_mode(drive, sensor, interval, duration=None, exit_on_q=True):
    """
    Repulse from obstacles using all three ultrasonic sensors.
    """
    start_time = time.time()
    last_trigger_time = start_time - interval
    last_print_time = 0.0
    last_label = None

    if exit_on_q:
        print("Herding/avoidance running. Type q then Enter to return.")

    while duration is None or time.time() - start_time < duration:
        if exit_on_q and _typed_quit():
            break

        now = time.time()
        if now - last_trigger_time >= interval:
            sensor.trigger()
            last_trigger_time = now

        distances = sensor.read()
        action, reverse, label = select_herding_command(distances)

        if action is None:
            drive.stop()
        else:
            drive.drive(action, reverse=reverse)

        if label != last_label or now - last_print_time > 0.25:
            print("Herding", label, distances)
            last_label = label
            last_print_time = now

    shutdown_resources([drive])


def select_wall_pwm_command(
    distances,
    side,
    nominal_distance=param.wall_follow_nominal_distance,
):
    """
    Return direct PWM levels for proportional wall following.
    """
    front = distances[1]
    if (
        front is not None
        and math.isfinite(front)
        and front < param.wall_follow_front_stop_distance
    ):
        return None, None, "front blocked"

    distance = distances[WALL_DISTANCE_INDEX[side]]
    if distance is None or not math.isfinite(distance):
        return None, None, "wall lost"

    error = WALL_ERROR_SIGN[side] * (distance - nominal_distance)
    limited_error = max(
        -param.wall_follow_max_error,
        min(param.wall_follow_max_error, error),
    )

    left_pwm = param.wall_follow_pwm_left_offset
    left_pwm += param.wall_follow_pwm_left_gain * limited_error
    right_pwm = param.wall_follow_pwm_right_offset
    right_pwm += param.wall_follow_pwm_right_gain * limited_error

    if limited_error != error:
        label = "pwm recover"
    else:
        label = "pwm e=%+.3f" % error

    return int(round(left_pwm)), int(round(right_pwm)), label


def run_wall_pwm_mode(drive, sensor, side, interval, duration):
    """
    Follow a left or right wall with proportional PWM feedback.
    """
    start_time = time.time()
    print("Wall following", duration, "seconds")
    last_trigger_time = start_time - interval
    last_print_time = 0.0
    last_label = None

    print(
        "PWM gains",
        param.wall_follow_pwm_left_offset,
        param.wall_follow_pwm_left_gain,
        param.wall_follow_pwm_right_offset,
        param.wall_follow_pwm_right_gain,
    )

    while duration is None or time.time() - start_time < duration:
        now = time.time()
        if now - last_trigger_time >= interval:
            sensor.trigger()
            last_trigger_time = now

        distances = sensor.read()
        left_pwm, right_pwm, label = select_wall_pwm_command(distances, side)

        if left_pwm is None:
            drive.stop()
        else:
            drive.pwm(left_pwm, right_pwm)

        if label != last_label or now - last_print_time > 0.25:
            print("Wall", side.name, label, left_pwm, right_pwm, distances)
            last_label = label
            last_print_time = now

    shutdown_resources([drive])


def run_interactive_menu(drive, sensor, interval, csv_filename):
    while True:
        mode = _prompt_choice(
            "\nMode [w]all-following/[h]erding-avoidance/[r]ead/[q]uit: ",
            MAIN_MODE_ALIASES,
        )

        if mode is TestMode.QUIT:
            return

        if mode is TestMode.READ:
            run_read_mode(
                sensor,
                interval,
                param.wall_follow_test_duration,
                csv_filename,
            )
        elif mode is TestMode.HERDING_AVOIDANCE:
            run_herding_mode(drive, sensor, interval, exit_on_q=True)
        elif mode is TestMode.WALL_FOLLOWING:
            side = _prompt_choice("Wall side [l]eft/[r]ight: ", WALL_SIDE_ALIASES)
            run_wall_pwm_mode(
                drive,
                sensor,
                side,
                interval,
                param.wall_follow_test_duration,
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test ultrasonic sensors and W6 proximity behaviors."
    )
    parser.add_argument(
        "mode",
        choices=tuple(CLI_MODE_ALIASES),
        nargs="?",
        default="interactive",
        help="Test mode to run.",
    )
    parser.add_argument(
        "--side",
        choices=tuple(WALL_SIDE_ALIASES),
        default="left",
        help="Wall side for wall-following modes.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=param.ultrasound_read_interval,
        help="Seconds between ultrasound triggers.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional run duration in seconds.",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="Optional CSV file for read-mode characterization data.",
    )
    args = parser.parse_args()
    args.mode = CLI_MODE_ALIASES[args.mode]
    args.side = WALL_SIDE_ALIASES[args.side]
    return args


def main():
    args = parse_args()
    io = _connected_io()
    sensor = None
    drive = None
    install_ctrl_c_shutdown(lambda: [drive, sensor, io])
    sensor = ProximitySensor(io)

    if args.mode is TestMode.READ:
        run_read_mode(sensor, args.interval, args.duration, args.csv)
    elif args.mode is TestMode.INTERACTIVE:
        drive = DriveSystem(io, pwm_max=param.pwm_max, pwm_freq=param.pwm_freq)
        run_interactive_menu(drive, sensor, args.interval, args.csv)
    else:
        drive = DriveSystem(io, pwm_max=param.pwm_max, pwm_freq=param.pwm_freq)
        if args.mode is TestMode.HERDING_AVOIDANCE:
            run_herding_mode(
                drive,
                sensor,
                args.interval,
                args.duration,
                exit_on_q=True,
            )
        elif args.mode in (TestMode.WALL_FOLLOWING, TestMode.WALL_PWM):
            run_wall_pwm_mode(
                drive,
                sensor,
                args.side,
                args.interval,
                args.duration or param.wall_follow_test_duration,
            )

    print("Turning off...")
    shutdown_resources([drive, sensor, io])


if __name__ == "__main__":
    main()

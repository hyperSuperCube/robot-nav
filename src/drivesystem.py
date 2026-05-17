#!/usr/bin/env python3
import pigpio
import signal
import sys
import time
import traceback
from enum import Enum, auto

PIN_MOTORR_LEGA = 7
PIN_MOTORR_LEGB = 8
PIN_MOTORL_LEGA = 6
PIN_MOTORL_LEGB = 5


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


class DriveAction(Enum):
    SPIN_LEFT = auto()
    HOOK_LEFT = auto()
    TURN_LEFT = auto()
    STEER_LEFT = auto()
    VEER_LEFT = auto()
    STRAIGHT = auto()
    VEER_RIGHT = auto()
    STEER_RIGHT = auto()
    TURN_RIGHT = auto()
    HOOK_RIGHT = auto()
    SPIN_RIGHT = auto()

    @property
    def label(self):
        return self.name.lower()


class Motor:
    """
    Low-level interface for one H-bridge controlled DC motor.

    API:
        motor = Motor(io, pin_a, pin_b, pwm_max, pwm_freq)
        motor.stop()
        motor.brake()
        motor.forward(level)
        motor.reverse(level)
        motor.setlevel(level)

    Args:
        io: Connected `pigpio.pi()` handle.
        pin_a: First motor control pin.
        pin_b: Second motor control pin.
        pwm_max: Maximum PWM duty value used in this project.
        pwm_freq: PWM frequency in Hz.
    """
    def __init__(self, io, pin_a, pin_b, pwm_max, pwm_freq):
        self.io = io
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pwm_max = pwm_max
        self.pwm_freq = pwm_freq

        self.io.set_mode(self.pin_a, pigpio.OUTPUT)
        self.io.set_mode(self.pin_b, pigpio.OUTPUT)

        self.io.set_PWM_range(self.pin_a, self.pwm_max)
        self.io.set_PWM_range(self.pin_b, self.pwm_max)

        self.io.set_PWM_frequency(self.pin_a, self.pwm_freq)
        self.io.set_PWM_frequency(self.pin_b, self.pwm_freq)

        self.stop()

    def stop(self):
        """
        Coast the motor by driving both PWM duty cycles to zero.
        """
        self.io.set_PWM_dutycycle(self.pin_a, 0)
        self.io.set_PWM_dutycycle(self.pin_b, 0)

    def brake(self):
        """
        Electrically brake the motor by driving both sides high.
        """
        self.io.set_PWM_dutycycle(self.pin_a, self.pwm_max)
        self.io.set_PWM_dutycycle(self.pin_b, self.pwm_max)

    def forward(self, level):
        """
        Spin forward with a magnitude from 0 to `pwm_max`.
        """
        level = max(0, min(self.pwm_max, int(level)))
        self.io.set_PWM_dutycycle(self.pin_a, self.pwm_max - level)
        self.io.set_PWM_dutycycle(self.pin_b, self.pwm_max)

    def reverse(self, level):
        """
        Spin reverse with a magnitude from 0 to `pwm_max`.
        """
        level = max(0, min(self.pwm_max, int(level)))
        self.io.set_PWM_dutycycle(self.pin_a, self.pwm_max)
        self.io.set_PWM_dutycycle(self.pin_b, self.pwm_max - level)

    def setlevel(self, level):
        """
        Set signed motor command: positive forward, negative reverse, zero stop.
        """
        level = int(level)
        if level > 0:
            self.forward(level)
        elif level < 0:
            self.reverse(-level)
        else:
            self.stop()


class DriveSystem:
    """
    Robot-level drive abstraction.

    Actions:
        `DriveAction.STRAIGHT`
        `DriveAction.VEER_LEFT` / `DriveAction.VEER_RIGHT`
        `DriveAction.STEER_LEFT` / `DriveAction.STEER_RIGHT`
        `DriveAction.TURN_LEFT` / `DriveAction.TURN_RIGHT`
        `DriveAction.HOOK_LEFT` / `DriveAction.HOOK_RIGHT`
        `DriveAction.SPIN_LEFT` / `DriveAction.SPIN_RIGHT`

    Tuning:
        Change `self.action_levels` in `__init__` to tune each motion directly.
        Each entry is (left_motor_level, right_motor_level).
        Positive => forward, negative => reverse, 0 => stop.
    """
    def __init__(self, io, pwm_max=255, pwm_freq=1000):
        """
        Args:
            io: Connected `pigpio.pi()` handle.
            pwm_max: Maximum PWM duty value for all motions.
            pwm_freq: PWM frequency in Hz for all motor pins.
        """
        self.io = io
        self.pwm_max = pwm_max
        self.pwm_freq = pwm_freq

        self.left = Motor(io, PIN_MOTORL_LEGA, PIN_MOTORL_LEGB, pwm_max, pwm_freq)
        self.right = Motor(io, PIN_MOTORR_LEGA, PIN_MOTORR_LEGB, pwm_max, pwm_freq)

        # ------------------------------------------------------------------
        # CENTRAL TUNING TABLE
        #
        # Format:
        #   DriveAction.ACTION_NAME: (left_motor_level, right_motor_level)
        # ------------------------------------------------------------------
        self.action_levels = {
            DriveAction.SPIN_LEFT:   ( -92,   95),
            DriveAction.HOOK_LEFT:   (   0,  116),
            DriveAction.TURN_LEFT:   (  80,  127),
            DriveAction.STEER_LEFT:  (  92,  115),
            DriveAction.VEER_LEFT:   ( 115,  127),
            DriveAction.STRAIGHT:    ( 109,  111),
            DriveAction.VEER_RIGHT:  ( 132,  111),
            DriveAction.STEER_RIGHT: ( 124,   96),
            DriveAction.TURN_RIGHT:  ( 135,   81),
            DriveAction.HOOK_RIGHT:  ( 122,    0),
            DriveAction.SPIN_RIGHT:  (  92,  -95),
        }

    def pwm(self, left_level, right_level):
        """
        Set signed left/right PWM levels directly.
        """
        self.left.setlevel(left_level)
        self.right.setlevel(right_level)

    def drive(self, action, reverse=False):
        """
        Apply one configured action from `self.action_levels`.

        Args:
            action: A `DriveAction` value. Strings are not accepted.
            reverse: If True, negate both motor levels to run the action
                backward.
        """
        left_level, right_level = self.action_levels[action]

        if reverse:
            left_level = -left_level
            right_level = -right_level

        self.pwm(left_level, right_level)

    def brake(self):
        """
        Brake both motors.
        """
        self.left.brake()
        self.right.brake()

    def stop(self):
        """
        Stop both motors.
        """
        self.left.stop()
        self.right.stop()

    def print_table(self):
        print("\nCurrent drive table:")
        print("Action         Left   Right")
        print("----------------------------")
        for action in DriveAction:
            l, r = self.action_levels[action]
            print(f"{action.label:12s} {l:5d} {r:6d}")
        print()

    def set_action_levels(self, action, left_level, right_level):
        """
        Override the motor levels for one named action.
        """
        self.action_levels[action] = (int(left_level), int(right_level))

    def scale_all(self, scale):
        """
        Uniformly scale all currently configured levels.
        Helpful if robot is globally too fast or too slow.
        """
        scaled_levels = {}
        for action, (l, r) in self.action_levels.items():
            new_l = max(-self.pwm_max, min(self.pwm_max, int(round(l * scale))))
            new_r = max(-self.pwm_max, min(self.pwm_max, int(round(r * scale))))
            scaled_levels[action] = (new_l, new_r)
        self.action_levels = scaled_levels


def flower_power(drive, run_time=4.0, use_brake=False):
    """
    Run all 11 actions one by one.
    After each segment, stop/brake and wait for Enter.
    """
    print("\nStarting flower-power demo.")
    print("Place robot at known start pose before each step.\n")

    for action in DriveAction:
        print(f"Running action: {action.label}")
        drive.drive(action)
        time.sleep(run_time)

        if use_brake:
            drive.brake()
        else:
            drive.stop()

        input("Hit return for next action... ")


def main():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        sys.exit(1)
    print("GPIO ready...")

    drive = None
    install_ctrl_c_shutdown(lambda: [drive, io])

    try:
        drive = DriveSystem(io, pwm_max=255, pwm_freq=1000)
        drive.print_table()
        flower_power(drive, run_time=4, use_brake=False)

    except Exception as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    finally:
        print("Turning off...")
        shutdown_resources([drive, io])


# if __name__ == "__main__":
#     main()

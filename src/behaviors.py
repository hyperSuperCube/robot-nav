#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import pigpio

from constants import param
from detectors import (
    BLACK,
    WHITE,
    BinaryDetector,
    SideEstimator,
    SideState,
)
from drivesystem import DriveAction, DriveSystem
from sensors import Gyroscope, LineSensor


class LineFollowerResult(Enum):
    INTERSECTION = auto()
    END_OF_STREET = auto()


class TurnDirection(Enum):
    LEFT = auto()
    RIGHT = auto()


TURN_SIGNS = {
    TurnDirection.LEFT: 1.0,
    TurnDirection.RIGHT: -1.0,
}

TURN_CONFIG = {
    TurnDirection.LEFT: (0, DriveAction.SPIN_LEFT),
    TurnDirection.RIGHT: (2, DriveAction.SPIN_RIGHT),
}


@dataclass
class TurnEstimate:
    """
    Angle estimate returned by the turn behavior.

    'angle' is the classified grid turn used by the brain/tracker.
    'gyro_angle' comes from integrating angular velocity during the turn.
    'timed_angle' comes from the nearest spin-time calibration table spot.
    """
    angle: float
    elapsed: float
    gyro_angle: Optional[float] = None
    timed_angle: float = 0.0

    @property
    def degrees(self):
        return math.degrees(self.angle)


def classify_turn_angle(direction, angle_radians):
    """
    Snap a turn angle to one legal grid class.
    """
    magnitude_degrees = abs(math.degrees(angle_radians))
    angle_degrees = min(
        param.turn_angle_classes_degrees,
        key=lambda candidate: abs(magnitude_degrees - candidate),
    )
    return math.radians(TURN_SIGNS[direction] * angle_degrees)


def estimate_turn_angle_from_time(direction, elapsed):
    """
    Classify turn angle from the nearest calibration table time.

    Positive angles are left turns, negative angles are right turns.
    """
    _, angle_degrees = min(
        param.turn_time_table[direction.name],
        key=lambda sample: abs(float(elapsed) - sample[0]),
    )
    return math.radians(TURN_SIGNS[direction] * angle_degrees)


def fuse_turn_angle(direction, gyro_angle, timed_angle):
    """
    Combine gyro integration with the time-table estimate, then classify it.

    The gyro is the main signal, while the table keeps the sign and magnitude
    sane if the gyro is noisy or unavailable. The returned angle is always one
    of the legal grid turn classes, never an arbitrary angle such as 20 deg.
    """
    if gyro_angle is None or not math.isfinite(gyro_angle):
        return timed_angle

    if abs(gyro_angle) < param.min_valid_gyro_angle:
        return timed_angle

    gyro_angle = TURN_SIGNS[direction] * abs(gyro_angle)

    fused_angle = param.gyro_blend * gyro_angle
    fused_angle += (1.0 - param.gyro_blend) * timed_angle
    return classify_turn_angle(direction, fused_angle)


class BasicBehaviors:
    """
    Robot body interface used by the brain.

    This layer owns GPIO and all lower hardware interfaces. The brain should
    call these behavior methods instead of touching motors or sensors directly.
    """
    def __init__(self):
        self.closed = False
        print("Setting up the GPIO...")
        self.io = pigpio.pi()
        if not self.io.connected:
            raise RuntimeError("Unable to connect to pigpio daemon")
        print("GPIO ready...")

        self.drive = DriveSystem(
            self.io,
            pwm_max=param.pwm_max,
            pwm_freq=param.pwm_freq,
        )
        self.line_sensor = LineSensor(
            self.io,
            param.pin_ir_left,
            param.pin_ir_middle,
            param.pin_ir_right,
        )
        self.gyro = Gyroscope(self.io)
        self.robot = RobotBehavior(self.drive, self.line_sensor, self.gyro)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is KeyboardInterrupt:
            print("\nCtrl+C received. Stopping robot and exiting...", flush=True)
            self.emergency_stop()
            return True

        self.shutdown()

    def follow_line(self):
        return self.robot.follow_line()

    def pull_forward(self):
        self.robot.pull_forward()

    def turn(self, direction):
        return self.robot.turn(direction)

    def turn_left(self):
        return self.turn(TurnDirection.LEFT)

    def turn_right(self):
        return self.turn(TurnDirection.RIGHT)

    def stop(self):
        self.drive.stop()

    def brake(self):
        self.drive.brake()

    def emergency_stop(self):
        if not self.closed:
            try:
                self.brake()
                self.stop()
            finally:
                self.closed = True

    def shutdown(self):
        if not self.closed:
            print("Turning off...", flush=True)
            self.brake()
            self.stop()
            self.io.stop()
            self.closed = True


LINE_ACTIONS = {
    (WHITE, WHITE, WHITE): None,
    (WHITE, BLACK, WHITE): DriveAction.STRAIGHT,
    (BLACK, BLACK, WHITE): DriveAction.TURN_LEFT,
    (BLACK, WHITE, WHITE): DriveAction.HOOK_LEFT,
    (WHITE, BLACK, BLACK): DriveAction.TURN_RIGHT,
    (WHITE, WHITE, BLACK): DriveAction.HOOK_RIGHT,
    (BLACK, BLACK, BLACK): DriveAction.STRAIGHT,
    (BLACK, WHITE, BLACK): DriveAction.STRAIGHT,
}


def choose_line_action(left, middle, right):
    """
    Map the current 3-sensor reading to the existing drive action table.

    Returns:
        A 'DriveAction', or 'None' when the sensors read '000' and the caller
        should fall back to the side estimator.
    """
    return LINE_ACTIONS[(left, middle, right)]


class RobotBehavior:
    """
    Own the robot's movement behaviors in one initialized object.

    API:
        behavior = RobotBehavior(drive, sensor, gyro)
        result = behavior.follow_line()
        behavior.pull_forward()
        turn_estimate = behavior.turn(TurnDirection.LEFT)

    Args:
        drive: 'DriveSystem' used to send motion commands.
        sensor: 'LineSensor' used to read the three IR sensors.
        gyro: Optional 'Gyroscope' used during turns.

    Tunable values come from 'constants.param'
    """
    def __init__(self, drive, sensor, gyro=None):
        self.drive = drive
        self.sensor = sensor
        self.gyro = gyro
        self.recovery_spin_level = float(param.recovery_spin_level)
        self.pull_forward_duration = float(param.pull_forward_duration)

        self.intersection_detector = BinaryDetector(
            param.intersection_time_constant,
            threshold=param.binary_threshold,
            initial_state=False,
        )
        self.end_detector = BinaryDetector(
            param.end_time_constant,
            threshold=param.binary_threshold,
            initial_state=False,
        )
        self.side_estimator = SideEstimator(
            param.side_time_constant,
            side_threshold=param.side_threshold,
            center_threshold=param.center_threshold,
            initial_state=SideState.CENTER,
            initial_level=0.0,
        )

        self.depart_detector = BinaryDetector(
            param.turn_depart_time_constant,
            threshold=param.turn_threshold,
            initial_state=False,
        )
        self.street_detector = BinaryDetector(
            param.turn_street_time_constant,
            threshold=param.turn_threshold,
            initial_state=False,
        )

    def reset_line_following(self):
        """
        Reset all detector state before starting a fresh line-following run.
        """
        self.intersection_detector.reset(initial_state=False, initial_level=0.0)
        self.end_detector.reset(initial_state=False, initial_level=0.0)
        self.side_estimator.reset(initial_state=SideState.CENTER, initial_level=0.0)

    def _recovery_action(self):
        """
            When robot sees a (0,0,0) it should identify pothole or street end
            When robot diviates from street, it try to recover to center position
        """
        if self.side_estimator.level <= -self.side_estimator.center_threshold:
            if abs(self.side_estimator.level) >= self.recovery_spin_level:
                return DriveAction.SPIN_RIGHT
            return DriveAction.HOOK_RIGHT

        if self.side_estimator.level >= self.side_estimator.center_threshold:
            if abs(self.side_estimator.level) >= self.recovery_spin_level:
                return DriveAction.SPIN_LEFT
            return DriveAction.HOOK_LEFT

        return DriveAction.STRAIGHT

    def _end_signal(self, reading):
        """
        Return the raw end-of-street evidence for this loop iteration.
        """
        if abs(self.side_estimator.level) > self.side_estimator.center_threshold:
            return 0.0
        return 1.0 if reading == (WHITE, WHITE, WHITE) else 0.0

    def follow_line(self):
        """
        Follow the current street until an exit condition is reached.

        Returns:
            'LineFollowerResult.INTERSECTION' when a valid intersection is
            detected.
            'LineFollowerResult.END_OF_STREET' when the road truly ends ahead.
        """
        self.reset_line_following()

        tlast = time.time()

        try:
            while True:
                reading = self.sensor.read()

                tnow = time.time()
                dt = tnow - tlast
                tlast = tnow

                self.side_estimator.update(reading, dt)
                intersection_raw = 1.0 if reading == (BLACK, BLACK, BLACK) else 0.0
                found_intersection = self.intersection_detector.update(
                    intersection_raw,
                    dt,
                )

                end_raw = self._end_signal(reading)
                found_end = self.end_detector.update(end_raw, dt)

                if found_intersection:
                    return LineFollowerResult.INTERSECTION

                if found_end:
                    return LineFollowerResult.END_OF_STREET

                action = choose_line_action(*reading)
                if action is None:
                    action = self._recovery_action()
                self.drive.drive(action)
        finally:
            self.drive.stop()

    def pull_forward(self):
        """
        Drive straight until the configured time expires, then stop.
        """
        t0 = time.time()
        try:
            while time.time() < t0 + self.pull_forward_duration:
                self.drive.drive(DriveAction.STRAIGHT)
        finally:
            self.drive.stop()

    def turn(self, direction):
        """
        Execute a left or right spin until the next street is acquired.

        Args:
            direction: 'TurnDirection.LEFT' or 'TurnDirection.RIGHT'.

        Returns:
            A 'TurnEstimate' containing the gyro-integrated angle, table-based
            angle, elapsed time, and fused angle.
        """
        leading_index, spin_action = TURN_CONFIG[direction]

        self.depart_detector.reset(initial_state=False, initial_level=0.0)
        self.street_detector.reset(initial_state=False, initial_level=0.0)

        gyro_angle = None
        if self.gyro is not None:
            self.drive.stop()
            self.gyro.resetzero()
            gyro_angle = 0.0

        start_time = time.time()
        tlast = start_time
        leaving_street = False

        try:
            while True:
                self.drive.drive(spin_action)
                reading = self.sensor.read()
                omega = self.gyro.read() if self.gyro is not None else None

                tnow = time.time()
                dt = tnow - tlast
                tlast = tnow

                if omega is not None:
                    gyro_angle += dt * omega

                leading = reading[leading_index]
                middle = reading[1]

                if not leaving_street:
                    departed = (leading == WHITE and middle == WHITE)
                    leaving_street = self.depart_detector.update(
                        1.0 if departed else 0.0,
                        dt,
                    )

                if leaving_street:
                    found_next_street = (leading == BLACK and middle == BLACK)
                    if self.street_detector.update(
                        1.0 if found_next_street else 0.0,
                        dt,
                    ):
                        elapsed = tnow - start_time
                        timed_angle = estimate_turn_angle_from_time(direction, elapsed)
                        angle = fuse_turn_angle(direction, gyro_angle, timed_angle)
                        return TurnEstimate(
                            angle=angle,
                            elapsed=elapsed,
                            gyro_angle=gyro_angle,
                            timed_angle=timed_angle,
                        )
        finally:
            self.drive.stop()

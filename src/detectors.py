#!/usr/bin/env python3
from enum import Enum, auto

BLACK = 1
WHITE = 0


class SideState(Enum):
    LEFT = auto()
    CENTER = auto()
    RIGHT = auto()


LEFT = SideState.LEFT
CENTER = SideState.CENTER
RIGHT = SideState.RIGHT


def _clamp(value, low, high):
    """
        bisectionally limit the result between low and high
    """
    return max(low, min(high, value))


class BinaryDetector:
    """
    First-order low-pass detector with binary hysteresis.

    API:
        detector = BinaryDetector(...)
        detector.reset(...)
        state = detector.update(raw, dt)

    Args:
        time_constant: Averaging time in seconds. Larger values reject more
            noise but react more slowly.
        threshold: ON threshold applied to the filtered level. OFF uses the
            mirrored value (1 - threshold) for hysteresis.
        initial_state: Initial boolean output state.
        initial_level: Optional initial filtered level in [0, 1]. If omitted,
            it starts at 1.0 for True and 0.0 for False.
    """
    def __init__(self, time_constant, threshold=0.632, initial_state=False, initial_level=None):
        self.time_constant = float(time_constant)
        self.threshold = float(threshold)
        self.reset(initial_state=initial_state, initial_level=initial_level)

    def reset(self, initial_state=False, initial_level=None):
        """
        Reinitialize the detector state and filtered internal level.
        """
        self.state = bool(initial_state)
        if initial_level is None:
            self.level = 1.0 if self.state else 0.0
        else:
            self.level = _clamp(float(initial_level), 0.0, 1.0)

    def update(self, raw, dt):
        """
        Advance the detector by one loop step.

        Args:
            raw: Instantaneous guess in [0, 1].
            dt: Time step since the previous loop iteration, in seconds.

        Returns:
            The boolean detector output after filtering and hysteresis.
        """
        raw = _clamp(float(raw), 0.0, 1.0)
        alpha = 1.0 if dt <= 0.0 else min(1.0, dt / self.time_constant)
        self.level += alpha * (raw - self.level)

        if self.level > self.threshold:
            self.state = True
        elif self.level < 1.0 - self.threshold:
            self.state = False

        return self.state


class SideEstimator:
    """
    First-order low-pass estimator with ternary hysteresis.

    API:
        estimator = SideEstimator(...)
        estimator.reset(...)
        raw = estimator.raw_guess(reading)
        side = estimator.update(reading, dt)

    Output states:
        `SideState.LEFT`, `SideState.CENTER`, `SideState.RIGHT`

    State meaning:
        The state describes where the robot is relative to the black street,
        not which sensor currently sees black.
        `SideState.RIGHT` means the robot has drifted right of the street and
        should steer left.
        `SideState.LEFT` means the robot has drifted left of the street and
        should steer right.

    Args:
        time_constant: Averaging time in seconds for side estimation.
        side_threshold: Magnitude required to commit to LEFT or RIGHT.
        center_threshold: Magnitude below which the estimate snaps back to
            CENTER.
        initial_state: Initial discrete side output.
        initial_level: Optional initial filtered level in [-1, 1].
    """
    def __init__(
        self,
        time_constant,
        side_threshold=0.35,
        center_threshold=0.15,
        initial_state=SideState.CENTER,
        initial_level=0.0,
    ):
        self.time_constant = float(time_constant)
        self.side_threshold = float(side_threshold)
        self.center_threshold = float(center_threshold)
        self.reset(initial_state=initial_state, initial_level=initial_level)

    def reset(self, initial_state=SideState.CENTER, initial_level=0.0):
        """
        Reinitialize the estimator state and filtered side level.
        """
        if initial_state not in (SideState.LEFT, SideState.CENTER, SideState.RIGHT):
            raise ValueError("invalid initial_state")

        self.state = initial_state
        self.level = _clamp(float(initial_level), -1.0, 1.0)

    @staticmethod
    def raw_guess(reading):
        """
        Convert one 3-sensor reading into a signed side guess.

        Assumes the street itself is black. The side guess is the negated
        lateral centroid of the sensors that currently still see black.

        Returns:
            `None`: no sensor currently sees black, so this sample carries no
                side information
            -1.0 / -0.5: robot is left of the street and should move right
             0.0: robot is centered on the black street
             0.5 / 1.0: robot is right of the street and should move left

        Examples:
            `(BLACK, WHITE, WHITE)` -> `+1.0`
                The left sensor still sees the black street, so the robot has
                drifted right and should steer left.
            `(WHITE, WHITE, BLACK)` -> `-1.0`
                The right sensor still sees the black street, so the robot has
                drifted left and should steer right.
        """
        left, middle, right = reading

        black_positions = []
        if left == BLACK:
            black_positions.append(1.0)
        if middle == BLACK:
            black_positions.append(0.0)
        if right == BLACK:
            black_positions.append(-1.0)

        if not black_positions:
            return None

        return sum(black_positions) / len(black_positions)

    def update(self, reading, dt):
        """
        Advance the estimator by one loop step from a 3-sensor reading.

        Args:
            reading: Tuple (left, middle, right) of raw IR values.
            dt: Time step since the previous loop iteration, in seconds.

        Returns:
            One of `SideState.LEFT`, `SideState.CENTER`, or `SideState.RIGHT`.
        """
        raw = self.raw_guess(reading)
        if raw is not None:
            alpha = 1.0 if dt <= 0.0 else min(1.0, dt / self.time_constant)
            self.level += alpha * (raw - self.level)
            # Restrict level between -1 and 1
            self.level = _clamp(self.level, -1.0, 1.0)

        if self.level >= self.side_threshold:
            self.state = SideState.RIGHT
        elif self.level <= -self.side_threshold:
            self.state = SideState.LEFT
        elif abs(self.level) <= self.center_threshold:
            self.state = SideState.CENTER

        return self.state

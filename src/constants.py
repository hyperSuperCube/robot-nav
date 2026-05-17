#!/usr/bin/env python3
import math


class Parameters:
    """
    Shared robot parameters.

    Import `param` and read values directly as `param.<name>`.
    No constructor call is needed.
    """
    pin_ir_left = 14
    pin_ir_middle = 15
    pin_ir_right = 18

    pin_ultrasound_left_trigger = 13
    pin_ultrasound_middle_trigger = 19
    pin_ultrasound_right_trigger = 26
    pin_ultrasound_left_echo = 16
    pin_ultrasound_middle_echo = 20
    pin_ultrasound_right_echo = 21

    ultrasound_speed_of_sound = 343.0
    ultrasound_min_trigger_interval = 0.040
    ultrasound_read_interval = 0.050

    pwm_max = 255
    pwm_freq = 1000
    map_filename = "mymap.pickle"
    assume_no_45_adjacent_streets = True

    turn_time_table = {
        "LEFT": (
            (0.462, 45.0),
            (0.788, 90.0),
            (1.101, 135.0),
            (1.576, 180.0),
            (2.438, 270.0),
            (3.068, 360.0),
        ),
        "RIGHT": (
            (0.546, 45.0),
            (0.820, 90.0),
            (1.131, 135.0),
            (1.584, 180.0),
            (2.313, 270.0),
            (3.094, 360.0),
        ),
    }

    gyro_blend = 0.80
    min_valid_gyro_angle = math.radians(3.0)
    turn_angle_classes_degrees = (45.0, 90.0, 135.0, 180.0, 270.0, 360.0)

    intersection_time_constant = 0.08
    end_time_constant = 0.15
    side_time_constant = 0.1
    binary_threshold = 0.632
    side_threshold = 0.35
    center_threshold = 0.15
    recovery_spin_level = 0.75
    pull_forward_duration = 0.45
    turn_depart_time_constant = 0.015
    turn_street_time_constant = 0.015
    turn_threshold = 0.632

    proximity_clear_distance = 0.20
    proximity_close_distance = 0.10

    wall_follow_nominal_distance = 0.30
    wall_follow_max_error = 0.10
    wall_follow_small_error = 0.02
    wall_follow_medium_error = 0.05
    wall_follow_large_error = 0.08
    wall_follow_front_stop_distance = 0.20

    wall_follow_pwm_left_offset = 112.43
    wall_follow_pwm_left_gain = -273.45
    wall_follow_pwm_right_offset = 109.71
    wall_follow_pwm_right_gain = 214.48
    wall_follow_test_duration = 4.0


param = Parameters

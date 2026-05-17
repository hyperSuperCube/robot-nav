#!/usr/bin/env python3
import math
import signal
import sys
import time
import traceback

import pigpio

from constants import param


TICK_ROLLOVER = 2 ** 32


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


class Ultrasound:
    """
    Interface for one ultrasonic distance sensor.

    API:
        sensor = Ultrasound(io, pintrig, pinecho)
        sensor.trigger()
        distance = sensor.read()
        time_of_flight = sensor.read_time_of_flight()

    The GPIO callback stores the most recent echo pulse. `read()` returns the
    last distance in meters, or `math.inf` before the first echo arrives.
    """
    def __init__(
        self,
        io,
        pintrig,
        pinecho,
        speed_of_sound=param.ultrasound_speed_of_sound,
        min_trigger_interval=param.ultrasound_min_trigger_interval,
    ):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho
        self.speed_of_sound = float(speed_of_sound)
        self.min_trigger_interval = float(min_trigger_interval)

        self.risetick = None
        self.echo_ticks = None
        self.time_of_flight = None
        self.distance = math.inf
        self.last_trigger_time = 0.0
        self.last_echo_time = None
        self.closed = False

        self.io.set_mode(self.pintrig, pigpio.OUTPUT)
        self.io.set_mode(self.pinecho, pigpio.INPUT)
        self.io.write(self.pintrig, 0)

        self.cbrise = self.io.callback(
            self.pinecho,
            pigpio.RISING_EDGE,
            self.rising,
        )
        self.cbfall = self.io.callback(
            self.pinecho,
            pigpio.FALLING_EDGE,
            self.falling,
        )

    def trigger(self):
        """
        Send one trigger pulse.

        Returns:
            True if a pulse was sent, False if the call came too soon after
            the previous trigger.
        """
        now = time.time()
        if now - self.last_trigger_time < self.min_trigger_interval:
            return False

        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 0)
        self.last_trigger_time = now
        return True

    def rising(self, pin, level, ticks):
        """
        Save the echo rising-edge tick for the later distance computation.
        """
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        """
        Convert the echo pulse width into a distance measurement.
        """
        if self.risetick is None:
            return

        deltatick = ticks - self.risetick
        if deltatick < 0:
            deltatick += TICK_ROLLOVER

        self.echo_ticks = deltatick
        self.time_of_flight = deltatick / 1_000_000.0
        self.distance = 0.5 * self.speed_of_sound * self.time_of_flight
        self.last_echo_time = time.time()

    def read(self):
        """
        Return the latest distance measurement in meters.
        """
        return self.distance

    def read_time_of_flight(self):
        """
        Return the latest echo pulse width in seconds.
        """
        return self.time_of_flight

    def cancel(self):
        """
        Remove GPIO callbacks when shutting down.
        """
        if self.closed:
            return
        self.cbrise.cancel()
        self.cbfall.cancel()
        self.closed = True


class ProximitySensor:
    """
    Combine the three ultrasonic sensors into one robot-level object.

    API:
        sensor = ProximitySensor(io)
        sensor.trigger()
        sensor.read() -> (left, middle, right) distances in meters
    """
    def __init__(
        self,
        io,
        left_trigger=param.pin_ultrasound_left_trigger,
        left_echo=param.pin_ultrasound_left_echo,
        middle_trigger=param.pin_ultrasound_middle_trigger,
        middle_echo=param.pin_ultrasound_middle_echo,
        right_trigger=param.pin_ultrasound_right_trigger,
        right_echo=param.pin_ultrasound_right_echo,
    ):
        self.left = Ultrasound(io, left_trigger, left_echo)
        self.middle = Ultrasound(io, middle_trigger, middle_echo)
        self.right = Ultrasound(io, right_trigger, right_echo)
        self.sensors = (self.left, self.middle, self.right)
        self.closed = False

    def trigger(self):
        """
        Trigger all three ultrasonic sensors.
        """
        return tuple(sensor.trigger() for sensor in self.sensors)

    def read(self):
        """
        Return the latest distances as (left, middle, right), in meters.
        """
        return tuple(sensor.read() for sensor in self.sensors)

    def read_time_of_flight(self):
        """
        Return latest echo pulse widths as (left, middle, right), in seconds.
        """
        return tuple(sensor.read_time_of_flight() for sensor in self.sensors)

    def cancel(self):
        """
        Remove all GPIO callbacks.
        """
        if self.closed:
            return
        for sensor in self.sensors:
            sensor.cancel()
        self.closed = True


def test():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        sys.exit(1)
    print("GPIO ready...")

    sensor = None
    install_ctrl_c_shutdown(lambda: [sensor, io])

    try:
        sensor = ProximitySensor(io)
        while True:
            sensor.trigger()
            time.sleep(param.ultrasound_read_interval)

            distances = sensor.read()
            times = sensor.read_time_of_flight()
            print(
                "TOF = (%8s, %8s, %8s)  Distances = (%6.3fm, %6.3fm, %6.3fm)"
                % (
                    _format_time_of_flight(times[0]),
                    _format_time_of_flight(times[1]),
                    _format_time_of_flight(times[2]),
                    distances[0],
                    distances[1],
                    distances[2],
                )
            )

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        print("Turning off...")
        shutdown_resources([sensor, io])


def _format_time_of_flight(value):
    if value is None:
        return "none"
    return "%6.1fus" % (value * 1_000_000.0)


def main():
    test()


if __name__ == "__main__":
    main()

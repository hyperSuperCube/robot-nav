import math
import signal
import sys
import time

import drivesystem as ds
import pigpio
from sensors import ADC


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


TURN_ACTION = ds.DriveAction.SPIN_LEFT
RUN_TIME = 4
TURN_ANGLE = 2 * math.pi
PLOT_FILENAME = "turn_adc_readings.png"
OFFSET = 133
KNOWN_OMEGA = TURN_ANGLE / RUN_TIME


def plot_adc_readings(sample_times, adc_readings):
    if not adc_readings:
        print("No ADC readings captured.")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed, so no plot was created.")
        return

    plt.figure()
    plt.plot(sample_times, adc_readings)
    plt.xlabel("Time (s)")
    plt.ylabel("ADC reading")
    plt.title("ADC Reading While Turning")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(PLOT_FILENAME)
    print("Saved plot to %s" % PLOT_FILENAME)
    plt.show()


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        return

    drive = None
    install_ctrl_c_shutdown(lambda: [drive, io])
    drive = ds.DriveSystem(io, pwm_max=255, pwm_freq=1000)
    adc = ADC(io)

    try:
        sample_times = []
        adc_readings = []

        print("Turning and recording ADC for %.1f seconds..." % RUN_TIME)
        start_time = time.perf_counter()
        drive.drive(TURN_ACTION)

        while time.perf_counter() < start_time + RUN_TIME:
            now = time.perf_counter()
            sample_times.append(now - start_time)
            adc_readings.append(adc.read())

        elapsed = time.perf_counter() - start_time
        drive.stop()
        print("Turn time: %.3f seconds" % elapsed)
        print("Known omega: %.6f rad/sec" % KNOWN_OMEGA)
        print("Captured %d ADC readings." % len(adc_readings))
        if adc_readings:
            average_adc = sum(adc_readings) / len(adc_readings)
            adc_delta = average_adc - OFFSET

            print("ADC min/max: %d / %d" % (min(adc_readings), max(adc_readings)))
            print("ADC average over full turn: %.3f" % average_adc)
            print("ADC offset: %d" % OFFSET)

            if adc_delta == 0:
                print("Scale cannot be computed because average equals offset.")
            else:
                scale = KNOWN_OMEGA / adc_delta
                print("Scale: %.6f rad/sec per ADC count" % scale)
        plot_adc_readings(sample_times, adc_readings)

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        shutdown_resources([drive, io])


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import signal
import sys
import time

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


def main():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        return
    print("GPIO ready...")
    install_ctrl_c_shutdown(lambda: [io])

    adc = ADC(io)

    try:
        while True:
            print(adc.read())
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Turning off...")
        shutdown_resources([io])


if __name__ == "__main__":
    main()

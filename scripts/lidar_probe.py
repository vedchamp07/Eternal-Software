#!/usr/bin/env python3
"""Raw RPLIDAR A1 protocol probe (no ROS, no Slamtec SDK).

Bypasses rplidar_ros to isolate device-vs-driver faults from first principles.
Opens the port at 115200 8N1, asserts DTR (motor/control line), then speaks the
Slamtec protocol directly: GET_HEALTH (0xA5 0x52) and START_SCAN (0xA5 0x20).

Usage:
    python3 lidar_probe.py [/dev/ttyUSB0]

How to read the output:
    - `health_desc: a5 5a 03 00 00 00 06` + `health_data: 00 00 00`
      -> UART path + SoC alive and healthy.
    - `scan_desc: a5 5a 05 00 00 40 81` + `pkt:` lines with bytes
      -> ranging core streams data. Lidar is GOOD; suspect driver/ROS side.
    - Valid descriptors but EMPTY `pkt:` lines while the head spins
      -> ranging core (laser / detector / head data path) produces nothing.
      Hardware fault: laser-glow test with phone camera, head flat-cable,
      swap USB cable, bench-test on a laptop.
    - Empty `health_desc`
      -> device silent. Check port mapping (/dev/serial/by-id), DTR control,
      competing holders (`fuser`), ModemManager interference.

Diagnosed 2026-09-28: valid handshake + zero data bytes = dead ranging core.
Requires pyserial (`pip install pyserial`).
"""
import serial
import sys
import time

READ_TIMEOUT = 1.0
N_PACKETS = 12


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    s = serial.Serial(port, 115200, timeout=READ_TIMEOUT)
    s.dtr = True  # A1 motor/control line; device is silent without it
    time.sleep(2.5)  # motor spin-up
    s.reset_input_buffer()

    s.write(bytes([0xA5, 0x52]))  # GET_HEALTH
    d = s.read(7)
    print('health_desc:', d.hex(' '), flush=True)
    if len(d) == 7 and d[0] == 0xA5 and d[1] == 0x5A:
        length = d[2] | (d[3] << 8) | (d[4] << 16) | (d[5] << 24)
        print('health_data:', s.read(length).hex(' '), flush=True)

    s.reset_input_buffer()
    s.write(bytes([0xA5, 0x20]))  # START_SCAN
    d = s.read(7)
    print('scan_desc:', d.hex(' '), flush=True)
    if len(d) == 7 and d[0] == 0xA5 and d[1] == 0x5A:
        print('SCAN DESCRIPTOR OK - reading packets', flush=True)
        for _ in range(N_PACKETS):
            print('pkt:', s.read(5).hex(' '), flush=True)

    s.write(bytes([0xA5, 0x25]))  # STOP
    s.dtr = False
    s.close()
    print('DONE', flush=True)


if __name__ == '__main__':
    main()

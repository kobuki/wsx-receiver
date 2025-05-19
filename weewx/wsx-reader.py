#!/usr/bin/env python3

# WeeWx 5.1+ is required. Copy to: /etc/weewx/bin/user/wsx-reader.py

import sys
import serial
import json
from datetime import datetime
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="WeeWx serial handler companion for the wsx-receiver firmware")
    parser.add_argument('--device', default='/dev/ttyS0', help='Serial device (default: /dev/ttyS0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    return parser.parse_args()

def main():
    args = parse_args()

    bytesize, parity, stopbits = serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE

    try:
        ser = serial.Serial(
            port=args.device,
            baudrate=args.baud,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}", file=sys.stderr)
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            try:
                data = json.loads(line)
                if isinstance(data, dict):
                    data['time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    print(json.dumps(data, separators=(',', ':')), flush=True)
            except json.JSONDecodeError:
                continue
        except KeyboardInterrupt:
            print("\nExiting.")
            break

if __name__ == '__main__':
    main()

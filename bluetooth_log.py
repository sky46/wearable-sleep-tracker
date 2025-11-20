import time
import datetime
import csv
from pathlib import Path

import serial

BT_COM_PORT = 'COM4'
BAUD_RATE = 38400
# Prints and clears buffer every time these bytes are read (print excludes EOL bytes themselves)
EOL_BYTES = b'\r\n' 

def open_log_file(filename: str):
    """Open file and write CSV header line if file empty."""
    Path(filename).touch(exist_ok=True)
    log_file = open(filename, 'r+', newline='')
    if (log_file.readline() == ""):
        log_writer = csv.writer(log_file)
        log_writer.writerow(['timestamp', 'accel_id'])
    return log_file

def log_line(read_line: bytes, csv_writer):
    csv_writer.writerow([
        datetime.datetime.now().isoformat(timespec='seconds'),
        read_line.decode('utf-8')
    ])
    print([
        datetime.datetime.now().isoformat(timespec='seconds'),
        read_line.decode('utf-8')
    ])

with (
    serial.Serial(BT_COM_PORT, BAUD_RATE) as ser,
    open_log_file('periodic_log.csv') as log_file
):
    log_writer = csv.writer(log_file)
    read_line = bytearray()
    while True:
        if ser.in_waiting > 0:
            read_bytes = ser.read(ser.in_waiting)
            eol_bytes_i = read_bytes.find(EOL_BYTES)
            while eol_bytes_i != -1:
                read_line.extend(read_bytes[:eol_bytes_i])
                log_line(read_line, log_writer)
                read_bytes = read_bytes[eol_bytes_i+len(EOL_BYTES):]
                read_line = bytearray()
                eol_bytes_i = read_bytes.find(EOL_BYTES)
            else:
                read_line.extend(read_bytes)
        time.sleep(0.01)

import serial
import time

BT_COM_PORT = 'COM4'
BAUD_RATE = 9600
# Prints and clears buffer every time these bytes are read (print excludes EOL bytes themselves)
EOL_BYTES = b'\n' 

with serial.Serial(BT_COM_PORT, BAUD_RATE) as ser:
    read_line = bytearray()
    while True:
        if ser.in_waiting > 0:
            read_bytes = ser.read(ser.in_waiting)
            eol_bytes_i = read_bytes.find(EOL_BYTES)
            while eol_bytes_i != -1:
                read_line.extend(read_bytes[:eol_bytes_i])
                print(read_line.decode('utf-8'))
                read_bytes = read_bytes[eol_bytes_i+1:]
                read_line = bytearray()
                eol_bytes_i = read_bytes.find(EOL_BYTES)
            else:
                read_line.extend(read_bytes)
        time.sleep(0.01)

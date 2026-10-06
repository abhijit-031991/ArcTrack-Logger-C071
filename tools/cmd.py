#!/usr/bin/env python3
# Bench helper for the logger's binary protocol.
#   python tools/cmd.py <PORT> <REQUEST_CODE>              -> send a 3-byte reqPing
#   python tools/cmd.py <PORT> set <freq> <tout> <hdop> <minsat>  -> send a settings frame
#
# Codes: 51 ALL, 50 NEW, 66 MEMORY_STATUS, 40 REQUEST_SETTINGS,
#        5 CALIBRATION_SUMMARY, 63 MEMORY_CLEAR, 3 FINISH, 200 REQ_DFU
# Close the serial monitor first (it holds the COM port).
import sys, time, struct, serial

TAG  = 10606
port = sys.argv[1] if len(sys.argv) > 1 else "COM45"
s = serial.Serial(port, 115200, timeout=0.2)
s.reset_input_buffer()

if len(sys.argv) > 2 and sys.argv[2] == "set":
    freq, tout, hdop, minsat = (int(sys.argv[i]) for i in (3, 4, 5, 6))
    # setttings: uint16 tag; int gpsFrq,gpsTout,hdop,minSat,radioFrq,startHour,endHour; bool scheduled
    frame = struct.pack("<H7iB", TAG, freq, tout, hdop, minsat, 0, 0, 0, 0)
    s.write(frame)
    print(f"sent settings freq={freq} tout={tout} hdop={hdop} minSat={minsat}  ({frame.hex(' ')})")
else:
    code = int(sys.argv[2]) if len(sys.argv) > 2 else 66
    frame = struct.pack("<HB", TAG, code)
    s.write(frame)
    print(f"sent {frame.hex(' ')}  (tag={TAG}, request={code})")

t0 = time.time()
while time.time() - t0 < 6:
    line = s.readline()
    if line:
        print(line.decode(errors="replace").rstrip())
s.close()

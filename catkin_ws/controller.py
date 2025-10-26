import serial
import json
import time

ser = serial.Serial(
    '/dev/ttyACM1',  # change to your device (e.g. /dev/ttyS0, /dev/ttyACM0)
    baudrate=115200,
    timeout=1
)

def send_json(obj):
    data = json.dumps(obj) + '\n'
    ser.write(data.encode('utf-8'))
    ser.flush()
    print("→ Sent:", data.strip())

def read_json():
    ser.reset_input_buffer()
    line = ser.readline().decode('utf-8').strip()
    if not line:
        print("ret nan")
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        print("⚠️ Invalid JSON:", line)
        return None

try:
    st = False
    led_tmr = time.time()

    while True:
        if time.time() - led_tmr > 0.2:
            send_json({"T":132,"IO1":255 * int(st),"IO2":255 * int(st)})
            left_tmr = time.time() 
            st = not st

        send_json({"T":600})

        response = read_json()
        if response:
            print("← Received:", response)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()


import pygame
import serial
import time

# ===== SERIAL CONFIG =====
SERIAL_PORT = '/dev/ttyACM0'   # change if needed
BAUD_RATE = 115200

# ===== INIT SERIAL =====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # allow Arduino reset

# ===== INIT GAMEPAD =====
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Using controller: {joystick.get_name()}")

# ===== MAIN LOOP =====
try:
    while True:
        pygame.event.pump()

        # AXES (depends on controller)
        steering_axis = joystick.get_axis(0)   # left stick X
        throttle_axis = joystick.get_axis(1)   # left stick Y

        # ===== PROCESS INPUT =====
        # Steering: already -1 → 1
        steering = round(steering_axis, 2)

        # Throttle mapping to your Arduino logic
        if throttle_axis < -0.5:
            throttle = 1      # forward
        elif throttle_axis > 0.5:
            throttle = -1     # reverse
        else:
            throttle = 0      # stop

        # ===== SEND SERIAL =====
        msg = f"{throttle},{steering}\n"
        ser.write(msg.encode())

        ser.write(msg.encode())
        print(f"TX: {msg.strip()}")

        while ser.in_waiting:
            print(ser.readline().decode(errors='ignore').strip())

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
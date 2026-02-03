from serial.tools import list_ports
from serial import Serial
import time
import numpy as np
from datetime import datetime

ARDUINO_VID = "2341"
SERIAL_BAUD = 921600

# Find and connect to Arduino
comport = None
port = None
ports = list_ports.comports()
for device in ports:
    if device.vid and "{:04X}".format(device.vid) == ARDUINO_VID:
        comport = device.name
        port = Serial(comport, SERIAL_BAUD, write_timeout=5)
        break

if not port:
    print("Arduino not found!")
    exit()

log_data = []
current_motor_speed = 0  # Track current motor speed (0-100)
motor_direction = 1      # 1 for forward, -1 for reverse

def present_logging_menu():    
    global current_motor_speed, motor_direction
    
    while True:
        print("\nAvailable commands:")
        print("s[0-100] - Set motor speed (forward)")
        print("r[0-100] - Set motor speed (reverse)")
        print("x - Stop motor")
        print("m - Start measurements (will prompt for speed) (Do not use)")
        print("z - Reset Z velocity")
        print("q - Stop measurements (Do not use)")
        print("l - Get last YPR (saves to file)")
        print("c - Calibrate IMU")
        print("\nEnter command:")
        user_in = input().strip().lower()
        
        if user_in.startswith(('s', 'r')) and len(user_in) > 1:
            try:
                speed = int(user_in[1:])
                if 0 <= speed <= 100:
                    current_motor_speed = speed
                    motor_direction = 1 if user_in[0] == 's' else -1
                    port.write(f"{user_in[0]}{speed}\n".encode())
                    print(f"Motor set to {speed}% {'forward' if user_in[0] == 's' else 'reverse'}")
                else:
                    print("Speed must be 0-100")
            except ValueError:
                print("Invalid speed value")
        elif user_in in ['x', 'z', 'q', 'c']:
            port.write(f"{user_in}\n".encode())
            if user_in == "c":
                wait_for_calibration_complete()
            elif user_in == "x":
                get_last_ypr()
                current_motor_speed = 0
                print("Motor stopped")
        elif user_in == "l":
            get_last_ypr()
        elif user_in == 'm':
            start_measurements_with_speed()
        else:
            print("Invalid command")

def get_last_ypr():
    """Send 'l' command to microcontroller and save the response to a file"""
    port.write(b'l')  # Send command to get last YPR
    
    # Wait for response
    start_time = time.time()
    timeout = 10.0  # 2 second timeout
    
    while time.time() - start_time < timeout:
        if port.in_waiting:
            line = port.readline().decode().strip()
            if line:
                if line.startswith('$'):
                    try:
                        parts = line[2:].split(',')
                        if len(parts) >= 3:
                            yaw = float(parts[0])
                            pitch = float(parts[1])
                            roll = float(parts[2])
                            
                            # Save to file
                            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                            filename = f"ypr_single_{current_motor_speed}p_{'fwd' if motor_direction == 1 else 'rev'}_{timestamp}.txt"
                            
                            with open(filename, 'w') as f:
                                f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                                f.write(f"Yaw: {yaw:.2f}°\n")
                                f.write(f"Pitch: {pitch:.2f}°\n")
                                f.write(f"Roll: {roll:.2f}°\n")
                            
                            print(f"Last YPR saved to {filename}")
                            print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
                            return
                    except ValueError as e:
                        print(f"Error parsing data: {e}")
                else:
                    print(line)  # Print any other messages from the microcontroller
    
    print("Timed out waiting for YPR data")

def start_measurements_with_speed():
    global current_motor_speed, motor_direction
    
    # Prompt for motor speed if not already set
    if current_motor_speed == 0:
        while True:
            try:
                speed = int(input("Enter motor speed (0-100): "))
                if 0 <= speed <= 100:
                    current_motor_speed = speed
                    direction = input("Direction (f for forward, r for reverse): ").lower()
                    motor_direction = 1 if direction == 'f' else -1
                    cmd = 's' if motor_direction == 1 else 'r'
                    port.write(f"{cmd}{speed}\n".encode())
                    break
                else:
                    print("Speed must be 0-100")
            except ValueError:
                print("Invalid speed value")
    
    print(f"\nStarting measurements at {current_motor_speed}% {'forward' if motor_direction == 1 else 'reverse'}")
    collect_measurements()

def wait_for_calibration_complete():
    print("Waiting for calibration to complete...")
    
    while True:
        if port.in_waiting:
            line = port.readline().decode().strip()
            if line:
                print(line)
                
                if "Starting calibration..." in line:
                    print("Calibration started")
                elif "Accelerometer calibration complete!" in line:
                    print("Accelerometer calibration complete")
                elif "Gyroscope calibration complete!" in line:
                    print("Gyroscope calibration complete")
                elif "DMP ready!" in line:
                    print("Calibration complete! DMP is ready")
                    return True
                elif "MPU6050 connection failed" in line:
                    print("Calibration failed - MPU6050 connection error")
                    return False

def collect_measurements():
    global log_data, current_motor_speed, motor_direction
    
    log_data = []
    yaw_rates = []
    prev_yaw = None
    prev_time = time.time()
    start = datetime.now()
    
    print("Collecting measurements... (press q to stop)")
    port.write(b'm\n')  # Start measurements on Arduino
    
    try:
        while True:
            if port.in_waiting:
                line = port.readline().decode().strip()
                if line:
                    if line.startswith('$'):
                        try:
                            current_time = time.time()
                            parts = line[2:].split(',')
                            if len(parts) >= 4:
                                yaw = float(parts[0])
                                pitch = float(parts[1])
                                roll = float(parts[2])
                                z_vel = float(parts[3])
                                
                                # Calculate yaw rate (degrees/second)
                                if prev_yaw is not None:
                                    dt = current_time - prev_time
                                    yaw_rate = (yaw - prev_yaw) / dt
                                    yaw_rates.append(yaw_rate)
                                
                                prev_yaw = yaw
                                prev_time = current_time
                                diff = (datetime.now()-start)
                                log_data.append((diff.seconds*1000+diff.microseconds/1000,yaw, pitch, roll, z_vel))
                                print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Z Vel: {z_vel:.2f} m/s")
                        except ValueError as e:
                            print(f"Error parsing data: {e}")
                    elif "Stop measure" in line:
                        break
                    else:
                        print(line)
                        
    except KeyboardInterrupt:
        port.write(b'q\n')
        port.write(b"x\n")
    finally:
        # Calculate and display average spin rate
        if len(yaw_rates) > 0:
            avg_yaw_rate = np.mean(yaw_rates)
            print(f"\nAverage spin rate: {abs(avg_yaw_rate):.2f}°/sec at {current_motor_speed}% power")
            print(f"Direction: {'clockwise' if (avg_yaw_rate * motor_direction) > 0 else 'counter-clockwise'}")
        
        print(f"Collected {len(log_data)} samples")
        if log_data:
            filename = f"imu_data_{current_motor_speed}p_{'fwd' if motor_direction == 1 else 'rev'}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w') as f:
                f.write("Time,Yaw (deg),Pitch (deg),Roll (deg),Z Velocity (m/s)\n")
                for sample in log_data:
                    f.write(f"{sample[0]},{sample[1]},{sample[2]},{sample[3]},{sample[4]}\n")
            print(f"Data saved to {filename}")

# Main communication loop
print("Searching for Arduino with MPU6050...")
while True:
    if port.in_waiting:
        line = port.readline().decode().strip()
        if line:
            print(line)
            
            if "MPU6050 connection successful" in line:
                print("\nFound MPU6050, starting calibration...")
                port.write(b'c\n')
                if wait_for_calibration_complete():
                    present_logging_menu()
                else:
                    print("Failed to calibrate IMU")
                    exit()
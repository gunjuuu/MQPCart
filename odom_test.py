import time
import sys
import csv
from basicmicro import Basicmicro
import matplotlib.pyplot as plt

# --- Configuration ---
COMPORT_NAME = "/dev/ttyS0"
BAUD_RATE = 38400

LEFT_RC_ADDR = 0x80
RIGHT_RC_ADDR = 0x81

MAX_QPPS = 2786
ACCEL_RATE = 12000 
TEST_SPEED = int(MAX_QPPS) # Target 40% max speed for the test

LOG_FILE = "odometry_data.csv"
PLOT_FILE = "odometry_plot.png"

def to_signed_32(val):
    """Converts a 32-bit unsigned integer from the RoboClaw back to a signed integer."""
    if val >= 2147483648: # If it's greater than or equal to 2^31
        return val - 4294967296 # Subtract 2^32 to wrap it back to negative
    return val

def set_chassis_velocity(rc, vx, vy, omega):
    """Translates chassis velocity into motor commands."""
    fl_desired = vx + vy + omega
    fr_desired = vx - vy - omega
    bl_desired = vx - vy + omega
    br_desired = vx + vy - omega

    # The Hardware Polarity Fix
    m1_80_qpps = int(bl_desired)
    m2_80_qpps = int(-fl_desired)
    m1_81_qpps = int(-fr_desired)
    m2_81_qpps = int(br_desired)

    try:
        rc.SpeedAccelM1M2(LEFT_RC_ADDR, ACCEL_RATE, m1_80_qpps, m2_80_qpps)
        rc.SpeedAccelM1M2(RIGHT_RC_ADDR, ACCEL_RATE, m1_81_qpps, m2_81_qpps)
    except Exception:
        pass

def stop_cart(rc):
    try:
        rc.SpeedAccelM1M2(LEFT_RC_ADDR, ACCEL_RATE, 0, 0)
        rc.SpeedAccelM1M2(RIGHT_RC_ADDR, ACCEL_RATE, 0, 0)
    except Exception:
        pass

if __name__ == "__main__":
    print("--- Mecanum Odometry Profiler (Patched) ---")
    
    rc = Basicmicro(COMPORT_NAME, BAUD_RATE)
    if not rc.Open():
        print("Fatal: Could not open serial port.")
        sys.exit(1)

    print("Test sequence starting in 3 seconds. Keep hands clear!")
    time.sleep(3)

    data_log = []
    start_time = time.time()
    
    print("Running Sequence: Forward -> Strafe -> Rotate")

    # --- Data Logging Loop ---
    while True:
        current_time = time.time() - start_time
        if current_time > 3.5:
            break
            
        # 1. Command the Test Sequence
        target_vx, target_vy, target_omega = 0, 0, 0
        if current_time < 1.0:
            target_vx = TEST_SPEED # Forward
        elif current_time < 2.0:
            target_vy = TEST_SPEED # Strafe Right
        elif current_time < 3.0:
            target_omega = TEST_SPEED # Rotate CW
        else:
            stop_cart(rc) # Brake at 6 seconds
            
        set_chassis_velocity(rc, target_vx, target_vy, target_omega)

        # 2. Read Actual Wheel Speeds
        try:
            # Reversing the polarity fix to get true wheel QPPS
            # Wrapped in to_signed_32() to fix the basicmicro unpacking bug
            raw_m2_80 = to_signed_32(rc.ReadSpeedM2(LEFT_RC_ADDR)[1])
            raw_m1_80 = to_signed_32(rc.ReadSpeedM1(LEFT_RC_ADDR)[1])
            raw_m1_81 = to_signed_32(rc.ReadSpeedM1(RIGHT_RC_ADDR)[1])
            raw_m2_81 = to_signed_32(rc.ReadSpeedM2(RIGHT_RC_ADDR)[1])
            
            fl_qpps = -raw_m2_80
            bl_qpps = raw_m1_80
            fr_qpps = -raw_m1_81
            br_qpps = raw_m2_81
            
            # 3. Calculate Forward Kinematics (Chassis Velocity)
            actual_vx = (fl_qpps + fr_qpps + bl_qpps + br_qpps) / 4.0
            actual_vy = (fl_qpps - fr_qpps - bl_qpps + br_qpps) / 4.0
            actual_omega = (fl_qpps - fr_qpps + bl_qpps - br_qpps) / 4.0

            data_log.append([
                current_time, target_vx, target_vy, target_omega, 
                actual_vx, actual_vy, actual_omega
            ])
            
        except Exception:
            pass 
            
        time.sleep(0.05)

    stop_cart(rc)
    print("Test complete. Motors stopped.")

    # --- Save to CSV ---
    print(f"Saving raw data to {LOG_FILE}...")
    with open(LOG_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time(s)", "Target_Vx", "Target_Vy", "Target_Omega", "Actual_Vx", "Actual_Vy", "Actual_Omega"])
        writer.writerows(data_log)

    # --- Generate Plot ---
    print(f"Generating graph at {PLOT_FILE}...")
    
    times = [row[0] for row in data_log]
    actual_vx = [row[4] for row in data_log]
    actual_vy = [row[5] for row in data_log]
    actual_omega = [row[6] for row in data_log]

    plt.figure(figsize=(12, 6))
    
    plt.plot(times, actual_vx, label='Linear X (Forward)', color='blue', linewidth=2)
    plt.plot(times, actual_vy, label='Linear Y (Strafe)', color='green', linewidth=2)
    plt.plot(times, actual_omega, label='Angular (Rotation)', color='red', linewidth=2)
    
    # Mark the transition phases
    plt.axvline(x=2.0, color='grey', linestyle='--', alpha=0.5)
    plt.axvline(x=4.0, color='grey', linestyle='--', alpha=0.5)
    plt.axvline(x=6.0, color='grey', linestyle='--', alpha=0.5)
    
    plt.text(1.0, TEST_SPEED*1.1, 'FORWARD', horizontalalignment='center')
    plt.text(3.0, TEST_SPEED*1.1, 'STRAFE', horizontalalignment='center')
    plt.text(5.0, TEST_SPEED*1.1, 'ROTATE', horizontalalignment='center')

    plt.title('Chassis Kinematics Profile')
    plt.xlabel('Time (Seconds)')
    plt.ylabel('Velocity (QPPS)')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    
    plt.savefig(PLOT_FILE, dpi=300, bbox_inches='tight')
    print("Done! You can now view the PNG file.")

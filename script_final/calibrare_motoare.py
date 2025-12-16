#!/usr/bin/env python3
import time
import sys
import json
import math
import threading
import smbus2
from gpiozero import OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

MOTOR1_STEP = 23  
MOTOR1_DIR = 24
MOTOR2_STEP = 25 
MOTOR2_DIR = 16

STEPS_PER_REV = 200
MICROSTEPS = 32
STEPS_FULL_REV = STEPS_PER_REV * MICROSTEPS
STEPS_PER_DEG = STEPS_FULL_REV / 360.0

ROLL_EXPANSION_FIX = 10.0 

I2C_BUS_0 = 0  
I2C_BUS_1 = 1 
AS5600_ADDR = 0x36
MPU6050_ADDR = 0x68

CALIBRATION_FILE = "system_calibration.json"
SPEED = 400
SETTLE_TIME = 1.0

class I2CManager:
    """Prevents data conflicts on I2C buses"""
    def __init__(self):
        self._locks = {
            0: threading.Lock(),
            1: threading.Lock()
        }
        self._buses = {}

    def get_bus(self, bus_num):
        if bus_num not in self._buses:
            self._buses[bus_num] = smbus2.SMBus(bus_num)
        return self._buses[bus_num]

    def read_byte(self, bus_num, addr, reg):
        with self._locks[bus_num]:
            return self.get_bus(bus_num).read_byte_data(addr, reg)

    def read_word(self, bus_num, addr, reg):
        with self._locks[bus_num]:
            return self.get_bus(bus_num).read_word_data(addr, reg)
            
    def write_byte(self, bus_num, addr, val):
        with self._locks[bus_num]:
            self.get_bus(bus_num).write_byte(addr, val)

    def write_reg(self, bus_num, addr, reg, val):
        with self._locks[bus_num]:
            self.get_bus(bus_num).write_byte_data(addr, reg, val)

i2c_mgr = I2CManager()


class MPU6050:
    """IMU Sensor for Gravity Reference"""
    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.bus_num = bus_num
        self.address = address
        self._init_sensor()

    def _init_sensor(self):
        try:
            # Wake up MPU6050
            i2c_mgr.write_reg(self.bus_num, self.address, 0x6B, 0)
        except Exception as e:
            print(f"IMU Init Error: {e}")

    def _read_word_2c(self, reg):
        try:
            high = i2c_mgr.read_byte(self.bus_num, self.address, reg)
            low = i2c_mgr.read_byte(self.bus_num, self.address, reg + 1)
            val = (high << 8) + low
            return val - 65536 if val >= 0x8000 else val
        except:
            return 0

    def get_orientation(self):
        """Returns (roll, pitch) in degrees"""
        ax = self._read_word_2c(0x3B) / 16384.0
        ay = self._read_word_2c(0x3D) / 16384.0
        az = self._read_word_2c(0x3F) / 16384.0
        
        try:
            pitch = math.atan2(ay, math.sqrt(ax*ax + az*az)) * 57.2958
            roll = math.atan2(-ax, az) * 57.2958
            return roll, pitch
        except:
            return 0.0, 0.0

class AS5600:
    """Magnetic Encoder with Expansion Fix"""
    def __init__(self, bus_num, address=AS5600_ADDR, expand_fix=0.0):
        self.bus_num = bus_num
        self.address = address
        self.expand_fix = expand_fix
        time.sleep(0.1)
        self.zero_position = self.read_raw_angle()
        
    def read_raw_angle(self):
        try:
            high = i2c_mgr.read_byte(self.bus_num, self.address, 0x0C)
            low = i2c_mgr.read_byte(self.bus_num, self.address, 0x0D)
            return (high << 8) | low
        except:
            return 0
            
    def get_angle_degrees(self):
        raw = self.read_raw_angle()
        return (raw * 360.0) / 4096.0
        
    def get_relative_angle(self):
        current = self.get_angle_degrees()
        zero_deg = (self.zero_position * 360.0 / 4096.0)
        
        relative = current - zero_deg

        while relative > 180.0: relative -= 360.0
        while relative < -180.0: relative += 360.0
        
        if abs(relative) > 1.0: 
            if relative > 0:
                relative += self.expand_fix 
            else:
                relative -= self.expand_fix 
                
        return relative

    def reset_zero(self):
        self.zero_position = self.read_raw_angle()

class StepperMotor:
    def __init__(self, step_pin, dir_pin, factory, name="Motor"):
        self.step_pin = OutputDevice(step_pin, pin_factory=factory)
        self.dir_pin = OutputDevice(dir_pin, pin_factory=factory)
        self.name = name
        self.current_step = 0
        
    def move_to_angle(self, target_angle, speed=SPEED):
        target_steps = int(target_angle * STEPS_PER_DEG)
        steps_to_move = target_steps - self.current_step
        
        if steps_to_move == 0: return
        
        if steps_to_move > 0:
            self.dir_pin.on()
        else:
            self.dir_pin.off()
            
        steps = abs(steps_to_move)
        delay = 1.0 / speed
        
        for _ in range(steps):
            self.step_pin.on()
            time.sleep(delay / 2)
            self.step_pin.off()
            time.sleep(delay / 2)
            
        self.current_step += steps_to_move

    def cleanup(self):
        self.step_pin.close()
        self.dir_pin.close()

def calibrate_axis(motor, encoder, axis_name, invert_readings=False):
    print(f"\n--- Calibrating {axis_name} ---")
    print(f"Expansion Fix: {encoder.expand_fix}°")
    
    print("Moving to 0°...")
    motor.move_to_angle(0)
    time.sleep(SETTLE_TIME)
    encoder.reset_zero()
    print(f"Zero Set. Encoder reads: {encoder.get_relative_angle():.2f}°")
    
    points = []
    test_angles = [-30, -15, 0, 15, 30]
    
    for target in test_angles:
        motor.move_to_angle(target)
        time.sleep(SETTLE_TIME)
        
        readings = []
        for _ in range(10):
            ang = encoder.get_relative_angle()
            readings.append(ang)
            time.sleep(0.02)
            
        avg_enc = sum(readings) / len(readings)
       
        if invert_readings: avg_enc = -avg_enc
        
        error = avg_enc - target
        print(f"Target: {target:3d}° | Encoder: {avg_enc:6.2f}° | Error: {error:5.2f}°")
        
        points.append({
            "target": target,
            "measured": avg_enc,
            "error": error
        })
        
    motor.move_to_angle(0)
    time.sleep(1.0)
    return points

def calibrate_imu_level(imu, motor1, motor2):
    print("\n--- IMU Level Calibration ---")
    print("Ensure robot is on a flat, stationary surface.")
    motor1.move_to_angle(0)
    motor2.move_to_angle(0)
    time.sleep(2.0)
    
    print("Sampling IMU (50 samples)...")
    rolls = []
    pitches = []
    
    for _ in range(50):
        r, p = imu.get_orientation()
        rolls.append(r)
        pitches.append(p)
        time.sleep(0.02)
        
    avg_roll = sum(rolls) / len(rolls)
    avg_pitch = sum(pitches) / len(pitches)
    
    print(f"Mechanical Zero results in:")
    print(f"  Roll Tilt:  {avg_roll:.2f}°")
    print(f"  Pitch Tilt: {avg_pitch:.2f}°")
    
    return {"roll_offset": avg_roll, "pitch_offset": avg_pitch}

# ==================== MAIN ====================
def main():
    print("="*60)
    print("SYSTEM CALIBRATION TOOL")
    print("="*60)
    
    factory = LGPIOFactory()
    enc_roll = AS5600(I2C_BUS_0, expand_fix=ROLL_EXPANSION_FIX)
    enc_pitch = AS5600(I2C_BUS_1, expand_fix=0.0)
    imu = MPU6050(I2C_BUS_1)
    mot_roll = StepperMotor(MOTOR1_STEP, MOTOR1_DIR, factory, "Roll")
    mot_pitch = StepperMotor(MOTOR2_STEP, MOTOR2_DIR, factory, "Pitch")
    
    try:
        roll_data = calibrate_axis(mot_roll, enc_roll, "Roll Axis", invert_readings=True)
        pitch_data = calibrate_axis(mot_pitch, enc_pitch, "Pitch Axis", invert_readings=True)
        imu_data = calibrate_imu_level(imu, mot_roll, mot_pitch)
        final_data = {
            "timestamp": time.time(),
            "roll_axis": {
                "expansion_fix": ROLL_EXPANSION_FIX,
                "inverted": True,
                "calibration_points": roll_data
            },
            "pitch_axis": {
                "expansion_fix": 0.0,
                "inverted": True,
                "calibration_points": pitch_data
            },
            "imu_level": imu_data
        }
        
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(final_data, f, indent=2)
            
    except KeyboardInterrupt:
        print("\nCancelled.")
    finally:
        mot_roll.cleanup()
        mot_pitch.cleanup()

if __name__ == "__main__":
    main()

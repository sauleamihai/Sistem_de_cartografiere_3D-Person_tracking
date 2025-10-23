#!/usr/bin/env python3

import lgpio
import smbus2
import time
import math
import json
import os

SERVO_PIN = 13
MPU6050_ADDR = 0x68

PULSE_MIN = 500
PULSE_MAX = 2500
PULSE_CENTER = 1500

CALIBRATION_FILE = "servo_calibration.json"

CALIBRATION_SETTLE = 0.3
ANGLE_TOLERANCE = 0.5

SWEEP_SPEED = 0.6
UPDATE_RATE = 0.05  # Redus de la 0.033 - mai lent pentru I2C


class MPU6050:
    
    def __init__(self, bus, address=0x68):
        self.bus = bus
        self.address = address
        self.max_retries = 3
        
        self._init_sensor()
        print("MPU6050 inițializat\n")
    
    def _init_sensor(self):
        try:
            # Wake up MPU6050
            self.bus.write_byte_data(self.address, 0x6B, 0)
            time.sleep(0.1)
        except OSError as e:
            print(f"⚠ Eroare inițializare MPU6050: {e}")
            time.sleep(0.5)
            # Retry
            self.bus.write_byte_data(self.address, 0x6B, 0)
            time.sleep(0.1)
    
    def _read_word_2c(self, reg, retry=True):
        for attempt in range(self.max_retries):
            try:
                high = self.bus.read_byte_data(self.address, reg)
                time.sleep(0.001)  # Delay mic între citiri
                low = self.bus.read_byte_data(self.address, reg + 1)
                
                value = (high << 8) + low
                return value - 65536 if value >= 0x8000 else value
                
            except OSError as e:
                if attempt < self.max_retries - 1:
                    time.sleep(0.05 * (attempt + 1))
                    
                    if attempt == self.max_retries - 2:
                        print(f"\n I2C error, reinițializare sensor...")
                        self._init_sensor()
                else:
                    print(f"\n I2C error persistent: {e}")
                   
                    return 0
        
        return 0
    
    def get_pitch(self):
              try:
            ax = self._read_word_2c(0x3B) / 16384.0
            time.sleep(0.002)  # Delay între citiri axe
            ay = self._read_word_2c(0x3D) / 16384.0
            time.sleep(0.002)
            az = self._read_word_2c(0x3F) / 16384.0
            
           
            if abs(ax) > 2.0 or abs(ay) > 2.0 or abs(az) > 2.0:
                print(f"\n⚠ Valori suspicioase: ax={ax:.2f} ay={ay:.2f} az={az:.2f}")
                return 0.0
            
            pitch = math.atan2(ax, math.sqrt(ay*ay + az*az)) * 57.3
            return pitch
            
        except Exception as e:
            print(f"\n⚠ Eroare citire pitch: {e}")
            return 0.0
    
    def get_stable_pitch(self, samples=10):
        readings = []
        for i in range(samples):
            pitch = self.get_pitch()
            readings.append(pitch)
            time.sleep(0.05)
        
        if len(readings) > 4:
            readings.sort()
            readings = readings[1:-1]  # Elimină extreme
        
        return sum(readings) / len(readings)


def set_servo_pulse(chip, pin, pulse_us):
    pulse_us = max(PULSE_MIN, min(PULSE_MAX, int(pulse_us)))
    duty = (pulse_us / 20000.0) * 100
    lgpio.tx_pwm(chip, pin, 50, duty)


def calibrate_angle(chip, pin, imu, target_angle, start_pulse=1500):
    print(f"\n→ Calibrare pentru target {target_angle:+.0f}°...")
    
    current_pulse = start_pulse
    set_servo_pulse(chip, pin, current_pulse)
    time.sleep(1.0)
    
    iteration = 0
    max_iterations = 200
    
    while iteration < max_iterations:
        actual_pitch = imu.get_stable_pitch(samples=5)
        error = target_angle - actual_pitch
        
        print(f"  Iter {iteration:3d}: Pulse={current_pulse:4.0f}μs | "
              f"IMU={actual_pitch:+6.2f}° | Error={error:+5.2f}°", end='\r')
        
        if abs(error) < ANGLE_TOLERANCE:
            print(f"\n Calibrat: Pulse={current_pulse:.0f}μs → IMU={actual_pitch:+.2f}°")
            return current_pulse, actual_pitch
        
        pulse_adjustment = error * 11.0
        max_adjustment = 20
        pulse_adjustment = max(-max_adjustment, min(max_adjustment, pulse_adjustment))
        
        current_pulse += pulse_adjustment
        current_pulse = max(PULSE_MIN, min(PULSE_MAX, current_pulse))
        
        set_servo_pulse(chip, pin, current_pulse)
        time.sleep(CALIBRATION_SETTLE)
        
        iteration += 1
    
    print(f"\n Calibrare nu a convergat după {max_iterations} iterații")
    actual_pitch = imu.get_stable_pitch()
    return current_pulse, actual_pitch


def run_calibration(chip, pin, imu):
    print("="*60)
    print("  FAZA DE CALIBRARE")
    print("="*60)
    print("\nServo-ul se va mișca FOARTE LENT pentru a găsi pozițiile reale.")
    print("Nu mișca platforma în timpul calibrării!\n")
    input("Apasă Enter pentru a începe calibrarea...")
    
    calibration = {}
    
    print("\n[1/3] Găsire poziție 0° (centru)...")
    pulse_0, angle_0 = calibrate_angle(chip, pin, imu, 0.0, start_pulse=1500)
    calibration['center'] = {
        'target': 0.0,
        'pulse': pulse_0,
        'actual': angle_0
    }
    time.sleep(1)
    
    print("\n[2/3] Găsire poziție +45°...")
    pulse_45, angle_45 = calibrate_angle(chip, pin, imu, 45.0, start_pulse=pulse_0 + 500)
    calibration['plus_45'] = {
        'target': 45.0,
        'pulse': pulse_45,
        'actual': angle_45
    }
    time.sleep(1)
    
    print("\n[3/3] Găsire poziție -45°...")
    pulse_m45, angle_m45 = calibrate_angle(chip, pin, imu, -45.0, start_pulse=pulse_0 - 500)
    calibration['minus_45'] = {
        'target': -45.0,
        'pulse': pulse_m45,
        'actual': angle_m45
    }
    
    print("\n Revenire la centru...")
    set_servo_pulse(chip, pin, pulse_0)
    time.sleep(1)
    
    print("\n" + "="*60)
    print("  REZULTATE CALIBRARE")
    print("="*60)
    print(f"\n{'Poziție':<15} {'Pulse (μs)':<12} {'IMU Real (°)':<15}")
    print("-" * 45)
    print(f"{'Centru (0°)':<15} {pulse_0:<12.0f} {angle_0:+14.2f}")
    print(f"{'Sus (+45°)':<15} {pulse_45:<12.0f} {angle_45:+14.2f}")
    print(f"{'Jos (-45°)':<15} {pulse_m45:<12.0f} {angle_m45:+14.2f}")
    print()
    
    return calibration


def save_calibration(calibration):
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration, f, indent=2)
    print(f"Calibrare salvată în {CALIBRATION_FILE}")


def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        return None
    
    with open(CALIBRATION_FILE, 'r') as f:
        calibration = json.load(f)
    
    print("Calibrare încărcată din fișier:")
    print(f"  0°:   {calibration['center']['pulse']:.0f}μs → {calibration['center']['actual']:+.2f}°")
    print(f"  +45°: {calibration['plus_45']['pulse']:.0f}μs → {calibration['plus_45']['actual']:+.2f}°")
    print(f"  -45°: {calibration['minus_45']['pulse']:.0f}μs → {calibration['minus_45']['actual']:+.2f}°")
    
    return calibration


class CalibratedServo:
    
    def __init__(self, chip, pin, calibration):
        self.chip = chip
        self.pin = pin
        self.cal = calibration
        
        self.slope = (calibration['plus_45']['pulse'] - calibration['minus_45']['pulse']) / 90.0
        self.intercept = calibration['center']['pulse']
        
        print(f"\n Servo calibrat:")
        print(f"  Slope: {self.slope:.2f} μs/°")
        print(f"  Intercept: {self.intercept:.0f} μs")
    
    def angle_to_pulse(self, angle):
        pulse = self.intercept + angle * self.slope
        return pulse
    
    def set_angle(self, angle):
        pulse = self.angle_to_pulse(angle)
        set_servo_pulse(self.chip, self.pin, pulse)
        return pulse


def run_sweep(chip, pin, imu, calibration):
    print("\n" + "="*60)
    print("  FAZA DE BALEIAJ")
    print("="*60)
    print("\nServoul va face baleiaj între +45° și -45° REALI.")
    print(f"Viteză: {SWEEP_SPEED}°/ciclu @ {1/UPDATE_RATE:.0f}Hz")
    print("Ctrl+C pentru stop\n")
    input("Apasă Enter pentru a începe baleiajul...")
    
    servo = CalibratedServo(chip, pin, calibration)
    
    target = 0.0
    direction = 1
    last_valid_pitch = 0.0
    error_count = 0
    
    try:
        while True:
            pulse = servo.set_angle(target)
            
            try:
                current_pitch = imu.get_pitch()
                
                if abs(current_pitch - last_valid_pitch) > 30 and last_valid_pitch != 0:
                    # Jump prea mare = probabil eroare
                    current_pitch = last_valid_pitch
                    error_count += 1
                else:
                    last_valid_pitch = current_pitch
                    error_count = 0
                
            except Exception as e:
                print(f"\n Eroare citire: {e}")
                current_pitch = last_valid_pitch
                error_count += 1
            
            if error_count > 10:
                print("\n Prea multe erori I2C consecutive. Oprire.")
                break
            
            error = target - current_pitch
            status = "!" if error_count > 0 else " "
            print(f"{status} Target: {target:+6.1f}° | "
                  f"IMU: {current_pitch:+6.1f}° | "
                  f"Error: {error:+5.2f}° | "
                  f"Pulse: {pulse:4.0f}μs",
                  end='  \r')
            
            target += direction * SWEEP_SPEED
            
            if target >= 45:
                direction = -1
                target = 45
                print()
            elif target <= -45:
                direction = 1
                target = -45
                print()
            
            time.sleep(UPDATE_RATE)
    
    except KeyboardInterrupt:
        print("\n\n Stop")


def main():
    print("="*60)
    print("  SISTEM CALIBRARE + BALEIAJ v3 (I2C Fix)")
    print("="*60)
    print("\nÎmbunătățiri:")
    print("I2C error handling și retry")
    print("Rate limiting pentru citiri")
    print("Fallback la ultima valoare validă\n")
    
    bus = smbus2.SMBus(1)
    h = lgpio.gpiochip_open(4)
    
    imu = MPU6050(bus)
    
    calibration = load_calibration()
    
    if calibration:
        response = input("\nFolosește calibrarea existentă? (da/nu): ").lower()
        if 'nu' in response or 'n' in response:
            calibration = None
    
    if not calibration:
        calibration = run_calibration(h, SERVO_PIN, imu)
        save_calibration(calibration)
        
        response = input("\nContinuă cu baleiajul? (da/nu): ").lower()
        if 'nu' in response or 'n' in response:
            set_servo_pulse(h, SERVO_PIN, calibration['center']['pulse'])
            time.sleep(0.3)
            lgpio.tx_pwm(h, SERVO_PIN, 50, 0)
            lgpio.gpiochip_close(h)
            bus.close()
            return
    
    try:
        run_sweep(h, SERVO_PIN, imu, calibration)
    finally:
        print("\nRevenire la centru calibrat...")
        set_servo_pulse(h, SERVO_PIN, calibration['center']['pulse'])
        time.sleep(0.5)
        lgpio.tx_pwm(h, SERVO_PIN, 50, 0)
        lgpio.gpiochip_close(h)
        bus.close()
        print("Cleanup complet")


if __name__ == "__main__":
    main()

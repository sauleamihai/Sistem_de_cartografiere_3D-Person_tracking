#!/usr/bin/env python3
import time
import sys
import threading
import math
import smbus2
from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

MOTOR1_STEP = 23
MOTOR1_DIR = 24
MOTOR2_STEP = 25
MOTOR2_DIR = 16

FAN1_PWM = 22
FAN1_IN1 = 17
FAN1_IN2 = 27

FAN2_PWM = 26
FAN2_IN3 = 5
FAN2_IN4 = 6

STEPS_PER_REV = 200
MICROSTEPS = 32
TARGET_ANGLE = 20.0 
STEPS_FULL_REV = STEPS_PER_REV * MICROSTEPS
STEPS_PER_DEG = STEPS_FULL_REV / 360.0
MOVE_STEPS = int(TARGET_ANGLE * STEPS_PER_DEG)

MAX_SPEED = 800.0
ACCELERATION = 400.0

TEMP_MIN = 25.0
TEMP_MAX = 40.0
TEMP2_MIN = 30.0
TEMP2_MAX = 50.0

I2C_BUS_0 = 0
I2C_BUS_1 = 1
AS5600_ADDR = 0x36
HTU21D_ADDR = 0x40
BMP280_ADDR = 0x76

class AS5600:
    def __init__(self, bus_num, address=AS5600_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        time.sleep(0.1)
        self.initial_angle = self.read_angle()
        
    def read_angle(self):
        try:
            high = self.bus.read_byte_data(self.address, 0x0C)
            low = self.bus.read_byte_data(self.address, 0x0D)
            raw_angle = (high << 8) | low
            return (raw_angle * 360.0) / 4096.0
        except Exception:
            return 0.0
            
    def get_relative_angle(self):
        raw = self.read_angle()
        if raw == 0.0: return 0.0
        rel = (raw - self.initial_angle) * -1.0
        if rel > 180.0: rel -= 360.0
        elif rel < -180.0: rel += 360.0
        return rel

class HTU21D:
    def __init__(self, bus_num, address=HTU21D_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
    
    def read_temperature(self):
        try:
            self.bus.write_byte(self.address, 0xF3)
            time.sleep(0.055) 
            data = self.bus.read_i2c_block_data(self.address, 0xF3, 3)
            temp_raw = (data[0] << 8) | data[1]
            temp_raw &= 0xFFFC
            return -46.85 + (175.72 * temp_raw / 65536.0)
        except Exception:
            return float('nan')

class BMP280:
    def __init__(self, bus_num, address=BMP280_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self._initialize()
        
    def _initialize(self):
        try:
            self.bus.write_byte_data(self.address, 0xF4, 0x27)
            self.bus.write_byte_data(self.address, 0xF5, 0x00)
            time.sleep(0.1)
            cal = self.bus.read_i2c_block_data(self.address, 0x88, 24)
            self.dig_T1 = cal[0] | (cal[1] << 8)
            self.dig_T2 = self._to_signed(cal[2] | (cal[3] << 8))
            self.dig_T3 = self._to_signed(cal[4] | (cal[5] << 8))
        except Exception:
            pass
            
    def _to_signed(self, val):
        return val - 65536 if val > 32767 else val
        
    def read_temperature(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, 0xFA, 3)
            adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            var1 = ((adc_T / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
            var2 = (((adc_T / 131072.0) - (self.dig_T1 / 8192.0)) ** 2) * self.dig_T3
            return (var1 + var2) / 5120.0
        except Exception:
            return float('nan')

class StepperMotor:
    def __init__(self, step_pin, dir_pin, factory):
        self.step_pin = OutputDevice(step_pin, pin_factory=factory)
        self.dir_pin = OutputDevice(dir_pin, pin_factory=factory)
        
        self.current_pos = 0
        self.target_pos = 0
        self.current_speed = 0.0
        self.max_speed = MAX_SPEED
        self.acceleration = ACCELERATION
        self.last_step_time = time.perf_counter()
        
    def move_to(self, target_steps):
        self.target_pos = target_steps
        
    def distance_to_go(self):
        return self.target_pos - self.current_pos
        
    def run(self):
        distance = self.target_pos - self.current_pos
        if distance == 0:
            return False
            
        current_time = time.perf_counter()
        
        if self.current_speed == 0.0:
            step_interval = 0.001
        else:
            step_interval = 1.0 / abs(self.current_speed)
            
        if (current_time - self.last_step_time) >= step_interval:
            required_accel = self.acceleration * step_interval
            stopping_dist = (self.current_speed * self.current_speed) / (2.0 * self.acceleration)
            
            if abs(distance) <= stopping_dist:
                if self.current_speed > 0:
                    self.current_speed -= required_accel
                    if self.current_speed < 10: self.current_speed = 10
                else:
                    self.current_speed += required_accel
                    if self.current_speed > -10: self.current_speed = -10
            else:
                if distance > 0:
                    if self.current_speed < self.max_speed:
                        self.current_speed += required_accel
                else:
                    if self.current_speed > -self.max_speed:
                        self.current_speed -= required_accel

            direction = 1 if self.current_speed > 0 else -1
            if direction > 0:
                self.dir_pin.on()
            else:
                self.dir_pin.off()
            
            self.step_pin.on()
            self.step_pin.off()
            
            self.current_pos += direction
            self.last_step_time = current_time
            return True
        return False

    def cleanup(self):
        self.step_pin.close()
        self.dir_pin.close()

class RTFan:
    def __init__(self, pwm_pin, in1_pin, in2_pin, factory):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=1000, pin_factory=factory)
        self.in1 = OutputDevice(in1_pin, pin_factory=factory)
        self.in2 = OutputDevice(in2_pin, pin_factory=factory)
        self.in1.on()
        self.in2.off()
        
    def set_speed(self, percentage):
        self.pwm.value = max(0, min(100, percentage)) / 100.0
        
    def cleanup(self):
        self.pwm.close()
        self.in1.close()
        self.in2.close()

class SystemMonitor(threading.Thread):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.running = True
        
    def run(self):
        print("Sensor Monitor Thread Started")
        while self.running:
            try:
                t1 = self.app.htu21d.read_temperature()
                t2 = self.app.bmp280.read_temperature()
                
                self._update_fan(self.app.fan1, t1, TEMP_MIN, TEMP_MAX)
                self._update_fan(self.app.fan2, t2, TEMP2_MIN, TEMP2_MAX)
                
                self._update_fsm(self.app.fsm1)
                self._update_fsm(self.app.fsm2)
                
                time.sleep(0.1) 
            except Exception as e:
                print(f"Monitor Error: {e}")

    def _update_fan(self, fan, temp, t_min, t_max):
        if math.isnan(temp):
            fan.set_speed(100)
        elif temp <= t_min:
            fan.set_speed(0)
        elif temp >= t_max:
            fan.set_speed(100)
        else:
            pct = ((temp - t_min) / (t_max - t_min)) * 100
            fan.set_speed(pct)

    def _update_fsm(self, fsm):
        now = time.monotonic()
        
        if fsm['state'] == 0:
            if fsm['motor'].distance_to_go() == 0:
                fsm['timer'] = now
                fsm['state'] = 1
                
        elif fsm['state'] == 1:
            if now - fsm['timer'] > 1.0:
                fsm['motor'].move_to(0)
                fsm['state'] = 2
                
        elif fsm['state'] == 2:
            if fsm['motor'].distance_to_go() == 0:
                fsm['timer'] = now
                fsm['state'] = 3

        elif fsm['state'] == 3:
            if now - fsm['timer'] > 1.0:
                fsm['motor'].move_to(-MOVE_STEPS)
                fsm['state'] = 4
                
        elif fsm['state'] == 4:
            if fsm['motor'].distance_to_go() == 0:
                fsm['timer'] = now
                fsm['state'] = 5
                
        elif fsm['state'] == 5:
            if now - fsm['timer'] > 1.0:
                fsm['motor'].move_to(0)
                fsm['state'] = 6
        
        elif fsm['state'] == 6:
            if fsm['motor'].distance_to_go() == 0:
                fsm['timer'] = now
                fsm['state'] = 7
        
        elif fsm['state'] == 7:
            if now - fsm['timer'] > 2.0:
                fsm['motor'].move_to(MOVE_STEPS)
                fsm['state'] = 0

class MainApplication:
    def __init__(self):
        self.factory = LGPIOFactory()
        
        self.encoder1 = AS5600(I2C_BUS_0)
        self.encoder2 = AS5600(I2C_BUS_1)
        self.htu21d = HTU21D(I2C_BUS_0)
        self.bmp280 = BMP280(I2C_BUS_0)
        
        self.motor1 = StepperMotor(MOTOR1_STEP, MOTOR1_DIR, self.factory)
        self.motor2 = StepperMotor(MOTOR2_STEP, MOTOR2_DIR, self.factory)
        self.fan1 = RTFan(FAN1_PWM, FAN1_IN1, FAN1_IN2, self.factory)
        self.fan2 = RTFan(FAN2_PWM, FAN2_IN3, FAN2_IN4, self.factory)
        
        self.fsm1 = {'state': 0, 'motor': self.motor1, 'encoder': self.encoder1, 'timer': 0}
        self.fsm2 = {'state': 0, 'motor': self.motor2, 'encoder': self.encoder2, 'timer': 0}
        
        self.motor1.move_to(MOVE_STEPS)
        self.motor2.move_to(MOVE_STEPS)
        
        self.monitor = SystemMonitor(self)

    def run(self):
        try:
            self.monitor.start()
            print("RT Loop Started - Ctrl+C to stop")
            while True:
                self.motor1.run()
                self.motor2.run()
                
        except KeyboardInterrupt:
            print("Stopping...")
            self.monitor.running = False
            self.monitor.join()
            self.motor1.cleanup()
            self.motor2.cleanup()
            self.fan1.cleanup()
            self.fan2.cleanup()

if __name__ == "__main__":
    app = MainApplication()
    app.run()
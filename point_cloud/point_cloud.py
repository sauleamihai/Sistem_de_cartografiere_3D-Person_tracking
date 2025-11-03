#!/usr/bin/env python3


import serial
import lgpio
import smbus2
import time
import math
import json
import os
import threading
from collections import deque
from math import atan, sin, cos, pi, sqrt
import numpy as np
from scipy.spatial import cKDTree


LIDAR_PORT = '/dev/ttyAMA0'
LIDAR_BAUD = 115200

SERVO_PIN = 13
MPU6050_ADDR = 0x68
CALIBRATION_FILE = "servo_calibration.json"


DETAIL_PRESETS = {
    'fast': {
        'ANGLE_STEP': 5.0,
        'SCANS_PER_POSITION': 3,
        'SERVO_STABILIZATION_TIME': 0.6,
        'VOXEL_SIZE': 50,
        'STAT_STDDEV': 2.0,
        'description': '~5 min - test rapid (18 poziții)'
    },
    'balanced': {
        'ANGLE_STEP': 3.0,
        'SCANS_PER_POSITION': 4,
        'SERVO_STABILIZATION_TIME': 0.7,
        'VOXEL_SIZE': 35,
        'STAT_STDDEV': 1.8,
        'description': '~10 min - detaliu bun (30 poziții)'
    },
    'high': {
        'ANGLE_STEP': 2.0,
        'SCANS_PER_POSITION': 5,
        'SERVO_STABILIZATION_TIME': 0.8,
        'VOXEL_SIZE': 25,
        'STAT_STDDEV': 1.5,
        'description': '~18 min - detaliu înalt (45 poziții)'
    },
    'ultra': {
        'ANGLE_STEP': 1.0,
        'SCANS_PER_POSITION': 6,
        'SERVO_STABILIZATION_TIME': 0.9,
        'VOXEL_SIZE': 20,
        'STAT_STDDEV': 1.3,
        'description': '~40 min - maximum detail (90 poziții)'
    }
}


ANGLE_STEP = 2.0
SCANS_PER_POSITION = 5
SERVO_STABILIZATION_TIME = 0.8
VOXEL_SIZE = 25


MIN_ANGLE = -45
MAX_ANGLE = 45


IMU_STABILITY_THRESHOLD = 0.3


MIN_DISTANCE = 150
MAX_DISTANCE = 7000
POINT_CLOUD_FILE = "scan_3d_detail.ply"


STATISTICAL_OUTLIER_K = 20
STATISTICAL_OUTLIER_STDDEV = 1.5  
RADIUS_OUTLIER_RADIUS = 120 
RADIUS_OUTLIER_MIN_NEIGHBORS = 4  


UPDATE_RATE = 0.05



class MPU6050:
    def __init__(self, bus, address=0x68):
        self.bus = bus
        self.address = address
        self.max_retries = 3
        self._init_sensor()
    
    def _init_sensor(self):
        try:
            self.bus.write_byte_data(self.address, 0x6B, 0)
            time.sleep(0.1)
        except OSError:
            time.sleep(0.5)
            self.bus.write_byte_data(self.address, 0x6B, 0)
            time.sleep(0.1)
    
    def _read_word_2c(self, reg):
        for attempt in range(self.max_retries):
            try:
                high = self.bus.read_byte_data(self.address, reg)
                time.sleep(0.001)
                low = self.bus.read_byte_data(self.address, reg + 1)
                value = (high << 8) + low
                return value - 65536 if value >= 0x8000 else value
            except OSError:
                if attempt < self.max_retries - 1:
                    time.sleep(0.05)
        return 0
    
    def get_pitch(self):
        try:
            ax = self._read_word_2c(0x3B) / 16384.0
            time.sleep(0.002)
            ay = self._read_word_2c(0x3D) / 16384.0
            time.sleep(0.002)
            az = self._read_word_2c(0x3F) / 16384.0
            
            if abs(ax) > 2.0 or abs(ay) > 2.0 or abs(az) > 2.0:
                return None
            
            pitch = math.atan2(ax, math.sqrt(ay*ay + az*az)) * 57.3
            return pitch
        except:
            return None
    
    def wait_for_complete_stability(self, target_angle, timeout=3.0):
        readings = deque(maxlen=20)
        start = time.time()
        
        while time.time() - start < timeout:
            pitch = self.get_pitch()
            if pitch is not None:
                readings.append(pitch)
                
                if len(readings) >= 20:
                    std_dev = np.std(list(readings))
                    mean_pitch = np.mean(list(readings))
                    error = abs(mean_pitch - target_angle)
                    
                    if std_dev < IMU_STABILITY_THRESHOLD and error < 1.0:
                        return mean_pitch
            
            time.sleep(0.05)
        
        return np.mean(list(readings)) if readings else None



class CalibratedServo:
    def __init__(self, chip, pin, calibration):
        self.chip = chip
        self.pin = pin
        self.cal = calibration
        
        self.slope = (calibration['plus_45']['pulse'] - 
                     calibration['minus_45']['pulse']) / 90.0
        self.intercept = calibration['center']['pulse']
    
    def angle_to_pulse(self, angle):
        return self.intercept + angle * self.slope
    
    def set_angle_and_hold(self, angle):
        pulse = self.angle_to_pulse(angle)
        pulse = max(500, min(2500, int(pulse)))
        duty = (pulse / 20000.0) * 100
        lgpio.tx_pwm(self.chip, self.pin, 50, duty)
        return pulse



class LidarReader:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=2)
        self.ser.reset_input_buffer()
        self.running = False
        self.thread = None
        self.latest_scan = []
        self.lock = threading.Lock()
        self.scan_count = 0
    
    def read_packet(self):
        while True:
            c = self.ser.read(1)
            if not c:
                return None
            
            if c[0] == 0xAA:
                c = self.ser.read(1)
                if c and c[0] == 0x55:
                    CT = ord(self.ser.read(1))
                    LSN = ord(self.ser.read(1))
                    FSA = int.from_bytes(self.ser.read(2), byteorder='little')
                    LSA = int.from_bytes(self.ser.read(2), byteorder='little')
                    CS = int.from_bytes(self.ser.read(2), byteorder='little')
                    
                    samples = []
                    for i in range(LSN):
                        Si = int.from_bytes(self.ser.read(2), byteorder='little')
                        dist = Si >> 2
                        
                        if MIN_DISTANCE <= dist <= MAX_DISTANCE:
                            Acorrect = 0 if (dist == 0) else atan(19.16 * (dist - 90.15) / (dist * 90.15))
                            F = (FSA >> 1) / 64.0
                            L = (LSA >> 1) / 64.0
                            angle = F + ((L - F) / LSN) * i - Acorrect
                            
                            samples.append((angle, dist))
                    
                    return {
                        'type': 'start' if CT == 1 else 'data',
                        'samples': samples
                    }
    
    def reader_thread(self):
        full_scan = []
        
        while self.running:
            packet = self.read_packet()
            
            if packet:
                full_scan.extend(packet['samples'])
                
                if packet['type'] == 'start' and full_scan:
                    with self.lock:
                        self.latest_scan = full_scan.copy()
                        self.scan_count += 1
                    full_scan = []
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.reader_thread, daemon=True)
        self.thread.start()
    
    def get_latest_scan(self):
        with self.lock:
            return self.latest_scan.copy(), self.scan_count
    
    def capture_multiple_scans(self, num_scans=5, timeout=10.0):
        scans = []
        initial_count = self.scan_count
        start = time.time()
        
        while len(scans) < num_scans and time.time() - start < timeout:
            scan, count = self.get_latest_scan()
            
            if count > initial_count + len(scans) and len(scan) > 0:
                scans.append(scan)
            
            time.sleep(0.05)
        
        return scans
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        self.ser.close()



class PointCloudFiltersOptimized:
    @staticmethod
    def statistical_outlier_removal_fast(points, k=20, std_dev_mult=1.5):
        if len(points) < k:
            return points
        
        print(f"  [SOR] Filtering {len(points):,} points (K={k}, σ={std_dev_mult})...")
        start = time.time()
        
        coords = np.array([p[:3] for p in points])
        tree = cKDTree(coords)
        distances, indices = tree.query(coords, k=k+1)
        distances = distances[:, 1:]
        mean_distances = np.mean(distances, axis=1)
        
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + std_dev_mult * global_std
        
        mask = mean_distances <= threshold
        filtered = [points[i] for i in range(len(points)) if mask[i]]
        
        elapsed = time.time() - start
        removed = len(points) - len(filtered)
        print(f"      → Removed {removed:,} ({removed/len(points)*100:.1f}%) in {elapsed:.1f}s")
        
        return filtered
    
    @staticmethod
    def radius_outlier_removal_fast(points, radius=120, min_neighbors=4):
        if len(points) == 0:
            return points
        
        print(f"  [ROR] Filtering (R={radius}mm, min_nb={min_neighbors})...")
        start = time.time()
        
        coords = np.array([p[:3] for p in points])
        tree = cKDTree(coords)
        
        neighbor_counts = np.array([
            len(tree.query_ball_point(coord, radius)) - 1
            for coord in coords
        ])
        
        mask = neighbor_counts >= min_neighbors
        filtered = [points[i] for i in range(len(points)) if mask[i]]
        
        elapsed = time.time() - start
        removed = len(points) - len(filtered)
        print(f"      → Removed {removed:,} ({removed/len(points)*100:.1f}%) in {elapsed:.1f}s")
        
        return filtered
    
    @staticmethod
    def voxel_grid_downsample_fast(points, voxel_size=25):
        if len(points) == 0:
            return points
        
        print(f"  [Voxel] Downsampling (size={voxel_size}mm)...")
        start = time.time()
        
        coords = np.array([p[:3] for p in points])
        colors = np.array([p[3:] for p in points])
        
        voxel_indices = np.floor(coords / voxel_size).astype(int)
        
        voxel_dict = {}
        for i in range(len(points)):
            key = tuple(voxel_indices[i])
            if key not in voxel_dict:
                voxel_dict[key] = {'coords': [], 'colors': []}
            voxel_dict[key]['coords'].append(coords[i])
            voxel_dict[key]['colors'].append(colors[i])
        
        downsampled = []
        for voxel_data in voxel_dict.values():
            avg_coord = np.mean(voxel_data['coords'], axis=0)
            avg_color = np.mean(voxel_data['colors'], axis=0).astype(int)
            downsampled.append(tuple(avg_coord) + tuple(avg_color))
        
        elapsed = time.time() - start
        removed = len(points) - len(downsampled)
        print(f"      → {len(downsampled):,} points ({removed/len(points)*100:.1f}% reduction) in {elapsed:.1f}s")
        
        return downsampled



class PointCloudBuilder:
    def __init__(self):
        self.raw_points = []
        self.filtered_points = []
        self.lock = threading.Lock()
        self.filters = PointCloudFiltersOptimized()
    
    def add_scans_at_fixed_position(self, scans_list, vertical_angle):
        if not scans_list:
            return
        
        angle_groups = {}
        
        for scan in scans_list:
            for h_angle, distance in scan:
                
                angle_key = round(h_angle, 0.5)  # 0.5° precision
                
                if angle_key not in angle_groups:
                    angle_groups[angle_key] = []
                
                angle_groups[angle_key].append(distance)
        
        for h_angle, distances in angle_groups.items():
            if len(distances) >= 2:  # Minim 2 măsurători (mai relaxat)
                distances_sorted = sorted(distances)
                
               
                trim = max(1, len(distances) // 7)
                if len(distances) > 2*trim:
                    distances_trimmed = distances_sorted[trim:-trim]
                else:
                    distances_trimmed = distances_sorted
                
                avg_distance = np.median(distances_trimmed)
                
                
                h_rad = h_angle * pi / 180.0
                v_rad = vertical_angle * pi / 180.0
                
                x = avg_distance * cos(v_rad) * sin(h_rad)
                y = avg_distance * cos(v_rad) * cos(h_rad)
                z = avg_distance * sin(v_rad)
                
             
                if avg_distance < 500:
                    r, g, b = 255, 0, 0  # Very close: red
                elif avg_distance < 1500:
                    r, g, b = 255, 128, 0  # Close: orange
                elif avg_distance < 3000:
                    r, g, b = 255, 255, 0  # Medium: yellow
                elif avg_distance < 5000:
                    r, g, b = 0, 255, 0  # Far: green
                else:
                    r, g, b = 0, 128, 255  # Very far: cyan
                
                with self.lock:
                    self.raw_points.append((x, y, z, r, g, b))
    
    def apply_filters(self, preset_stddev=1.5, voxel_size=25):
        with self.lock:
            points = self.raw_points.copy()
        
        if not points:
            print("⚠ Nu sunt puncte!")
            return
        
        print(f"\n🔧 Filtrare: {len(points):,} puncte raw")
        print("="*60)
        
        total_start = time.time()
        
     
        points = self.filters.statistical_outlier_removal_fast(
            points, STATISTICAL_OUTLIER_K, preset_stddev)
        
        points = self.filters.radius_outlier_removal_fast(
            points, RADIUS_OUTLIER_RADIUS, RADIUS_OUTLIER_MIN_NEIGHBORS)
        
        points = self.filters.voxel_grid_downsample_fast(
            points, voxel_size)
        
        total_elapsed = time.time() - total_start
        
        print("="*60)
        print(f" Filtrare în {total_elapsed:.1f}s: {len(self.raw_points):,} → {len(points):,}")
        print(f"  Detaliu păstrat: {len(points)/len(self.raw_points)*100:.1f}%")
        
        with self.lock:
            self.filtered_points = points
    
    def get_point_count(self, filtered=False):
        with self.lock:
            return len(self.filtered_points if filtered else self.raw_points)
    
    def save_ply(self, filename, use_filtered=True):
        with self.lock:
            points = self.filtered_points if use_filtered else self.raw_points
        
        if not points:
            return False
        
        print(f"\n Salvare {len(points):,} puncte în {filename}...")
        
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            
            for x, y, z, r, g, b in points:
                f.write(f"{x:.2f} {y:.2f} {z:.2f} {int(r)} {int(g)} {int(b)}\n")
        
        print(f"✓ Salvat: {filename}")
        return True



def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        print(f"{CALIBRATION_FILE} lipsește!")
        return None
    
    with open(CALIBRATION_FILE, 'r') as f:
        return json.load(f)


def select_detail_preset():
    print("\n Presets de detaliu disponibile:")
    print("-" * 60)
    for i, (name, config) in enumerate(DETAIL_PRESETS.items(), 1):
        num_pos = int(90 / config['ANGLE_STEP']) + 1
        print(f"  {i}. {name.upper():8s} - {config['description']}")
        print(f"             ({num_pos} poziții, voxel={config['VOXEL_SIZE']}mm)")
    
    print("\n Pentru imaginea ta sparse, recomand: 3 (HIGH) sau 4 (ULTRA)")
    choice = input("\nAlege preset (1-4) [3=high]: ").strip() or "3"
    
    preset_map = {'1': 'fast', '2': 'balanced', '3': 'high', '4': 'ultra'}
    preset_name = preset_map.get(choice, 'high')
    preset = DETAIL_PRESETS[preset_name]
    
    print(f"\n Selectat: {preset_name.upper()}")
    print(f"  {preset['description']}")
    
    return preset



def run_3d_scan_detail(chip, pin, imu, servo, lidar, point_cloud, preset):
    num_positions = int((MAX_ANGLE - MIN_ANGLE) / preset['ANGLE_STEP']) + 1
    
    print("\n" + "="*70)
    print("  3D SCANNER v3.1 - HIGH DETAIL ".center(70))
    print("="*70)
    
    print(f"\n  Setări detaliu:")
    print(f"  • Pași: {preset['ANGLE_STEP']}° (mai mult detaliu)")
    print(f"  • Poziții: {num_positions}")
    print(f"  • Scanări/poziție: {preset['SCANS_PER_POSITION']}")
    print(f"  • Voxel size: {preset['VOXEL_SIZE']}mm (mai fin)")
    print(f"  • Stabilizare: {preset['SERVO_STABILIZATION_TIME']}s")
    
    est_time = num_positions * (preset['SERVO_STABILIZATION_TIME'] + 1.5)
    print(f"\n Timp estimat: ~{est_time/60:.0f} minute")
    print(f"   ({num_positions} poziții × ~{preset['SERVO_STABILIZATION_TIME'] + 1.5:.1f}s)")
    
    input("\nApasă Enter pentru START...")
    
    positions_completed = 0
    start_time = time.time()
    
    try:
        current_angle = MIN_ANGLE
        
        while current_angle <= MAX_ANGLE:
            progress = (positions_completed + 1) / num_positions * 100
            
            print(f"\n [{progress:5.1f}%] Poziție {positions_completed + 1}/{num_positions}: {current_angle:+.1f}°")
            
        
            servo.set_angle_and_hold(current_angle)
            
        
            time.sleep(preset['SERVO_STABILIZATION_TIME'])
            stable_pitch = imu.wait_for_complete_stability(current_angle, timeout=3.0)
            
            if stable_pitch is None:
                print(f"       IMU instabil - SKIP")
                current_angle += preset['ANGLE_STEP']
                continue
            
            print(f"       Stabil: {stable_pitch:+.2f}° (target: {current_angle:+.1f}°)")
            
       
            scans = lidar.capture_multiple_scans(num_scans=preset['SCANS_PER_POSITION'], timeout=10.0)
            
            if scans:
                point_cloud.add_scans_at_fixed_position(scans, stable_pitch)
                positions_completed += 1
                
                elapsed = time.time() - start_time
                raw_count = point_cloud.get_point_count(filtered=False)
                eta = (est_time - elapsed) / 60
                
                print(f"       {raw_count:,} puncte | Timp: {elapsed/60:.1f}m | ETA: {eta:.1f}m")
            else:
                print(f"      Niciun scan - SKIP")
            
            current_angle += preset['ANGLE_STEP']
        
        elapsed = time.time() - start_time
        print(f"\n Scanare completă în {elapsed/60:.1f} minute")
        print(f"  Poziții: {positions_completed}/{num_positions}")
        print(f"  Puncte raw: {point_cloud.get_point_count(filtered=False):,}")
    
    except KeyboardInterrupt:
        print(f"\n  Stop manual")


def main():
    print("="*70)
    print("  M1C1_Mini 3D SCANNER v3.1 - HIGH DETAIL".center(70))
    print("="*70)
    print("\n Îmbunătățiri față de v3.0:")
    print("   ✓ Mai multe poziții (până la 90 vs 18)")
    print("   ✓ Voxel mai fin (până la 20mm vs 50mm)")
    print("   ✓ Filtering mai puțin agresiv (mai mult detaliu)")
    print("   ✓ 5 gradații de culoare pentru depth perception")
    print("   ✓ 4 presets: Fast/Balanced/High/Ultra\n")
    
  
    preset = select_detail_preset()
    
   
    global ANGLE_STEP, SCANS_PER_POSITION, SERVO_STABILIZATION_TIME, VOXEL_SIZE, STATISTICAL_OUTLIER_STDDEV
    ANGLE_STEP = preset['ANGLE_STEP']
    SCANS_PER_POSITION = preset['SCANS_PER_POSITION']
    SERVO_STABILIZATION_TIME = preset['SERVO_STABILIZATION_TIME']
    VOXEL_SIZE = preset['VOXEL_SIZE']
    STATISTICAL_OUTLIER_STDDEV = preset['STAT_STDDEV']
    
    calibration = load_calibration()
    if not calibration:
        return
    
    print("\n Hardware init...")
    bus = smbus2.SMBus(1)
    chip = lgpio.gpiochip_open(4)
    
    imu = MPU6050(bus)
    servo = CalibratedServo(chip, SERVO_PIN, calibration)
    lidar = LidarReader(LIDAR_PORT, LIDAR_BAUD)
    point_cloud = PointCloudBuilder()
    
    lidar.start()
    time.sleep(2)
    
    scan, count = lidar.get_latest_scan()
    print(f"✓ LiDAR: {len(scan)} puncte/scan\n")
    
    try:
        run_3d_scan_detail(chip, SERVO_PIN, imu, servo, lidar, point_cloud, preset)
        
        
        point_cloud.apply_filters(
            preset_stddev=preset['STAT_STDDEV'],
            voxel_size=preset['VOXEL_SIZE']
        )
        
        point_cloud.save_ply(POINT_CLOUD_FILE, use_filtered=True)
        point_cloud.save_ply("scan_3d_raw_v31.ply", use_filtered=False)
        
        print("\n" + "="*70)
        print("  ✓ SCANARE COMPLETĂ".center(70))
        print("="*70)
        print(f"\n Fișiere:")
        print(f"   • {POINT_CLOUD_FILE} ← folosește asta!")
        print(f"   • scan_3d_raw_v31.ply (raw)")
        
        print(f"\n Vizualizează:")
        print(f"   meshlab {POINT_CLOUD_FILE}")
        print(f"   cloudcompare {POINT_CLOUD_FILE}")
        
    finally:
        print("\n Cleanup...")
        servo.set_angle_and_hold(0)
        time.sleep(0.5)
        lgpio.tx_pwm(chip, SERVO_PIN, 50, 0)
        lidar.stop()
        lgpio.gpiochip_close(chip)
        bus.close()
        print(" Done\n")


if __name__ == "__main__":
    main()
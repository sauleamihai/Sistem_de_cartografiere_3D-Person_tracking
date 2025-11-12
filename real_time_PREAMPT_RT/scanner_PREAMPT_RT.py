#!/usr/bin/env python3

import serial
import lgpio
import smbus2
import time
import math
import json
import os
import sys
import signal
import threading
import asyncio
import logging
import ctypes
import resource
from collections import deque
from contextlib import contextmanager
from threading import RLock, Event, Condition
from math import atan, sin, cos, pi, sqrt
import numpy as np
from scipy.spatial import cKDTree
import heapq
import queue

libc = ctypes.CDLL('libc.so.6')

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

LIDAR_PORT = '/dev/ttyAMA0'
LIDAR_BAUD = 115200
SERVO_PIN = 13
MPU6050_ADDR = 0x68
CALIBRATION_FILE = "servo_calibration.json"

DETAIL_PRESETS = {
    'fast': {'ANGLE_STEP': 5.0, 'SCANS_PER_POSITION': 3, 'SERVO_STABILIZATION_TIME': 0.6, 
             'VOXEL_SIZE': 50, 'STAT_STDDEV': 2.0, 'description': '~5 min'},
    'balanced': {'ANGLE_STEP': 3.0, 'SCANS_PER_POSITION': 4, 'SERVO_STABILIZATION_TIME': 0.7,
                 'VOXEL_SIZE': 35, 'STAT_STDDEV': 1.8, 'description': '~10 min'},
    'high': {'ANGLE_STEP': 2.0, 'SCANS_PER_POSITION': 5, 'SERVO_STABILIZATION_TIME': 0.8,
             'VOXEL_SIZE': 25, 'STAT_STDDEV': 1.5, 'description': '~18 min'},
    'ultra': {'ANGLE_STEP': 1.0, 'SCANS_PER_POSITION': 6, 'SERVO_STABILIZATION_TIME': 0.9,
              'VOXEL_SIZE': 20, 'STAT_STDDEV': 1.3, 'description': '~40 min'}
}

MIN_ANGLE, MAX_ANGLE = -45, 45
IMU_STABILITY_THRESHOLD = 0.3
MIN_DISTANCE, MAX_DISTANCE = 150, 7000
POINT_CLOUD_FILE = "scan_3d_detail.ply"
STATISTICAL_OUTLIER_K = 20
RADIUS_OUTLIER_RADIUS = 120
RADIUS_OUTLIER_MIN_NEIGHBORS = 4


class RTKernelManager:
    SCHED_OTHER = 0
    SCHED_FIFO = 1
    SCHED_RR = 2
    SCHED_BATCH = 3
    SCHED_IDLE = 5
    SCHED_DEADLINE = 6

    @staticmethod
    def set_sched_fifo(priority=90, pid=None):
        if pid is None:
            pid = os.getpid()
        
        try:
            sched_param = (ctypes.c_int * 1)(priority)
            result = libc.sched_setscheduler(pid, RTKernelManager.SCHED_FIFO, ctypes.byref(sched_param))
            
            if result == 0:
                logger.info(f"✓ SCHED_FIFO priority {priority} set for PID {pid}")
                return True
            else:
                logger.warning(f"⚠ Cannot set SCHED_FIFO (may need sudo): {result}")
                return False
        except Exception as e:
            logger.warning(f"⚠ sched_setscheduler failed: {e}")
            return False

    @staticmethod
    def set_cpu_affinity(cpu_set, pid=None):
        if pid is None:
            pid = os.getpid()
        
        try:
            os.sched_setaffinity(pid, cpu_set)
            affinity = os.sched_getaffinity(pid)
            logger.info(f"✓ CPU affinity {affinity} set for PID {pid}")
            return True
        except Exception as e:
            logger.warning(f"⚠ Cannot set affinity: {e}")
            return False

    @staticmethod
    def lock_memory():
        try:
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
            logger.info("✓ Memory locked (mlockall)")
            return True
        except Exception as e:
            logger.warning(f"⚠ Cannot lock memory (need sudo): {e}")
            return False

    @staticmethod
    def check_kernel():
        try:
            with open('/proc/version', 'r') as f:
                kernel_info = f.read()
                if 'PREEMPT_RT' in kernel_info or 'PREEMPT' in kernel_info:
                    logger.info(f"✓ PREEMPT_RT kernel detected")
                    return True
                else:
                    logger.warning("⚠ Standard kernel (PREEMPT_RT recommended)")
                    return False
        except:
            return False

    @staticmethod
    def check_isolcpus():
        try:
            with open('/proc/cmdline', 'r') as f:
                cmdline = f.read()
                if 'isolcpus=3' in cmdline and 'nohz_full=3' in cmdline and 'rcu_nocbs=3' in cmdline:
                    logger.info("✓ CPU isolation parameters present")
                    return True
                else:
                    logger.warning("⚠ CPU isolation not configured")
                    return False
        except:
            return False

    @staticmethod
    def get_current_sched():
        try:
            pid = os.getpid()
            sched_param = (ctypes.c_int * 1)()
            sched = libc.sched_getscheduler(pid)
            libc.sched_getparam(pid, ctypes.byref(sched_param))
            
            sched_names = {0: 'SCHED_OTHER', 1: 'SCHED_FIFO', 2: 'SCHED_RR', 
                          3: 'SCHED_BATCH', 5: 'SCHED_IDLE', 6: 'SCHED_DEADLINE'}
            priority = sched_param[0]
            
            logger.info(f"Current scheduling: {sched_names.get(sched, 'UNKNOWN')} priority {priority}")
            return sched, priority
        except Exception as e:
            logger.warning(f"Cannot get scheduling info: {e}")
            return None, None


class PriorityTaskScheduler:
    def __init__(self):
        self.task_heap = []
        self.task_counter = 0
        self.lock = RLock()
        self.condition = Condition(self.lock)
        self.running = False

    def add_task(self, priority, callback, args=None, task_name=""):
        with self.lock:
            task_id = self.task_counter
            self.task_counter += 1
            heapq.heappush(self.task_heap, (priority, task_id, callback, args or (), task_name))
            self.condition.notify()
            return task_id

    def get_next_task(self):
        with self.lock:
            if self.task_heap:
                return heapq.heappop(self.task_heap)
            return None

    def wait_for_task(self, timeout=1.0):
        with self.condition:
            if not self.task_heap:
                self.condition.wait(timeout=timeout)
            if self.task_heap:
                return heapq.heappop(self.task_heap)
            return None


class ThreadSafeMutex:
    def __init__(self, name="", priority=90):
        self.lock = RLock()
        self.name = name
        self.priority = priority

    @contextmanager
    def acquire(self, timeout=None):
        acquired = self.lock.acquire(timeout=timeout)
        if not acquired:
            logger.warning(f"Mutex '{self.name}' acquisition timeout")
            yield False
        else:
            try:
                yield True
            finally:
                self.lock.release()


class MPU6050:
    def __init__(self, bus, address=MPU6050_ADDR):
        self.bus = bus
        self.address = address
        self.max_retries = 3
        self.data_mutex = ThreadSafeMutex("IMU_DATA", priority=95)
        self.pitch_cache = None
        self.cache_timestamp = 0
        self.cache_valid_time = 0.01
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
        with self.data_mutex.acquire(timeout=0.1) as acquired:
            if not acquired:
                return self.pitch_cache
            
            current_time = time.time()
            if self.pitch_cache is not None and (current_time - self.cache_timestamp) < self.cache_valid_time:
                return self.pitch_cache

            try:
                ax = self._read_word_2c(0x3B) / 16384.0
                time.sleep(0.002)
                ay = self._read_word_2c(0x3D) / 16384.0
                time.sleep(0.002)
                az = self._read_word_2c(0x3F) / 16384.0

                if abs(ax) > 2.0 or abs(ay) > 2.0 or abs(az) > 2.0:
                    return None

                pitch = math.atan2(ax, math.sqrt(ay*ay + az*az)) * 57.3
                self.pitch_cache = pitch
                self.cache_timestamp = current_time
                return pitch
            except:
                return None

    async def wait_for_stability(self, target_angle, timeout=3.0):
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

            await asyncio.sleep(0.05)

        return np.mean(list(readings)) if readings else None


class CalibratedServo:
    def __init__(self, chip, pin, calibration):
        self.chip = chip
        self.pin = pin
        self.cal = calibration
        self.control_mutex = ThreadSafeMutex("SERVO_CONTROL", priority=92)
        self.current_angle = 0

        self.slope = (calibration['plus_45']['pulse'] - 
                     calibration['minus_45']['pulse']) / 90.0
        self.intercept = calibration['center']['pulse']

    def angle_to_pulse(self, angle):
        return self.intercept + angle * self.slope

    def set_angle_and_hold(self, angle):
        with self.control_mutex.acquire(timeout=0.5) as acquired:
            if not acquired:
                logger.warning("Servo lock timeout")
                return None
            
            pulse = self.angle_to_pulse(angle)
            pulse = max(500, min(2500, int(pulse)))
            duty = (pulse / 20000.0) * 100
            lgpio.tx_pwm(self.chip, self.pin, 50, duty)
            self.current_angle = angle
            return pulse

    async def async_set_angle(self, angle):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.set_angle_and_hold, angle)


class LidarReader:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=2)
        self.ser.reset_input_buffer()
        self.running = False
        self.thread = None
        self.scan_queue = queue.PriorityQueue(maxsize=50)
        self.scan_mutex = ThreadSafeMutex("LIDAR_SCAN", priority=95)
        self.scan_count = 0
        self.latest_scan = []

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
        RTKernelManager.set_cpu_affinity({2}, threading.current_thread().ident)
        RTKernelManager.set_sched_fifo(priority=88)
        
        full_scan = []

        while self.running:
            packet = self.read_packet()

            if packet:
                full_scan.extend(packet['samples'])

                if packet['type'] == 'start' and full_scan:
                    with self.scan_mutex.acquire(timeout=0.05) as acquired:
                        if acquired:
                            self.latest_scan = full_scan.copy()
                            self.scan_count += 1
                            priority = 10 - (self.scan_count % 10)
                            try:
                                self.scan_queue.put_nowait((priority, self.scan_count, full_scan.copy()))
                            except queue.Full:
                                logger.warning("Scan queue full, dropping oldest")
                                try:
                                    self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait((priority, self.scan_count, full_scan.copy()))
                                except:
                                    pass
                    full_scan = []

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.reader_thread, daemon=True)
        self.thread.start()

    async def capture_multiple_scans(self, num_scans=5, timeout=10.0):
        scans = []
        start = time.time()

        while len(scans) < num_scans and time.time() - start < timeout:
            try:
                _, _, scan = self.scan_queue.get(timeout=0.5)
                if len(scan) > 0:
                    scans.append(scan)
            except queue.Empty:
                await asyncio.sleep(0.05)

        return scans

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        self.ser.close()


class PointCloudBuilder:
    def __init__(self):
        self.raw_points = []
        self.filtered_points = []
        self.data_mutex = ThreadSafeMutex("POINT_CLOUD", priority=85)

    def add_scans_at_fixed_position(self, scans_list, vertical_angle):
        if not scans_list:
            return

        angle_groups = {}

        for scan in scans_list:
            for h_angle, distance in scan:
                angle_key = round(h_angle, 0.5)

                if angle_key not in angle_groups:
                    angle_groups[angle_key] = []

                angle_groups[angle_key].append(distance)

        points_to_add = []

        for h_angle, distances in angle_groups.items():
            if len(distances) >= 2:
                distances_sorted = sorted(distances)
                trim = max(1, len(distances) // 7)
                distances_trimmed = distances_sorted[trim:-trim] if len(distances) > 2*trim else distances_sorted

                avg_distance = np.median(distances_trimmed)

                h_rad = h_angle * pi / 180.0
                v_rad = vertical_angle * pi / 180.0

                x = avg_distance * cos(v_rad) * sin(h_rad)
                y = avg_distance * cos(v_rad) * cos(h_rad)
                z = avg_distance * sin(v_rad)

                if avg_distance < 500:
                    r, g, b = 255, 0, 0
                elif avg_distance < 1500:
                    r, g, b = 255, 128, 0
                elif avg_distance < 3000:
                    r, g, b = 255, 255, 0
                elif avg_distance < 5000:
                    r, g, b = 0, 255, 0
                else:
                    r, g, b = 0, 128, 255

                points_to_add.append((x, y, z, r, g, b))

        with self.data_mutex.acquire(timeout=0.5) as acquired:
            if acquired:
                self.raw_points.extend(points_to_add)

    def apply_filters(self, preset_stddev=1.5, voxel_size=25):
        with self.data_mutex.acquire(timeout=2.0) as acquired:
            if not acquired:
                logger.error("Cannot acquire point cloud lock for filtering")
                return

            points = self.raw_points.copy()

        if not points:
            logger.warning("No points to filter")
            return

        logger.info(f"Filtering: {len(points):,} raw points")

        coords = np.array([p[:3] for p in points])
        tree = cKDTree(coords)
        distances, indices = tree.query(coords, k=min(STATISTICAL_OUTLIER_K + 1, len(coords)))
        
        if distances.ndim == 1:
            distances = distances.reshape(-1, 1)
        
        distances = distances[:, 1:] if distances.shape[1] > 1 else distances
        mean_distances = np.mean(distances, axis=1)
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + preset_stddev * global_std

        mask = mean_distances <= threshold
        points = [points[i] for i in range(len(points)) if mask[i]]

        coords = np.array([p[:3] for p in points])
        tree = cKDTree(coords)
        neighbor_counts = np.array([len(tree.query_ball_point(coord, RADIUS_OUTLIER_RADIUS)) - 1 for coord in coords])
        mask = neighbor_counts >= RADIUS_OUTLIER_MIN_NEIGHBORS
        points = [points[i] for i in range(len(points)) if mask[i]]

        colors = np.array([p[3:] for p in points])
        coords = np.array([p[:3] for p in points])
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

        with self.data_mutex.acquire(timeout=2.0) as acquired:
            if acquired:
                self.filtered_points = downsampled

        logger.info(f"Filtered: {len(self.raw_points):,} → {len(downsampled):,}")

    def get_point_count(self, filtered=False):
        with self.data_mutex.acquire(timeout=0.5) as acquired:
            if acquired:
                return len(self.filtered_points if filtered else self.raw_points)
            return 0

    def save_ply(self, filename, use_filtered=True):
        with self.data_mutex.acquire(timeout=2.0) as acquired:
            if not acquired:
                logger.error("Cannot acquire lock for PLY save")
                return False
            points = self.filtered_points if use_filtered else self.raw_points

        if not points:
            return False

        logger.info(f"Saving {len(points):,} points to {filename}")

        with open(filename, 'w') as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")

            for x, y, z, r, g, b in points:
                f.write(f"{x:.2f} {y:.2f} {z:.2f} {int(r)} {int(g)} {int(b)}\n")

        logger.info(f"Saved: {filename}")
        return True


def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        logger.error(f"{CALIBRATION_FILE} missing")
        return None

    with open(CALIBRATION_FILE, 'r') as f:
        return json.load(f)


def select_detail_preset():
    print("\nDetail Presets:")
    print("-" * 60)
    for i, (name, config) in enumerate(DETAIL_PRESETS.items(), 1):
        num_pos = int(90 / config['ANGLE_STEP']) + 1
        print(f"  {i}. {name.upper():8s} - {config['description']} ({num_pos} positions)")

    choice = input("\nSelect preset (1-4) [3=high]: ").strip() or "3"
    preset_map = {'1': 'fast', '2': 'balanced', '3': 'high', '4': 'ultra'}
    preset_name = preset_map.get(choice, 'high')
    preset = DETAIL_PRESETS[preset_name]

    logger.info(f"Selected: {preset_name.upper()}")
    return preset


class ScannerAsyncController:
    def __init__(self, chip, pin, imu, servo, lidar, point_cloud, preset):
        self.chip = chip
        self.pin = pin
        self.imu = imu
        self.servo = servo
        self.lidar = lidar
        self.point_cloud = point_cloud
        self.preset = preset
        self.scheduler = PriorityTaskScheduler()
        self.stop_event = Event()
        self.interrupt_handled = False

    def setup_signal_handlers(self):
        def signal_handler(signum, frame):
            logger.info("Interrupt signal received")
            self.stop_event.set()
            self.interrupt_handled = True

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    async def scan_position(self, angle):
        if self.stop_event.is_set():
            return None

        logger.info(f"Moving to angle: {angle:+.1f}°")
        await self.servo.async_set_angle(angle)
        await asyncio.sleep(self.preset['SERVO_STABILIZATION_TIME'])

        stable_pitch = await self.imu.wait_for_stability(angle, timeout=3.0)

        if stable_pitch is None:
            logger.warning(f"IMU unstable at {angle:+.1f}°")
            return None

        logger.info(f"Stable: {stable_pitch:+.2f}° (target: {angle:+.1f}°)")

        scans = await self.lidar.capture_multiple_scans(
            num_scans=self.preset['SCANS_PER_POSITION'], timeout=10.0)

        if scans:
            self.point_cloud.add_scans_at_fixed_position(scans, stable_pitch)
            raw_count = self.point_cloud.get_point_count(filtered=False)
            logger.info(f"Points: {raw_count:,}")
            return True

        logger.warning("No scans captured")
        return None

    async def run_scan(self):
        self.setup_signal_handlers()

        num_positions = int((MAX_ANGLE - MIN_ANGLE) / self.preset['ANGLE_STEP']) + 1

        logger.info(f"Starting scan: {num_positions} positions")
        logger.info(f"Angle step: {self.preset['ANGLE_STEP']}°")
        logger.info(f"Scans per position: {self.preset['SCANS_PER_POSITION']}")

        positions_completed = 0
        start_time = time.time()

        current_angle = MIN_ANGLE
        
        while current_angle <= MAX_ANGLE and not self.stop_event.is_set():
            result = await self.scan_position(current_angle)

            if result is not None:
                positions_completed += 1
                progress = (positions_completed / num_positions) * 100
                elapsed = time.time() - start_time
                logger.info(f"Progress: {progress:.1f}% [{positions_completed}/{num_positions}] Time: {elapsed/60:.1f}m")

            current_angle += self.preset['ANGLE_STEP']

        elapsed = time.time() - start_time
        logger.info(f"Scan completed in {elapsed/60:.1f} minutes")
        logger.info(f"Positions: {positions_completed}/{num_positions}")
        logger.info(f"Raw points: {self.point_cloud.get_point_count(filtered=False):,}")

        if not self.interrupt_handled:
            logger.info("Starting filtering...")
            self.point_cloud.apply_filters(
                preset_stddev=self.preset['STAT_STDDEV'],
                voxel_size=self.preset['VOXEL_SIZE']
            )

            self.point_cloud.save_ply(POINT_CLOUD_FILE, use_filtered=True)
            self.point_cloud.save_ply("scan_3d_raw.ply", use_filtered=False)

            logger.info(f"Output: {POINT_CLOUD_FILE}")

    async def cleanup(self):
        logger.info("Cleaning up...")
        self.servo.set_angle_and_hold(0)
        await asyncio.sleep(0.5)
        lgpio.tx_pwm(self.chip, self.pin, 50, 0)
        self.lidar.stop()
        lgpio.gpiochip_close(self.chip)
        logger.info("Done")


async def main_async():
    logger.info("="*70)
    logger.info("3D SCANNER v4.0 - PREEMPT_RT FULL INTEGRATION".center(70))
    logger.info("="*70)
    
    logger.info("\n[1] Verifying RT Kernel Configuration")
    if RTKernelManager.check_kernel():
        logger.info("✓ PREEMPT_RT kernel detected")
    
    if RTKernelManager.check_isolcpus():
        logger.info("✓ CPU isolation configured (isolcpus=3)")
    
    RTKernelManager.get_current_sched()
    
    logger.info("\n[2] Setting Up Real-Time Parameters")
    RTKernelManager.set_cpu_affinity({3}, os.getpid())
    RTKernelManager.lock_memory()
    RTKernelManager.set_sched_fifo(priority=90)
    
    preset = select_detail_preset()

    calibration = load_calibration()
    if not calibration:
        return

    logger.info("\n[3] Initializing Hardware...")
    bus = smbus2.SMBus(1)
    chip = lgpio.gpiochip_open(4)

    imu = MPU6050(bus)
    servo = CalibratedServo(chip, SERVO_PIN, calibration)
    lidar = LidarReader(LIDAR_PORT, LIDAR_BAUD)
    point_cloud = PointCloudBuilder()

    lidar.start()
    await asyncio.sleep(2)

    scan, count = lidar.latest_scan, lidar.scan_count
    logger.info(f"LiDAR ready: {len(scan)} points/scan")

    controller = ScannerAsyncController(chip, SERVO_PIN, imu, servo, lidar, point_cloud, preset)

    try:
        logger.info("\n[4] Starting Async Scanner")
        await controller.run_scan()
    finally:
        logger.info("\n[5] Cleaning Up Resources")
        await controller.cleanup()
        bus.close()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
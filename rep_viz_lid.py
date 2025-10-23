#!/usr/bin/env python3

import serial
import time
from math import atan, sin, cos, pi
import os

PORT = '/dev/ttyAMA0'
BAUD = 115200

RADAR_SIZE = 40  # Caractere (80x40 terminal)
CENTER = RADAR_SIZE // 2

def clear_screen():

    os.system('clear' if os.name != 'nt' else 'cls')

def create_empty_radar():

    radar = [[' ' for _ in range(RADAR_SIZE)] for _ in range(RADAR_SIZE)]
    
    radar[CENTER][CENTER] = '⊕'
    
    for radius in [10, 20, 30]:
        for angle in range(0, 360, 5):
            x = int(CENTER + radius * sin(angle * pi / 180))
            y = int(CENTER - radius * cos(angle * pi / 180))
            if 0 <= x < RADAR_SIZE and 0 <= y < RADAR_SIZE:
                if radar[y][x] == ' ':
                    radar[y][x] = '·'
    
    for i in range(RADAR_SIZE):
        if radar[CENTER][i] == ' ':
            radar[CENTER][i] = '─'  # Orizontal
        if radar[i][CENTER] == ' ':
            radar[i][CENTER] = '│'  # Vertical
    
    return radar

def add_point_to_radar(radar, angle, distance, max_range=8000):    
    if distance == 0 or distance > max_range:
        return
    
    scale = (RADAR_SIZE / 2 - 2) / max_range  # -2 pentru margini
    scaled_dist = distance * scale
    
    x = int(CENTER + scaled_dist * sin(angle * pi / 180))
    y = int(CENTER - scaled_dist * cos(angle * pi / 180))
    
    if 0 <= x < RADAR_SIZE and 0 <= y < RADAR_SIZE:
        if distance < 1000:
            radar[y][x] = '█'  # Aproape (< 1m)
        elif distance < 3000:
            radar[y][x] = '▓'  # Mediu (1-3m)
        else:
            radar[y][x] = '▒'  # Departe (> 3m)

def print_radar(radar, stats):
    clear_screen()
    
    print("="*80)
    print("  M1C1_Mini LiDAR - ASCII Radar Viewer".center(80))
    print("="*80)
    
    print("\n  " + "+" + "-"*RADAR_SIZE + "+")
    for row in radar:
        print("  |" + "".join(row) + "|")
    print("  " + "+" + "-"*RADAR_SIZE + "+")
    
    print("\n  Range: 0-8m | █=<1m  ▓=1-3m  ▒=>3m  ·=grid  ⊕=center")
    
    print(f"\n  Packets: {stats['packets']}")
    print(f"  Objects detected: {stats['objects']}")
    print(f"  Closest: {stats['closest']}mm ({stats['closest']/10:.1f}cm)")
    print(f"  Farthest: {stats['farthest']}mm ({stats['farthest']/10:.1f}cm)")
    print("\n  [Ctrl+C to exit]")

def read_packet(ser):
    while True:
        c = ser.read(1)
        if not c:
            return None
        
        if c[0] == 0xAA:
            c = ser.read(1)
            if c and c[0] == 0x55:
                CT = ord(ser.read(1))
                LSN = ord(ser.read(1))
                
                FSA = int.from_bytes(ser.read(2), byteorder='little')
                LSA = int.from_bytes(ser.read(2), byteorder='little')
                CS = int.from_bytes(ser.read(2), byteorder='little')
                
                samples = []
                for i in range(LSN):
                    Si = int.from_bytes(ser.read(2), byteorder='little')
                    dist = Si >> 2
                    
                    Acorrect = 0 if (dist == 0) else atan(19.16 * (dist - 90.15) / (dist * 90.15))
                    F = (FSA >> 1) / 64.0
                    L = (LSA >> 1) / 64.0
                    angle = F + ((L - F) / LSN) * i - Acorrect
                    
                    samples.append((angle, dist))
                
                return {
                    'type': 'start' if CT == 1 else 'data',
                    'samples': samples
                }

def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    ser.reset_input_buffer()
    time.sleep(1)
    
    stats = {
        'packets': 0,
        'objects': 0,
        'closest': 9999,
        'farthest': 0
    }
    
    full_scan = []
    
    try:
        while True:
            packet = read_packet(ser)
            
            if packet:
                stats['packets'] += 1
                full_scan.extend(packet['samples'])
                if packet['type'] == 'start' and full_scan:
                    radar = create_empty_radar()
                    
                    objects_in_scan = 0
                    closest = 9999
                    farthest = 0
                    
                    for angle, distance in full_scan:
                        if distance > 0:
                            add_point_to_radar(radar, angle, distance)
                            objects_in_scan += 1
                            
                            if distance < closest:
                                closest = distance
                            if distance > farthest:
                                farthest = distance
                    
                    if objects_in_scan > 0:
                        stats['objects'] = objects_in_scan
                        stats['closest'] = closest
                        stats['farthest'] = farthest
                    
                    print_radar(radar, stats)
                    
                    full_scan = []
                    
                    time.sleep(0.1)  # Refresh rate
    
    except KeyboardInterrupt:
        print("\n\n  Stopped by user")
        ser.close()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
WebSocket Bridge for Real-Time LiDAR Scanner (Ultra High Performance)
- Optimized for V5 Dense Scanner
- 16MB UDP Buffer to prevent packet loss
- Burst Mode Reading
- Threaded Point Collection (Zero-Copy)
- Async PLY Export (Background Thread)
"""

import asyncio
import websockets
import socket
import struct
import json
import time
from datetime import datetime
import os
import threading
from queue import Queue
from collections import deque

# Configurare
SCANNER_IP = "10.230.226.8"
SCANNER_PORT = 5005
LOCAL_UDP_PORT = 5006
WEBSOCKET_PORT = 8080
PLY_OUTPUT_DIR = "./scans"

# Performance settings
MAX_POINTS_BUFFER = 1000000  # 1M points max before auto-export
PLY_WRITE_BATCH_SIZE = 10000  # Write in batches for efficiency

class PointCollector:
    """Thread-safe point collector with efficient storage"""
    def __init__(self):
        self.points = []
        self.lock = threading.Lock()
        self.total_collected = 0
        
    def add_batch(self, points_batch):
        """Add a batch of points (called from UDP thread)"""
        with self.lock:
            self.points.extend(points_batch)
            self.total_collected += len(points_batch)
    
    def get_count(self):
        """Thread-safe point count"""
        with self.lock:
            return len(self.points)
    
    def get_all_and_clear(self):
        """Get all points and clear buffer (for export)"""
        with self.lock:
            points_copy = self.points.copy()
            self.points.clear()
            return points_copy
    
    def get_all(self):
        """Get all points without clearing"""
        with self.lock:
            return self.points.copy()

class PLYExporter:
    """Background PLY exporter thread"""
    def __init__(self):
        self.queue = Queue()
        self.thread = None
        self.running = False
        os.makedirs(PLY_OUTPUT_DIR, exist_ok=True)
        
    def start(self):
        """Start background export thread"""
        self.running = True
        self.thread = threading.Thread(target=self._export_worker, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Stop background thread"""
        self.running = False
        self.queue.put(None)  # Signal to stop
        if self.thread:
            self.thread.join(timeout=5)
    
    def queue_export(self, points, timestamp):
        """Queue points for export (non-blocking)"""
        self.queue.put((points, timestamp))
        
    def _export_worker(self):
        """Background worker that writes PLY files"""
        while self.running:
            try:
                item = self.queue.get(timeout=1)
                if item is None:
                    break
                    
                points, timestamp = item
                if len(points) > 0:
                    self._write_ply(points, timestamp)
                    
            except Exception as e:
                if self.running:  # Don't print timeout errors
                    print(f" PLY export error: {e}")
    
    def _write_ply(self, points, timestamp):
        """Write PLY file to disk"""
        try:
            filename = f"scan_{timestamp}.ply"
            filepath = os.path.join(PLY_OUTPUT_DIR, filename)
            
            print(f" Exporting {len(points)} points to {filepath}")
            start_time = time.time()
            
            with open(filepath, 'w') as f:
                # PLY Header
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"comment Created by LiDAR Scanner\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
                f.write("end_header\n")
                
                # Write in batches for better performance
                batch = []
                for i, point in enumerate(points):
                    batch.append(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f} "
                                f"{point['r']} {point['g']} {point['b']}\n")
                    
                    if len(batch) >= PLY_WRITE_BATCH_SIZE:
                        f.writelines(batch)
                        batch.clear()
                
                # Write remaining
                if batch:
                    f.writelines(batch)
            
            elapsed = time.time() - start_time
            print(f" PLY export complete: {filepath} ({elapsed:.2f}s)")
            
        except Exception as e:
            print(f" PLY write failed: {e}")

class UDPToWebSocketBridge:
    def __init__(self):
        self.udp_sock = None
        self.scan_sock = None
        self.websocket = None
        self.running = False
        self.point_collector = PointCollector()
        self.ply_exporter = PLYExporter()
        self.scan_start_time = None
        
    async def handle_client(self, websocket):
        """Handle a single WebSocket client connection"""
        print(f"🔌 Client Web conectat: {websocket.remote_address}")
        self.websocket = websocket
        self.running = True
        self.scan_start_time = datetime.now()
        
        # Start background PLY exporter
        self.ply_exporter.start()
        
        # Setup UDP socket with LARGE BUFFER
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
            buff_size = self.udp_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            print(f" UDP Buffer size set to: {buff_size/1024/1024:.1f} MB")
        except Exception as e:
            print(f" Could not set socket buffer: {e}")

        self.udp_sock.bind(("0.0.0.0", LOCAL_UDP_PORT))
        self.udp_sock.setblocking(False)
        
        # Setup scanner command socket
        self.scan_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Send initial scan command
        #try:
            #bytes_sent = self.scan_sock.sendto(b"SCAN:GRID", (SCANNER_IP, SCANNER_PORT))
            #print(f" Comandă SCAN trimisă: {bytes_sent} bytes către {SCANNER_IP}:{SCANNER_PORT}")
            #await asyncio.sleep(0.05)
        #except Exception as e:
            #print(f" Error sending SCAN command: {e}")
        
        # Send initial status to web client
        await websocket.send(json.dumps({
            'type': 'connection',
            'msg': 'Connected (Ultra High Performance + Background PLY Export)'
        }))
        
        try:
            await asyncio.gather(self.relay_loop(), self.command_listener(websocket))
        except websockets.exceptions.ConnectionClosed:
            print(" Client deconectat")
        except Exception as e:
            print(f" Error: {e}")
        finally:
            await self.cleanup()
    
    async def relay_loop(self):
        """Main loop: Maximum throughput UDP -> WebSocket"""
        last_heartbeat = time.time()
        last_point_report = 0
        
        while self.running:
            packets_processed = 0
            
            try:
                # --- ULTRA BURST MODE ---
                # Process as many packets as possible without yielding
                while True:
                    try:
                        data, addr = self.udp_sock.recvfrom(65536)
                        packets_processed += 1
                        
                        if data.startswith(b'PTS:'):
                            await self.handle_point_packet(data)
                        elif data.startswith(b'STATUS:'):
                            await self.handle_status_packet(data)
                        elif data.startswith(b'TEMP:'):
                            await self.handle_temperature_packet(data)
                        elif data.startswith(b'SCAN_COMPLETE'):
                            await self.handle_scan_complete()
                        
                        # Yield less frequently - only every 100 packets
                        if packets_processed % 100 == 0:
                            await asyncio.sleep(0)
                            
                    except BlockingIOError:
                        break
                        
            except Exception as e:
                print(f" Relay error: {e}")
            
            # Minimal sleep if no packets
            if packets_processed == 0:
                await asyncio.sleep(0.0005)  # 500us instead of 1ms
            
            # Heartbeat with stats
            now = time.time()
            if now - last_heartbeat > 2.0:
                try:
                    point_count = self.point_collector.get_count()
                    await self.websocket.send(json.dumps({
                        'type': 'heartbeat',
                        'timestamp': now,
                        'points_collected': point_count
                    }))
                    
                    # Print collection rate
                    if point_count > last_point_report:
                        rate = (point_count - last_point_report) / (now - last_heartbeat)
                        print(f" Collection rate: {rate:.0f} points/sec | Total: {point_count}")
                        last_point_report = point_count
                    
                    last_heartbeat = now
                except:
                    break
    async def command_listener(self, websocket):
        """Read commands coming FROM the browser and forward them to the scanner."""
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                await self.handle_command(msg)
        finally:
            self.running = False  # socket closed -> let the relay loop exit too

    async def handle_command(self, msg):
        cmd = msg.get('cmd')
        if cmd == 'start':
            mode = msg.get('mode', 'grid')
            payload = b"SCAN:GRID" if mode == 'grid' else b"SCAN"
            self.scan_sock.sendto(payload, (SCANNER_IP, SCANNER_PORT))
            print(f"▶️  START ({mode}) -> scanner")
        elif cmd == 'stop':
            self.scan_sock.sendto(b"STOP", (SCANNER_IP, SCANNER_PORT))
            print("⏹️  STOP -> scanner")

    async def handle_point_packet(self, data):
        """Parse point cloud data - OPTIMIZED"""
        try:
            if len(data) < 8:
                return
            
            count = struct.unpack('I', data[4:8])[0]
            offset = 8
            
            # Pre-allocate lists
            points_for_ws = []
            points_for_collector = []
            
            # Single pass through data
            for _ in range(count):
                if offset + 15 > len(data):
                    break
                
                # Unpack once
                x, y, z, r, g, b = struct.unpack('fffBBB', data[offset:offset+15])
                
                # For WebSocket (rounded)
                points_for_ws.append({
                    'x': round(x, 1),
                    'y': round(y, 1),
                    'z': round(z, 1),
                    'r': r, 'g': g, 'b': b
                })
                
                # For PLY collector (full precision)
                points_for_collector.append({
                    'x': x, 'y': y, 'z': z,
                    'r': r, 'g': g, 'b': b
                })
                
                offset += 15
            
            # Batch add to collector (thread-safe)
            if points_for_collector:
                self.point_collector.add_batch(points_for_collector)
            
            # Send to WebSocket
            if points_for_ws:
                await self.websocket.send(json.dumps({
                    'type': 'points',
                    'count': len(points_for_ws),
                    'data': points_for_ws
                }))
                
        except Exception as e:
            pass  # Silent failures to avoid console spam
    
    async def handle_status_packet(self, data):
        try:
            msg = data.decode('utf-8', errors='ignore').split(':', 1)[1]
            await self.websocket.send(json.dumps({'type': 'status', 'msg': msg}))
            print(f" Status: {msg}")
        except:
            pass
    
    async def handle_temperature_packet(self, data):
        try:
            payload = data.decode('utf-8', errors='ignore').split(':', 1)[1]
            values = payload.split(',')
            if len(values) >= 4:
                await self.websocket.send(json.dumps({
                    'type': 'telemetry',
                    'temp1': float(values[0]),
                    'temp2': float(values[1]),
                    'fan1': int(values[2]),
                    'fan2': int(values[3])
                }))
        except:
            pass
    
    async def handle_scan_complete(self):
        """Handle scan completion - queue PLY export in background"""
        try:
            print(" Scan complete signal received")
            
            point_count = self.point_collector.get_count()
            timestamp = self.scan_start_time.strftime("%Y%m%d_%H%M%S")
            
            # Get points and queue for background export
            points = self.point_collector.get_all_and_clear()
            self.ply_exporter.queue_export(points, timestamp)
            
            # Notify client immediately (export happens in background)
            await self.websocket.send(json.dumps({
                'type': 'scan_complete',
                'points_total': point_count,
                'ply_file': f"scan_{timestamp}.ply",
                'status': 'exporting_background'
            }))
            
            print(f" Queued {point_count} points for background export")
            
        except Exception as e:
            print(f" Error in scan completion: {e}")
    
    async def cleanup(self):
        print(" Cleanup...")
        self.running = False
        
        # Queue final export if we have points
        point_count = self.point_collector.get_count()
        if point_count > 0:
            print(f"Queuing final export: {point_count} points")
            timestamp = self.scan_start_time.strftime("%Y%m%d_%H%M%S")
            points = self.point_collector.get_all()
            self.ply_exporter.queue_export(points, timestamp)
        
        # Stop background exporter (will finish current export)
        self.ply_exporter.stop()
        
        if self.scan_sock:
            try:
                self.scan_sock.sendto(b"STOP", (SCANNER_IP, SCANNER_PORT))
            except:
                pass
            self.scan_sock.close()
        
        if self.udp_sock:
            self.udp_sock.close()
        
        print("Cleanup complete")

async def main():
    print("=" * 70)
    print(" WebSocket Bridge V6 (ULTRA HIGH PERFORMANCE + THREADED PLY)")
    print("=" * 70)
    print(f" PLY files: {os.path.abspath(PLY_OUTPUT_DIR)}")
    print(f" Background export: ENABLED")
    print(f" Zero-copy point collection: ENABLED")
    print("=" * 70)
    
    bridge = UDPToWebSocketBridge()
    
    async with websockets.serve(
        bridge.handle_client,
        "0.0.0.0",
        WEBSOCKET_PORT,
        max_size=None,
        ping_interval=None
    ):
        print(f" Server activ pe port {WEBSOCKET_PORT}")
        print(" Aștept conexiuni de la clienți web...")
        await asyncio.get_running_loop().create_future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n Server oprit manual")
    except Exception as e:
        print(f"\n Server error: {e}")
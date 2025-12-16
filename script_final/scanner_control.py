#!/usr/bin/env python3
"""
WebSocket Bridge for Real-Time LiDAR Scanner (High Performance)
- Optimized for V5 Dense Scanner
- 16MB UDP Buffer to prevent packet loss
- Burst Mode Reading
"""

import asyncio
import websockets
import socket
import struct
import json
import time

# Configurare
SCANNER_IP = "10.115.82.8"  # IP-ul Local (daca ruleaza pe acelasi Pi) sau IP-ul Pi-ului
SCANNER_PORT = 5005
LOCAL_UDP_PORT = 5006
WEBSOCKET_PORT = 8080

class UDPToWebSocketBridge:
    def __init__(self):
        self.udp_sock = None
        self.scan_sock = None
        self.websocket = None
        self.running = False
        
    async def handle_client(self, websocket):
        """Handle a single WebSocket client connection"""
        print(f"🔌 Client Web conectat: {websocket.remote_address}")
        self.websocket = websocket
        self.running = True
        
        # Setup UDP socket with LARGE BUFFER
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # --- CRITICAL FIX: Increase OS UDP Receive Buffer to 16MB ---
        try:
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
            buff_size = self.udp_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            print(f"✅ UDP Buffer size set to: {buff_size/1024/1024:.1f} MB")
        except Exception as e:
            print(f"⚠️  Could not set socket buffer: {e}")

        self.udp_sock.bind(("0.0.0.0", LOCAL_UDP_PORT))
        self.udp_sock.setblocking(False)
        
        # Setup scanner command socket
        self.scan_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Send initial scan command
        self.scan_sock.sendto(b"SCAN:GRID", (SCANNER_IP, SCANNER_PORT))
        print("📡 Comandă SCAN trimisă la scaner")
        
        # Send initial status to web client
        await websocket.send(json.dumps({
            'type': 'connection',
            'msg': 'Connected to scanner (High Performance Mode)'
        }))
        
        try:
            # Main UDP -> WebSocket relay loop
            await self.relay_loop()
            
        except websockets.exceptions.ConnectionClosed:
            print("❌ Client deconectat")
        except Exception as e:
            print(f"⚠️  Error: {e}")
        finally:
            await self.cleanup()
    
    async def relay_loop(self):
        """Main loop: Reads ALL available UDP packets before yielding"""
        last_heartbeat = time.time()
        
        while self.running:
            # --- BURST READ MODE ---
            # Read as many packets as possible to clear the OS buffer
            packets_processed = 0
            
            try:
                while True:
                    try:
                        # Non-blocking read
                        data, addr = self.udp_sock.recvfrom(65536)
                        packets_processed += 1
                        
                        if data.startswith(b'PTS:'):
                            await self.handle_point_packet(data)
                        elif data.startswith(b'STATUS:'):
                            await self.handle_status_packet(data)
                        elif data.startswith(b'TEMP:'):
                            await self.handle_temperature_packet(data)
                            
                        # Yield to event loop every 50 packets to allow WebSocket to send
                        # This prevents the loop from blocking heartbeat/sending too long
                        if packets_processed % 50 == 0:
                            await asyncio.sleep(0)
                            
                    except BlockingIOError:
                        # No more data in buffer, break inner loop and wait
                        break
                        
            except Exception as e:
                print(f"⚠️  Relay error: {e}")
            
            # If we processed no packets, sleep briefly to prevent CPU burn
            if packets_processed == 0:
                await asyncio.sleep(0.001)
            
            # Send periodic heartbeat (every 2s)
            if time.time() - last_heartbeat > 2.0:
                try:
                    await self.websocket.send(json.dumps({
                        'type': 'heartbeat',
                        'timestamp': time.time()
                    }))
                    last_heartbeat = time.time()
                except:
                    break
    
    async def handle_point_packet(self, data):
        """Parse and forward point cloud data"""
        try:
            # Format: 'PTS:' (4 bytes) + COUNT (4 bytes) + POINTS (15 bytes each)
            if len(data) < 8: return
            
            count = struct.unpack('I', data[4:8])[0]
            points = []
            offset = 8
            
            # Pre-calculate struct format size (15 bytes)
            # Struct: float(4) + float(4) + float(4) + u8(1) + u8(1) + u8(1)
            
            for _ in range(count):
                if offset + 15 > len(data):
                    break
                    
                chunk = data[offset:offset+15]
                # Unpack: x, y, z, r, g, b
                x, y, z, r, g, b = struct.unpack('fffBBB', chunk)
                
                points.append({
                    'x': round(x, 1), # Reduce JSON size by rounding
                    'y': round(y, 1),
                    'z': round(z, 1),
                    'r': r,
                    'g': g,
                    'b': b
                })
                offset += 15
            
            if points:
                # Send to WebSocket
                await self.websocket.send(json.dumps({
                    'type': 'points',
                    'count': len(points),
                    'data': points
                }))
                
        except Exception as e:
            # Don't print for every error to avoid console lag
            pass
    
    async def handle_status_packet(self, data):
        try:
            msg = data.decode('utf-8', errors='ignore').split(':', 1)[1]
            await self.websocket.send(json.dumps({'type': 'status', 'msg': msg}))
            print(f"📊 Status: {msg}")
        except: pass
    
    async def handle_temperature_packet(self, data):
        try:
            # Format: TEMP:temp1,temp2,fan1,fan2
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
                # Optional: Print to console occasionally
        except: pass
    
    async def cleanup(self):
        print("🧹 Cleanup...")
        self.running = False
        if self.scan_sock:
            try:
                self.scan_sock.sendto(b"STOP", (SCANNER_IP, SCANNER_PORT))
            except: pass
            self.scan_sock.close()
        if self.udp_sock:
            self.udp_sock.close()

async def main():
    print("=" * 60)
    print("🚀 WebSocket Bridge V5 (High Speed)")
    print("=" * 60)
    
    bridge = UDPToWebSocketBridge()
    
    # Increase WebSocket max size limit just in case
    async with websockets.serve(
        bridge.handle_client, 
        "0.0.0.0", 
        WEBSOCKET_PORT,
        max_size=None, 
        ping_interval=None
    ):
        print(f"✅ Server activ pe port {WEBSOCKET_PORT}")
        print("⏳ Aștept conexiuni de la clienți web...")
        await asyncio.get_running_loop().create_future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Server oprit manual")
    except Exception as e:
        print(f"\n❌ Server error: {e}")
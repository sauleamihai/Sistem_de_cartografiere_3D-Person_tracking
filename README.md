# Sistem Real-Time de Cartografiere 3D și Urmărire Persoane

Sistem de cartografiere 3D și urmărire multi-persoane în timp real pentru medii interioare, utilizând fuziune senzorială LiDAR-cameră pe Raspberry Pi 5 cu kernel PREEMPT_RT.

## Prezentare

Senzorul LiDAR M1C1 (UART 230400 baud) montat pe platformă pan-tilt cu servomotoare permite scanarea sferică completă. Raspberry Pi Camera Module 3 (CSI-2) realizează detecția persoanelor prin OpenCV/YOLO. Fuziunea datelor mapează coordonatele 2D cu distanțe 3D precise, rezultând acuratețe de urmărire ±5-10cm.

Funcționalitatea real-time este garantată prin kernel PREEMPT_RT cu izolarea CPU cores 2-3, programare SCHED_FIFO (P95 pentru LiDAR/encodere, P90 pentru fuziune) și memory locking, atingând latență deterministă sub 100ms. Două encodere rotative (600 PPR) furnizează feedback closed-loop pentru precizie de poziționare ±0.1°.

## Specificații Tehnice

- **Platformă**: Raspberry Pi 5 (8GB RAM, kernel PREEMPT_RT)
- **LiDAR**: M1C1 ToF (12m, ±2cm, UART 230400 baud)
- **Cameră**: Pi Camera Module 3 (12MP, CSI-2, 1080p@60fps)
- **Encodere**: 2× 600 PPR rotative, quadrature
- **Servomotoare**: 2× MG996R, PWM hardware
- **Latență**: <100ms end-to-end, <50μs kernel
- **Acuratețe**: ±5-10cm poziție 3D, ±0.1° unghi
- **Comunicație**: UART, CSI-2, I2C, PWM, WiFi 802.11ac

## Arhitectură

### Componente Hardware

| Componentă | Model | Interfață |
|-----------|-------|-----------|
| SBC | Raspberry Pi 5 8GB | - |
| LiDAR | M1C1 (12m) | UART (GPIO 14/15) |
| Cameră | Pi Camera Module 3 | CSI-2 |
| Encodere | 2× 600 PPR | GPIO 17,27,22 + 23,24,25 |
| Servomotoare | 2× MG996R | PWM (GPIO 12/13) |
| IMU | MPU6050 (opțional) | I2C (GPIO 2/3) |

### Priorități Real-Time
```
P95: LiDAR UART + Encodere    Nucleu 2 (izolat)
P90: Fuziune & Tracking        Nucleu 3 (izolat)
P85: Point Cloud               Nucleu 3 (izolat)
P80: Control PWM Servomotoare  Nucleu 1
P75: Detecție YOLO             Nucleu 1
P50: ROS 2 Middleware          Nucleu 0
```
### Diagrama electrica 
![diagrama_electrica](https://github.com/user-attachments/assets/2cf363c5-7584-4f55-a7d3-8d71baac3c18)
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1400 950" style="background-color: #ffffff;">
  <!-- Title -->
  <text x="700" y="35" font-family="Arial, sans-serif" font-size="22" font-weight="bold" text-anchor="middle" fill="#1a1a1a">
    Diagrama Electrică - Sistem RT de Cartografiere 3D
  </text>
  <text x="700" y="58" font-family="Arial, sans-serif" font-size="13" text-anchor="middle" fill="#666666">
    Raspberry Pi 5 cu Kernel PREEMPT_RT | Izolarea Nucleelor 2-3
  </text>

  <!-- Raspberry Pi 5 Central Unit -->
  <g id="raspberry-pi">
    <rect x="550" y="150" width="300" height="550" rx="8" fill="#2c5aa0" stroke="#1a4278" stroke-width="3"/>
    <text x="700" y="180" font-family="Arial, sans-serif" font-size="17" font-weight="bold" text-anchor="middle" fill="white">
      Raspberry Pi 5
    </text>
    <text x="700" y="200" font-family="Arial, sans-serif" font-size="11" text-anchor="middle" fill="#d0e0f0">
      8GB RAM | BCM2712 Quad-Core @ 2.4GHz
    </text>
    
    <!-- GPIO Pin Section -->
    <rect x="570" y="220" width="260" height="420" rx="4" fill="#1a4278" opacity="0.3"/>
    <text x="700" y="240" font-family="Arial, sans-serif" font-size="12" font-weight="bold" text-anchor="middle" fill="white">
      Alocarea GPIO
    </text>
    
    <!-- UART Section -->
    <text x="580" y="265" font-family="Arial, sans-serif" font-size="10" font-weight="bold" fill="#ffeb99">UART (LiDAR):</text>
    <circle cx="590" cy="285" r="6" fill="#e74c3c"/>
    <text x="605" y="290" font-family="monospace" font-size="9" fill="white">GPIO 14 (TXD)</text>
    <circle cx="590" cy="305" r="6" fill="#e67e22"/>
    <text x="605" y="310" font-family="monospace" font-size="9" fill="white">GPIO 15 (RXD)</text>
    
    <!-- PWM Section -->
    <text x="580" y="335" font-family="Arial, sans-serif" font-size="10" font-weight="bold" fill="#ffeb99">PWM Hardware:</text>
    <circle cx="590" cy="355" r="6" fill="#9b59b6"/>
    <text x="605" y="360" font-family="monospace" font-size="9" fill="white">GPIO 12 (PWM0) Pan</text>
    <circle cx="590" cy="375" r="6" fill="#8e44ad"/>
    <text x="605" y="380" font-family="monospace" font-size="9" fill="white">GPIO 13 (PWM1) Tilt</text>
    
    <!-- Encoder Section -->
    <text x="580" y="405" font-family="Arial, sans-serif" font-size="10" font-weight="bold" fill="#ffeb99">Encodere (600 PPR):</text>
    <circle cx="590" cy="425" r="6" fill="#27ae60"/>
    <text x="605" y="430" font-family="monospace" font-size="9" fill="white">GPIO 17 Pan A</text>
    <circle cx="590" cy="445" r="6" fill="#27ae60"/>
    <text x="605" y="450" font-family="monospace" font-size="9" fill="white">GPIO 27 Pan B</text>
    <circle cx="590" cy="465" r="6" fill="#27ae60"/>
    <text x="605" y="470" font-family="monospace" font-size="9" fill="white">GPIO 22 Pan Index</text>
    
    <circle cx="590" cy="490" r="6" fill="#229954"/>
    <text x="605" y="495" font-family="monospace" font-size="9" fill="white">GPIO 23 Tilt A</text>
    <circle cx="590" cy="510" r="6" fill="#229954"/>
    <text x="605" y="515" font-family="monospace" font-size="9" fill="white">GPIO 24 Tilt B</text>
    <circle cx="590" cy="530" r="6" fill="#229954"/>
    <text x="605" y="535" font-family="monospace" font-size="9" fill="white">GPIO 25 Tilt Index</text>
    
    <!-- I2C Section -->
    <text x="580" y="560" font-family="Arial, sans-serif" font-size="10" font-weight="bold" fill="#ffeb99">I2C (IMU Optional):</text>
    <circle cx="590" cy="580" r="6" fill="#3498db"/>
    <text x="605" y="585" font-family="monospace" font-size="9" fill="white">GPIO 2 (SDA1)</text>
    <circle cx="590" cy="600" r="6" fill="#2980b9"/>
    <text x="605" y="605" font-family="monospace" font-size="9" fill="white">GPIO 3 (SCL1)</text>
    
    <!-- Power Control -->
    <text x="580" y="630" font-family="Arial, sans-serif" font-size="10" font-weight="bold" fill="#ffeb99">Control:</text>
    <circle cx="590" cy="650" r="6" fill="#f39c12"/>
    <text x="605" y="655" font-family="monospace" font-size="9" fill="white">GPIO 26 Power Timer</text>
  </g>

  <!-- CSI Camera Port -->
  <g id="csi-port">
    <rect x="870" y="280" width="100" height="60" rx="5" fill="#34495e" stroke="#2c3e50" stroke-width="2"/>
    <text x="920" y="305" font-family="Arial, sans-serif" font-size="11" font-weight="bold" text-anchor="middle" fill="white">
      Port CSI-2
    </text>
    <text x="920" y="325" font-family="monospace" font-size="8" text-anchor="middle" fill="#ecf0f1">
      4-lane
    </text>
    <text x="920" y="338" font-family="monospace" font-size="8" text-anchor="middle" fill="#ecf0f1">
      10 Gbps
    </text>
  </g>

  <!-- LiDAR M1C1 -->
  <g id="lidar">
    <rect x="120" y="200" width="220" height="160" rx="7" fill="#e74c3c" stroke="#c0392b" stroke-width="3"/>
    <text x="230" y="230" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      LiDAR M1C1
    </text>
    <text x="230" y="248" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#ffe6e6">
      Time-of-Flight 12m
    </text>
    
    <rect x="140" y="265" width="180" height="85" rx="3" fill="#c0392b" opacity="0.4"/>
    <text x="150" y="285" font-family="monospace" font-size="9" fill="white">VCC (5V) ← Alimentare</text>
    <text x="150" y="303" font-family="monospace" font-size="9" fill="white">GND ← Masă</text>
    <text x="150" y="321" font-family="monospace" font-size="9" fill="white">TX → GPIO 15 (RXD)</text>
    <circle cx="320" cy="317" r="5" fill="#e67e22"/>
    <text x="150" y="339" font-family="monospace" font-size="9" fill="white">RX ← GPIO 14 (TXD)</text>
    <circle cx="320" cy="335" r="5" fill="#e74c3c"/>
    
    <rect x="130" y="355" width="200" height="20" rx="3" fill="#34495e" opacity="0.7"/>
    <text x="230" y="369" font-family="monospace" font-size="8" text-anchor="middle" fill="#ecf0f1">
      230400 baud | 8N1 | DMA | P95
    </text>
  </g>

  <!-- Servomotor Pan -->
  <g id="servo-pan">
    <rect x="120" y="410" width="220" height="120" rx="7" fill="#9b59b6" stroke="#8e44ad" stroke-width="3"/>
    <text x="230" y="440" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Servomotor Pan
    </text>
    <text x="230" y="458" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#e8d5f0">
      MG996R | ±180°
    </text>
    
    <rect x="140" y="475" width="180" height="45" rx="3" fill="#8e44ad" opacity="0.4"/>
    <text x="150" y="492" font-family="monospace" font-size="9" fill="white">PWM ← GPIO 12 (50Hz)</text>
    <circle cx="320" cy="488" r="5" fill="#9b59b6"/>
    <text x="150" y="510" font-family="monospace" font-size="9" fill="white">VCC/GND ← Rail 5V/2A</text>
  </g>

  <!-- Servomotor Tilt -->
  <g id="servo-tilt">
    <rect x="120" y="560" width="220" height="120" rx="7" fill="#8e44ad" stroke="#7d3c98" stroke-width="3"/>
    <text x="230" y="590" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Servomotor Tilt
    </text>
    <text x="230" y="608" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#e8d5f0">
      MG996R | ±90°
    </text>
    
    <rect x="140" y="625" width="180" height="45" rx="3" fill="#7d3c98" opacity="0.4"/>
    <text x="150" y="642" font-family="monospace" font-size="9" fill="white">PWM ← GPIO 13 (50Hz)</text>
    <circle cx="320" cy="638" r="5" fill="#8e44ad"/>
    <text x="150" y="660" font-family="monospace" font-size="9" fill="white">VCC/GND ← Rail 5V/2A</text>
  </g>

  <!-- Encoder Pan -->
  <g id="encoder-pan">
    <rect x="120" y="720" width="220" height="100" rx="7" fill="#27ae60" stroke="#229954" stroke-width="3"/>
    <text x="230" y="750" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Encoder Pan
    </text>
    <text x="230" y="768" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#d5f4e6">
      600 PPR Incremental
    </text>
    
    <rect x="140" y="780" width="180" height="30" rx="3" fill="#229954" opacity="0.4"/>
    <text x="150" y="798" font-family="monospace" font-size="8" fill="white">A→GPIO 17 | B→GPIO 27 | I→GPIO 22</text>
    <circle cx="320" cy="794" r="4" fill="#27ae60"/>
  </g>

  <!-- Encoder Tilt -->
  <g id="encoder-tilt">
    <rect x="120" y="850" width="220" height="80" rx="7" fill="#229954" stroke="#1e7e34" stroke-width="3"/>
    <text x="230" y="880" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Encoder Tilt
    </text>
    <text x="230" y="898" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#d5f4e6">
      600 PPR Incremental
    </text>
    
    <text x="150" y="918" font-family="monospace" font-size="8" fill="white">A→GPIO 23 | B→GPIO 24 | I→GPIO 25</text>
    <circle cx="320" cy="914" r="4" fill="#229954"/>
  </g>

  <!-- Camera -->
  <g id="camera">
    <rect x="1040" y="230" width="240" height="140" rx="7" fill="#3498db" stroke="#2980b9" stroke-width="3"/>
    <text x="1160" y="265" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Pi Camera Module 3
    </text>
    <text x="1160" y="285" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#d6eaf8">
      12MP | Autofocus | HDR
    </text>
    
    <rect x="1060" y="300" width="200" height="60" rx="3" fill="#2980b9" opacity="0.4"/>
    <text x="1070" y="320" font-family="monospace" font-size="9" fill="white">Interfață: CSI-2 (4-lane)</text>
    <text x="1070" y="338" font-family="monospace" font-size="9" fill="white">Rezoluție: 1080p @ 60fps</text>
    <text x="1070" y="356" font-family="monospace" font-size="9" fill="white">Latență: <10ms | DMA | P75</text>
  </g>

  <!-- IMU -->
  <g id="imu">
    <rect x="1040" y="410" width="240" height="130" rx="7" fill="#3498db" stroke="#2980b9" stroke-width="3"/>
    <text x="1160" y="445" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      IMU MPU6050
    </text>
    <text x="1160" y="463" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#d6eaf8">
      Opțional | Gyro + Accel
    </text>
    
    <rect x="1060" y="478" width="200" height="52" rx="3" fill="#2980b9" opacity="0.4"/>
    <text x="1070" y="497" font-family="monospace" font-size="9" fill="white">SDA → GPIO 2 (I2C1)</text>
    <circle cx="1020" cy="493" r="5" fill="#3498db"/>
    <text x="1070" y="515" font-family="monospace" font-size="9" fill="white">SCL → GPIO 3 (I2C1)</text>
    <circle cx="1020" cy="511" r="5" fill="#2980b9"/>
    <text x="1070" y="533" font-family="monospace" font-size="8" fill="white">Addr: 0x68 | 400kHz | P70</text>
  </g>

  <!-- Power Supply -->
  <g id="power">
    <rect x="1040" y="580" width="240" height="160" rx="7" fill="#f39c12" stroke="#d68910" stroke-width="3"/>
    <text x="1160" y="615" font-family="Arial, sans-serif" font-size="15" font-weight="bold" text-anchor="middle" fill="white">
      Alimentare
    </text>
    
    <rect x="1060" y="635" width="200" height="95" rx="3" fill="#d68910" opacity="0.4"/>
    <text x="1160" y="655" font-family="monospace" font-size="10" font-weight="bold" text-anchor="middle" fill="white">
      Sursă 5V 4A (20W)
    </text>
    <text x="1070" y="675" font-family="monospace" font-size="8" fill="white">USB-C → Raspberry Pi 5</text>
    <text x="1070" y="691" font-family="monospace" font-size="8" fill="white">Rail 5V → Servomotoare (4W)</text>
    <text x="1070" y="707" font-family="monospace" font-size="8" fill="white">Rail 5V → LiDAR (0.5W)</text>
    <text x="1070" y="723" font-family="monospace" font-size="8" fill="white">Consum total: 3.5W idle, 12W peak</text>
  </g>

  <!-- Connection Lines -->
  <!-- UART -->
  <line x1="340" y1="317" x2="590" y2="305" stroke="#e67e22" stroke-width="2.5" marker-end="url(#arrow)"/>
  <line x1="340" y1="335" x2="590" y2="285" stroke="#e74c3c" stroke-width="2.5" marker-end="url(#arrow)"/>
  
  <!-- PWM -->
  <line x1="340" y1="488" x2="590" y2="355" stroke="#9b59b6" stroke-width="2.5" marker-end="url(#arrow)"/>
  <line x1="340" y1="638" x2="590" y2="375" stroke="#8e44ad" stroke-width="2.5" marker-end="url(#arrow)"/>
  
  <!-- Encoders -->
  <line x1="340" y1="794" x2="590" y2="435" stroke="#27ae60" stroke-width="2" marker-end="url(#arrow)"/>
  <line x1="340" y1="914" x2="590" y2="500" stroke="#229954" stroke-width="2" marker-end="url(#arrow)"/>
  
  <!-- I2C -->
  <line x1="1020" y1="493" x2="590" y2="580" stroke="#3498db" stroke-width="2" marker-end="url(#arrow)"/>
  <line x1="1020" y1="511" x2="590" y2="600" stroke="#2980b9" stroke-width="2" marker-end="url(#arrow)"/>
  
  <!-- CSI -->
  <line x1="1040" y1="310" x2="970" y2="310" stroke="#34495e" stroke-width="3" marker-end="url(#arrow)"/>

  <!-- Legend -->
  <g id="legend">
    <rect x="380" y="20" width="940" height="85" rx="5" fill="#f8f9fa" stroke="#bdc3c7" stroke-width="2"/>
    <text x="850" y="45" font-family="Arial, sans-serif" font-size="13" font-weight="bold" text-anchor="middle" fill="#2c3e50">
      Legenda Conexiuni
    </text>
    
    <line x1="400" y1="60" x2="450" y2="60" stroke="#e74c3c" stroke-width="2.5"/>
    <text x="460" y="65" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">UART (230400 baud, DMA)</text>
    
    <line x1="660" y1="60" x2="710" y2="60" stroke="#9b59b6" stroke-width="2.5"/>
    <text x="720" y="65" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">PWM Hardware (50Hz)</text>
    
    <line x1="920" y1="60" x2="970" y2="60" stroke="#27ae60" stroke-width="2"/>
    <text x="980" y="65" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">Encodere (Quadrature)</text>
    
    <line x1="1170" y1="60" x2="1220" y2="60" stroke="#3498db" stroke-width="2"/>
    <text x="1230" y="65" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">I2C (400kHz)</text>
    
    <line x1="400" y1="85" x2="450" y2="85" stroke="#34495e" stroke-width="3"/>
    <text x="460" y="90" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">CSI-2 (10 Gbps, 4-lane)</text>
    
    <line x1="710" y1="85" x2="760" y2="85" stroke="#f39c12" stroke-width="2.5"/>
    <text x="770" y="90" font-family="Arial, sans-serif" font-size="10" fill="#2c3e50">Alimentare 5V</text>
  </g>

  <!-- Arrow marker -->
  <defs>
    <marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto">
      <polygon points="0 0, 8 3, 0 6" fill="#34495e"/>
    </marker>
  </defs>

  <!-- Footer -->
  <text x="700" y="945" font-family="Arial, sans-serif" font-size="10" text-anchor="middle" fill="#7f8c8d">
    Sistem RT de Cartografiere 3D | Versiune Hardware 1.0 | Kernel PREEMPT_RT
  </text>
</svg>


![diagrama_electrica](https://github.com/user-attachments/assets/ffc89975-4b0d-41bc-8c6c-76fb02ca8f77)

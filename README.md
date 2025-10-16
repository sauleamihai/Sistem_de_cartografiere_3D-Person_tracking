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
<img width="1220" height="795" alt="image" src="https://github.com/user-attachments/assets/7804b242-13bb-4cf3-8e83-629972862c23" />


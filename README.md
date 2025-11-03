# Secure Multi-Node IoT System – Thesis Firmware and Tools

This repository contains all firmware, software, and analysis tools for the master’s thesis  
**“Scalable and Secure RoboRebound-based Multi-Robot Embedded Systems with Edge Intelligence.”**

The project extends the **RoboRebound** protocol—a real-time coordination framework for distributed robots—by adding:
1. **Scalability Testing:** evaluate throughput, jitter, and broker load with 10 + robots.  
2. **Edge ML Perception:** deploy MobileNet / YOLOv5-nano inference on Raspberry Pi 5 for improved vision.  
3. **Enhanced Security:** add TLS 1.3 + ECC to existing HMAC authentication.  
4. **Energy Profiling:** characterize STM32 / Raspberry Pi power draw under crypto + ML workloads.  
5. **PCB & Firmware Validation:** integrate and benchmark the RoboRebound custom board.


---

## 🧩 Core Components

| Component | Platform | Description |
|------------|-----------|-------------|
| **RoboRebound Protocol** | STM32 + ESP32 + Raspberry Pi | Deterministic coordination with bounded-time message exchange |
| **S-Node Firmware** | STM32H753ZI | Sensor acquisition, SHA-256, UART TX to C-Node |
| **C-Node Software** | Raspberry Pi 5 | UART RX → hash verify → ML inference → MQTT publish |
| **Wireless Module** | ESP32-C6 | TLS 1.3 + ECC secure MQTT communication |
| **Edge ML** | TensorFlow Lite / YOLOv5-nano | Object detection / event classification on Pi |
| **Power Profiler** | INA219 / Joulescope | Energy measurement during crypto + ML workloads |
| **Broker & Monitoring** | Mosquitto + Python metrics | Throughput, jitter, and packet-loss logging |



---

## 🧩 Active Development Components

1. STM32 bring-up (UART + HASH): done
2. UART framing + C-Node receover: in progress
3. ML Pipeline: In progress
4. TLS 1.3 ECC integration
5. Energy profiling + optimization
6. Full fleet demo (10 robots)


---

## 🧠 S-Node Hardware Summary

| Feature | Details |
|----------|----------|
| MCU | STM32H753ZIT6, Cortex-M7 @ 400 MHz |
| Board | NUCLEO-H753ZI |
| UART | USART3 @ 115200 8N1 (PD8 TX, PD9 RX via ST-LINK VCP) |
| LEDs | PB0 (Green), PE1 (Yellow), PB14 (Red) |
| Crypto | Hardware HASH peripheral (SHA-1/SHA-224/SHA-256/MD5) |
| RNG | On-chip True Random Number Generator |

---


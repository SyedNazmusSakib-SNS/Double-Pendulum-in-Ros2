# 📡 ROS 2 QoS (Quality of Service) Demo

A comprehensive educational package to understand **ROS 2 communication quality parameters**:
Frequency, Latency, Buffer/Queue Size, Reliability, and Bandwidth.

---

## 📋 Table of Contents

1. [Overview](#-overview)
2. [Quick Start](#-quick-start)
3. [Concepts Explained](#-concepts-explained-in-detail)
   - [Frequency](#1%EF%B8%8F⃣-frequency-publishing-rate)
   - [Latency](#2%EF%B8%8F⃣-latency-message-delay)
   - [Queue Size / Buffer](#3%EF%B8%8F⃣-queue-size--buffer)
   - [Reliability](#4%EF%B8%8F⃣-reliability)
   - [QoS Profiles](#5%EF%B8%8F⃣-qos-profiles)
   - [Bandwidth](#6%EF%B8%8F⃣-bandwidth)
4. [Experiments](#-experiments-to-try)
5. [Monitoring Commands](#-monitoring-commands)
6. [Common Issues](#-common-issues--solutions)

---

## 🎯 Overview

This package provides two configurable nodes:

```
┌────────────────────────┐                    ┌────────────────────────┐
│  CONFIGURABLE          │   /sensor_data     │  CONFIGURABLE          │
│  PUBLISHER             │ ─────────────────▶ │  SUBSCRIBER            │
│                        │                    │                        │
│  Parameters:           │                    │  Parameters:           │
│  • frequency (Hz)      │                    │  • queue_size          │
│  • queue_size          │                    │  • reliable            │
│  • reliable            │                    │  • simulate_slow       │
│  • message_size        │                    │                        │
│                        │                    │  Tracks:               │
│  Publishes stats to:   │                    │  • Latency (ms)        │
│  /publisher_stats      │                    │  • Dropped messages    │
└────────────────────────┘                    │  • Receive rate        │
                                              └────────────────────────┘
```

---

## 🚀 Quick Start

### Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qos_demo
source install/setup.bash
```

### Run with Defaults

**Terminal 1 - Publisher (1 Hz):**
```bash
ros2 run qos_demo publisher
```

**Terminal 2 - Subscriber:**
```bash
ros2 run qos_demo subscriber
```

### Run with Custom Parameters

```bash
# High frequency publisher (50 Hz)
ros2 run qos_demo publisher --ros-args -p frequency:=50.0 -p queue_size:=20

# Slow subscriber (simulates processing delay)
ros2 run qos_demo subscriber --ros-args -p simulate_slow:=0.1
```

---

## 📚 Concepts Explained in Detail

---

### 1️⃣ Frequency (Publishing Rate)

#### What is Frequency?
**Frequency** is how often a publisher sends messages, measured in **Hertz (Hz)**.

```
Frequency = Messages per Second

1 Hz   = 1 message/second  (every 1000 ms)
10 Hz  = 10 messages/second (every 100 ms)
100 Hz = 100 messages/second (every 10 ms)
```

#### Visual Representation

```
Time ────────────────────────────────────────────────────────▶

1 Hz:    📩                    📩                    📩
         |←──── 1 second ────→|←──── 1 second ────→|

10 Hz:   📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩 📩
         |←────────── 1 second ──────────→|←────────── 1 second ──────────→|
```

#### How to Control in Code

```python
# Set timer period (inverse of frequency)
frequency = 10.0  # Hz
timer_period = 1.0 / frequency  # = 0.1 seconds

self.timer = self.create_timer(timer_period, self.callback)
```

#### How to Control at Runtime

```bash
# Start publisher at 50 Hz
ros2 run qos_demo publisher --ros-args -p frequency:=50.0

# Check actual frequency
ros2 topic hz /sensor_data
```

#### Typical Frequencies in Robotics

| Sensor/Data Type | Typical Frequency |
|------------------|-------------------|
| IMU (Accelerometer) | 100-1000 Hz |
| Lidar | 10-40 Hz |
| Camera | 15-60 Hz |
| GPS | 1-10 Hz |
| Temperature | 0.1-1 Hz |
| Motor commands | 50-500 Hz |

#### Trade-offs

| Higher Frequency | Lower Frequency |
|------------------|-----------------|
| ✅ More responsive | ✅ Less CPU usage |
| ✅ Finer control | ✅ Less bandwidth |
| ❌ More CPU usage | ❌ Slower response |
| ❌ More bandwidth | ❌ Less smooth control |

---

### 2️⃣ Latency (Message Delay)

#### What is Latency?
**Latency** is the time delay between when a message is **sent** and when it is **received**.

```
Publisher                                          Subscriber
    │                                                  │
    │  📩 Message Sent                                 │
    │  t = 0 ms                                        │
    │  ─────────────────────────────────────────────▶  │
    │                                                  │  📥 Message Received
    │                                                  │  t = 5 ms
    │                                                  │
    └──────────────── Latency = 5 ms ──────────────────┘
```

#### Components of Latency

```
Total Latency = Serialization + Network + Deserialization + Queue Wait

┌──────────┐   ┌─────────┐   ┌──────────┐   ┌──────────────┐   ┌──────────┐
│ Serialize│ → │ Network │ → │Deserialize│ → │ Queue/Buffer │ → │ Callback │
│  ~0.1ms  │   │ ~1-10ms │   │  ~0.1ms   │   │  0-???ms     │   │          │
└──────────┘   └─────────┘   └──────────┘   └──────────────┘   └──────────┘
```

#### Factors Affecting Latency

| Factor | Impact | Solution |
|--------|--------|----------|
| Network type | WiFi > Ethernet > Localhost | Use wired connections |
| Message size | Larger = slower | Compress or reduce data |
| Queue size | Large queues add delay | Use smaller queues |
| CPU load | Busy = slower | Optimize code |
| QoS Reliability | RELIABLE adds overhead | Use BEST_EFFORT if acceptable |

#### Measuring Latency

```bash
# Our subscriber reports latency automatically!
# Or use:
ros2 topic delay /sensor_data  # (requires timestamped messages)
```

#### Typical Latency Values

| Scenario | Expected Latency |
|----------|------------------|
| Same process | < 0.1 ms |
| Same machine | 0.1 - 1 ms |
| Wired LAN | 1 - 5 ms |
| WiFi | 5 - 50 ms |
| Internet | 50 - 200+ ms |

---

### 3️⃣ Queue Size / Buffer

#### What is Queue Size?
The **queue** (or buffer) stores messages when the subscriber can't process them fast enough.

```
                           Queue Size = 5
Publisher (Fast)          ┌─────────────────────┐          Subscriber (Slow)
    │                     │ [5][4][3][2][1]     │                │
    │  📩📩📩📩📩 ───────▶│ ←── Messages wait ──│──▶ 📥           │
    │  10 msg/sec         │     in queue        │    2 msg/sec   │
    │                     └─────────────────────┘                │
                                   ↓
                          If queue is full,
                          OLD messages are DROPPED!
```

#### Queue Behavior

```
Queue Size = 3, Publisher = 5 msg/s, Subscriber = 2 msg/s

Time 0s: Queue [ ][ ][ ]  (empty)
Time 1s: Queue [5][4][3]  (full!) → Messages 1,2 processed, 3,4,5 waiting
Time 2s: Queue [7][6][5]  (Messages 3,4 DROPPED! - couldn't fit)

With Queue Size = 10:
Time 0s: Queue [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
Time 1s: Queue [5][4][3][2][1][ ][ ][ ][ ][ ]  (no drops yet)
Time 2s: Queue [8][7][6][5][4][3][ ][ ][ ][ ]  (still OK)
```

#### How to Set Queue Size

```python
# In publisher
self.publisher = self.create_publisher(String, 'topic', 10)  # queue_size = 10

# In subscriber
self.subscription = self.create_subscription(String, 'topic', callback, 10)
```

#### Queue Size Guidelines

| Queue Size | When to Use |
|------------|-------------|
| **1** | Only latest data matters (real-time control) |
| **5-10** | General purpose (good balance) |
| **50-100** | Cannot lose any messages (logging, recording) |

#### Trade-offs

| Large Queue | Small Queue |
|-------------|-------------|
| ✅ Fewer dropped messages | ✅ Lower latency |
| ✅ Handles bursts | ✅ Always latest data |
| ❌ Higher latency | ❌ More dropped messages |
| ❌ More memory | ❌ Sensitive to processing speed |

---

### 4️⃣ Reliability

#### What is Reliability?
Reliability determines whether message delivery is **guaranteed** or **best-effort**.

#### Two Modes

```
┌─────────────────────────────────────────────────────────────────┐
│  RELIABLE (Like TCP)                                            │
│                                                                  │
│  Publisher ──📩──▶ Subscriber                                   │
│       │              │                                          │
│       │◀────ACK──────│  "Got it!"                               │
│       │              │                                          │
│  If no ACK, Publisher RESENDS the message                       │
│  ✅ Guaranteed delivery                                         │
│  ❌ Slower (waits for confirmation)                             │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  BEST_EFFORT (Like UDP)                                         │
│                                                                  │
│  Publisher ──📩──▶ Subscriber                                   │
│       │              │                                          │
│       │   (no ACK)   │  Fire and forget!                        │
│       │              │                                          │
│  If message lost, it's GONE                                     │
│  ✅ Faster (no waiting)                                         │
│  ❌ May lose messages                                           │
└─────────────────────────────────────────────────────────────────┘
```

#### How to Set Reliability

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable (guaranteed delivery)
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

# Best Effort (faster, may lose messages)
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)
```

#### Compatibility Rules

⚠️ **IMPORTANT:** Publisher and Subscriber QoS must be compatible!

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| RELIABLE | RELIABLE | ✅ Yes |
| RELIABLE | BEST_EFFORT | ✅ Yes |
| BEST_EFFORT | RELIABLE | ❌ **NO!** |
| BEST_EFFORT | BEST_EFFORT | ✅ Yes |

**Rule:** Subscriber cannot be MORE strict than Publisher.

#### When to Use Which

| Use RELIABLE for: | Use BEST_EFFORT for: |
|-------------------|----------------------|
| Configuration messages | Sensor data (cameras, lidar) |
| Commands (start/stop) | High-frequency data |
| Critical state updates | Data where latest matters most |
| Low-frequency data | Bandwidth-constrained systems |

---

### 5️⃣ QoS Profiles

#### Complete QoS Configuration

```python
from rclpy.qos import (
    QoSProfile, 
    ReliabilityPolicy, 
    HistoryPolicy, 
    DurabilityPolicy
)

qos_profile = QoSProfile(
    # RELIABILITY: Guarantee delivery?
    reliability=ReliabilityPolicy.RELIABLE,
    
    # HISTORY: Which messages to keep?
    history=HistoryPolicy.KEEP_LAST,  # or KEEP_ALL
    
    # DEPTH: Queue size
    depth=10,
    
    # DURABILITY: Keep messages for late subscribers?
    durability=DurabilityPolicy.VOLATILE  # or TRANSIENT_LOCAL
)
```

#### Built-in QoS Preset Profiles

```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# For sensor data (cameras, lidar, IMU)
# - Best effort, small queue
self.create_subscription(Image, 'camera', callback, qos_profile_sensor_data)

# For general purpose
# - Reliable, medium queue
self.create_publisher(String, 'status', qos_profile_system_default)
```

| Profile | Reliability | History | Depth | Durability |
|---------|-------------|---------|-------|------------|
| `sensor_data` | BEST_EFFORT | KEEP_LAST | 5 | VOLATILE |
| `system_default` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| `services_default` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| `parameters` | RELIABLE | KEEP_LAST | 1000 | VOLATILE |

---

### 6️⃣ Bandwidth

#### What is Bandwidth?
**Bandwidth** is the amount of data transferred per second, measured in **bytes/second** or **bits/second**.

```
Bandwidth = Message Size × Frequency

Example:
  Message Size = 1000 bytes
  Frequency = 30 Hz
  Bandwidth = 1000 × 30 = 30,000 bytes/second = 30 KB/s
```

#### Calculating Bandwidth

| Data Type | Typical Size | @ 30 Hz |
|-----------|--------------|---------|
| Float64 | 8 bytes | 240 B/s |
| String (100 char) | ~100 bytes | 3 KB/s |
| Image (640×480 RGB) | ~921 KB | 27 MB/s! |
| Point Cloud (10k points) | ~120 KB | 3.6 MB/s |

#### Monitoring Bandwidth

```bash
# Check bandwidth of a topic
ros2 topic bw /sensor_data

# Example output:
# Subscribed to [/sensor_data]
# 1.23 KB/s from 10 messages
```

#### Reducing Bandwidth

| Technique | Description |
|-----------|-------------|
| Lower frequency | Publish less often |
| Compress data | Use image compression, downsample |
| Send deltas | Only send changes, not full data |
| Use smaller types | Int16 instead of Int64, etc. |

---

## 🧪 Experiments to Try

### Experiment 1: Frequency Effects

```bash
# Terminal 1: Low frequency (1 Hz)
ros2 run qos_demo publisher --ros-args -p frequency:=1.0

# Terminal 2: Subscriber
ros2 run qos_demo subscriber

# Terminal 3: Check actual rate
ros2 topic hz /sensor_data
```

Then try 10 Hz, 50 Hz, 100 Hz!

### Experiment 2: Queue Overflow

```bash
# Fast publisher, slow subscriber with small queue
# Terminal 1:
ros2 run qos_demo publisher --ros-args -p frequency:=20.0

# Terminal 2: Slow processing, small queue
ros2 run qos_demo subscriber --ros-args -p simulate_slow:=0.1 -p queue_size:=5

# Watch for "DROPPED messages" warnings!
```

### Experiment 3: QoS Mismatch

```bash
# Terminal 1: Best effort publisher
ros2 run qos_demo publisher --ros-args -p reliable:=false

# Terminal 2: Reliable subscriber (THIS WON'T WORK!)
ros2 run qos_demo subscriber --ros-args -p reliable:=true

# Subscriber won't receive messages due to QoS incompatibility
```

### Experiment 4: Latency Measurement

```bash
# Terminal 1: High frequency publisher
ros2 run qos_demo publisher --ros-args -p frequency:=50.0

# Terminal 2: Watch latency stats
ros2 run qos_demo subscriber

# Check the latency reports every 5 seconds
```

---

## 🔍 Monitoring Commands

| Command | Description |
|---------|-------------|
| `ros2 topic hz /topic` | Measure actual publishing frequency |
| `ros2 topic bw /topic` | Measure bandwidth (bytes/second) |
| `ros2 topic delay /topic` | Measure latency (needs header) |
| `ros2 topic info /topic -v` | Detailed topic info with QoS |
| `ros2 topic echo /topic` | View messages in real-time |
| `rqt_graph` | Visual node/topic diagram |

### Check QoS Settings

```bash
ros2 topic info /sensor_data --verbose
```

Shows:
```
Topic: /sensor_data
Publisher count: 1
Subscription count: 1

Node name: configurable_publisher
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  History: KEEP_LAST (depth: 10)
```

---

## ❌ Common Issues & Solutions

### Issue 1: Subscriber Not Receiving Messages

**Symptom:** Subscriber running but no messages received.

**Causes & Solutions:**
| Cause | Solution |
|-------|----------|
| QoS mismatch | Make reliability settings compatible |
| Different topic names | Check topic name spelling |
| Different Domain ID | `export ROS_DOMAIN_ID=same_number` |
| Network issues | Check `ros2 topic list` from both machines |

### Issue 2: Dropping Messages

**Symptom:** Subscriber reports dropped messages.

**Solutions:**
- Increase queue size
- Reduce publisher frequency
- Speed up subscriber processing
- Use RELIABLE QoS if acceptable

### Issue 3: High Latency

**Symptom:** Large delay between publish and receive.

**Solutions:**
- Reduce queue size (for fresher data)
- Use BEST_EFFORT instead of RELIABLE
- Reduce message size
- Check network conditions

---

## 📁 Package Structure

```
qos_demo/
├── qos_demo/
│   ├── __init__.py
│   ├── publisher.py       # Configurable publisher node
│   └── subscriber.py      # Configurable subscriber node
├── resource/
│   └── qos_demo
├── package.xml
├── setup.cfg
├── setup.py
└── README.md              # This file
```

---

## 📊 Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS 2 QoS QUICK REFERENCE                   │
├──────────────┬──────────────────────────────────────────────────┤
│ FREQUENCY    │ Messages per second (Hz)                         │
│              │ Control: timer_period = 1/frequency              │
│              │ Measure: ros2 topic hz /topic                    │
├──────────────┼──────────────────────────────────────────────────┤
│ QUEUE SIZE   │ Buffer for messages (depth)                      │
│              │ Small (1-5): Real-time, latest data              │
│              │ Large (50+): Don't lose messages                 │
├──────────────┼──────────────────────────────────────────────────┤
│ RELIABILITY  │ RELIABLE: Guarantees delivery (slower)           │
│              │ BEST_EFFORT: Fire-and-forget (faster)            │
├──────────────┼──────────────────────────────────────────────────┤
│ LATENCY      │ Time from publish to receive                     │
│              │ Affected by: network, queue, CPU, message size   │
├──────────────┼──────────────────────────────────────────────────┤
│ BANDWIDTH    │ Data rate = size × frequency                     │
│              │ Measure: ros2 topic bw /topic                    │
├──────────────┼──────────────────────────────────────────────────┤
│ DOMAIN ID    │ Isolate ROS 2 networks (0-232)                   │
│              │ export ROS_DOMAIN_ID=42                          │
└──────────────┴──────────────────────────────────────────────────┘
```

---

## 📝 License

MIT License - Educational Use

---

*Created for learning ROS 2 Quality of Service concepts* 🤖

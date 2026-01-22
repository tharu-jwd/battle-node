# Battle-Node

A C++17 backend that ingests multiple noisy, asynchronous battlefield sensor streams (GPS, vision, RF, radar, lidar), fuses them into a single real-time estimated state per entity using a Kalman filter, and publishes the fused state via CLI and WebSocket server.

## Functionalities
- Real-time, multi-source ingestion with **asynchronous**, delayed, and missing updates.
- Multi-entity tracking with separate per-entity filter state.
- Sensor fusion using a high-performance Kalman filter with a constant-velocity motion model.
- Concurrency: dedicated ingestion, fusion, and output loops designed for low latency and high throughput.
- A synthetic sensor generator for reproducible demos without hardware.

## Architecture

### Data flow

```text
+----------------------------------------------------------------+
|                       Sensor Layer                             |
|  +----------+  +----------+  +----------+  +----------+         |
|  |   GPS    |  |  RADAR   |  |  VISION  |  |  LIDAR   |         |
|  +----+-----+  +----+-----+  +----+-----+  +----+-----+         |
+-------+-------------+-------------+-------------+---------------+
        |             |             |             |
        +-------------+-------------+-------------+
                      |
            +---------v---------+
            |   Fusion Engine   |
            | - Kalman Filter   |
            | - Entity Tracker  |
            | - Cleanup / TTL   |
            +---------+---------+
                      |
        +-------------+-------------+
        |                           |
  +-----v------+          +---------v--------+
  |    CLI     |          | WebSocket Server |
  | Visualizer |          |  (port 8080)     |
  +------------+          +------------------+
```

### Threading model (typical)

- **Sensor generator(s)**: 1 thread per synthetic sensor (GPS/Radar/Vision/etc.)
- **Fusion engine**:
  - **Fusion thread**: consumes measurements from a thread-safe queue and updates per-entity trackers
  - **Output thread**: periodically publishes fused states at a configurable rate
- **Main thread**: orchestration + signal handling (Ctrl+C)

### Fused state output

Each fused update includes:

- `entityId`
- `entityType`
- `position` (x, y, z)
- `velocity` (vx, vy, vz)
- `confidence` (0..1)
- `timestamp` + `lastUpdateTime`
- `measurementCount`
- `contributingSensors` (recent sensor types that updated the track)

The WebSocket server broadcasts JSON messages at ~10 Hz to all connected clients.

## Repository layout

```text
battle-node/
├── CMakeLists.txt
├── README.md
├── index.html (WebSocket test client)
├── include/
│   ├── common/
│   ├── sensors/
│   ├── fusion/
│   ├── output/
│   └── system/
├── src/
│   ├── common/
│   ├── sensors/
│   ├── fusion/
│   ├── output/
│   └── system/
└── demo/
    └── demo_script.cpp
```

## Prerequisites

- CMake 3.15+
- A C++17 compiler (GCC / Clang / MSVC)
- Eigen3 (for matrix math)
- Boost (for WebSocket server via Boost.Beast)

### Install Dependencies

**Eigen3**

**Ubuntu/Debian**

```bash
sudo apt-get update
sudo apt-get install -y libeigen3-dev
```

**macOS (Homebrew)**

```bash
brew install eigen
```

**Windows (vcpkg)**

```bash
vcpkg install eigen3
```

**Boost**

**Ubuntu/Debian**

```bash
sudo apt-get install -y libboost-all-dev
```

**macOS (Homebrew)**

```bash
brew install boost
```

**Windows (vcpkg)**

```bash
vcpkg install boost
```

## Build

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

Executables produced:

- `sensor_fusion` (full system run)
- `demo` (short portfolio demo)

## Run the portfolio demo

```bash
cd build
./demo
```

What you should see:

- A terminal dashboard updating live every ~0.5–1s (depending on configured output rate).
- Stable fused tracks even when individual sensors drop or jitter.

## Run the full system

```bash
cd build
./sensor_fusion
```

Default behavior (from [src/main.cpp](src/main.cpp)):

- Starts multiple synthetic sensors with different update rates/noise profiles.
- Tracks multiple entities simultaneously.
- Streams fused results to CLI and WebSocket server on port 8080.
- Runs until Ctrl+C.

To test the WebSocket connection, open `index.html` in your browser and click "Connect".

## Configuration points (quick edits)

You can adjust these in [src/main.cpp](src/main.cpp):

- **Fusion output rate:**
  ```cpp
  fusionEngine->setOutputRateHz(5.0);
  ```

- **Stale track eviction:**
  ```cpp
  fusionEngine->setStaleEntityTimeout(std::chrono::seconds(15));
  ```

- **Per-sensor behavior (noise/update/dropout/delay):**
  ```cpp
  SyntheticSensorGenerator(SensorType::GPS, 1.0, 5.0);
  setDropoutProbability(0.10);
  setDelayMs(10, 50);
  ```

- **Entity trajectories:**
  - Initial position + constant velocity for each entity.

## WebSocket Output (JSON)

The server broadcasts JSON updates at ~10 Hz on `ws://localhost:8080`.

Example JSON for a single entity update:

```json
{
  "entityId": 101,
  "type": "VEHICLE",
  "position": {"x": 150.23, "y": 80.45, "z": 0.00},
  "velocity": {"vx": 15.00, "vy": 10.00, "vz": 0.00},
  "confidence": 0.92,
  "measurements": 145
}
```

**Implementation:**

- Production-ready WebSocket server using Boost.Beast
- Async I/O with session management
- Thread-safe message queuing with backpressure handling
- Supports multiple concurrent clients
- Graceful connection/disconnection handling

**Testing:**

Open `index.html` in your browser or connect with any WebSocket client:

```javascript
const ws = new WebSocket('ws://localhost:8080');
ws.onmessage = (e) => console.log(JSON.parse(e.data));
```

## Extending the system

### Add a new sensor type

1. Add an enum value in [include/common/Types.h](include/common/Types.h).
2. Implement a new class that implements `SensorInterface` (or extend `SyntheticSensorGenerator`).
3. Define:
   - update rate
   - measurement covariance (noise model)
   - optional velocity availability
   - confidence behavior

### Add a new output sink

1. Implement `OutputInterface`.
2. Register it in `SensorFusionSystem` via `addOutputInterface(...)`.
3. Publish fused states in your preferred protocol:
   - file logger
   - UDP multicast
   - gRPC streaming
   - additional WebSocket endpoints

### Improve fusion realism (portfolio upgrades)

- Constant-turn / acceleration model (EKF/UKF).
- Outlier rejection (Mahalanobis gating).
- Per-sensor latency compensation + measurement-time ordering.
- Track management (init/confirm/delete logic).
- Metrics endpoint (rates, queue depths, per-entity update intervals).

## Known limitations

- Motion model is constant velocity (good baseline; easy to extend).
- Entity classification is simplified; entity type is currently defaulted in the tracker creation path.

## License

MIT License

# Battlefield Sensor Fusion Backend (C++)

A portfolio-ready C++17 backend that ingests multiple noisy, asynchronous battlefield sensor streams (GPS, vision, RF, radar, lidar), fuses them into a single real-time estimated state per entity using a Kalman filter, and publishes the fused state via CLI and a lightweight network output.

## What this project demonstrates
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
  |    CLI     |          |   Network Output |
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

The network output currently emits JSON messages (suitable for a thin dashboard).

## Repository layout
text
BattlefieldSensorFusion/
├── CMakeLists.txt
├── README.md
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

### Install Eigen3

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
- Streams fused results to CLI and to the network output on port 8080.
- Runs until Ctrl+C.

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

## Network output (JSON)

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

**Notes:**

- This backend keeps the output format straightforward to enable a thin visualization layer.
- A next step for a portfolio upgrade is to plug in a real WebSocket implementation (Boost.Beast or similar) or add gRPC streaming.

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
   - WebSocket
   - gRPC streaming

### Improve fusion realism (portfolio upgrades)

- Constant-turn / acceleration model (EKF/UKF).
- Outlier rejection (Mahalanobis gating).
- Per-sensor latency compensation + measurement-time ordering.
- Track management (init/confirm/delete logic).
- Metrics endpoint (rates, queue depths, per-entity update intervals).

## Known limitations (intentional for a self-contained demo)

- Network output is a lightweight stub designed for easy integration; it does not yet implement full WebSocket framing/handshakes.
- Motion model is constant velocity (good baseline; easy to extend).
- Entity classification is simplified; entity type is currently defaulted in the tracker creation path.

## License

MIT (recommended for portfolio projects). Add a LICENSE file if you plan to publish the repository publicly.
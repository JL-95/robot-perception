# Robot Perception

Camera and LiDAR frame processing for Caldova sidewalk delivery robots.

The library decodes the compact perception frames emitted by robot sensor pods,
normalizes camera pixels, and projects LiDAR points into the robot coordinate
space used by route planning.

## Features

- Decode camera frames from the Caldova edge transport envelope
- Convert range readings into occupancy grid coordinates
- Fuse camera and LiDAR timing into a single perception packet
- C API for runtime embedding
- Python ctypes binding for operations tooling

## Layout

- `include\caldova\perception\` public headers
- `src\` C++ implementation and C ABI wrapper
- `python\` Python binding

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
```

## Python usage

```python
from caldova_perception import PerceptionLibrary

lib = PerceptionLibrary(r".\build\Release\caldova_perception.dll")
result = lib.decode_frame(packet_bytes)
print(result.width, result.height, result.channels)
```

## Frame envelope

Frames are little-endian and start with a fixed 32-byte header:

| Field | Size |
| --- | ---: |
| Magic `RBPF` | 4 |
| Version | 2 |
| Header bytes | 2 |
| Width | 4 |
| Height | 4 |
| Timestamp milliseconds | 8 |
| Camera payload bytes | 4 |
| LiDAR point count | 4 |

The camera payload begins immediately after the header. LiDAR points follow as
packed triples of 32-bit floats: `x`, `y`, and `intensity`.

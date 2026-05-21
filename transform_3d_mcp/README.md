# 3D Transform MCP Server

An MCP (Model Context Protocol) server for converting between all common 3D pose representations: rotation matrix, quaternion, axis-angle, axis-magnitude with angle, Euler angles, and 4x4 homogeneous transformation matrices.

Based on the algorithms and conventions from [3D Transform Viewer](https://dugas.ch/transform_viewer/index.html) by Daniel Dugas.

## Source & Inspiration

This project is directly inspired by the excellent [3D Transform Viewer](https://dugas.ch/transform_viewer/index.html) — an interactive WebGL-based tool built with Three.js that visualizes 3D transformations in real-time. The viewer lets you adjust any 3D pose representation and see the corresponding 3D axes update live, making it an invaluable resource for understanding spatial relationships between coordinate frames.

This MCP server reimplements all the mathematical transforms from that viewer as a programmatic API, enabling LLMs to perform precise 3D pose conversions in robotics, computer graphics, and computer vision workflows.

## How It Works

3D rotations can be expressed in multiple mathematically equivalent forms, each with different use cases:

| Representation | Description | Typical Use Case |
|---|---|---|
| **Rotation Matrix** (3×3) | Orthogonal matrix with determinant +1 | Linear algebra operations, rendering pipelines |
| **Quaternion** (x, y, z, w) | Compact 4-parameter representation | Smooth interpolation (SLERP), avoiding gimbal lock |
| **Axis-Angle** | Rotation axis + angle in radians | Intuitive description of rotation, robot kinematics |
| **Axis-Magnitude** | axis × angle as a single 3D vector | Lie algebra (so(3)), exponential map |
| **Euler Angles** | Three sequential rotations (6 intrinsic orders) | Human-readable angles, Three.js convention |

The server converts any input representation to all others simultaneously, using the same conventions as the Transform Viewer:

- **Quaternions** use the Hamilton convention (matching Three.js)
- **Euler angles** support all 6 intrinsic rotation orders (`XYZ`, `XZY`, `YXZ`, `YZX`, `ZXY`, `ZYX`) using the Three.js convention (`xy'z''` notation)
- **Axis-angle** uses Rodrigues' rotation formula
- **4×4 matrices** use homogeneous coordinates for combined rotation + translation

All conversions round-trip through the rotation matrix as the canonical intermediate form.

## Tools

The server exposes 7 MCP tools:

| Tool | Description |
|---|---|
| `transform_from_quaternion` | Quaternion → all other representations |
| `transform_from_matrix` | Rotation matrix → all other representations |
| `transform_from_axis_angle` | Axis-angle → all other representations |
| `transform_from_axis_magnitude` | Axis-magnitude vector → all other representations |
| `transform_from_euler` | Euler angles → all other representations |
| `transform_from_any` | Unified: input any one representation → all others |
| `transform_compose` | Compose a 4×4 transformation matrix from rotation + translation |
| `transform_apply_point` | Apply a 4×4 matrix to a 3D point (frame transformation) |

## Usage

### Installation

```bash
# Install dependencies
pip install mcp pydantic
```

### Running the server

```bash
python server.py
```

### MCP Client Configuration

Add to your `crush.json` or MCP client config:

```json
{
  "mcpServers": {
    "transform_3d_mcp": {
      "command": "python",
      "args": ["/path/to/transform_3d_mcp/server.py"]
    }
  }
}
```

### Example Queries

Once connected, you can ask:

- "Convert quaternion (0, 0, 0, 1) to rotation matrix and Euler angles"
- "What's the axis-angle for this rotation matrix: [[0,-1,0],[1,0,0],[0,0,1]]?"
- "Rotate point (1, 2, 3) by 90 degrees around Z axis then translate by (0, 0, 5)"
- "What are all Euler angle representations of rotation around X by 30 degrees?"

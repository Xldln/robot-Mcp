# Robot-MCP

A collection of MCP (Model Context Protocol) servers and AI agent skills for robotics applications 

## Components

### `transform_3d_mcp/` — 3D Transform MCP Server

An MCP server for converting between all common 3D pose representations:

- Rotation matrix (3×3)
- Quaternion (x, y, z, w)
- Axis-angle
- Axis-magnitude vector
- Euler angles (all 6 intrinsic orders)
- 4×4 homogeneous transformation matrix (compose + apply)

Based on the algorithms from [3D Transform Viewer](https://dugas.ch/transform_viewer/index.html) by Daniel Dugas.

**Tools:** `transform_from_quaternion`, `transform_from_matrix`, `transform_from_axis_angle`, `transform_from_axis_magnitude`, `transform_from_euler`, `transform_from_any`, `transform_compose`, `transform_apply_point`

→ See [transform_3d_mcp/README.md](transform_3d_mcp/README.md) for details.

### `skills/transform_3d/` — AI Agent Skill for 3D Orientation

A Crush skill that enables AI agents to:

- Parse natural-language descriptions of robot gripper/end-effector orientation
- Convert them to quaternion (xyzw) values using the `transform_3d_mcp` server
- Write the results into structured YAML config files at specified key paths

Handles multiple input patterns: axis-angle, Euler angles, axis-relative angle descriptions ("x vector + z vector = degree 60"), and approach-direction descriptions.

→ See [skills/transform_3d/SKILL.md](skills/transform_3d/SKILL.md) for details.


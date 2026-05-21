---
name: transform_3d
description: Guide AI agents to convert natural-language robot gripper orientation descriptions into quaternion (xyzw) values using the transform_3d_mcp server, and inject them into YAML config files at specified keys.
user-invocable: true
license: yy
---

# transform_3d Skill

Makes AI agents capable of parsing spatial orientation descriptions (typically for robot gripper/end-effector poses), converting them to quaternion values via the `transform_3d_mcp` server, and writing them into structured YAML configuration files.

## Prerequisites

The `transform_3d_mcp` MCP server must be configured and running. See its README for setup.

## Workflow

### Step 1 — Understand the orientation description

Users describe gripper orientation in natural language. Common patterns:

| Pattern | Example | Meaning |
|---|---|---|
| **Axis-angle** | "rotate 90° around Z axis" | `axis=(0,0,1), angle=π/2` |
| **Euler angles** | "roll 30°, pitch 0°, yaw 45° in XYZ order" | `e0=π/6, e1=0, e2=π/4, order=XYZ` |
| **Axis relative angles** | "x vector + z vector = 60°, x vector + y vector = 0°" | Gripper x-axis forms 60° with world Z, 0° with world Y |
| **Approach direction** | "gripper points downward, rotated 45° around approach axis" | Euler or axis-angle derived from direction vector |
| **Quaternion directly** | "quaternion (0, 0, 0.707, 0.707)" | Pass through directly |

#### Interpreting "axis relative angle" descriptions

When user says something like _"x vector + z vector = degree 60 and x vector + y vector = degree 0"_:

This describes the **direction of the gripper's x-axis vector** in the world frame, expressed as angles to two world axes:

- `x vector + z vector = 60°` → The gripper's x-axis is 60° away from the world z-axis
- `x vector + y vector = 0°` → The gripper's x-axis is 0° away from the world y-axis (i.e., aligned with it)

**Derivation logic:**

1. From `x vector + y vector = 0°`, the x-axis direction is aligned with the y-axis → unit direction vector `v = (0, 1, 0)` in ideal case.
2. From `x vector + z vector = 60°`, the x-axis makes a 60° angle with z-axis.
3. To find a full rotation matrix, also need the gripper's z-axis (approach direction) or y-axis orientation.
4. If only x-axis is constrained, assume a standard orientation (e.g., z-axis is world -z for downward approach) and compute the rotation that aligns the gripper frame with these constraints.

**General procedure for axis-relative descriptions:**

1. Extract each stated axis-constraint pair: `{gripper_axis} + {world_axis} = {degrees}`
2. Convert degrees to radians: `angle_rad = degrees × π / 180`
3. Build a direction vector for the gripper axis from the constraints
4. Use the cross product to complete the orthonormal basis (gripper frame)
5. Assemble the 3×3 rotation matrix from the three basis vectors
6. Feed the matrix into `transform_from_matrix` to get the quaternion

### Step 2 — Convert to quaternion via MCP

Call the appropriate `transform_3d_mcp` tool based on the parsed representation:

**If you have Euler angles:**
```
transform_from_euler with:
  e0: <first angle in radians>
  e1: <second angle in radians>
  e2: <third angle in radians>
  order: "<XYZ|XZY|YXZ|YZX|ZXY|ZYX>"
```
→ Extract `quaternion.x`, `quaternion.y`, `quaternion.z`, `quaternion.w` from the response.

**If you have an axis-angle:**
```
transform_from_axis_angle with:
  ux: <axis x>
  uy: <axis y>
  uz: <axis z>
  angle_rad: <angle in radians>
```
→ Extract the quaternion from the response.

**If you have a rotation matrix:**
```
transform_from_matrix with:
  m00: <...>  m01: <...>  m02: <...>
  m10: <...>  m11: <...>  m12: <...>
  m20: <...>  m21: <...>  m22: <...>
```
→ Extract the quaternion from the response.

**If you have an axis-magnitude vector:**
```
transform_from_axis_magnitude with:
  ux: <axis × angle in radians>
  uy: <axis × angle in radians>
  uz: <axis × angle in radians>
```

**Unified entry (any format):**
```
transform_from_any with:
  use_<format>: true
  <corresponding fields>: <values>
```

### Step 3 — Write quaternion values into the YAML file

Parse the JSON response from the tool, extract `quaternion.x/y/z/w`, and write them into the target YAML at the specified key path.

**For the example** (writing to `pencil.pick.action` quaternion fields):

```yaml
pencil:
  pick:
    action:
      x: <quaternion.x>, y: <quaternion.y>,z: <quaternion.z>,w: <quaternion.w>
      
```

Use the `edit` or `write` tools to update the YAML file.

## Complete Example

**User request:**

> 帮我转换为一个四元值使用 transformer_3d_mcp, 机器人 robot 夹抓 x vector + z vector = degree 60 and x vector + y vector = degree 0, 生成四元值并填入 /path/to/test.yaml 的 pencil : pick : action 键的所有 xyzw 的位置

**Agent procedure:**

1. **Parse**: Interpret the orientation description.
   - `x vector + y vector = 0°` → x-axis is parallel to world Y → x-axis direction is along `(0, 1, 0)` or `(0, -1, 0)`
   - `x vector + z vector = 60°` → x-axis makes 60° angle with world Z
   - If the two constraints are geometrically contradictory (e.g., x parallel to Y means x·z = 0, cannot also have x·z = 0.5), treat the description as a **compound rotation**: the gripper starts in a default orientation, then is rotated 60° around an axis derived from the two constraints.
   - **Practical fallback**: Convert the description to axis-angle: the axis is the cross product of the two reference axes, the angle is the stated value. Ask user for clarification if the constraints are clearly contradictory.

2. **Convert**: Call `transform_from_axis_angle` (or `transform_from_euler`) depending on interpretation.
3. **Write**: Edit the YAML file with the four quaternion values at the target key path.

## YAML Writing Convention

When updating a YAML file, use the `edit` tool with exact indentation matching. The quaternion keys are typically `x`, `y`, `z`, `w` at the target path:

```
edit file_path: <path>
old_string: "<existing YAML context>"
new_string: "<updated YAML context>"
```

Always preserve adjacent keys and indentation — only change the quaternion values.

## Common Robot Gripper Conventions

| Convention | Approach axis | Open/close axis | Notes |
|---|---|---|---|
| **ROS / MoveIt** | Z | X | Standard REP-103 |
| **Universal Robots** | Z | Y | UR script convention |
| **ABB** | Z | X | RAPID convention |
| **Fanuc** | Z | Y | TP/Karel convention |
| **Three.js / graphics** | Z (local -Z often forward) | X | Used in transform viewer |

When interpreting orientation descriptions, prefer the ROS convention (approach = +Z) unless the user specifies otherwise.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "Matrix is not orthogonal" | Invalid rotation matrix from ambiguous constraints | Use axis-angle or Euler instead |
| Quaternion has unexpected signs | Different handedness convention | Negate all components (q → -q represents same rotation) |
| Angles in degrees vs radians | User provided degrees but tool expects radians | Convert: rad = deg × π / 180 |
| Gimbal lock in Euler extraction | Middle angle close to ±90° | Use axis-angle representation instead |

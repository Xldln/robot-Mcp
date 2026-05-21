#!/usr/bin/env python3
'''
MCP Server for 3D Pose Transformation Conversions.

Provides tools to convert between all common 3D pose representations:
rotation matrix, quaternion, axis-angle, Euler angles, and 4x4
transformation matrices. Based on the transforms from
https://dugas.ch/transform_viewer/index.html
'''

import json
import math
from typing import Optional, List
from pydantic import BaseModel, Field, field_validator, ConfigDict
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("transform_3d_mcp")

# ─── Constants ───────────────────────────────────────────────────────────────

_EPS = 1e-10

_EULER_ORDERS = {
    "xyz": 0, "yzx": 1, "zxy": 2, "xzy": 3, "yxz": 4, "zyx": 5,
    "XYZ": 0, "YZX": 1, "ZXY": 2, "XZY": 3, "YXZ": 4, "ZYX": 5,
}

# ─── Shared helpers ──────────────────────────────────────────────────────────

def _mat3_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    '''Multiply two 3x3 matrices.'''
    return [
        [
            A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def _mat3_vector_mul(M: List[List[float]], v: List[float]) -> List[float]:
    '''Multiply 3x3 matrix by 3D vector.'''
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    ]


def _mat3_transpose(M: List[List[float]]) -> List[List[float]]:
    '''Transpose a 3x3 matrix.'''
    return [[M[j][i] for j in range(3)] for i in range(3)]


def _mat3_identity() -> List[List[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _rot_x(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_z(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _format_matrix(M: List[List[float]], decimals: int = 6) -> str:
    '''Format a matrix as a readable string.'''
    return "\n".join(
        "[" + ", ".join(f"{v:.{decimals}f}" for v in row) + "]"
        for row in M
    )


def _format_vec(v: List[float], decimals: int = 6) -> str:
    return f"[{', '.join(f'{x:.{decimals}f}' for x in v)}]"


def _check_rotation_matrix(M: List[List[float]]) -> Optional[str]:
    '''Validate a 3x3 rotation matrix, return error message or None.'''
    if len(M) != 3 or any(len(row) != 3 for row in M):
        return "Matrix must be 3x3"
    # Check orthogonality: R^T @ R ≈ I
    for i in range(3):
        for j in range(3):
            dot = sum(M[k][i] * M[k][j] for k in range(3))
            expected = 1.0 if i == j else 0.0
            if abs(dot - expected) > 1e-4:
                return f"Matrix is not orthogonal (column {i} · column {j} = {dot:.4f}, expected {expected})"
    # Check determinant ≈ 1
    det = (
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
        - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
        + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
    )
    if abs(det - 1.0) > 1e-4:
        return f"Matrix determinant is {det:.4f}, expected 1.0"
    return None


# ─── Core conversion functions ───────────────────────────────────────────────

def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    '''
    Convert quaternion (xyzw) to 3x3 rotation matrix.

    Formula from the transform viewer (standard Hamilton convention).
    '''
    xx = 1 - 2 * qy * qy - 2 * qz * qz
    xy = 2 * qx * qy + 2 * qz * qw
    xz = 2 * qx * qz - 2 * qy * qw
    yx = 2 * qx * qy - 2 * qz * qw
    yy = 1 - 2 * qx * qx - 2 * qz * qz
    yz = 2 * qy * qz + 2 * qx * qw
    zx = 2 * qx * qz + 2 * qy * qw
    zy = 2 * qy * qz - 2 * qx * qw
    zz = 1 - 2 * qx * qx - 2 * qy * qy
    return [[xx, yx, zx], [xy, yy, zy], [xz, yz, zz]]


def matrix_to_quaternion(M: List[List[float]]) -> tuple:
    '''
    Convert 3x3 rotation matrix to quaternion (x, y, z, w).

    Uses the stable algorithm from the transform viewer.
    '''
    trace = M[0][0] + M[1][1] + M[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return (
            (M[2][1] - M[1][2]) * s,
            (M[0][2] - M[2][0]) * s,
            (M[1][0] - M[0][1]) * s,
            0.25 / s,
        )
    if M[0][0] > M[1][1] and M[0][0] > M[2][2]:
        s = 2.0 * math.sqrt(1.0 + M[0][0] - M[1][1] - M[2][2])
        return (
            0.25 * s,
            (M[0][1] + M[1][0]) / s,
            (M[0][2] + M[2][0]) / s,
            (M[2][1] - M[1][2]) / s,
        )
    if M[1][1] > M[2][2]:
        s = 2.0 * math.sqrt(1.0 + M[1][1] - M[0][0] - M[2][2])
        return (
            (M[0][1] + M[1][0]) / s,
            0.25 * s,
            (M[1][2] + M[2][1]) / s,
            (M[0][2] - M[2][0]) / s,
        )
    s = 2.0 * math.sqrt(1.0 + M[2][2] - M[0][0] - M[1][1])
    return (
        (M[0][2] + M[2][0]) / s,
        (M[1][2] + M[2][1]) / s,
        0.25 * s,
        (M[1][0] - M[0][1]) / s,
    )


def axis_angle_to_matrix(ux: float, uy: float, uz: float, angle: float) -> List[List[float]]:
    '''
    Convert axis-angle to 3x3 rotation matrix (Rodrigues' formula).

    The axis (ux, uy, uz) is normalized automatically.
    angle is in radians.
    '''
    norm = math.sqrt(ux * ux + uy * uy + uz * uz)
    if norm < _EPS:
        return _mat3_identity()
    ux, uy, uz = ux / norm, uy / norm, uz / norm
    c, s = math.cos(angle), math.sin(angle)
    return [
        [ux * ux * (1 - c) + c, ux * uy * (1 - c) - uz * s, ux * uz * (1 - c) + uy * s],
        [uy * ux * (1 - c) + uz * s, uy * uy * (1 - c) + c, uy * uz * (1 - c) - ux * s],
        [uz * ux * (1 - c) - uy * s, uz * uy * (1 - c) + ux * s, uz * uz * (1 - c) + c],
    ]


def matrix_to_axis_angle(M: List[List[float]]) -> tuple:
    '''
    Convert 3x3 rotation matrix to axis-angle (ux, uy, uz, angle_rad).

    Returns (ux, uy, uz, angle) where angle is in radians.
    '''
    angle = math.acos(max(-1.0, min(1.0, (M[0][0] + M[1][1] + M[2][2] - 1) / 2)))
    if angle < _EPS:
        return (1.0, 0.0, 0.0, 0.0)
    denom = math.sqrt(
        (M[2][1] - M[1][2]) ** 2
        + (M[0][2] - M[2][0]) ** 2
        + (M[1][0] - M[0][1]) ** 2
    )
    if denom < _EPS:
        return (1.0, 0.0, 0.0, 0.0)
    ux = (M[2][1] - M[1][2]) / denom
    uy = (M[0][2] - M[2][0]) / denom
    uz = (M[1][0] - M[0][1]) / denom
    return (ux, uy, uz, angle)


def axis_magnitude_to_matrix(ux: float, uy: float, uz: float) -> List[List[float]]:
    '''Convert axis-with-angle-magnitude vector to 3x3 rotation matrix.'''
    angle = math.sqrt(ux * ux + uy * uy + uz * uz)
    if angle < _EPS:
        return _mat3_identity()
    return axis_angle_to_matrix(ux / angle, uy / angle, uz / angle, angle)


def matrix_to_axis_magnitude(M: List[List[float]]) -> tuple:
    '''Convert 3x3 rotation matrix to axis-with-angle-magnitude vector.'''
    ux, uy, uz, angle = matrix_to_axis_angle(M)
    return (ux * angle, uy * angle, uz * angle)


def euler_to_matrix(e0: float, e1: float, e2: float, order: str) -> List[List[float]]:
    '''
    Convert Euler angles to 3x3 rotation matrix.

    The `order` uses the intrinsic (local axis) convention matching THREE.js:
      'XYZ' = rotate X, then Y', then Z''  (extrinsic ZYX)
      'ZYX' = rotate Z, then Y', then X''  (extrinsic XYZ)
    Full mapping of intrinsic orders:
      xy'z'' → 'XYZ'  (extrinsic ZYX)
      xz'y'' → 'XZY'  (extrinsic YZX)
      yx'z'' → 'YXZ'  (extrinsic ZXY)
      yz'x'' → 'YZX'  (extrinsic XZY)
      zx'y'' → 'ZXY'  (extrinsic YXZ)
      zy'x'' → 'ZYX'  (extrinsic XYZ)
    '''
    order_key = order.lower()
    if order_key not in ('xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx'):
        raise ValueError(f"Invalid Euler order '{order}'. Use one of: XYZ, XZY, YXZ, YZX, ZXY, ZYX")

    # For intrinsic rotations: R = R_first(e0) @ R_second(e1) @ R_third(e2)
    rot_map = {'x': _rot_x, 'y': _rot_y, 'z': _rot_z}
    r0 = rot_map[order[0].lower()](e0)
    r1 = rot_map[order[1].lower()](e1)
    r2 = rot_map[order[2].lower()](e2)
    return _mat3_mul(r0, _mat3_mul(r1, r2))


def _euler_from_matrix_single_order(M: List[List[float]], order: str) -> tuple:
    '''
    Extract Euler angles from rotation matrix for a specific order.

    Returns (e0, e1, e2) in radians.
    Handles gimbal lock cases.
    '''
    # For intrinsic rotation order 'XYZ': R = Rx(e0) @ Ry(e1) @ Rz(e2)
    # Elements of R:
    # R[0][2] = cos(e1)*sin(e2)   -> e2
    # R[1][2] = cos(e1)*cos(e2)   -> e2 = atan2(R[0][2], R[1][2])
    # R[0][0] = cos(e1)*cos(e2)   -> no, let me derive properly

    # For intrinsic 'XYZ': R = Rx(α) @ Ry(β) @ Rz(γ)
    # Rx = [[1,0,0],[0,cα,-sα],[0,sα,cα]]
    # Ry = [[cβ,0,sβ],[0,1,0],[-sβ,0,cβ]]
    # Rz = [[cγ,-sγ,0],[sγ,cγ,0],[0,0,1]]
    #
    # R = Rx @ Ry @ Rz
    #
    # R[0][:] = [cβ*cγ, -cβ*sγ, sβ]
    # R[1][:] = [cα*sγ + sα*sβ*cγ, cα*cγ - sα*sβ*sγ, -sα*cβ]
    # R[2][:] = [sα*sγ - cα*sβ*cγ, sα*cγ + cα*sβ*sγ, cα*cβ]
    #
    # So: e1(β) = asin(R[0][2])
    #     e2(γ) = atan2(-R[0][1], R[0][0])
    #     e0(α) = atan2(-R[1][2], R[2][2])

    # Let me use a general approach. For any order, I'll compute using
    # the known patterns of each element.

    # I'll implement a lookup-based approach using the sy (sin of middle angle) pattern.

    # There are 12 possible Euler angle conventions, but we only need the 6 intrinsic ones.
    # Following THREE.js approach for each order:

    m = M  # shorthand

    # For intrinsic rotations: The rotation matrix is built as R = R_a @ R_b @ R_c
    # where a, b, c are the first, second, third rotation axes.

    # Using the approach from: https://www.geometrictools.com/Documentation/EulerAngles.pdf

    order_lower = order.lower()
    # Map axis to index: 0=x, 1=y, 2=z
    axis_idx = {'x': 0, 'y': 1, 'z': 2}
    i = axis_idx[order_lower[0]]
    j = axis_idx[order_lower[1]]
    k = axis_idx[order_lower[2]]

    # Check if the rotation is "proper" (i != k) - for intrinsic Euler, the first and last
    # axes may be the same in some conventions... Actually in Euler angles, i != j, j != k, but i can equal k.
    # Wait, for proper Euler angles the first and last axis are the same (e.g., 'ZXZ').
    # But for the 6 orders we handle (Tait-Bryan / Cardan), all three axes are different.
    # So i, j, k are all different.

    # Map each axis to a sine/cosine position in the matrix
    # The middle rotation (j) determines if there's gimbal lock

    # I'll use the following approach:
    # For each order, the matrix entries are at known positions.
    # Rather than a fully general solution, I'll implement each order explicitly.

    def _extract_xyz() -> tuple:
        # R = Rx(e0) @ Ry(e1) @ Rz(e2)
        # R[0][2] = sβ,  R[0][1]/R[0][0] for γ, R[1][2]/R[2][2] for α
        sy = m[0][2]
        if abs(sy) > 0.99999:  # gimbal lock
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(-m[1][0], m[1][1])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(-m[1][2], m[2][2])
            e2 = math.atan2(-m[0][1], m[0][0])
        return (e0, e1, e2)

    def _extract_xzy() -> tuple:
        # R = Rx(e0) @ Rz(e1) @ Ry(e2)
        # R[0][1] = -sβ  (wait, this needs proper derivation)
        # Let me derive: Rx @ Rz @ Ry
        # Rx = [[1,0,0],[0,cα,-sα],[0,sα,cα]]
        # Rz = [[cβ,-sβ,0],[sβ,cβ,0],[0,0,1]]
        # Ry = [[cγ,0,sγ],[0,1,0],[-sγ,0,cγ]]
        # R = Rx @ (Rz @ Ry)
        # Rz @ Ry = [[cβ*cγ, -sβ, cβ*sγ], [sβ*cγ, cβ, sβ*sγ], [-sγ, 0, cγ]]
        # R = [[cβ*cγ, -sβ, cβ*sγ], [cα*sβ*cγ+sα*sγ, cα*cβ, cα*sβ*sγ-sα*cγ], [sα*sβ*cγ-cα*sγ, sα*cβ, sα*sβ*sγ+cα*cγ]]
        # R[0][1] = -sβ
        sy = -m[0][1]
        if abs(sy) > 0.99999:
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(-m[0][2], m[2][1])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(m[1][1], m[2][1])
            e2 = math.atan2(m[0][2], m[0][0])
        return (e0, e1, e2)

    def _extract_yxz() -> tuple:
        # R = Ry(e0) @ Rx(e1) @ Rz(e2)
        # Ry = [[cα,0,sα],[0,1,0],[-sα,0,cα]]
        # Rx = [[1,0,0],[0,cβ,-sβ],[0,sβ,cβ]]
        # Rz = [[cγ,-sγ,0],[sγ,cγ,0],[0,0,1]]
        # R = Ry @ (Rx @ Rz)
        # Rx @ Rz = [[cγ, -sγ, 0], [cβ*sγ, cβ*cγ, -sβ], [sβ*sγ, sβ*cγ, cβ]]
        # R = [[cα*cγ+sα*sβ*sγ, -cα*sγ+sα*sβ*cγ, sα*cβ], [cβ*sγ, cβ*cγ, -sβ], [-sα*cγ+cα*sβ*sγ, sα*sγ+cα*sβ*cγ, cα*cβ]]
        # R[1][2] = -sβ
        sy = -m[1][2]
        if abs(sy) > 0.99999:
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(m[0][1], m[1][1])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(m[0][2], m[2][2])
            e2 = math.atan2(m[1][0], m[1][1])
        return (e0, e1, e2)

    def _extract_yzx() -> tuple:
        # R = Ry(e0) @ Rz(e1) @ Rx(e2)
        # Ry = [[cα,0,sα],[0,1,0],[-sα,0,cα]]
        # Rz = [[cβ,-sβ,0],[sβ,cβ,0],[0,0,1]]
        # Rx = [[1,0,0],[0,cγ,-sγ],[0,sγ,cγ]]
        # R = Ry @ (Rz @ Rx)
        # Rz@Rx = [[cβ, -sβ*cγ, sβ*sγ], [sβ, cβ*cγ, -cβ*sγ], [0, sγ, cγ]]
        # R = [[cα*cβ, -cα*sβ*cγ+sα*sγ, cα*sβ*sγ+sα*cγ], [sβ, cβ*cγ, -cβ*sγ], [-sα*cβ, sα*sβ*cγ+cα*sγ, -sα*sβ*sγ+cα*cγ]]
        # R[1][0] = sβ
        sy = m[1][0]
        if abs(sy) > 0.99999:
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(-m[2][1], m[1][1])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(-m[2][0], m[0][0])
            e2 = math.atan2(-m[1][2], m[1][1])
        return (e0, e1, e2)

    def _extract_zxy() -> tuple:
        # R = Rz(e0) @ Rx(e1) @ Ry(e2)
        # Rz = [[cα,-sα,0],[sα,cα,0],[0,0,1]]
        # Rx = [[1,0,0],[0,cβ,-sβ],[0,sβ,cβ]]
        # Ry = [[cγ,0,sγ],[0,1,0],[-sγ,0,cγ]]
        # R = Rz @ (Rx @ Ry)
        # Rx@Ry = [[cγ, 0, sγ], [sβ*sγ, cβ, -sβ*cγ], [-cβ*sγ, sβ, cβ*cγ]]
        # R = [[cα*cγ-sα*sβ*sγ, -sα*cβ, cα*sγ+sα*sβ*cγ], [sα*cγ+cα*sβ*sγ, cα*cβ, sα*sγ-cα*sβ*cγ], [-cβ*sγ, sβ, cβ*cγ]]
        # R[2][1] = sβ
        sy = m[2][1]
        if abs(sy) > 0.99999:
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(m[0][2], m[0][0])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(-m[0][1], m[1][1])
            e2 = math.atan2(-m[2][0], m[2][2])
        return (e0, e1, e2)

    def _extract_zyx() -> tuple:
        # R = Rz(e0) @ Ry(e1) @ Rx(e2)
        # Rz = [[cα,-sα,0],[sα,cα,0],[0,0,1]]
        # Ry = [[cβ,0,sβ],[0,1,0],[-sβ,0,cβ]]
        # Rx = [[1,0,0],[0,cγ,-sγ],[0,sγ,cγ]]
        # R = Rz @ (Ry @ Rx)
        # Ry@Rx = [[cβ, sβ*sγ, sβ*cγ], [0, cγ, -sγ], [-sβ, cβ*sγ, cβ*cγ]]
        # R = [[cα*cβ, cα*sβ*sγ-sα*cγ, cα*sβ*cγ+sα*sγ], [sα*cβ, sα*sβ*sγ+cα*cγ, sα*sβ*cγ-cα*sγ], [-sβ, cβ*sγ, cβ*cγ]]
        # R[2][0] = -sβ
        sy = -m[2][0]
        if abs(sy) > 0.99999:
            e1 = math.copysign(math.pi / 2, sy)
            e0 = 0.0
            e2 = math.atan2(-m[2][1], m[1][1])
        else:
            e1 = math.asin(max(-1.0, min(1.0, sy)))
            e0 = math.atan2(m[1][0], m[0][0])
            e2 = math.atan2(m[2][1], m[2][2])
        return (e0, e1, e2)

    extractors = {
        'xyz': _extract_xyz,
        'xzy': _extract_xzy,
        'yxz': _extract_yxz,
        'yzx': _extract_yzx,
        'zxy': _extract_zxy,
        'zyx': _extract_zyx,
    }
    return extractors[order_lower]()


# ─── Pydantic Input Models ───────────────────────────────────────────────────

class QuaternionInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    qx: float = Field(..., description="Quaternion x component")
    qy: float = Field(..., description="Quaternion y component")
    qz: float = Field(..., description="Quaternion z component")
    qw: float = Field(..., description="Quaternion w (scalar) component")


class RotationMatrixInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    m00: float = Field(default=1.0, description="Element [0][0] of 3x3 rotation matrix")
    m01: float = Field(default=0.0, description="Element [0][1] of 3x3 rotation matrix")
    m02: float = Field(default=0.0, description="Element [0][2] of 3x3 rotation matrix")
    m10: float = Field(default=0.0, description="Element [1][0] of 3x3 rotation matrix")
    m11: float = Field(default=1.0, description="Element [1][1] of 3x3 rotation matrix")
    m12: float = Field(default=0.0, description="Element [1][2] of 3x3 rotation matrix")
    m20: float = Field(default=0.0, description="Element [2][0] of 3x3 rotation matrix")
    m21: float = Field(default=0.0, description="Element [2][1] of 3x3 rotation matrix")
    m22: float = Field(default=1.0, description="Element [2][2] of 3x3 rotation matrix")

    def to_matrix(self) -> List[List[float]]:
        return [
            [self.m00, self.m01, self.m02],
            [self.m10, self.m11, self.m12],
            [self.m20, self.m21, self.m22],
        ]

    @field_validator("m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22")
    @classmethod
    def check_finite(cls, v: float) -> float:
        if not math.isfinite(v):
            raise ValueError(f"Matrix element must be a finite number, got {v}")
        return v


class AxisAngleInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    ux: float = Field(default=1.0, description="Axis x component (vector will be normalized)")
    uy: float = Field(default=0.0, description="Axis y component")
    uz: float = Field(default=0.0, description="Axis z component")
    angle_rad: float = Field(..., description="Rotation angle in radians")


class AxisMagnitudeInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    ux: float = Field(default=0.0, description="Axis * angle, x component (radians)")
    uy: float = Field(default=0.0, description="Axis * angle, y component (radians)")
    uz: float = Field(default=0.0, description="Axis * angle, z component (radians)")


class EulerInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    e0: float = Field(..., description="First Euler angle (radians)")
    e1: float = Field(..., description="Second Euler angle (radians)")
    e2: float = Field(..., description="Third Euler angle (radians)")
    order: str = Field(
        default="XYZ",
        description="Intrinsic rotation order. Options: XYZ, XZY, YXZ, YZX, ZXY, ZYX. "
                    "Matches the transform viewer notation:\n"
                    "  xy'z'' → XYZ (extrinsic ZYX)\n"
                    "  xz'y'' → XZY (extrinsic YZX)\n"
                    "  yx'z'' → YXZ (extrinsic ZXY)\n"
                    "  yz'x'' → YZX (extrinsic XZY)\n"
                    "  zx'y'' → ZXY (extrinsic YXZ)\n"
                    "  zy'x'' → ZYX (extrinsic XYZ)"
    )

    @field_validator('order')
    @classmethod
    def validate_order(cls, v: str) -> str:
        v = v.upper()
        if v not in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            raise ValueError(f"Invalid Euler order '{v}'. Choose from: XYZ, XZY, YXZ, YZX, ZXY, ZYX")
        return v


class ComposeInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    m00: float = Field(default=1.0, description="Element [0][0] of 3x3 rotation matrix")
    m01: float = Field(default=0.0, description="Element [0][1] of 3x3 rotation matrix")
    m02: float = Field(default=0.0, description="Element [0][2] of 3x3 rotation matrix")
    m10: float = Field(default=0.0, description="Element [1][0] of 3x3 rotation matrix")
    m11: float = Field(default=1.0, description="Element [1][1] of 3x3 rotation matrix")
    m12: float = Field(default=0.0, description="Element [1][2] of 3x3 rotation matrix")
    m20: float = Field(default=0.0, description="Element [2][0] of 3x3 rotation matrix")
    m21: float = Field(default=0.0, description="Element [2][1] of 3x3 rotation matrix")
    m22: float = Field(default=1.0, description="Element [2][2] of 3x3 rotation matrix")
    tx: float = Field(default=0.0, description="Translation x component")
    ty: float = Field(default=0.0, description="Translation y component")
    tz: float = Field(default=0.0, description="Translation z component")

    def to_rotation_matrix(self) -> List[List[float]]:
        return [
            [self.m00, self.m01, self.m02],
            [self.m10, self.m11, self.m12],
            [self.m20, self.m21, self.m22],
        ]


class ApplyTransformInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    m00: float = Field(default=1.0, description="Element [0][0] of 4x4 transformation matrix")
    m01: float = Field(default=0.0, description="Element [0][1] of 4x4 transformation matrix")
    m02: float = Field(default=0.0, description="Element [0][2] of 4x4 transformation matrix")
    m03: float = Field(default=0.0, description="Element [0][3] (translation x) of 4x4 transformation matrix")
    m10: float = Field(default=0.0, description="Element [1][0] of 4x4 transformation matrix")
    m11: float = Field(default=1.0, description="Element [1][1] of 4x4 transformation matrix")
    m12: float = Field(default=0.0, description="Element [1][2] of 4x4 transformation matrix")
    m13: float = Field(default=0.0, description="Element [1][3] (translation y) of 4x4 transformation matrix")
    m20: float = Field(default=0.0, description="Element [2][0] of 4x4 transformation matrix")
    m21: float = Field(default=0.0, description="Element [2][1] of 4x4 transformation matrix")
    m22: float = Field(default=1.0, description="Element [2][2] of 4x4 transformation matrix")
    m23: float = Field(default=0.0, description="Element [2][3] (translation z) of 4x4 transformation matrix")
    px: float = Field(..., description="Point x coordinate")
    py: float = Field(..., description="Point y coordinate")
    pz: float = Field(..., description="Point z coordinate")


class FromAnyInput(BaseModel):
    """Input for converting from any representation to all others."""
    model_config = ConfigDict(str_strip_whitespace=True, validate_assignment=True)

    # Rotation matrix
    use_matrix: bool = Field(default=False, description="Set to true to input a rotation matrix")
    m00: float = Field(default=1.0, description="Element [0][0] of 3x3 rotation matrix")
    m01: float = Field(default=0.0, description="Element [0][1]")
    m02: float = Field(default=0.0, description="Element [0][2]")
    m10: float = Field(default=0.0, description="Element [1][0]")
    m11: float = Field(default=1.0, description="Element [1][1]")
    m12: float = Field(default=0.0, description="Element [1][2]")
    m20: float = Field(default=0.0, description="Element [2][0]")
    m21: float = Field(default=0.0, description="Element [2][1]")
    m22: float = Field(default=1.0, description="Element [2][2]")

    # Quaternion
    use_quaternion: bool = Field(default=False, description="Set to true to input a quaternion")
    qx: float = Field(default=0.0, description="Quaternion x component")
    qy: float = Field(default=0.0, description="Quaternion y component")
    qz: float = Field(default=0.0, description="Quaternion z component")
    qw: float = Field(default=1.0, description="Quaternion w (scalar) component")

    # Axis-angle
    use_axis_angle: bool = Field(default=False, description="Set to true to input axis-angle")
    aa_ux: float = Field(default=1.0, description="Axis x component")
    aa_uy: float = Field(default=0.0, description="Axis y component")
    aa_uz: float = Field(default=0.0, description="Axis z component")
    aa_angle_rad: float = Field(default=0.0, description="Angle in radians")

    # Axis magnitude
    use_axis_magnitude: bool = Field(default=False, description="Set to true to input axis-magnitude vector")
    am_ux: float = Field(default=0.0, description="Axis * angle, x component")
    am_uy: float = Field(default=0.0, description="Axis * angle, y component")
    am_uz: float = Field(default=0.0, description="Axis * angle, z component")

    # Euler angles
    use_euler: bool = Field(default=False, description="Set to true to input Euler angles")
    e0: float = Field(default=0.0, description="First Euler angle (radians)")
    e1: float = Field(default=0.0, description="Second Euler angle (radians)")
    e2: float = Field(default=0.0, description="Third Euler angle (radians)")
    euler_order: str = Field(
        default="XYZ",
        description="Intrinsic rotation order. Options: XYZ, XZY, YXZ, YZX, ZXY, ZYX"
    )

    @field_validator('euler_order')
    @classmethod
    def validate_order(cls, v: str) -> str:
        v = v.upper()
        if v not in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            raise ValueError(f"Invalid Euler order '{v}'")
        return v


# ─── Tools ───────────────────────────────────────────────────────────────────

def _build_result(rot_matrix: List[List[float]]) -> str:
    '''Convert a rotation matrix to all representations.'''
    qx, qy, qz, qw = matrix_to_quaternion(rot_matrix)
    ux, uy, uz, angle = matrix_to_axis_angle(rot_matrix)
    amx, amy, amz = matrix_to_axis_magnitude(rot_matrix)

    result = {
        "rotation_matrix": rot_matrix,
        "quaternion": {"x": qx, "y": qy, "z": qz, "w": qw},
        "axis_angle": {"ux": ux, "uy": uy, "uz": uz, "angle_rad": angle},
        "axis_magnitude": {"x": amx, "y": amy, "z": amz},
        "euler_angles": {},
    }

    # Compute Euler angles for all 6 orders
    for order_name in ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']:
        try:
            e0, e1, e2 = _euler_from_matrix_single_order(rot_matrix, order_name)
            # Map to transform viewer notation
            notation_map = {
                'XYZ': "xy'z'' (ZYX)",
                'XZY': "xz'y'' (YZX)",
                'YXZ': "yx'z'' (ZXY)",
                'YZX': "yz'x'' (XZY)",
                'ZXY': "zx'y'' (YXZ)",
                'ZYX': "zy'x'' (XYZ)",
            }
            label = notation_map.get(order_name, order_name)
            result["euler_angles"][label] = {"e0": e0, "e1": e1, "e2": e2}
        except Exception:
            result["euler_angles"][order_name] = "gimbal lock / singular"

    return json.dumps(result, indent=2)


@mcp.tool(
    name="transform_from_quaternion",
    annotations={
        "title": "Convert Quaternion to All Representations",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_quaternion(params: QuaternionInput) -> str:
    '''
    Convert a quaternion (x, y, z, w) to all other 3D rotation representations.

    Returns rotation matrix, axis-angle, axis-magnitude, and Euler angles
    for all 6 intrinsic orders (matching the transform viewer).

    The quaternion uses the Hamilton convention (same as THREE.js).

    Args:
        params (QuaternionInput): Validated input containing:
            - qx (float): Quaternion x component
            - qy (float): Quaternion y component
            - qz (float): Quaternion z component
            - qw (float): Quaternion w (scalar) component

    Returns:
        str: JSON string with all rotation representations including:
            - rotation_matrix: 3x3 matrix (list of lists)
            - quaternion: input quaternion echoed back
            - axis_angle: {ux, uy, uz, angle_rad}
            - axis_magnitude: {x, y, z}
            - euler_angles: all 6 intrinsic orders with angles in radians

    Examples:
        - Use when: "Convert quaternion (0, 0, 0, 1) to rotation matrix"
        - Don't use when: You already have a rotation matrix (use transform_from_matrix)
    '''
    try:
        norm = math.sqrt(params.qx ** 2 + params.qy ** 2 + params.qz ** 2 + params.qw ** 2)
        if norm < _EPS:
            return "Error: Quaternion must have non-zero norm (all components are zero)"
        # Normalize
        qx, qy, qz, qw = params.qx / norm, params.qy / norm, params.qz / norm, params.qw / norm
        rot_matrix = quaternion_to_matrix(qx, qy, qz, qw)
        return _build_result(rot_matrix)
    except Exception as e:
        return f"Error: Failed to convert quaternion: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_from_matrix",
    annotations={
        "title": "Convert Rotation Matrix to All Representations",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_matrix(params: RotationMatrixInput) -> str:
    '''
    Convert a 3x3 rotation matrix to all other 3D rotation representations.

    Returns quaternion, axis-angle, axis-magnitude, and Euler angles
    for all 6 intrinsic orders (matching the transform viewer).

    The matrix is validated to be a proper rotation (orthogonal, det=1).

    Args:
        params (RotationMatrixInput): Validated input containing all 9 elements
            of a 3x3 rotation matrix (m00 through m22).

    Returns:
        str: JSON string with all rotation representations including:
            - rotation_matrix: input matrix echoed back
            - quaternion: {x, y, z, w}
            - axis_angle: {ux, uy, uz, angle_rad}
            - axis_magnitude: {x, y, z}
            - euler_angles: all 6 intrinsic orders with angles in radians

    Examples:
        - Use when: "Convert this rotation matrix to quaternion",
          "What are the Euler angles for matrix [[1,0,0],[0,0,-1],[0,1,0]]"
        - Don't use when: The matrix might not be a proper rotation
    '''
    try:
        M = params.to_matrix()
        err = _check_rotation_matrix(M)
        if err:
            return f"Error: {err}"
        return _build_result(M)
    except Exception as e:
        return f"Error: Failed to process matrix: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_from_axis_angle",
    annotations={
        "title": "Convert Axis-Angle to All Representations",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_axis_angle(params: AxisAngleInput) -> str:
    '''
    Convert axis-angle (axis vector + angle in radians) to all other
    3D rotation representations.

    The axis vector is automatically normalized. A zero-axis results
    in identity rotation.

    Args:
        params (AxisAngleInput): Validated input containing:
            - ux, uy, uz (float): Axis components (vector will be normalized)
            - angle_rad (float): Rotation angle in radians

    Returns:
        str: JSON string with all rotation representations

    Examples:
        - Use when: "Convert rotation of 90 degrees around Z axis",
          "axis=(0,1,0), angle=pi/2 to rotation matrix"
        - Don't use when: You have axis * angle as a single vector
          (use transform_from_axis_magnitude)
    '''
    try:
        rot_matrix = axis_angle_to_matrix(params.ux, params.uy, params.uz, params.angle_rad)
        return _build_result(rot_matrix)
    except Exception as e:
        return f"Error: Failed to convert axis-angle: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_from_axis_magnitude",
    annotations={
        "title": "Convert Axis-Magnitude Vector to All Representations",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_axis_magnitude(params: AxisMagnitudeInput) -> str:
    '''
    Convert an axis-magnitude vector (axis * angle as a single 3D vector)
    to all other 3D rotation representations.

    The vector direction is the rotation axis, its magnitude is the
    rotation angle in radians.

    Args:
        params (AxisMagnitudeInput): Validated input containing:
            - ux, uy, uz (float): Axis * angle components (radians)

    Returns:
        str: JSON string with all rotation representations

    Examples:
        - Use when: "Convert the vector (0, 1.57, 0) to quaternion"
          (rotation of ~90° around Y)
        - Don't use when: You have separate axis and angle values
          (use transform_from_axis_angle)
    '''
    try:
        rot_matrix = axis_magnitude_to_matrix(params.ux, params.uy, params.uz)
        return _build_result(rot_matrix)
    except Exception as e:
        return f"Error: Failed to convert axis-magnitude: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_from_euler",
    annotations={
        "title": "Convert Euler Angles to All Representations",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_euler(params: EulerInput) -> str:
    '''
    Convert Euler angles (with configurable intrinsic rotation order)
    to all other 3D rotation representations.

    Uses the intrinsic (local axis) convention matching THREE.js:
      xy'z'' → 'XYZ' order  (extrinsic ZYX)
      zy'x'' → 'ZYX' order  (extrinsic XYZ)

    Args:
        params (EulerInput): Validated input containing:
            - e0 (float): First Euler angle in radians
            - e1 (float): Second Euler angle in radians
            - e2 (float): Third Euler angle in radians
            - order (str): Intrinsic rotation order.
              Options: XYZ, XZY, YXZ, YZX, ZXY, ZYX (default: XYZ)

    Returns:
        str: JSON string with all rotation representations

    Examples:
        - Use when: "Convert Euler angles (0, pi/2, 0) in XYZ order to quaternion"
        - Don't use when: You need a specific extrinsic convention
          (use the complementary order, e.g., intrinsic ZYX = extrinsic XYZ)
    '''
    try:
        rot_matrix = euler_to_matrix(params.e0, params.e1, params.e2, params.order)
        return _build_result(rot_matrix)
    except Exception as e:
        return f"Error: Failed to convert Euler angles: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_compose",
    annotations={
        "title": "Compose 4x4 Transformation Matrix",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_compose(params: ComposeInput) -> str:
    '''
    Compose a 4x4 homogeneous transformation matrix from a 3x3 rotation
    matrix and a 3D translation vector.

    The resulting 4x4 matrix can be used to transform points from a child
    frame to a parent frame (as described in the transform viewer).

    Args:
        params (ComposeInput): Validated input containing:
            - m00..m22 (float): 3x3 rotation matrix elements
            - tx, ty, tz (float): Translation components

    Returns:
        str: JSON string with the 4x4 transformation matrix

    Examples:
        - Use when: "Combine this rotation matrix and translation (1,2,3)"
    '''
    try:
        M = params.to_rotation_matrix()
        err = _check_rotation_matrix(M)
        if err:
            return f"Error: {err}"

        tx, ty, tz = params.tx, params.ty, params.tz
        mat4 = [
            [M[0][0], M[0][1], M[0][2], tx],
            [M[1][0], M[1][1], M[1][2], ty],
            [M[2][0], M[2][1], M[2][2], tz],
            [0.0, 0.0, 0.0, 1.0],
        ]
        result = {
            "transformation_matrix": mat4,
            "description": "child_to_parent_transform. "
            "Use transform_apply_point to transform 3D points with this matrix.",
            "usage": "parent_point = matrix @ child_point  (where point is [x, y, z, 1])",
        }
        return json.dumps(result, indent=2)
    except Exception as e:
        return f"Error: Failed to compose transform: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_apply_point",
    annotations={
        "title": "Apply Transformation Matrix to 3D Point",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_apply_point(params: ApplyTransformInput) -> str:
    '''
    Apply a 4x4 homogeneous transformation matrix to a 3D point.

    The point is treated as [x, y, z, 1] and transformed by the matrix.
    This implements: point_parent = T_parent_child @ point_child

    Args:
        params (ApplyTransformInput): Validated input containing:
            - m00..m23 (float): 4x4 transformation matrix elements
            - px, py, pz (float): 3D point coordinates in source frame

    Returns:
        str: JSON string with the transformed point coordinates

    Examples:
        - Use when: "Transform point (1, 2, 3) by this 4x4 matrix"
        - Use when chained with transform_compose to apply rotation+translation
    '''
    try:
        # Build 4x4 matrix
        T = [
            [params.m00, params.m01, params.m02, params.m03],
            [params.m10, params.m11, params.m12, params.m13],
            [params.m20, params.m21, params.m22, params.m23],
            [0.0, 0.0, 0.0, 1.0],
        ]

        # Transform point [px, py, pz, 1]
        p = [params.px, params.py, params.pz, 1.0]
        result = [sum(T[i][j] * p[j] for j in range(4)) for i in range(4)]

        # Homogeneous to Cartesian
        w = result[3]
        if abs(w) < _EPS:
            return "Error: Transformation resulted in point at infinity (w=0)"
        cartesian = [result[0] / w, result[1] / w, result[2] / w]

        return json.dumps({
            "input_point": {"x": params.px, "y": params.py, "z": params.pz},
            "transformed_point": {"x": cartesian[0], "y": cartesian[1], "z": cartesian[2]},
            "description": "Point in parent frame after applying transformation",
        }, indent=2)
    except Exception as e:
        return f"Error: Failed to apply transform: {type(e).__name__}: {e}"


@mcp.tool(
    name="transform_from_any",
    annotations={
        "title": "Convert Any Rotation Representation to All Others",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
)
async def transform_from_any(params: FromAnyInput) -> str:
    '''
    Convert from ANY single rotation representation to ALL others.

    Set exactly one of these flags to True to select the input format:
      - use_matrix: Input a 3x3 rotation matrix
      - use_quaternion: Input a quaternion (x, y, z, w)
      - use_axis_angle: Input axis (ux, uy, uz) + angle_rad
      - use_axis_magnitude: Input axis*angle vector (ux, uy, uz)
      - use_euler: Input Euler angles (e0, e1, e2, order)

    Args:
        params (FromAnyInput): Set exactly one `use_*` flag to True and
            fill in the corresponding fields.

    Returns:
        str: JSON string with ALL representations (rotation matrix, quaternion,
            axis-angle, axis-magnitude, Euler angles for all 6 orders)

    Examples:
        - Use when: "I have a rotation matrix [[0,-1,0],[1,0,0],[0,0,1]]"
          → set use_matrix=true, m00=0, m01=-1, m02=0, etc.
        - Use when: "What's the axis-angle of (0.707, 0, 0, 0.707) quaternion?"
          → set use_quaternion=true, qx=0.707, qw=0.707
        - Use when: "Convert Euler (pi/2, 0, 0) in XYZ order to quaternion"
          → set use_euler=true, e0=1.5708, order='XYZ'
    '''
    try:
        active_flags = sum([
            params.use_matrix,
            params.use_quaternion,
            params.use_axis_angle,
            params.use_axis_magnitude,
            params.use_euler,
        ])

        if active_flags == 0:
            return "Error: Set one of use_matrix, use_quaternion, use_axis_angle, use_axis_magnitude, or use_euler to True"
        if active_flags > 1:
            return "Error: Set only ONE of use_matrix, use_quaternion, use_axis_angle, use_axis_magnitude, or use_euler to True"

        if params.use_matrix:
            M = [
                [params.m00, params.m01, params.m02],
                [params.m10, params.m11, params.m12],
                [params.m20, params.m21, params.m22],
            ]
            err = _check_rotation_matrix(M)
            if err:
                return f"Error: {err}"
            return _build_result(M)

        if params.use_quaternion:
            norm = math.sqrt(params.qx ** 2 + params.qy ** 2 + params.qz ** 2 + params.qw ** 2)
            if norm < _EPS:
                return "Error: Quaternion has zero norm"
            qx, qy, qz, qw = params.qx / norm, params.qy / norm, params.qz / norm, params.qw / norm
            M = quaternion_to_matrix(qx, qy, qz, qw)
            return _build_result(M)

        if params.use_axis_angle:
            M = axis_angle_to_matrix(params.aa_ux, params.aa_uy, params.aa_uz, params.aa_angle_rad)
            return _build_result(M)

        if params.use_axis_magnitude:
            M = axis_magnitude_to_matrix(params.am_ux, params.am_uy, params.am_uz)
            return _build_result(M)

        if params.use_euler:
            M = euler_to_matrix(params.e0, params.e1, params.e2, params.euler_order)
            return _build_result(M)

    except Exception as e:
        return f"Error: Conversion failed: {type(e).__name__}: {e}"

    return "Error: No conversion method selected"


if __name__ == "__main__":
    mcp.run()

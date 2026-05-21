# 3D 变换 MCP 服务器

一个基于 MCP（模型上下文协议）的服务器，用于在多种常见的 3D 位姿表示之间进行转换：旋转矩阵、四元数、轴角、轴幅矢量、欧拉角以及 4×4 齐次变换矩阵。

算法和约定源自 Daniel Dugas 开发的 [3D Transform Viewer](https://dugas.ch/transform_viewer/index.html)。

## 来源与灵感

本项目直接受优秀的 [3D Transform Viewer](https://dugas.ch/transform_viewer/index.html) 启发——这是一个基于 Three.js 构建的交互式 WebGL 工具，能够实时可视化 3D 变换。在该工具中调整任意 3D 位姿表示时，对应的 3D 坐标轴会实时更新，是理解坐标系之间空间关系的宝贵资源。

本 MCP 服务器将该查看器中所有的数学变换以编程 API 的形式重新实现，使 LLM 能够在机器人学、计算机图形学和计算机视觉工作流中执行精确的 3D 位姿转换。

## 原理

3D 旋转可以用多种数学等价形式表达，各有不同的适用场景：

| 表示形式 | 描述 | 典型用途 |
|---|---|---|
| **旋转矩阵** (3×3) | 正交矩阵，行列式为 +1 | 线性代数运算、渲染管线 |
| **四元数** (x, y, z, w) | 紧凑的 4 参数表示 | 平滑插值（SLERP）、避免万向锁 |
| **轴角** | 旋转轴 + 弧度角 | 直观描述旋转、机器人运动学 |
| **轴幅矢量** | 轴 × 角度作为一个 3D 矢量 | 李代数 so(3)、指数映射 |
| **欧拉角** | 三个顺序旋转（6 种内旋顺序） | 人可读的角度、Three.js 约定 |

该服务器将任意输入表示同时转换为所有其他表示，使用与 Transform Viewer 相同的约定：

- **四元数**使用 Hamilton 约定（与 Three.js 一致）
- **欧拉角**支持全部 6 种内旋顺序（`XYZ`、`XZY`、`YXZ`、`YZX`、`ZXY`、`ZYX`），采用 Three.js 的 `xy'z''` 记号约定
- **轴角**使用 Rodrigues 旋转公式
- **4×4 矩阵**使用齐次坐标实现旋转 + 平移的组合

所有转换都以旋转矩阵作为规范中间形式进行往返。

## 工具

该服务器提供 7 个 MCP 工具：

| 工具 | 描述 |
|---|---|
| `transform_from_quaternion` | 四元数 → 所有其他表示 |
| `transform_from_matrix` | 旋转矩阵 → 所有其他表示 |
| `transform_from_axis_angle` | 轴角 → 所有其他表示 |
| `transform_from_axis_magnitude` | 轴幅矢量 → 所有其他表示 |
| `transform_from_euler` | 欧拉角 → 所有其他表示 |
| `transform_from_any` | 统一入口：输入任意一种表示 → 所有其他表示 |
| `transform_compose` | 从旋转矩阵和平移向量组合 4×4 变换矩阵 |
| `transform_apply_point` | 对 3D 点应用 4×4 矩阵（坐标系变换） |

## 使用方法

### 安装

```bash
# 安装依赖
pip install mcp pydantic
```

### 启动服务器

```bash
python server.py
```

### MCP 客户端配置

添加到 `crush.json` 或 MCP 客户端配置中：

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

### 示例查询

连接后可以询问：

- "将四元数 (0, 0, 0, 1) 转换为旋转矩阵和欧拉角"
- "旋转矩阵 [[0,-1,0],[1,0,0],[0,0,1]] 的轴角表示是什么？"
- "将点 (1, 2, 3) 绕 Z 轴旋转 90 度，然后平移 (0, 0, 5)"
- "绕 X 轴旋转 30 度的所有欧拉角表示是什么？"

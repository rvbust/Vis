

[English](./README_EN.md) | 简体中文

<div align = "center">
<img src=./Data/Images/logo.png width="180"/>
</div>

# Vis: 异步3D可视化工具

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/std/the-standard)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-green.svg)]()

Vis 是一款交互式异步3D可视化工具，旨在让3D视觉和机器人应用开发更简单。

## ✨ 核心功能

- 🎨 **图形绘制** - 支持点、线、面、各种几何体的绘制
- 📦 **3D模型导入** - 支持 STL、DAE、3DS 等常见格式
- 🖱️ **交互工具** - 多种拾取模式，支持物体选择
- 🔧 **Gizmo** - 可视化操作器，支持平移、旋转、缩放
- 🐍 **Python API** - 完整的 Python 绑定，便于快速开发

---

## 📐 架构设计

### 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              公共 API 层                                     │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────┐  ┌──────────────┐      │
│  │  View   │  │ Handle  │  │ Config  │  │ Geometry │  │ Interaction  │      │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬─────┘  └──────┬───────┘      │
└───────┼────────────┼────────────┼────────────┼───────────────┼──────────────┘
        │            │            │            │               │
        ▼            ▼            ▼            ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              命令层                                          │
│  ┌──────────────────┐  ┌───────────────────┐  ┌─────────────────────┐       │
│  │  CommandQueue    │  │  CommandExecutor  │  │  CommandFactory     │       │
│  │  (线程安全队列)   │  │  (异步执行器)      │  │  (类型安全工厂)      │       │
│  └──────────────────┘  └───────────────────┘  └─────────────────────┘       │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              场景层                                          │
│  ┌───────────────┐  ┌──────────────────┐  ┌────────────────────────┐        │
│  │ SceneManager  │  │  NodeRegistry    │  │  TransformManager      │        │
│  │ (场景管理器)   │  │  (句柄映射)       │  │  (变换管理)            │        │
│  └───────────────┘  └──────────────────┘  └────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           几何体工厂层                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐     │
│  │PointFactory│  │ MeshFactory │  │ ShapeFactory│  │ ModelLoader     │     │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────┘     │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              交互层                                          │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────────────────┐      │
│  │ PickerManager   │  │  GizmoController │  │  CameraController      │      │
│  │ (策略模式)       │  │  (状态机)         │  │  (相机操控)            │      │
│  └─────────────────┘  └──────────────────┘  └────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           渲染后端层 (抽象)                                   │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        IRenderBackend                                 │   │
│  │  ┌─────────────────────┐         ┌─────────────────────────────────┐ │   │
│  │  │    OSGBackend       │         │   VulkanBackend (未来扩展)       │ │   │
│  │  │   (OpenSceneGraph)  │         │                                 │ │   │
│  │  └─────────────────────┘         └─────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 核心模块说明

| 模块 | 职责 | 关键类 |
|------|------|--------|
| **公共 API** | 提供用户接口 | `View`, `Handle`, `ViewConfig` |
| **命令层** | 线程安全的命令执行 | `CommandQueue`, `ICommand` |
| **场景层** | 管理场景图和对象 | `SceneManager`, `NodeRegistry` |
| **几何体工厂** | 创建各种几何体 | `ShapeFactory`, `MeshFactory` |
| **交互层** | 处理用户交互 | `PickerManager`, `GizmoController` |
| **渲染后端** | 抽象渲染接口 | `IRenderBackend`, `OSGBackend` |

### 设计模式

- **命令模式** - 封装操作请求，支持异步执行
- **策略模式** - 可切换的拾取策略
- **状态机** - Gizmo 状态管理
- **工厂模式** - 几何体创建
- **Pimpl 模式** - 隐藏实现细节

---

## 🚀 安装

### Linux

```shell
# 安装必要的依赖项
sudo apt-get install build-essential python3-dev cmake git

# 安装 OpenSceneGraph
sudo apt install libopenscenegraph-3.4-dev
# 如果找不到以上版本，请下载源码编译
wget https://github.com/openscenegraph/OpenSceneGraph/archive/refs/tags/OpenSceneGraph-3.4.1.tar.gz

# 下载 Vis 源码
git clone https://github.com/rvbust/Vis.git

# 编译安装
mkdir build && cd build
cmake ..
sudo make install -j

# 配置 Python 路径
echo 'export PYTHONPATH="$PYTHONPATH:/opt/RVBUST/Vis/Python"' >> ~/.bashrc
source ~/.bashrc
```

### Windows

1. **安装 OSG**

   下载 OpenSceneGraph-3.4.1：[点击下载](https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.4.1-VC2017-64-Release.7z)

2. **设置环境变量**

   ```shell
   OSG_ROOT = C:\OpenSceneGraph-3.4.1-VC2017-64-Release
   OSG_BIN_DIR = %OSG_ROOT%\bin
   OSG_INCLUDE_DIR = %OSG_ROOT%\include
   OSG_LIB_DIR = %OSG_ROOT%\lib
   ```

   将 `OSG_BIN_DIR` 添加到 `PATH`

3. **安装 PyVis**

   ```shell
   python3 Setup.py install
   ```

---

## 📖 使用教程

### 快速开始

```python
from RVBUST import Vis

# 创建视图
v = Vis.View("My Visualization")

# 绘制坐标系
axes = v.Axes([0, 0, 0], [0, 0, 0, 1], 1.0, 3.0)

# 绘制立方体
box = v.Box([0, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0])

# 返回 Home 视角
v.Home()
```

### 窗口配置

```python
# 共享场景 - 多个窗口显示同一场景
v1 = Vis.View("View1", shared=True)
v2 = Vis.View("View2", shared=True)

# 独立场景 - 每个窗口独立
v1 = Vis.View("View1", shared=False)
v2 = Vis.View("View2", shared=False)

# 详细配置
cfg = Vis.ViewConfig()
cfg.name = "Custom View"
cfg.x, cfg.y = 100, 100
cfg.width, cfg.height = 1280, 720
cfg.bgcolor = [0.2, 0.2, 0.2, 1.0]
v = Vis.View(cfg, shared=False)
```

### 几何体绘制

| 名称 | 描述 | 示例 |
|------|------|------|
| **Axes** | 坐标系 | <img src=./Data/Images/Axes.png width="80"> |
| **Point** | 点/点云 | <img src=./Data/Images/Point.png width="80"> |
| **Line** | 线段 | <img src=./Data/Images/Line.png width="80"> |
| **Box** | 立方体 | <img src=./Data/Images/Box.png width="80"> |
| **Sphere** | 球体 | <img src=./Data/Images/Sphere.png width="80"> |
| **Cone** | 圆锥 | <img src=./Data/Images/Cone.png width="80"> |
| **Cylinder** | 圆柱 | <img src=./Data/Images/Cylinder.png width="80"> |
| **Arrow** | 箭头 | <img src=./Data/Images/Arrow.png width="80"> |
| **Mesh** | 网格 | <img src=./Data/Images/Mesh.png width="80"> |
| **Plane** | 平面 | <img src=./Data/Images/Plane.png width="80"> |

> **单位约定**: 长度单位为米(m)，角度单位为弧度(rad)，四元数格式为 (x, y, z, w)

### 模型导入

```python
from RVBUST import Vis

v = Vis.View("Robot Visualization")

# 加载机器人模型
hs = v.Load(
    ["BaseLink.stl", "Link1.stl", "Link2.stl", "Link3.stl", "Link4.stl", "Link5.stl", "Link6.stl"],
    [[0, 0, 0], [0, 0, 0], [0.05, 0, 0.33], [0.05, 0, 0.66], [0.05, 0, 0.695], [0.385, 0, 0.695], [0.385, 0, 0.615]],
    [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0.707, 0, 0.707], [0, 0.707, 0, 0.707]]
)
```

<img src=./Data/Images/Robot.gif width="400">

### 交互工具

#### 拾取模式

| 模式 | 功能 | 适用场景 |
|------|------|----------|
| `IntersectorMode_Polytope` | 拾取任意对象 | 通用选择 |
| `IntersectorMode_LineSegment` | 拾取表面点 | 获取坐标 |
| `IntersectorMode_Point` | 拾取点云中的点 | 点云处理 |
| `IntersectorMode_Line` | 拾取线段 | 线条选择 |

```python
# 设置拾取模式
v.SetIntersectorMode(Vis.IntersectorMode_LineSegment)

# 获取拾取结果
handle = v.Picked()
position_normal = v.PickedPlane()  # [x, y, z, nx, ny, nz]
```

<img src=./Data/Images/IntersectorModeLineSegment.gif width="500">

#### Gizmo 操作器

```python
# 启用 Gizmo (1=移动, 2=旋转, 3=缩放, 4=移动+旋转)
v.EnableGizmo(box, 4)
v.SetGizmoDisplayScale(1.5)

# 禁用 Gizmo
v.DisableGizmo()
```

<img src=./Data/Images/Gizmo.gif width="500">

### 完整示例

```python
from RVBUST import Vis

def main():
    # 创建视图
    cfg = Vis.ViewConfig()
    cfg.name = "Vis Demo"
    cfg.width, cfg.height = 800, 600
    v = Vis.View(cfg, shared=False)
    
    # 设置日志级别
    Vis.SetLogLevel("debug")
    
    # 绘制坐标系
    v.Axes([0, 0, 0], [0, 0, 0, 1], 2, 1)
    
    # 绘制几何体
    box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    sphere = v.Sphere([2, 0, 0], 0.5, [0, 1, 0])
    
    # 启用 Gizmo
    v.EnableGizmo(box, 4)
    v.SetGizmoDisplayScale(2)
    
    # 回到主视角
    v.Home()
    
    # 交互式 Shell
    from IPython import embed
    embed()

if __name__ == "__main__":
    main()
```

<img src=./Data/Images/move_box.gif width="700">

---

## 📁 项目结构

```
Vis/
├── Include/Vis/          # 公共头文件
│   ├── Vis.h             # 主头文件
│   ├── Handle.h          # 句柄系统
│   ├── Types.h           # 基础类型
│   └── View.h            # View 类
├── Src/                  # 源代码
│   ├── Core/             # 核心组件
│   ├── Commands/         # 命令系统
│   ├── Backend/          # 渲染后端
│   ├── Interaction/      # 交互系统
│   └── Utils/            # 工具类
├── Examples/             # 示例代码
│   ├── C++/
│   └── Python/
├── Data/                 # 资源文件
└── Externals/            # 外部依赖
```

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 License

[MIT License](./LICENSE)

---

<div align="center">
Made with ❤️ by <a href="https://github.com/rvbust">RVBUST Inc.</a>
</div>

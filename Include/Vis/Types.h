/***********************************************************************
 **
 ** Copyright (c) 2012-2024 RVBUST Inc.
 **
 ** Permission is hereby granted, free of charge, to any person obtaining
 ** a copy of this software and associated documentation files (the
 ** "Software"), to deal in the Software without restriction, including
 ** without limitation the rights to use, copy, modify, merge, publish,
 ** distribute, sublicense, and/or sell copies of the Software, and to
 ** permit persons to whom the Software is furnished to do so, subject to
 ** the following conditions:
 **
 ** The above copyright notice and this permission notice shall be
 ** included in all copies or substantial portions of the Software.
 **
 ** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 ** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 ** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 ** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 ** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 ** OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 ** WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ***********************************************************************/

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace Vis {

//============================================================================
// Basic Math Types
//============================================================================

/// 2D Vector
struct Vec2f {
    float x = 0.0f, y = 0.0f;

    Vec2f() = default;
    Vec2f(float x_, float y_) : x(x_), y(y_) {}

    float& operator[](size_t i) { return (&x)[i]; }
    float operator[](size_t i) const { return (&x)[i]; }

    Vec2f operator+(const Vec2f& v) const { return {x + v.x, y + v.y}; }
    Vec2f operator-(const Vec2f& v) const { return {x - v.x, y - v.y}; }
    Vec2f operator*(float s) const { return {x * s, y * s}; }
    Vec2f operator/(float s) const { return {x / s, y / s}; }

    float dot(const Vec2f& v) const { return x * v.x + y * v.y; }
    float length() const { return std::sqrt(x * x + y * y); }
    float lengthSquared() const { return x * x + y * y; }

    Vec2f normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Vec2f{};
    }

    bool operator==(const Vec2f& v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vec2f& v) const { return !(*this == v); }
};

/// 3D Vector
struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3f() = default;
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    explicit Vec3f(const std::array<float, 3>& arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    float& operator[](size_t i) { return (&x)[i]; }
    float operator[](size_t i) const { return (&x)[i]; }

    Vec3f operator+(const Vec3f& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3f operator-(const Vec3f& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3f operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3f operator-() const { return {-x, -y, -z}; }

    Vec3f& operator+=(const Vec3f& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3f& operator-=(const Vec3f& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3f& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float dot(const Vec3f& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3f cross(const Vec3f& v) const {
        return {
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        };
    }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float lengthSquared() const { return x * x + y * y + z * z; }

    Vec3f normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Vec3f{};
    }

    void normalize() {
        float len = length();
        if (len > 0.0f) {
            x /= len; y /= len; z /= len;
        }
    }

    std::array<float, 3> toArray() const { return {x, y, z}; }

    bool operator==(const Vec3f& v) const { return x == v.x && y == v.y && z == v.z; }
    bool operator!=(const Vec3f& v) const { return !(*this == v); }

    static Vec3f Zero() { return {0, 0, 0}; }
    static Vec3f UnitX() { return {1, 0, 0}; }
    static Vec3f UnitY() { return {0, 1, 0}; }
    static Vec3f UnitZ() { return {0, 0, 1}; }
};

inline Vec3f operator*(float s, const Vec3f& v) { return v * s; }

/// 4D Vector
struct Vec4f {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;

    Vec4f() = default;
    Vec4f(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4f(const Vec3f& v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

    float& operator[](size_t i) { return (&x)[i]; }
    float operator[](size_t i) const { return (&x)[i]; }

    Vec3f xyz() const { return {x, y, z}; }
};

/// Quaternion (x, y, z, w) where w is the scalar part
struct Quatf {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    Quatf() = default;
    Quatf(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    explicit Quatf(const std::array<float, 4>& arr) : x(arr[0]), y(arr[1]), z(arr[2]), w(arr[3]) {}

    /// Create from axis-angle (angle in radians)
    static Quatf fromAxisAngle(const Vec3f& axis, float angle) {
        float halfAngle = angle * 0.5f;
        float s = std::sin(halfAngle);
        Vec3f n = axis.normalized();
        return {n.x * s, n.y * s, n.z * s, std::cos(halfAngle)};
    }

    /// Create from Euler angles (roll, pitch, yaw in radians)
    static Quatf fromEuler(float roll, float pitch, float yaw) {
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);

        return {
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        };
    }

    Quatf operator*(const Quatf& q) const {
        return {
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        };
    }

    Vec3f rotate(const Vec3f& v) const {
        Vec3f qv{x, y, z};
        Vec3f uv = qv.cross(v);
        Vec3f uuv = qv.cross(uv);
        return v + ((uv * w) + uuv) * 2.0f;
    }

    Quatf conjugate() const { return {-x, -y, -z, w}; }

    float length() const { return std::sqrt(x * x + y * y + z * z + w * w); }

    Quatf normalized() const {
        float len = length();
        return len > 0.0f ? Quatf{x / len, y / len, z / len, w / len} : Quatf{};
    }

    void normalize() {
        float len = length();
        if (len > 0.0f) {
            x /= len; y /= len; z /= len; w /= len;
        }
    }

    std::array<float, 4> toArray() const { return {x, y, z, w}; }

    static Quatf Identity() { return {0, 0, 0, 1}; }
};

/// RGBA Color (values in [0, 1])
struct Color4f {
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;

    Color4f() = default;
    Color4f(float r_, float g_, float b_, float a_ = 1.0f) : r(r_), g(g_), b(b_), a(a_) {}
    
    explicit Color4f(const std::vector<float>& v) {
        if (v.size() >= 3) {
            r = v[0]; g = v[1]; b = v[2];
            a = v.size() >= 4 ? v[3] : 1.0f;
        }
    }

    std::vector<float> toVector() const { return {r, g, b, a}; }
    std::array<float, 4> toArray() const { return {r, g, b, a}; }

    static Color4f Red() { return {1, 0, 0, 1}; }
    static Color4f Green() { return {0, 1, 0, 1}; }
    static Color4f Blue() { return {0, 0, 1, 1}; }
    static Color4f White() { return {1, 1, 1, 1}; }
    static Color4f Black() { return {0, 0, 0, 1}; }
    static Color4f Gray() { return {0.5f, 0.5f, 0.5f, 1}; }
    static Color4f Yellow() { return {1, 1, 0, 1}; }
    static Color4f Cyan() { return {0, 1, 1, 1}; }
    static Color4f Magenta() { return {1, 0, 1, 1}; }
};

/// 4x4 Matrix (column-major order for OpenGL compatibility)
struct Mat4f {
    float data[16];

    Mat4f() {
        std::fill(std::begin(data), std::end(data), 0.0f);
        data[0] = data[5] = data[10] = data[15] = 1.0f;  // Identity
    }

    float& operator()(int row, int col) { return data[col * 4 + row]; }
    float operator()(int row, int col) const { return data[col * 4 + row]; }

    float* ptr() { return data; }
    const float* ptr() const { return data; }

    static Mat4f Identity() { return Mat4f{}; }

    static Mat4f Translation(const Vec3f& t) {
        Mat4f m;
        m(0, 3) = t.x;
        m(1, 3) = t.y;
        m(2, 3) = t.z;
        return m;
    }

    static Mat4f Scale(const Vec3f& s) {
        Mat4f m;
        m(0, 0) = s.x;
        m(1, 1) = s.y;
        m(2, 2) = s.z;
        return m;
    }

    static Mat4f FromQuaternion(const Quatf& q) {
        Mat4f m;
        float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        m(0, 0) = 1.0f - 2.0f * (yy + zz);
        m(0, 1) = 2.0f * (xy - wz);
        m(0, 2) = 2.0f * (xz + wy);
        m(1, 0) = 2.0f * (xy + wz);
        m(1, 1) = 1.0f - 2.0f * (xx + zz);
        m(1, 2) = 2.0f * (yz - wx);
        m(2, 0) = 2.0f * (xz - wy);
        m(2, 1) = 2.0f * (yz + wx);
        m(2, 2) = 1.0f - 2.0f * (xx + yy);
        return m;
    }

    Mat4f operator*(const Mat4f& other) const {
        Mat4f result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result(i, j) = 0;
                for (int k = 0; k < 4; ++k) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }

    Vec3f transformPoint(const Vec3f& p) const {
        float w = data[3] * p.x + data[7] * p.y + data[11] * p.z + data[15];
        return {
            (data[0] * p.x + data[4] * p.y + data[8] * p.z + data[12]) / w,
            (data[1] * p.x + data[5] * p.y + data[9] * p.z + data[13]) / w,
            (data[2] * p.x + data[6] * p.y + data[10] * p.z + data[14]) / w
        };
    }

    Vec3f transformVector(const Vec3f& v) const {
        return {
            data[0] * v.x + data[4] * v.y + data[8] * v.z,
            data[1] * v.x + data[5] * v.y + data[9] * v.z,
            data[2] * v.x + data[6] * v.y + data[10] * v.z
        };
    }
};

//============================================================================
// Transform
//============================================================================

/// Combined position and rotation transform
struct Transform {
    Vec3f position;
    Quatf rotation;
    Vec3f scale{1.0f, 1.0f, 1.0f};

    Transform() = default;
    Transform(const Vec3f& pos, const Quatf& rot) : position(pos), rotation(rot) {}
    Transform(const Vec3f& pos, const Quatf& rot, const Vec3f& scl) 
        : position(pos), rotation(rot), scale(scl) {}

    Mat4f toMatrix() const {
        Mat4f r = Mat4f::FromQuaternion(rotation);
        Mat4f s = Mat4f::Scale(scale);
        Mat4f t = Mat4f::Translation(position);
        return t * r * s;
    }

    Transform inverse() const {
        Quatf invRot = rotation.conjugate();
        Vec3f invScale{1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};
        Vec3f invPos = invRot.rotate(-position);
        invPos.x *= invScale.x;
        invPos.y *= invScale.y;
        invPos.z *= invScale.z;
        return {invPos, invRot, invScale};
    }

    Vec3f transformPoint(const Vec3f& p) const {
        Vec3f scaled{p.x * scale.x, p.y * scale.y, p.z * scale.z};
        return position + rotation.rotate(scaled);
    }

    Vec3f transformVector(const Vec3f& v) const {
        Vec3f scaled{v.x * scale.x, v.y * scale.y, v.z * scale.z};
        return rotation.rotate(scaled);
    }

    static Transform Identity() { return {}; }
};

//============================================================================
// Geometry Types
//============================================================================

/// Axis-Aligned Bounding Box
struct AABB {
    Vec3f min{std::numeric_limits<float>::max(), 
              std::numeric_limits<float>::max(), 
              std::numeric_limits<float>::max()};
    Vec3f max{std::numeric_limits<float>::lowest(), 
              std::numeric_limits<float>::lowest(), 
              std::numeric_limits<float>::lowest()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }

    Vec3f center() const { return (min + max) * 0.5f; }
    Vec3f extents() const { return (max - min) * 0.5f; }
    Vec3f size() const { return max - min; }

    void expand(const Vec3f& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    void expand(const AABB& other) {
        expand(other.min);
        expand(other.max);
    }

    bool contains(const Vec3f& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }
};

/// Ray for intersection tests
struct Ray {
    Vec3f origin;
    Vec3f direction;

    Ray() = default;
    Ray(const Vec3f& o, const Vec3f& d) : origin(o), direction(d.normalized()) {}

    Vec3f pointAt(float t) const { return origin + direction * t; }
};

//============================================================================
// Configuration Types
//============================================================================

/// Window configuration
struct WindowConfig {
    std::string name = "Vis3D";
    int x = 0;
    int y = 0;
    int width = 800;
    int height = 600;
    Color4f backgroundColor{1.0f, 1.0f, 1.0f, 1.0f};
    bool useDecoration = true;
    int screenNumber = -1;  // -1 means default screen
};

/// View configuration (alias for backward compatibility)
using ViewConfig = WindowConfig;

//============================================================================
// Enumerations
//============================================================================

/// Intersector mode for picking
enum class IntersectorMode {
    Disabled = 0,
    Polytope,       ///< Pick any object, cannot get intersection point
    LineSegment,    ///< Pick surfaces, get intersection point
    Point,          ///< Pick from point clouds
    Line            ///< Pick lines
};

/// Gizmo operation type
enum class GizmoType {
    None = 0,
    Move = 1,
    Rotate = 2,
    Scale = 3,
    MoveRotate = 4
};

/// Gizmo axis mask
enum class GizmoAxisMask : uint32_t {
    None = 0,
    X = 1,
    Y = 2,
    Z = 4,
    XY = X | Y,
    XZ = X | Z,
    YZ = Y | Z,
    All = X | Y | Z
};

inline GizmoAxisMask operator|(GizmoAxisMask a, GizmoAxisMask b) {
    return static_cast<GizmoAxisMask>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline GizmoAxisMask operator&(GizmoAxisMask a, GizmoAxisMask b) {
    return static_cast<GizmoAxisMask>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

/// Animation loop mode
enum class AnimationLoopMode {
    Swing = 0,   ///< Ping-pong animation
    Loop = 1,    ///< Loop from start
    NoLoop = 2   ///< Play once
};

}  // namespace Vis


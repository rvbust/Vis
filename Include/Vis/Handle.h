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

#include <cstdint>
#include <functional>
#include <optional>
#include <type_traits>

namespace Vis {

//============================================================================
// Object Type Tags
//============================================================================

/// Tags for compile-time type safety of handles
namespace HandleTags {
    struct ViewTag {};
    struct AxesTag {};
    struct PointTag {};
    struct LineTag {};
    struct BoxTag {};
    struct SphereTag {};
    struct ConeTag {};
    struct CylinderTag {};
    struct ArrowTag {};
    struct MeshTag {};
    struct PlaneTag {};
    struct ModelTag {};
    struct TextTag {};
    struct Text2DTag {};
    struct Quad2DTag {};
    struct GizmoTag {};
    struct GenericTag {};  // For backward compatibility
}  // namespace HandleTags

//============================================================================
// Object Type Enumeration
//============================================================================

/// Runtime type identification for objects
enum class ObjectType : uint64_t {
    None = 0,
    View,
    Axes,
    Point,
    Line,
    Box,
    Sphere,
    Cone,
    Cylinder,
    Arrow,
    Mesh,
    Plane,
    Model,
    Text,
    Text2D,
    Quad2D,
    Gizmo,
    Generic,
    
    // Keep this last for counting
    Count
};

/// Convert ObjectType to string
inline const char* objectTypeToString(ObjectType type) {
    switch (type) {
        case ObjectType::None: return "None";
        case ObjectType::View: return "View";
        case ObjectType::Axes: return "Axes";
        case ObjectType::Point: return "Point";
        case ObjectType::Line: return "Line";
        case ObjectType::Box: return "Box";
        case ObjectType::Sphere: return "Sphere";
        case ObjectType::Cone: return "Cone";
        case ObjectType::Cylinder: return "Cylinder";
        case ObjectType::Arrow: return "Arrow";
        case ObjectType::Mesh: return "Mesh";
        case ObjectType::Plane: return "Plane";
        case ObjectType::Model: return "Model";
        case ObjectType::Text: return "Text";
        case ObjectType::Text2D: return "Text2D";
        case ObjectType::Quad2D: return "Quad2D";
        case ObjectType::Gizmo: return "Gizmo";
        case ObjectType::Generic: return "Generic";
        default: return "Unknown";
    }
}

//============================================================================
// Type Traits for Tag -> ObjectType mapping
//============================================================================

namespace Detail {

template<typename Tag>
struct TagToObjectType {
    static constexpr ObjectType value = ObjectType::None;
};

#define VIS_DEFINE_TAG_MAPPING(TagName, TypeValue)           \
    template<>                                                \
    struct TagToObjectType<HandleTags::TagName> {            \
        static constexpr ObjectType value = ObjectType::TypeValue; \
    }

VIS_DEFINE_TAG_MAPPING(ViewTag, View);
VIS_DEFINE_TAG_MAPPING(AxesTag, Axes);
VIS_DEFINE_TAG_MAPPING(PointTag, Point);
VIS_DEFINE_TAG_MAPPING(LineTag, Line);
VIS_DEFINE_TAG_MAPPING(BoxTag, Box);
VIS_DEFINE_TAG_MAPPING(SphereTag, Sphere);
VIS_DEFINE_TAG_MAPPING(ConeTag, Cone);
VIS_DEFINE_TAG_MAPPING(CylinderTag, Cylinder);
VIS_DEFINE_TAG_MAPPING(ArrowTag, Arrow);
VIS_DEFINE_TAG_MAPPING(MeshTag, Mesh);
VIS_DEFINE_TAG_MAPPING(PlaneTag, Plane);
VIS_DEFINE_TAG_MAPPING(ModelTag, Model);
VIS_DEFINE_TAG_MAPPING(TextTag, Text);
VIS_DEFINE_TAG_MAPPING(Text2DTag, Text2D);
VIS_DEFINE_TAG_MAPPING(Quad2DTag, Quad2D);
VIS_DEFINE_TAG_MAPPING(GizmoTag, Gizmo);
VIS_DEFINE_TAG_MAPPING(GenericTag, Generic);

#undef VIS_DEFINE_TAG_MAPPING

}  // namespace Detail

//============================================================================
// TypedHandle - Compile-time type-safe handle
//============================================================================

/**
 * @brief Type-safe handle for scene objects.
 * 
 * TypedHandle provides compile-time type safety for handles to scene objects.
 * Each handle type is distinct at compile time, preventing accidental mixing
 * of handle types.
 * 
 * @tparam Tag A tag type that identifies the handle type (e.g., AxesTag, BoxTag)
 */
template<typename Tag>
class TypedHandle {
public:
    /// Default constructor creates an invalid handle
    constexpr TypedHandle() noexcept : m_id(0) {}

    /// Construct from a unique ID
    constexpr explicit TypedHandle(uint64_t id) noexcept : m_id(id) {}

    /// Get the unique ID
    constexpr uint64_t id() const noexcept { return m_id; }

    /// Check if the handle is valid (non-zero)
    constexpr bool valid() const noexcept { return m_id != 0; }

    /// Explicit conversion to bool for validity check
    constexpr explicit operator bool() const noexcept { return valid(); }

    /// Get the object type
    static constexpr ObjectType objectType() noexcept {
        return Detail::TagToObjectType<Tag>::value;
    }

    /// Reset to invalid state
    void reset() noexcept { m_id = 0; }

    /// Comparison operators
    constexpr bool operator==(const TypedHandle& other) const noexcept {
        return m_id == other.m_id;
    }

    constexpr bool operator!=(const TypedHandle& other) const noexcept {
        return m_id != other.m_id;
    }

    constexpr bool operator<(const TypedHandle& other) const noexcept {
        return m_id < other.m_id;
    }

private:
    uint64_t m_id;
};

//============================================================================
// Concrete Handle Type Aliases
//============================================================================

using ViewHandle     = TypedHandle<HandleTags::ViewTag>;
using AxesHandle     = TypedHandle<HandleTags::AxesTag>;
using PointHandle    = TypedHandle<HandleTags::PointTag>;
using LineHandle     = TypedHandle<HandleTags::LineTag>;
using BoxHandle      = TypedHandle<HandleTags::BoxTag>;
using SphereHandle   = TypedHandle<HandleTags::SphereTag>;
using ConeHandle     = TypedHandle<HandleTags::ConeTag>;
using CylinderHandle = TypedHandle<HandleTags::CylinderTag>;
using ArrowHandle    = TypedHandle<HandleTags::ArrowTag>;
using MeshHandle     = TypedHandle<HandleTags::MeshTag>;
using PlaneHandle    = TypedHandle<HandleTags::PlaneTag>;
using ModelHandle    = TypedHandle<HandleTags::ModelTag>;
using TextHandle     = TypedHandle<HandleTags::TextTag>;
using Text2DHandle   = TypedHandle<HandleTags::Text2DTag>;
using Quad2DHandle   = TypedHandle<HandleTags::Quad2DTag>;
using GizmoHandle    = TypedHandle<HandleTags::GizmoTag>;
using GenericHandle  = TypedHandle<HandleTags::GenericTag>;

//============================================================================
// Handle - Type-erased handle (for runtime polymorphism)
//============================================================================

/**
 * @brief Type-erased handle that can hold any TypedHandle.
 * 
 * Handle provides runtime type information while still being able to
 * interoperate with TypedHandle. Use this when you need to store
 * handles of different types in the same container.
 * 
 * For backward compatibility, this also supports the legacy (type, uid) interface.
 */
class Handle {
public:
    /// Default constructor creates an invalid handle
    constexpr Handle() noexcept : m_type(ObjectType::None), m_id(0) {}

    /// Construct from type and ID (legacy interface)
    constexpr Handle(uint64_t type, uint64_t id) noexcept 
        : m_type(static_cast<ObjectType>(type)), m_id(id) {}

    /// Construct from ObjectType and ID
    constexpr Handle(ObjectType type, uint64_t id) noexcept 
        : m_type(type), m_id(id) {}

    /// Implicit conversion from TypedHandle
    template<typename Tag>
    constexpr Handle(TypedHandle<Tag> h) noexcept 
        : m_type(TypedHandle<Tag>::objectType()), m_id(h.id()) {}

    /// Get the object type
    constexpr ObjectType objectType() const noexcept { return m_type; }

    /// Get the type as uint64_t (legacy interface)
    constexpr uint64_t type() const noexcept { return static_cast<uint64_t>(m_type); }

    /// Get the unique ID
    constexpr uint64_t id() const noexcept { return m_id; }
    
    /// Legacy interface: uid property
    constexpr uint64_t uid() const noexcept { return m_id; }

    /// Check if the handle is valid
    constexpr bool valid() const noexcept { return m_id != 0; }

    /// Explicit conversion to bool
    constexpr explicit operator bool() const noexcept { return valid(); }

    /// Reset to invalid state
    void reset() noexcept {
        m_type = ObjectType::None;
        m_id = 0;
    }

    /// Try to convert to a TypedHandle
    template<typename Tag>
    std::optional<TypedHandle<Tag>> as() const noexcept {
        if (m_type == Detail::TagToObjectType<Tag>::value) {
            return TypedHandle<Tag>(m_id);
        }
        return std::nullopt;
    }

    /// Check if this handle is of a specific type
    template<typename Tag>
    constexpr bool is() const noexcept {
        return m_type == Detail::TagToObjectType<Tag>::value;
    }

    /// Comparison operators
    constexpr bool operator==(const Handle& other) const noexcept {
        return m_type == other.m_type && m_id == other.m_id;
    }

    constexpr bool operator!=(const Handle& other) const noexcept {
        return !(*this == other);
    }

    constexpr bool operator<(const Handle& other) const noexcept {
        if (m_type != other.m_type) return m_type < other.m_type;
        return m_id < other.m_id;
    }

private:
    ObjectType m_type;
    uint64_t m_id;
};

//============================================================================
// Hash support for handles
//============================================================================

/// Hash functor for TypedHandle
template<typename Tag>
struct TypedHandleHasher {
    std::size_t operator()(const TypedHandle<Tag>& h) const noexcept {
        return std::hash<uint64_t>{}(h.id());
    }
};

/// Hash functor for Handle
struct HandleHasher {
    std::size_t operator()(const Handle& h) const noexcept {
        return std::hash<uint64_t>{}(h.type()) ^ 
               (std::hash<uint64_t>{}(h.id()) << 1);
    }
};

}  // namespace Vis

//============================================================================
// std::hash specializations
//============================================================================

namespace std {

template<typename Tag>
struct hash<Vis::TypedHandle<Tag>> {
    size_t operator()(const Vis::TypedHandle<Tag>& h) const noexcept {
        return hash<uint64_t>{}(h.id());
    }
};

template<>
struct hash<Vis::Handle> {
    size_t operator()(const Vis::Handle& h) const noexcept {
        return hash<uint64_t>{}(h.type()) ^ (hash<uint64_t>{}(h.id()) << 1);
    }
};

}  // namespace std


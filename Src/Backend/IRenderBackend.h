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

#include <Vis/Handle.h>
#include <Vis/Types.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Vis {

//============================================================================
// Backend Types
//============================================================================

/// Internal node handle used by the backend
using BackendNodeId = uint64_t;
constexpr BackendNodeId InvalidBackendNodeId = 0;

/// Mesh data for the backend
struct BackendMeshData {
    std::vector<float> vertices;          // xyz triplets
    std::vector<unsigned int> indices;    // Triangle indices
    std::vector<float> normals;           // Optional normals
    std::vector<float> colors;            // Optional per-vertex colors
    std::vector<float> texCoords;         // Optional texture coordinates
};

/// Shape types
enum class ShapeType {
    Box,
    Sphere,
    Cone,
    Cylinder,
    Capsule
};

/// Shape parameters
struct ShapeParams {
    // Common
    Vec3f center{0, 0, 0};
    
    // Box
    Vec3f extents{0.5f, 0.5f, 0.5f};  // Half-extents
    
    // Sphere
    float radius = 0.5f;
    
    // Cone/Cylinder/Capsule
    float height = 1.0f;
    
    // Color
    Color4f color = Color4f::Red();
};

/// Camera pose
struct CameraPose {
    Vec3f eye{0, 0, 5};
    Vec3f center{0, 0, 0};
    Vec3f up{0, 1, 0};
};

//============================================================================
// IRenderBackend Interface
//============================================================================

/**
 * @brief Abstract interface for rendering backends.
 * 
 * This interface decouples the Vis library from specific rendering
 * implementations (like OSG, Vulkan, etc.). All rendering operations
 * go through this interface.
 */
class IRenderBackend {
public:
    virtual ~IRenderBackend() = default;

    //========================================================================
    // Lifecycle
    //========================================================================

    /// Initialize the backend
    virtual bool initialize() = 0;

    /// Shutdown the backend
    virtual void shutdown() = 0;

    /// Check if the backend is initialized
    virtual bool isInitialized() const = 0;

    /// Check if the rendering loop is running
    virtual bool isRunning() const = 0;

    //========================================================================
    // Window Management
    //========================================================================

    /// Create a new window/view
    virtual BackendNodeId createWindow(const WindowConfig& config) = 0;

    /// Destroy a window
    virtual void destroyWindow(BackendNodeId windowId) = 0;

    /// Set window position and size
    virtual void setWindowRectangle(BackendNodeId windowId, int x, int y, 
                                     int width, int height) = 0;

    /// Get window position and size
    virtual void getWindowRectangle(BackendNodeId windowId, int& x, int& y,
                                     int& width, int& height) = 0;

    /// Show/hide window decoration
    virtual void setWindowDecoration(BackendNodeId windowId, bool enabled) = 0;

    /// Raise window to front
    virtual void raiseWindow(BackendNodeId windowId) = 0;

    /// Check if window is closed
    virtual bool isWindowClosed(BackendNodeId windowId) const = 0;

    //========================================================================
    // Scene Graph
    //========================================================================

    /// Create an empty transform node
    virtual BackendNodeId createTransformNode() = 0;

    /// Destroy a node and its children
    virtual void destroyNode(BackendNodeId nodeId) = 0;

    /// Set node visibility
    virtual void setNodeVisible(BackendNodeId nodeId, bool visible) = 0;

    /// Set node transform (4x4 matrix)
    virtual void setNodeTransform(BackendNodeId nodeId, const Mat4f& matrix) = 0;

    /// Get node transform
    virtual Mat4f getNodeTransform(BackendNodeId nodeId) const = 0;

    /// Parent a node under another node
    virtual void setNodeParent(BackendNodeId childId, BackendNodeId parentId) = 0;

    /// Unparent a node (move to root)
    virtual void unparentNode(BackendNodeId nodeId) = 0;

    /// Clone a node
    virtual BackendNodeId cloneNode(BackendNodeId nodeId) = 0;

    //========================================================================
    // Geometry Creation
    //========================================================================

    /// Create axes geometry
    virtual BackendNodeId createAxes(const Vec3f& position, const Quatf& rotation,
                                      float length, float size) = 0;

    /// Create points geometry
    virtual BackendNodeId createPoints(const std::vector<float>& positions,
                                        float pointSize,
                                        const std::vector<float>& colors) = 0;

    /// Create lines geometry
    virtual BackendNodeId createLines(const std::vector<float>& vertices,
                                       float lineWidth,
                                       const std::vector<float>& colors) = 0;

    /// Create a shape (box, sphere, etc.)
    virtual BackendNodeId createShape(ShapeType type, const ShapeParams& params) = 0;

    /// Create an arrow
    virtual BackendNodeId createArrow(const std::vector<float>& tails,
                                       const std::vector<float>& heads,
                                       float radius,
                                       const std::vector<float>& colors) = 0;

    /// Create a mesh
    virtual BackendNodeId createMesh(const BackendMeshData& data,
                                      const Color4f& color) = 0;

    /// Create a plane/grid
    virtual BackendNodeId createPlane(float xLength, float yLength,
                                       int xCells, int yCells,
                                       const Color4f& color) = 0;

    /// Create a ground plane
    virtual BackendNodeId createGround(int halfCells, float cellSize,
                                        const Color4f& color) = 0;

    //========================================================================
    // Model Loading
    //========================================================================

    /// Load a model from file
    virtual BackendNodeId loadModel(const std::string& filepath) = 0;

    //========================================================================
    // Text
    //========================================================================

    /// Create 3D text
    virtual BackendNodeId createText3D(const std::string& content,
                                        const Vec3f& position,
                                        float fontSize,
                                        const Color4f& color) = 0;

    /// Create 2D text (screen space)
    virtual BackendNodeId createText2D(const std::string& content,
                                        const Vec2f& position,
                                        float fontSize,
                                        const Color4f& color) = 0;

    /// Update text content
    virtual void updateText(BackendNodeId nodeId, const std::string& content) = 0;

    /// Set text font
    virtual bool setTextFont(const std::string& fontPath) = 0;

    //========================================================================
    // Appearance
    //========================================================================

    /// Set node color
    virtual void setNodeColor(BackendNodeId nodeId, const Color4f& color) = 0;

    /// Get node color
    virtual Color4f getNodeColor(BackendNodeId nodeId) const = 0;

    /// Set node transparency (0 = opaque, 1 = transparent)
    virtual void setNodeTransparency(BackendNodeId nodeId, float transparency) = 0;

    //========================================================================
    // Camera
    //========================================================================

    /// Set camera pose
    virtual void setCameraPose(BackendNodeId windowId, const CameraPose& pose) = 0;

    /// Get camera pose
    virtual CameraPose getCameraPose(BackendNodeId windowId) const = 0;

    /// Set home pose (default camera position)
    virtual void setHomePose(BackendNodeId windowId, const CameraPose& pose) = 0;

    /// Get home pose
    virtual CameraPose getHomePose(BackendNodeId windowId) const = 0;

    /// Go to home position
    virtual void goHome(BackendNodeId windowId) = 0;

    //========================================================================
    // Picking
    //========================================================================

    /// Pick result
    struct PickResult {
        BackendNodeId nodeId = InvalidBackendNodeId;
        Vec3f position;
        Vec3f normal;
        bool hit = false;
    };

    /// Set picking mode
    virtual void setPickingMode(IntersectorMode mode, bool hoverEnabled) = 0;

    /// Get current picking mode
    virtual IntersectorMode getPickingMode() const = 0;

    /// Get last picked result
    virtual PickResult getLastPick() const = 0;

    /// Get multi-pick results
    virtual std::vector<PickResult> getMultiPick() const = 0;

    /// Clear pick results
    virtual void clearPickResults() = 0;

    //========================================================================
    // Gizmo
    //========================================================================

    /// Enable gizmo on a node
    virtual bool enableGizmo(BackendNodeId nodeId, GizmoType type) = 0;

    /// Disable gizmo
    virtual void disableGizmo() = 0;

    /// Set gizmo type
    virtual void setGizmoType(GizmoType type) = 0;

    /// Set gizmo axis mask
    virtual void setGizmoAxisMask(GizmoType type, uint32_t mask) = 0;

    /// Set gizmo display scale
    virtual void setGizmoDisplayScale(float scale) = 0;

    /// Set gizmo detection range
    virtual void setGizmoDetectionRange(float range) = 0;

    /// Get the node currently being manipulated by gizmo
    virtual BackendNodeId getGizmoTarget() const = 0;

    //========================================================================
    // Animation
    //========================================================================

    /// Set object animation path
    virtual bool setObjectAnimation(BackendNodeId nodeId, bool enable,
                                     float duration, AnimationLoopMode loopMode,
                                     const std::vector<Vec3f>& positions,
                                     const std::vector<Quatf>& rotations) = 0;

    /// Set camera animation path
    virtual bool setCameraAnimation(BackendNodeId windowId, bool enable,
                                     float duration, AnimationLoopMode loopMode,
                                     const std::vector<CameraPose>& poses) = 0;

    //========================================================================
    // Frame Update
    //========================================================================

    /// Process one frame (must be called from rendering thread)
    virtual void frame() = 0;

    /// Request a redraw
    virtual void requestRedraw() = 0;

    //========================================================================
    // Callbacks
    //========================================================================

    using FrameCallback = std::function<void()>;
    using PickCallback = std::function<void(const PickResult&)>;
    using GizmoCallback = std::function<void(BackendNodeId, const Transform&)>;

    /// Set frame callback (called once per frame)
    virtual void setFrameCallback(FrameCallback callback) = 0;

    /// Set pick callback
    virtual void setPickCallback(PickCallback callback) = 0;

    /// Set gizmo transform callback
    virtual void setGizmoCallback(GizmoCallback callback) = 0;
};

/// Unique pointer to render backend
using RenderBackendPtr = std::unique_ptr<IRenderBackend>;

}  // namespace Vis


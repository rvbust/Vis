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
#include "../Backend/IRenderBackend.h"
#include "../Commands/GeometryCommands.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Vis {

/**
 * @brief Manages the scene graph and object lifecycle.
 * 
 * SceneManager provides a high-level interface for creating and manipulating
 * scene objects. It wraps the rendering backend and manages the mapping
 * between public handles and internal backend nodes.
 */
class SceneManager {
public:
    /**
     * @brief Construct a SceneManager with a rendering backend.
     * @param backend The rendering backend to use
     */
    explicit SceneManager(RenderBackendPtr backend);

    ~SceneManager();

    // Non-copyable, non-movable (owns unique resources)
    SceneManager(const SceneManager&) = delete;
    SceneManager& operator=(const SceneManager&) = delete;
    SceneManager(SceneManager&&) = delete;
    SceneManager& operator=(SceneManager&&) = delete;

    //========================================================================
    // Lifecycle
    //========================================================================

    /// Initialize the scene manager
    bool initialize();

    /// Shutdown the scene manager
    void shutdown();

    /// Check if initialized
    bool isInitialized() const;

    //========================================================================
    // Window Management
    //========================================================================

    /// Create a new view/window
    ViewHandle createView(const WindowConfig& config);

    /// Close a view
    bool closeView(ViewHandle handle);

    /// Check if a view is closed
    bool isViewClosed(ViewHandle handle) const;

    /// Set window rectangle
    void setWindowRectangle(ViewHandle handle, int x, int y, int width, int height);

    /// Get window rectangle
    void getWindowRectangle(ViewHandle handle, int& x, int& y, int& width, int& height) const;

    /// Set window decoration
    void setWindowDecoration(ViewHandle handle, bool enabled);

    /// Raise window
    void raiseWindow(ViewHandle handle);

    /// Go home
    void goHome(ViewHandle handle);

    //========================================================================
    // Geometry Creation
    //========================================================================

    /// Create axes
    AxesHandle createAxes(const CreateAxesData& data);
    
    /// Create multiple axes
    std::vector<AxesHandle> createAxes(const std::vector<CreateAxesData>& data);

    /// Create points
    PointHandle createPoints(const CreatePointData& data);

    /// Create lines
    LineHandle createLines(const CreateLineData& data);

    /// Create box
    BoxHandle createBox(const CreateBoxData& data);

    /// Create sphere
    SphereHandle createSphere(const CreateSphereData& data);

    /// Create multiple spheres
    SphereHandle createSpheres(const CreateSpheresData& data);

    /// Create cone
    ConeHandle createCone(const CreateConeData& data);

    /// Create cylinder
    CylinderHandle createCylinder(const CreateCylinderData& data);

    /// Create arrow
    ArrowHandle createArrow(const CreateArrowData& data);

    /// Create mesh
    MeshHandle createMesh(const CreateMeshData& data);

    /// Create plane
    PlaneHandle createPlane(const CreatePlaneData& data);

    /// Create ground
    PlaneHandle createGround(int halfCells, float cellSize, const Color4f& color);

    //========================================================================
    // Model Loading
    //========================================================================

    /// Load a model from file
    ModelHandle loadModel(const std::string& filepath);

    /// Load a model with transform
    ModelHandle loadModel(const std::string& filepath, 
                          const Vec3f& position, 
                          const Quatf& rotation);

    /// Load multiple models
    std::vector<ModelHandle> loadModels(const std::vector<std::string>& filepaths);

    /// Load multiple models with transforms
    std::vector<ModelHandle> loadModels(const std::vector<std::string>& filepaths,
                                         const std::vector<Vec3f>& positions,
                                         const std::vector<Quatf>& rotations);

    //========================================================================
    // Text
    //========================================================================

    /// Create 3D text
    TextHandle createText3D(const std::string& content, const Vec3f& position,
                            float fontSize, const Color4f& color);

    /// Create 2D text
    Text2DHandle createText2D(const std::string& content, const Vec2f& position,
                              float fontSize, const Color4f& color);

    /// Update text content
    bool updateText(Handle handle, const std::string& content);

    /// Set text font
    bool setTextFont(const std::string& fontPath);

    //========================================================================
    // Node Operations
    //========================================================================

    /// Check if a handle is valid
    bool exists(Handle handle) const;

    /// Delete a node
    bool deleteNode(Handle handle);

    /// Delete multiple nodes
    bool deleteNodes(const std::vector<Handle>& handles);

    /// Clear all nodes
    bool clearAll();

    /// Show a node
    bool show(Handle handle);

    /// Hide a node
    bool hide(Handle handle);

    /// Clone a node
    Handle clone(Handle handle);

    /// Clone a node with transform
    Handle clone(Handle handle, const Vec3f& position, const Quatf& rotation);

    /// Clone multiple nodes
    std::vector<Handle> clone(const std::vector<Handle>& handles);

    //========================================================================
    // Transform Operations
    //========================================================================

    /// Set position
    bool setPosition(Handle handle, const Vec3f& position);

    /// Set rotation
    bool setRotation(Handle handle, const Quatf& rotation);

    /// Set transform
    bool setTransform(Handle handle, const Vec3f& position, const Quatf& rotation);

    /// Set multiple transforms
    bool setTransforms(const std::vector<Handle>& handles,
                       const std::vector<Vec3f>& positions,
                       const std::vector<Quatf>& rotations);

    /// Get position
    std::optional<Vec3f> getPosition(Handle handle) const;

    /// Get rotation
    std::optional<Quatf> getRotation(Handle handle) const;

    /// Get transform
    std::optional<Transform> getTransform(Handle handle) const;

    //========================================================================
    // Appearance
    //========================================================================

    /// Set color
    bool setColor(Handle handle, const Color4f& color);

    /// Get color
    std::optional<Color4f> getColor(Handle handle) const;

    /// Set transparency
    bool setTransparency(Handle handle, float transparency);

    //========================================================================
    // Hierarchy
    //========================================================================

    /// Chain nodes (parent-child relationship)
    bool chain(const std::vector<Handle>& handles);

    /// Unchain nodes
    bool unchain(const std::vector<Handle>& handles);

    //========================================================================
    // Camera
    //========================================================================

    /// Set camera pose
    bool setCameraPose(ViewHandle view, const Vec3f& eye, 
                       const Vec3f& center, const Vec3f& up);

    /// Get camera pose
    bool getCameraPose(ViewHandle view, Vec3f& eye, Vec3f& center, Vec3f& up) const;

    /// Set home pose
    bool setHomePose(ViewHandle view, const Vec3f& eye,
                     const Vec3f& center, const Vec3f& up);

    /// Get home pose
    bool getHomePose(ViewHandle view, Vec3f& eye, Vec3f& center, Vec3f& up) const;

    //========================================================================
    // Picking
    //========================================================================

    /// Set intersector mode
    void setIntersectorMode(IntersectorMode mode, bool hoverEnabled = true);

    /// Get intersector mode
    IntersectorMode getIntersectorMode() const;

    /// Get picked handle
    Handle getPicked() const;

    /// Get multi-picked handles
    std::vector<Handle> getMultiPicked() const;

    /// Get picked position and normal
    bool getPickedPlane(Vec3f& position, Vec3f& normal) const;

    /// Get picked point axes handles
    std::vector<Handle> getPickedPointAxes() const;

    /// Clear picked point axes
    void clearPickedPointAxes();

    //========================================================================
    // Gizmo
    //========================================================================

    /// Enable gizmo
    bool enableGizmo(Handle handle, GizmoType type);

    /// Disable gizmo
    bool disableGizmo();

    /// Set gizmo type
    bool setGizmoType(GizmoType type);

    /// Set gizmo draw mask
    bool setGizmoDrawMask(GizmoType type, uint32_t mask);

    /// Set gizmo display scale
    bool setGizmoDisplayScale(float scale);

    /// Set gizmo detection range
    bool setGizmoDetectionRange(float range);

    //========================================================================
    // Animation
    //========================================================================

    /// Set object animation
    bool setObjectAnimation(Handle handle, bool enable, float duration,
                            AnimationLoopMode loopMode,
                            const std::vector<Vec3f>& positions,
                            const std::vector<Quatf>& rotations);

    /// Set camera animation
    bool setCameraAnimation(ViewHandle view, bool enable, float duration,
                            AnimationLoopMode loopMode,
                            const std::vector<Vec3f>& eyes,
                            const std::vector<Vec3f>& centers,
                            const std::vector<Vec3f>& ups);

    //========================================================================
    // Frame Processing
    //========================================================================

    /// Process one frame
    void frame();

    /// Get the rendering backend (for advanced use)
    IRenderBackend* getBackend() const { return m_backend.get(); }

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
    RenderBackendPtr m_backend;
};

}  // namespace Vis


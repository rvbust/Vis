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

#include "Export.h"
#include "Handle.h"
#include "Types.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace Vis {

/**
 * @brief Main view class for 3D visualization.
 * 
 * View provides a high-level API for creating 3D visualizations.
 * It manages a window and scene, providing methods for:
 * - Creating geometric primitives (points, lines, boxes, spheres, etc.)
 * - Loading 3D models
 * - Manipulating object transforms
 * - Interactive picking and gizmo manipulation
 * - Camera control
 * 
 * Views can optionally share a common scene, allowing multiple windows
 * to display the same content.
 * 
 * @code
 * // Create a simple view
 * Vis::View view("My Visualization");
 * 
 * // Add some geometry
 * auto axes = view.Axes({0,0,0}, {0,0,0,1}, 1.0f, 3.0f);
 * auto box = view.Box({0,0,0}, {0.5,0.5,0.5}, {1,0,0});
 * 
 * // Enable gizmo for manipulation
 * view.EnableGizmo(box, Vis::GizmoType::MoveRotate);
 * @endcode
 */
class VIS_API View {
public:
    //========================================================================
    // Construction / Destruction
    //========================================================================

    /**
     * @brief Create a view with the given name.
     * @param viewname Name displayed on the window title
     * @param shared If true, share the scene with other shared views
     */
    explicit View(const std::string& viewname = "Vis3D", bool shared = true);

    /**
     * @brief Create a view with detailed configuration.
     * @param config Window configuration
     * @param shared If true, share the scene with other shared views
     */
    explicit View(const ViewConfig& config, bool shared = true);

    /**
     * @brief Destructor. Closes the view if still open.
     */
    virtual ~View();

    // Non-copyable
    View(const View&) = delete;
    View& operator=(const View&) = delete;

    // Movable
    View(View&& other) noexcept;
    View& operator=(View&& other) noexcept;

    //========================================================================
    // Window Management
    //========================================================================

    /// Close the view window
    bool Close();

    /// Check if the view is closed
    bool IsClosed() const;

    /// Enable/disable window decoration (title bar, borders)
    void WindowSetDecoration(bool enable);

    /// Set window position and size
    void WindowSetRectangle(int x, int y, int width, int height);

    /// Get window position and size
    void WindowGetRectangle(int& x, int& y, int& width, int& height);

    /// Raise window to the top
    void WindowRaise();

    /// Hide the window, returns the previous rectangle
    std::array<int, 4> WindowHide();

    /// Show the window at the given rectangle
    void WindowShow(const std::array<int, 4>& r);

    /// Get the view size
    bool GetViewSize(int& width, int& height);

    //========================================================================
    // Camera Control
    //========================================================================

    /// Set camera pose
    bool SetCameraPose(const std::array<float, 3>& eye,
                       const std::array<float, 3>& center,
                       const std::array<float, 3>& up);

    /// Get camera pose
    bool GetCameraPose(std::array<float, 3>& eye,
                       std::array<float, 3>& center,
                       std::array<float, 3>& up);

    /// Set home pose (default camera position when pressing Space)
    bool SetHomePose(const std::array<float, 3>& eye,
                     const std::array<float, 3>& center,
                     const std::array<float, 3>& up);

    /// Get home pose
    bool GetHomePose(std::array<float, 3>& eye,
                     std::array<float, 3>& center,
                     std::array<float, 3>& up);

    /// Go to home position
    bool Home();

    //========================================================================
    // Node Management
    //========================================================================

    /// Delete a node from the scene
    bool Delete(const Handle& h);

    /// Delete multiple nodes
    bool Delete(const std::vector<Handle>& handles);

    /// Clear all nodes from the scene
    bool Clear();

    /// Show a node
    bool Show(const Handle& h);

    /// Hide a node
    bool Hide(const Handle& h);

    /// Check if a handle is still valid
    bool IsAlive(const Handle& h) const;

    /// Clone a node
    Handle Clone(const Handle& h);

    /// Clone a node with a new transform
    Handle Clone(const Handle& h, const std::array<float, 3>& pos,
                 const std::array<float, 4>& quat);

    /// Clone multiple nodes
    std::vector<Handle> Clone(const std::vector<Handle>& handles);

    /// Clone multiple nodes with new transforms
    std::vector<Handle> Clone(const std::vector<Handle>& handles,
                              const std::vector<std::array<float, 3>>& positions,
                              const std::vector<std::array<float, 4>>& rotations);

    //========================================================================
    // Hierarchy
    //========================================================================

    /// Chain nodes together (parent-child relationships)
    bool Chain(const std::vector<Handle>& links);

    /// Unchain nodes
    bool Unchain(const std::vector<Handle>& links);

    //========================================================================
    // Transform
    //========================================================================

    /// Set node position
    bool SetPosition(const Handle& h, const std::array<float, 3>& pos);

    /// Set node rotation (quaternion: x, y, z, w)
    bool SetRotation(const Handle& h, const std::array<float, 4>& quat);

    /// Set node transform
    bool SetTransform(const Handle& h, const std::array<float, 3>& pos,
                      const std::array<float, 4>& quat);

    /// Set multiple transforms
    bool SetTransforms(const std::vector<Handle>& handles,
                       const std::vector<std::array<float, 3>>& positions,
                       const std::vector<std::array<float, 4>>& rotations);

    /// Set transforms for a chain of linked nodes
    bool SetTransforms(const Handle& h,
                       const std::vector<std::array<float, 3>>& positions,
                       const std::vector<std::array<float, 4>>& rotations);

    /// Get node position
    bool GetPosition(const Handle& h, std::array<float, 3>& pos);

    /// Get node rotation
    bool GetRotation(const Handle& h, std::array<float, 4>& quat);

    /// Get node transform
    bool GetTransform(const Handle& h, std::array<float, 3>& pos,
                      std::array<float, 4>& quat);

    //========================================================================
    // Appearance
    //========================================================================

    /// Set node color
    bool SetColor(const Handle& h, const std::vector<float>& color);

    /// Get node color
    std::vector<float> GetColor(const Handle& h) const;

    /// Set transparency (0 = fully transparent, 1 = opaque)
    bool SetTransparency(const Handle& h, float alpha);

    //========================================================================
    // Geometry Creation
    //========================================================================

    /// Create coordinate axes
    Handle Axes(const std::array<float, 3>& position = {0, 0, 0},
                const std::array<float, 4>& quaternion = {0, 0, 0, 1},
                float axisLength = 1.0f, float axisSize = 3.0f);

    /// Create axes from a 4x4 transform matrix
    Handle Axes(const std::array<float, 16>& transform,
                float axisLength = 1.0f, float axisSize = 3.0f);

    /// Create multiple axes
    std::vector<Handle> Axes(const std::vector<std::array<float, 3>>& positions,
                             const std::vector<std::array<float, 4>>& quaternions,
                             float axisLength = 1.0f, float axisSize = 3.0f);

    /// Create points
    Handle Point(const std::vector<float>& positions, float pointSize = 1.0f,
                 const std::vector<float>& colors = {1.0f, 0, 0});

    /// Create lines (every 6 floats = one line: start xyz, end xyz)
    Handle Line(const std::vector<float>& lines, float lineWidth = 1.0f,
                const std::vector<float>& colors = {1.0f, 0, 0});

    /// Create a box
    Handle Box(const std::array<float, 3>& center,
               const std::array<float, 3>& extents,
               const std::vector<float>& color = {1.0f, 0, 0});

    /// Create a sphere
    Handle Sphere(const std::array<float, 3>& center, float radius,
                  const std::vector<float>& color = {1.0f, 0, 0});

    /// Create multiple spheres
    Handle Spheres(const std::vector<float>& centers,
                   std::vector<float>& radii,
                   const std::vector<float>& colors = {1.0f, 0, 0, 1.0f});

    /// Create a cone
    Handle Cone(const std::array<float, 3>& center, float radius, float height,
                const std::vector<float>& color = {1.0f, 0, 0});

    /// Create a cylinder
    Handle Cylinder(const std::array<float, 3>& center, float radius, float height,
                    const std::vector<float>& color = {1.0f, 0, 0});

    /// Create an arrow (or multiple arrows)
    Handle Arrow(const std::vector<float>& tails, const std::vector<float>& heads,
                 float radius, const std::vector<float>& colors = {1.0f, 0, 0});

    /// Create a mesh from vertices and indices
    Handle Mesh(const std::vector<float>& vertices,
                const std::vector<unsigned int>& indices,
                const std::vector<float>& colors = {1.0f, 0, 0});

    /// Create a plane
    Handle Plane(float xLength, float yLength,
                 int halfXCells = 8, int halfYCells = 8,
                 const std::vector<float>& color = {0.5f, 0.5f, 0.5f});

    /// Create a plane with transform
    Handle Plane(float xLength, float yLength,
                 int halfXCells, int halfYCells,
                 const std::array<float, 3>& position,
                 const std::array<float, 4>& quaternion,
                 const std::vector<float>& color = {0.5f, 0.5f, 0.5f});

    /// Create a ground plane
    Handle Ground(int halfCells, float cellSize, const std::vector<float>& color);

    //========================================================================
    // Model Loading
    //========================================================================

    /// Load a model from file
    Handle Load(const std::string& filename);

    /// Load a model with initial transform
    Handle Load(const std::string& filename,
                const std::array<float, 3>& position,
                const std::array<float, 4>& quaternion);

    /// Load multiple models
    std::vector<Handle> Load(const std::vector<std::string>& filenames);

    /// Load multiple models with transforms
    std::vector<Handle> Load(const std::vector<std::string>& filenames,
                             const std::vector<std::array<float, 3>>& positions,
                             const std::vector<std::array<float, 4>>& quaternions);

    //========================================================================
    // Text
    //========================================================================

    /// Create text (2D if position has 2 elements, 3D if 3)
    Handle Text(const std::string& content, const std::vector<float>& position,
                float fontSize = 0.02f,
                const std::vector<float>& colors = {0, 0, 0});

    /// Update existing text
    bool SetText(Handle& h, const std::string& content,
                 const std::vector<float>& position,
                 float fontSize = 0.02f,
                 const std::vector<float>& colors = {0, 0, 0});

    /// Set text font
    bool SetTextFont(const std::string& fontPath);

    /// Create a 2D quad
    Handle Quad2D(const std::vector<float>& vertices,
                  const std::vector<float>& color = {1.0f, 0, 0},
                  int mode = 0);

    //========================================================================
    // Picking
    //========================================================================

    /// Set intersector mode for picking
    void SetIntersectorMode(IntersectorMode mode, bool hover = true);

    /// Get current intersector mode
    IntersectorMode GetIntersectorMode() const;

    /// Get the last picked handle
    Handle Picked();

    /// Get multiple picked handles (Ctrl+click)
    std::vector<Handle> MultiPicked() const;

    /// Get the picked position and normal
    std::array<float, 6> PickedPlane();

    /// Get handles of axes drawn at picked points
    std::vector<Handle>& GetPickedPointAxes();

    /// Clear picked point axes
    void ClearPickedPointAxes();

    //========================================================================
    // Gizmo
    //========================================================================

    /// Enable gizmo on an object
    /// @param h Target object handle
    /// @param gizmoType 1=Move, 2=Rotate, 3=Scale, 4=Move+Rotate
    bool EnableGizmo(const Handle& h, int gizmoType);

    /// Disable gizmo
    bool DisableGizmo();

    /// Set gizmo type (1=Move, 2=Rotate, 3=Scale)
    bool SetGizmoType(int gizmoType);

    /// Set gizmo axis draw mask
    bool SetGizmoDrawMask(int gizmoType, unsigned int mask);

    /// Set gizmo display scale
    bool SetGizmoDisplayScale(float scale);

    /// Set gizmo detection range
    bool SetGizmoDetectionRange(float range);

    //========================================================================
    // Animation
    //========================================================================

    /// Set object animation path
    bool SetObjectAnimation(const Handle& h, bool enable,
                            float duration = 6.0f, int loopMode = 1,
                            const std::vector<float>& positions = {},
                            const std::vector<float>& quaternions = {});

    /// Set camera animation path
    bool SetCameraAnimation(bool enable, float duration = 6.0f, int loopMode = 1,
                            const std::vector<float>& positions = {},
                            const std::vector<float>& quaternions = {});

    /// Set camera animation using eye/center/up vectors
    bool SetCameraAnimation(bool enable, float duration, int loopMode,
                            const std::vector<float>& eyes,
                            const std::vector<float>& centers,
                            const std::vector<float>& ups);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Vis


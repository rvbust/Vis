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
#include "../Utils/Signal.h"

namespace Vis {

// Forward declarations
class IRenderBackend;
class NodeRegistry;

/**
 * @brief Current state of the gizmo.
 */
enum class GizmoState {
    Inactive,   ///< No gizmo active
    Idle,       ///< Gizmo visible but not being used
    Hovering,   ///< Mouse hovering over gizmo
    Dragging    ///< Gizmo being dragged
};

/**
 * @brief Which axis/plane is being manipulated.
 */
enum class GizmoAxis {
    None,
    X,
    Y,
    Z,
    XY,
    XZ,
    YZ,
    All,        ///< Uniform scale
    Screen      ///< Screen-space rotation
};

/**
 * @brief Controls the gizmo for manipulating objects.
 * 
 * GizmoController manages the state machine for gizmo interactions
 * and coordinates with the rendering backend for visual feedback.
 */
class GizmoController {
public:
    GizmoController(IRenderBackend& backend, NodeRegistry& registry);
    ~GizmoController();

    // Non-copyable
    GizmoController(const GizmoController&) = delete;
    GizmoController& operator=(const GizmoController&) = delete;

    //========================================================================
    // Gizmo Control
    //========================================================================

    /// Enable the gizmo on a specific object
    bool enable(Handle target, GizmoType type);

    /// Disable the gizmo
    void disable();

    /// Check if the gizmo is enabled
    bool isEnabled() const { return m_state != GizmoState::Inactive; }

    /// Get the current target handle
    Handle getTarget() const { return m_target; }

    //========================================================================
    // Gizmo Type
    //========================================================================

    /// Set the gizmo type (move/rotate/scale)
    void setType(GizmoType type);

    /// Get the current gizmo type
    GizmoType getType() const { return m_type; }

    //========================================================================
    // Display Settings
    //========================================================================

    /// Set which axes are displayed/enabled
    void setAxisMask(GizmoType type, uint32_t mask);

    /// Get the axis mask for a gizmo type
    uint32_t getAxisMask(GizmoType type) const;

    /// Set the display scale
    void setDisplayScale(float scale);

    /// Get the display scale
    float getDisplayScale() const { return m_displayScale; }

    /// Set the detection range (how close mouse needs to be to interact)
    void setDetectionRange(float range);

    /// Get the detection range
    float getDetectionRange() const { return m_detectionRange; }

    //========================================================================
    // State
    //========================================================================

    /// Get the current state
    GizmoState getState() const { return m_state; }

    /// Get which axis is currently active
    GizmoAxis getActiveAxis() const { return m_activeAxis; }

    //========================================================================
    // Input Handling
    //========================================================================

    /// Handle mouse button press
    /// @return true if the gizmo captured the input
    bool onMouseDown(const Vec2f& screenPos, int button);

    /// Handle mouse movement
    /// @return true if the gizmo captured the input
    bool onMouseMove(const Vec2f& screenPos);

    /// Handle mouse button release
    void onMouseUp(const Vec2f& screenPos, int button);

    //========================================================================
    // Signals
    //========================================================================

    /// Signal emitted when a transform operation starts
    Signal<Handle> onTransformBegin;

    /// Signal emitted during a transform operation
    Signal<Handle, const Transform&> onTransformChanged;

    /// Signal emitted when a transform operation ends
    Signal<Handle, const Transform&> onTransformEnd;

private:
    void updateGizmoTransform();
    void beginDrag(GizmoAxis axis, const Vec2f& screenPos);
    void updateDrag(const Vec2f& screenPos);
    void endDrag();

    IRenderBackend& m_backend;
    NodeRegistry& m_registry;

    // State
    GizmoState m_state = GizmoState::Inactive;
    GizmoType m_type = GizmoType::None;
    GizmoAxis m_activeAxis = GizmoAxis::None;
    Handle m_target;

    // Settings
    float m_displayScale = 1.0f;
    float m_detectionRange = 0.1f;
    uint32_t m_moveAxisMask = 0x3F;    // All axes
    uint32_t m_rotateAxisMask = 0x1F;  // All axes + screen
    uint32_t m_scaleAxisMask = 0x3F;   // All axes

    // Drag state
    Vec2f m_dragStartPos;
    Transform m_initialTransform;
    
    // The gizmo's own transformation matrix
    float m_gizmoMatrix[16];

    // Capture flag for coordinating with camera
    int m_captureFlags = 0;
};

}  // namespace Vis


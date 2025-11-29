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

#include <memory>
#include <vector>

namespace Vis {

// Forward declarations
class IRenderBackend;
class NodeRegistry;

/**
 * @brief Pick result from a picking operation.
 */
struct PickResult {
    Handle handle;              ///< The picked handle (invalid if no hit)
    Vec3f position{0, 0, 0};    ///< World position of the hit
    Vec3f normal{0, 0, 1};      ///< Surface normal at the hit point
    float distance = 0.0f;      ///< Distance from camera
    bool hit = false;           ///< Whether something was hit

    /// Check if this is a valid pick
    bool valid() const { return hit && handle.valid(); }
};

/**
 * @brief Interface for pick strategies.
 * 
 * Different picking strategies can be used depending on the type
 * of objects being picked (surfaces, points, lines, etc.).
 */
class IPickStrategy {
public:
    virtual ~IPickStrategy() = default;

    /// Get the name of this strategy
    virtual std::string name() const = 0;

    /// Get the intersector mode this strategy handles
    virtual IntersectorMode mode() const = 0;

    /// Perform a pick at the given screen coordinates
    virtual PickResult pick(const Vec2f& screenPos, IRenderBackend& backend) = 0;
};

/**
 * @brief Manages picking operations and pick state.
 * 
 * PickerManager coordinates picking operations, manages the current
 * pick mode, and maintains the list of picked objects.
 */
class PickerManager {
public:
    PickerManager(IRenderBackend& backend, NodeRegistry& registry);
    ~PickerManager();

    // Non-copyable
    PickerManager(const PickerManager&) = delete;
    PickerManager& operator=(const PickerManager&) = delete;

    //========================================================================
    // Mode Management
    //========================================================================

    /// Set the current intersector mode
    void setMode(IntersectorMode mode);

    /// Get the current intersector mode
    IntersectorMode getMode() const { return m_mode; }

    /// Enable/disable hover highlighting
    void setHoverEnabled(bool enabled) { m_hoverEnabled = enabled; }

    /// Check if hover is enabled
    bool isHoverEnabled() const { return m_hoverEnabled; }

    //========================================================================
    // Picking Operations
    //========================================================================

    /// Perform a pick at the given screen coordinates
    PickResult pick(const Vec2f& screenPos);

    /// Add to multi-pick selection
    void addToSelection(const PickResult& result);

    /// Clear the current selection
    void clearSelection();

    /// Get the last single pick result
    const PickResult& getLastPick() const { return m_lastPick; }

    /// Get all picked handles (multi-pick)
    const std::vector<Handle>& getSelection() const { return m_selection; }

    /// Get picked position and normal from last pick
    bool getPickedPlane(Vec3f& position, Vec3f& normal) const;

    //========================================================================
    // Picked Point Axes
    //========================================================================

    /// Add a picked point axes handle
    void addPickedPointAxes(Handle axesHandle);

    /// Get all picked point axes
    const std::vector<Handle>& getPickedPointAxes() const { return m_pickedPointAxes; }

    /// Clear picked point axes
    void clearPickedPointAxes();

    //========================================================================
    // Hover State
    //========================================================================

    /// Update hover state for mouse move
    void updateHover(const Vec2f& screenPos);

    /// Get the currently hovered handle
    Handle getHoveredHandle() const { return m_hoveredHandle; }

    /// Clear hover state
    void clearHover();

    //========================================================================
    // Signals
    //========================================================================

    /// Signal emitted when an object is picked
    Signal<const PickResult&> onPicked;

    /// Signal emitted when hover state changes
    Signal<Handle> onHoverChanged;

    /// Signal emitted when selection changes
    Signal<const std::vector<Handle>&> onSelectionChanged;

private:
    void updatePickStrategy();
    void highlightHandle(Handle handle, bool highlight);

    IRenderBackend& m_backend;
    NodeRegistry& m_registry;

    IntersectorMode m_mode = IntersectorMode::Disabled;
    bool m_hoverEnabled = true;

    std::unique_ptr<IPickStrategy> m_strategy;

    PickResult m_lastPick;
    std::vector<Handle> m_selection;
    std::vector<Handle> m_pickedPointAxes;
    Handle m_hoveredHandle;

    // Colors for highlighting
    Color4f m_hoverColor{1.0f, 0.5f, 0.5f, 1.0f};
    Color4f m_selectColor{1.0f, 1.0f, 0.5f, 1.0f};
};

}  // namespace Vis


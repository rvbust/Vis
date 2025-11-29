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
#include "../Backend/IRenderBackend.h"

#include <atomic>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace Vis {

/**
 * @brief Manages the mapping between public handles and backend node IDs.
 * 
 * NodeRegistry provides thread-safe operations for registering and
 * looking up the relationship between user-facing handles and internal
 * backend identifiers.
 */
class NodeRegistry {
public:
    NodeRegistry() = default;
    ~NodeRegistry() = default;

    // Non-copyable
    NodeRegistry(const NodeRegistry&) = delete;
    NodeRegistry& operator=(const NodeRegistry&) = delete;

    /**
     * @brief Register a new handle with a backend node ID.
     * @param type The object type
     * @param backendId The backend's node ID
     * @return The newly created handle
     */
    Handle registerNode(ObjectType type, BackendNodeId backendId) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        uint64_t id = ++m_nextId;
        Handle handle(type, id);
        
        m_handleToBackend[handle] = backendId;
        m_backendToHandle[backendId] = handle;
        
        return handle;
    }

    /**
     * @brief Register a typed handle.
     */
    template<typename Tag>
    TypedHandle<Tag> registerTypedNode(BackendNodeId backendId) {
        Handle h = registerNode(Detail::TagToObjectType<Tag>::value, backendId);
        return TypedHandle<Tag>(h.id());
    }

    /**
     * @brief Unregister a handle.
     * @param handle The handle to unregister
     * @return true if the handle was found and removed
     */
    bool unregisterNode(Handle handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto it = m_handleToBackend.find(handle);
        if (it == m_handleToBackend.end()) {
            return false;
        }
        
        BackendNodeId backendId = it->second;
        m_handleToBackend.erase(it);
        m_backendToHandle.erase(backendId);
        m_colors.erase(handle);
        
        return true;
    }

    /**
     * @brief Get the backend node ID for a handle.
     * @param handle The handle to look up
     * @return The backend node ID, or nullopt if not found
     */
    std::optional<BackendNodeId> getBackendId(Handle handle) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto it = m_handleToBackend.find(handle);
        if (it != m_handleToBackend.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    /**
     * @brief Get the handle for a backend node ID.
     * @param backendId The backend node ID to look up
     * @return The handle, or nullopt if not found
     */
    std::optional<Handle> getHandle(BackendNodeId backendId) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto it = m_backendToHandle.find(backendId);
        if (it != m_backendToHandle.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    /**
     * @brief Check if a handle exists.
     */
    bool exists(Handle handle) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_handleToBackend.find(handle) != m_handleToBackend.end();
    }

    /**
     * @brief Get all handles of a specific type.
     */
    std::vector<Handle> getHandlesOfType(ObjectType type) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        std::vector<Handle> result;
        for (const auto& [handle, _] : m_handleToBackend) {
            if (handle.objectType() == type) {
                result.push_back(handle);
            }
        }
        return result;
    }

    /**
     * @brief Get all handles.
     */
    std::vector<Handle> getAllHandles() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        std::vector<Handle> result;
        result.reserve(m_handleToBackend.size());
        for (const auto& [handle, _] : m_handleToBackend) {
            result.push_back(handle);
        }
        return result;
    }

    /**
     * @brief Clear all registrations.
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_handleToBackend.clear();
        m_backendToHandle.clear();
        m_colors.clear();
    }

    /**
     * @brief Get the number of registered handles.
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_handleToBackend.size();
    }

    //========================================================================
    // Color storage (for preserving original colors)
    //========================================================================

    void setColor(Handle handle, const Color4f& color) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_colors[handle] = color;
    }

    std::optional<Color4f> getColor(Handle handle) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_colors.find(handle);
        if (it != m_colors.end()) {
            return it->second;
        }
        return std::nullopt;
    }

private:
    mutable std::mutex m_mutex;
    std::atomic<uint64_t> m_nextId{0};
    
    std::unordered_map<Handle, BackendNodeId, HandleHasher> m_handleToBackend;
    std::unordered_map<BackendNodeId, Handle> m_backendToHandle;
    std::unordered_map<Handle, Color4f, HandleHasher> m_colors;
};

}  // namespace Vis


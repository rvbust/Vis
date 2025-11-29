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

#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <atomic>

namespace Vis {

/**
 * @brief Connection handle for signal-slot connections.
 * 
 * ScopedConnection automatically disconnects when destroyed,
 * providing RAII-style connection management.
 */
class ScopedConnection {
public:
    ScopedConnection() = default;
    
    ScopedConnection(std::function<void()> disconnector)
        : m_disconnector(std::make_shared<std::function<void()>>(std::move(disconnector)))
        , m_connected(true) {}

    ScopedConnection(ScopedConnection&& other) noexcept
        : m_disconnector(std::move(other.m_disconnector))
        , m_connected(other.m_connected.load()) {
        other.m_connected = false;
    }

    ScopedConnection& operator=(ScopedConnection&& other) noexcept {
        if (this != &other) {
            disconnect();
            m_disconnector = std::move(other.m_disconnector);
            m_connected = other.m_connected.load();
            other.m_connected = false;
        }
        return *this;
    }

    // Non-copyable
    ScopedConnection(const ScopedConnection&) = delete;
    ScopedConnection& operator=(const ScopedConnection&) = delete;

    ~ScopedConnection() {
        disconnect();
    }

    /// Disconnect the slot from the signal
    void disconnect() {
        if (m_connected && m_disconnector && *m_disconnector) {
            (*m_disconnector)();
            m_connected = false;
        }
    }

    /// Check if the connection is still active
    bool connected() const { return m_connected; }

    /// Release ownership (connection will not be disconnected on destruction)
    void release() {
        m_disconnector.reset();
        m_connected = false;
    }

private:
    std::shared_ptr<std::function<void()>> m_disconnector;
    std::atomic<bool> m_connected{false};
};

/**
 * @brief A lightweight, thread-safe signal implementation.
 * 
 * Signal allows multiple slots (callbacks) to be connected and called
 * when the signal is emitted.
 * 
 * @tparam Args The argument types that will be passed to connected slots
 * 
 * Usage:
 * @code
 * Signal<int, const std::string&> mySignal;
 * 
 * // Connect a lambda
 * auto conn = mySignal.connect([](int i, const std::string& s) {
 *     std::cout << i << ": " << s << std::endl;
 * });
 * 
 * // Emit the signal
 * mySignal.emit(42, "Hello");
 * 
 * // Connection is automatically disconnected when 'conn' goes out of scope
 * @endcode
 */
template<typename... Args>
class Signal {
public:
    using SlotType = std::function<void(Args...)>;

    Signal() = default;
    
    // Non-copyable, non-movable (slots hold references to internal state)
    Signal(const Signal&) = delete;
    Signal& operator=(const Signal&) = delete;
    Signal(Signal&&) = delete;
    Signal& operator=(Signal&&) = delete;

    ~Signal() = default;

    /**
     * @brief Connect a slot to this signal.
     * @param slot The callback function to connect
     * @return A ScopedConnection that automatically disconnects when destroyed
     */
    [[nodiscard]] ScopedConnection connect(SlotType slot) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        uint64_t id = m_nextId++;
        m_slots.emplace_back(id, std::move(slot));

        // Create a weak reference to avoid preventing destruction
        std::weak_ptr<bool> weakAlive = m_alive;
        
        return ScopedConnection([this, id, weakAlive]() {
            if (auto alive = weakAlive.lock()) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_slots.erase(
                    std::remove_if(m_slots.begin(), m_slots.end(),
                        [id](const auto& pair) { return pair.first == id; }),
                    m_slots.end()
                );
            }
        });
    }

    /**
     * @brief Connect a slot without receiving a ScopedConnection.
     * The slot will remain connected until the signal is destroyed.
     * @param slot The callback function to connect
     */
    void connectForever(SlotType slot) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_slots.emplace_back(m_nextId++, std::move(slot));
    }

    /**
     * @brief Emit the signal, calling all connected slots.
     * @param args Arguments to pass to the slots
     */
    void emit(Args... args) {
        // Copy slots to avoid holding lock during calls
        std::vector<std::pair<uint64_t, SlotType>> slotsCopy;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            slotsCopy = m_slots;
        }

        for (const auto& [id, slot] : slotsCopy) {
            if (slot) {
                slot(args...);
            }
        }
    }

    /**
     * @brief Call operator for emitting the signal.
     * Equivalent to emit().
     */
    void operator()(Args... args) {
        emit(std::forward<Args>(args)...);
    }

    /**
     * @brief Disconnect all slots.
     */
    void disconnectAll() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_slots.clear();
    }

    /**
     * @brief Get the number of connected slots.
     */
    size_t slotCount() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_slots.size();
    }

    /**
     * @brief Check if any slots are connected.
     */
    bool hasConnections() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return !m_slots.empty();
    }

private:
    mutable std::mutex m_mutex;
    std::vector<std::pair<uint64_t, SlotType>> m_slots;
    uint64_t m_nextId = 0;
    std::shared_ptr<bool> m_alive = std::make_shared<bool>(true);
};

/**
 * @brief Helper class to collect multiple ScopedConnections.
 * 
 * All connections are automatically disconnected when the collector
 * is destroyed or when disconnectAll() is called.
 */
class ConnectionCollector {
public:
    ConnectionCollector() = default;
    ~ConnectionCollector() = default;

    // Movable
    ConnectionCollector(ConnectionCollector&&) = default;
    ConnectionCollector& operator=(ConnectionCollector&&) = default;

    // Non-copyable
    ConnectionCollector(const ConnectionCollector&) = delete;
    ConnectionCollector& operator=(const ConnectionCollector&) = delete;

    /// Add a connection to the collector
    void add(ScopedConnection&& conn) {
        m_connections.push_back(std::move(conn));
    }

    /// Operator += for convenient syntax
    ConnectionCollector& operator+=(ScopedConnection&& conn) {
        add(std::move(conn));
        return *this;
    }

    /// Disconnect all collected connections
    void disconnectAll() {
        m_connections.clear();
    }

    /// Get the number of connections
    size_t size() const { return m_connections.size(); }

private:
    std::vector<ScopedConnection> m_connections;
};

}  // namespace Vis


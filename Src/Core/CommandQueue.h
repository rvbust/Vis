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

#include "../Commands/ICommand.h"

#include <condition_variable>
#include <mutex>
#include <queue>
#include <chrono>
#include <optional>

namespace Vis {

/**
 * @brief Thread-safe command queue.
 * 
 * CommandQueue provides a way to pass commands between threads safely.
 * It supports blocking and non-blocking operations.
 */
class CommandQueue {
public:
    CommandQueue() = default;
    ~CommandQueue() = default;

    // Non-copyable, non-movable
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;
    CommandQueue(CommandQueue&&) = delete;
    CommandQueue& operator=(CommandQueue&&) = delete;

    /**
     * @brief Push a command onto the queue.
     * @param cmd The command to push
     */
    void push(CommandPtr cmd) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(std::move(cmd));
        }
        m_cv.notify_one();
    }

    /**
     * @brief Pop a command from the queue (blocking).
     * Waits until a command is available.
     * @return The popped command
     */
    CommandPtr pop() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this] { return !m_queue.empty() || m_shutdown; });
        
        if (m_shutdown && m_queue.empty()) {
            return nullptr;
        }
        
        CommandPtr cmd = std::move(m_queue.front());
        m_queue.pop();
        return cmd;
    }

    /**
     * @brief Try to pop a command (non-blocking).
     * @return The command if available, nullptr otherwise
     */
    CommandPtr tryPop() {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        if (m_queue.empty()) {
            return nullptr;
        }
        
        CommandPtr cmd = std::move(m_queue.front());
        m_queue.pop();
        return cmd;
    }

    /**
     * @brief Try to pop a command with timeout.
     * @param timeout Maximum time to wait
     * @return The command if available within timeout, nullptr otherwise
     */
    template<typename Rep, typename Period>
    CommandPtr tryPopFor(const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lock(m_mutex);
        
        if (!m_cv.wait_for(lock, timeout, [this] { return !m_queue.empty() || m_shutdown; })) {
            return nullptr;
        }
        
        if (m_shutdown && m_queue.empty()) {
            return nullptr;
        }
        
        CommandPtr cmd = std::move(m_queue.front());
        m_queue.pop();
        return cmd;
    }

    /**
     * @brief Check if the queue is empty.
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    /**
     * @brief Get the number of commands in the queue.
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    /**
     * @brief Clear all commands from the queue.
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::queue<CommandPtr> empty;
        std::swap(m_queue, empty);
    }

    /**
     * @brief Signal shutdown to unblock waiting threads.
     */
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_shutdown = true;
        }
        m_cv.notify_all();
    }

    /**
     * @brief Reset shutdown state.
     */
    void reset() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_shutdown = false;
    }

    /**
     * @brief Check if shutdown was signaled.
     */
    bool isShutdown() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_shutdown;
    }

private:
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::queue<CommandPtr> m_queue;
    bool m_shutdown = false;
};

}  // namespace Vis


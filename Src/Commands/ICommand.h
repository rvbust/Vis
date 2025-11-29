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

#include <future>
#include <memory>
#include <string>

namespace Vis {

// Forward declarations
class SceneManager;

/**
 * @brief Base interface for all commands.
 * 
 * Commands encapsulate operations that can be executed asynchronously
 * on the rendering thread.
 */
class ICommand {
public:
    virtual ~ICommand() = default;

    /// Execute the command on the given scene manager
    virtual void execute(SceneManager& scene) = 0;

    /// Get the command name (for debugging/logging)
    virtual std::string name() const = 0;

    /// Check if the command requires execution on the rendering thread
    virtual bool requiresRenderThread() const { return true; }
};

/**
 * @brief Command with a typed result.
 * 
 * This template class provides a way to get results back from commands
 * executed asynchronously.
 * 
 * @tparam ResultType The type of the result returned by the command
 */
template<typename ResultType>
class Command : public ICommand {
public:
    using Result = ResultType;

    Command() = default;
    virtual ~Command() = default;

    /// Get a future that will contain the result once the command is executed
    std::future<Result> getFuture() {
        return m_promise.get_future();
    }

protected:
    /// Set the result (called by subclasses after execution)
    void setResult(Result result) {
        m_promise.set_value(std::move(result));
    }

    /// Set an exception if the command fails
    void setException(std::exception_ptr ex) {
        m_promise.set_exception(ex);
    }

private:
    std::promise<Result> m_promise;
};

/**
 * @brief Specialization for void result (commands that don't return a value).
 */
template<>
class Command<void> : public ICommand {
public:
    using Result = void;

    Command() = default;
    virtual ~Command() = default;

    std::future<void> getFuture() {
        return m_promise.get_future();
    }

protected:
    void setResult() {
        m_promise.set_value();
    }

    void setException(std::exception_ptr ex) {
        m_promise.set_exception(ex);
    }

private:
    std::promise<void> m_promise;
};

/// Alias for a unique pointer to a command
using CommandPtr = std::unique_ptr<ICommand>;

}  // namespace Vis


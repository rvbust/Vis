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

#include "ICommand.h"
#include <Vis/Handle.h>
#include <Vis/Types.h>

#include <vector>

namespace Vis {

//============================================================================
// Node Operations
//============================================================================

/// Command to delete a node
class DeleteNodeCommand : public Command<bool> {
public:
    explicit DeleteNodeCommand(Handle handle) : m_handle(handle) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "DeleteNode"; }

private:
    Handle m_handle;
};

/// Command to delete multiple nodes
class DeleteNodesCommand : public Command<bool> {
public:
    explicit DeleteNodesCommand(std::vector<Handle> handles) 
        : m_handles(std::move(handles)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "DeleteNodes"; }

private:
    std::vector<Handle> m_handles;
};

/// Command to clear all nodes
class ClearNodesCommand : public Command<bool> {
public:
    void execute(SceneManager& scene) override;
    std::string name() const override { return "ClearNodes"; }
};

/// Command to show a node
class ShowNodeCommand : public Command<bool> {
public:
    explicit ShowNodeCommand(Handle handle) : m_handle(handle) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "ShowNode"; }

private:
    Handle m_handle;
};

/// Command to hide a node
class HideNodeCommand : public Command<bool> {
public:
    explicit HideNodeCommand(Handle handle) : m_handle(handle) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "HideNode"; }

private:
    Handle m_handle;
};

/// Command to check if a node exists
class HasNodeCommand : public Command<bool> {
public:
    explicit HasNodeCommand(Handle handle) : m_handle(handle) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "HasNode"; }

private:
    Handle m_handle;
};

/// Command to clone a node
class CloneNodeCommand : public Command<Handle> {
public:
    explicit CloneNodeCommand(Handle handle, 
                              Vec3f position = Vec3f::Zero(),
                              Quatf rotation = Quatf::Identity())
        : m_handle(handle), m_position(position), m_rotation(rotation) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CloneNode"; }

private:
    Handle m_handle;
    Vec3f m_position;
    Quatf m_rotation;
};

//============================================================================
// Transform Operations
//============================================================================

/// Command to set node position
class SetPositionCommand : public Command<bool> {
public:
    SetPositionCommand(Handle handle, Vec3f position)
        : m_handle(handle), m_position(position) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetPosition"; }

private:
    Handle m_handle;
    Vec3f m_position;
};

/// Command to set node rotation
class SetRotationCommand : public Command<bool> {
public:
    SetRotationCommand(Handle handle, Quatf rotation)
        : m_handle(handle), m_rotation(rotation) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetRotation"; }

private:
    Handle m_handle;
    Quatf m_rotation;
};

/// Command to set node transform
class SetTransformCommand : public Command<bool> {
public:
    SetTransformCommand(Handle handle, Vec3f position, Quatf rotation)
        : m_handle(handle), m_position(position), m_rotation(rotation) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetTransform"; }

private:
    Handle m_handle;
    Vec3f m_position;
    Quatf m_rotation;
};

/// Command to set multiple transforms
class SetTransformsCommand : public Command<bool> {
public:
    SetTransformsCommand(std::vector<Handle> handles,
                         std::vector<Vec3f> positions,
                         std::vector<Quatf> rotations)
        : m_handles(std::move(handles))
        , m_positions(std::move(positions))
        , m_rotations(std::move(rotations)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetTransforms"; }

private:
    std::vector<Handle> m_handles;
    std::vector<Vec3f> m_positions;
    std::vector<Quatf> m_rotations;
};

/// Command to get node transform
class GetTransformCommand : public Command<Transform> {
public:
    explicit GetTransformCommand(Handle handle) : m_handle(handle) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "GetTransform"; }

private:
    Handle m_handle;
};

//============================================================================
// Appearance Operations
//============================================================================

/// Command to set node color
class SetColorCommand : public Command<bool> {
public:
    SetColorCommand(Handle handle, Color4f color)
        : m_handle(handle), m_color(color) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetColor"; }

private:
    Handle m_handle;
    Color4f m_color;
};

/// Command to set node transparency
class SetTransparencyCommand : public Command<bool> {
public:
    SetTransparencyCommand(Handle handle, float alpha)
        : m_handle(handle), m_alpha(alpha) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "SetTransparency"; }

private:
    Handle m_handle;
    float m_alpha;
};

//============================================================================
// Hierarchy Operations
//============================================================================

/// Command to chain nodes together
class ChainNodesCommand : public Command<bool> {
public:
    explicit ChainNodesCommand(std::vector<Handle> handles)
        : m_handles(std::move(handles)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "ChainNodes"; }

private:
    std::vector<Handle> m_handles;
};

/// Command to unchain nodes
class UnchainNodesCommand : public Command<bool> {
public:
    explicit UnchainNodesCommand(std::vector<Handle> handles)
        : m_handles(std::move(handles)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "UnchainNodes"; }

private:
    std::vector<Handle> m_handles;
};

}  // namespace Vis


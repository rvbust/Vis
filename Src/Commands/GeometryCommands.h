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

#include <string>
#include <vector>

namespace Vis {

//============================================================================
// Command Data Structures
//============================================================================

/// Data for creating axes
struct CreateAxesData {
    Vec3f position{0, 0, 0};
    Quatf rotation = Quatf::Identity();
    float axisLength = 1.0f;
    float axisSize = 3.0f;
};

/// Data for creating points
struct CreatePointData {
    std::vector<float> positions;  // xyz triplets
    float pointSize = 1.0f;
    Color4f color = Color4f::Red();
    std::vector<float> colors;     // Optional per-point colors
};

/// Data for creating lines
struct CreateLineData {
    std::vector<float> vertices;   // xyz pairs (start, end)
    float lineWidth = 1.0f;
    Color4f color = Color4f::Red();
    std::vector<float> colors;     // Optional per-line colors
};

/// Data for creating a box
struct CreateBoxData {
    Vec3f center{0, 0, 0};
    Vec3f extents{0.5f, 0.5f, 0.5f};  // Half-extents
    Color4f color = Color4f::Red();
};

/// Data for creating a sphere
struct CreateSphereData {
    Vec3f center{0, 0, 0};
    float radius = 0.5f;
    Color4f color = Color4f::Red();
};

/// Data for creating multiple spheres
struct CreateSpheresData {
    std::vector<float> centers;    // xyz triplets
    std::vector<float> radii;
    std::vector<float> colors;     // Optional per-sphere colors (rgba)
};

/// Data for creating a cone
struct CreateConeData {
    Vec3f center{0, 0, 0};
    float radius = 0.5f;
    float height = 1.0f;
    Color4f color = Color4f::Red();
};

/// Data for creating a cylinder
struct CreateCylinderData {
    Vec3f center{0, 0, 0};
    float radius = 0.5f;
    float height = 1.0f;
    Color4f color = Color4f::Red();
};

/// Data for creating an arrow
struct CreateArrowData {
    std::vector<float> tails;      // xyz triplets for tail positions
    std::vector<float> heads;      // xyz triplets for head positions
    float radius = 0.1f;
    std::vector<float> colors;     // Optional per-arrow colors
};

/// Data for creating a mesh
struct CreateMeshData {
    std::vector<float> vertices;           // xyz triplets
    std::vector<unsigned int> indices;     // Triangle indices
    std::vector<float> colors;             // Optional per-vertex colors
    std::vector<float> normals;            // Optional normals
};

/// Data for creating a plane
struct CreatePlaneData {
    float xLength = 2.0f;
    float yLength = 2.0f;
    int halfXCells = 8;
    int halfYCells = 8;
    Vec3f position{0, 0, 0};
    Quatf rotation = Quatf::Identity();
    Color4f color = Color4f::Gray();
};

/// Data for loading a model
struct LoadModelData {
    std::string filepath;
    Vec3f position{0, 0, 0};
    Quatf rotation = Quatf::Identity();
};

/// Data for loading multiple models
struct LoadModelsData {
    std::vector<std::string> filepaths;
    std::vector<Vec3f> positions;
    std::vector<Quatf> rotations;
};

//============================================================================
// Geometry Commands
//============================================================================

/// Command to create axes
class CreateAxesCommand : public Command<AxesHandle> {
public:
    explicit CreateAxesCommand(CreateAxesData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateAxes"; }

    const CreateAxesData& data() const { return m_data; }

private:
    CreateAxesData m_data;
};

/// Command to create multiple axes
class CreateMultipleAxesCommand : public Command<std::vector<AxesHandle>> {
public:
    explicit CreateMultipleAxesCommand(std::vector<CreateAxesData> data) 
        : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateMultipleAxes"; }

private:
    std::vector<CreateAxesData> m_data;
};

/// Command to create points
class CreatePointCommand : public Command<PointHandle> {
public:
    explicit CreatePointCommand(CreatePointData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreatePoint"; }

private:
    CreatePointData m_data;
};

/// Command to create lines
class CreateLineCommand : public Command<LineHandle> {
public:
    explicit CreateLineCommand(CreateLineData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateLine"; }

private:
    CreateLineData m_data;
};

/// Command to create a box
class CreateBoxCommand : public Command<BoxHandle> {
public:
    explicit CreateBoxCommand(CreateBoxData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateBox"; }

private:
    CreateBoxData m_data;
};

/// Command to create a sphere
class CreateSphereCommand : public Command<SphereHandle> {
public:
    explicit CreateSphereCommand(CreateSphereData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateSphere"; }

private:
    CreateSphereData m_data;
};

/// Command to create a cone
class CreateConeCommand : public Command<ConeHandle> {
public:
    explicit CreateConeCommand(CreateConeData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateCone"; }

private:
    CreateConeData m_data;
};

/// Command to create a cylinder
class CreateCylinderCommand : public Command<CylinderHandle> {
public:
    explicit CreateCylinderCommand(CreateCylinderData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateCylinder"; }

private:
    CreateCylinderData m_data;
};

/// Command to create an arrow
class CreateArrowCommand : public Command<ArrowHandle> {
public:
    explicit CreateArrowCommand(CreateArrowData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateArrow"; }

private:
    CreateArrowData m_data;
};

/// Command to create a mesh
class CreateMeshCommand : public Command<MeshHandle> {
public:
    explicit CreateMeshCommand(CreateMeshData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreateMesh"; }

private:
    CreateMeshData m_data;
};

/// Command to create a plane
class CreatePlaneCommand : public Command<PlaneHandle> {
public:
    explicit CreatePlaneCommand(CreatePlaneData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "CreatePlane"; }

private:
    CreatePlaneData m_data;
};

/// Command to load a model
class LoadModelCommand : public Command<ModelHandle> {
public:
    explicit LoadModelCommand(LoadModelData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "LoadModel"; }

private:
    LoadModelData m_data;
};

/// Command to load multiple models
class LoadModelsCommand : public Command<std::vector<ModelHandle>> {
public:
    explicit LoadModelsCommand(LoadModelsData data) : m_data(std::move(data)) {}

    void execute(SceneManager& scene) override;
    std::string name() const override { return "LoadModels"; }

private:
    LoadModelsData m_data;
};

}  // namespace Vis


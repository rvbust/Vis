/***********************************************************************
 **
 ** Copyright (c) 2012-2021 RVBUST Inc.
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

#include <Vis/Vis.h>
#include <fmt/format.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace Vis;

PYBIND11_MODULE(PyVis, m) {
    namespace py = pybind11;
    using namespace py::literals;

    m.def("GetVersion", &GetVersion);
    m.def("SetLogLevel", &SetLogLevel, "levelname"_a, "debug/info/warn/error/off");

    py::enum_<IntersectorMode>(m, "IntersectorMode")
        .value("IntersectorMode_Disable", IntersectorMode::IntersectorMode_Disable)
        .value("IntersectorMode_Polytope", IntersectorMode::IntersectorMode_Polytope)
        .value("IntersectorMode_LineSegment", IntersectorMode::IntersectorMode_LineSegment)
        .value("IntersectorMode_Point", IntersectorMode::IntersectorMode_Point)
        .value("IntersectorMode_Line", IntersectorMode::IntersectorMode_Line)
        .export_values();

    py::class_<Handle>(m, "Handle")
        .def(py::init<>())
        .def(py::init<uint64_t, uint64_t>(), "type"_a, "uid"_a)
        .def("Copy", [](const Handle &self) { return Handle(self.type, self.uid); })
        .def_readwrite("type", &Handle::type)
        .def_readwrite("uid", &Handle::uid)
        .def("__repr__",
             [](const Handle &h) { return fmt::format("(type:{0}, uid:{1})", h.type, h.uid); });

    py::class_<ViewConfig>(m, "ViewConfig")
        .def(py::init<>())
        .def(
            py::init<const std::string &, int, int, int, int, const std::array<float, 4> &, bool>(),
            "name"_a, "x"_a, "y"_a, "width"_a, "height"_a, "bgcolor"_a, "use_decoration"_a = true)
        .def("Copy",
             [](const ViewConfig &self) {
                 return ViewConfig(self.name, self.x, self.y, self.width, self.height, self.bgcolor,
                                   self.use_decoration);
             })
        .def_readwrite("name", &ViewConfig::name)
        .def_readwrite("x", &ViewConfig::x)
        .def_readwrite("y", &ViewConfig::y)
        .def_readwrite("width", &ViewConfig::width)
        .def_readwrite("height", &ViewConfig::height)
        .def_readwrite("bgcolor", &ViewConfig::bgcolor)
        .def_readwrite("use_decoration", &ViewConfig::use_decoration)
        .def_readwrite("screen_num", &ViewConfig::screen_num)
        .def("__repr__", [](const ViewConfig &self) {
            return fmt::format(
                "name:{0}, x:{1}, y:{2}, width:{3}, height:{4}, bgcolor:({6},{7},{8},{9}), "
                "use_decoration:{5}, screen_num: {10}",
                self.name, self.x, self.y, self.width, self.height, self.use_decoration,
                self.bgcolor[0], self.bgcolor[1], self.bgcolor[2], self.bgcolor[3],
                self.screen_num);
        });

    py::class_<View>(m, "View")
        .def(py::init<const std::string &, bool>(), "name"_a = "Vis", "shared"_a = true)
        .def(py::init<const ViewConfig &, bool>(), "cfg"_a, "shared"_a = true)
        .def("GetViewSize",
             [](View &v) {
                 int width = 0, height = 0;
                 v.GetViewSize(width, height);
                 return std::make_tuple(width, height);
             },
             "Get View window size.")
        .def("WindowSetDecoration", &View::WindowSetDecoration, "enable"_a)
        .def("WindowSetRectangle", &View::WindowSetRectangle, "x"_a, "y"_a, "width"_a, "height"_a)
        .def("WindowGetRectangle",
             [](View &v) {
                 std::array<int, 4> r;
                 v.WindowGetRectangle(r[0], r[1], r[2], r[3]);
                 return r;
             })
        .def("WindowRaise", &View::WindowRaise, "Raise the window to the top")
        .def("WindowHide", &View::WindowHide, "Hide the window.Return the rectangle of the window.")
        .def("WindowShow", &View::WindowShow, "rectangle"_a,
             "Show the at certain position with certain size.")

        .def("SetIntersectorMode", &View::SetIntersectorMode,
             "Set intersector mode for picking in scene\n"
             "\t-IntersectorMode_Disable, Disable picking mode\n"
             "\t-IntersectorMode_Polytope, use PolytopeIntersector to pick object (including "
             "point, line, etc.), cann't calculate intersection point\n"
             "\t-IntersectorMode_LineSegment, use LineSegmentIntersector to pick a node (except "
             "line,point)\n"
             "\t-IntersectorMode_Line, use LineIntersector (derived from LineSegmentIntersector) "
             "to pick a line (drawn with viewer.Line(...)), could be replaced with "
             "IntersectorMode_Polytope\n"
             "\t-IntersectorMode_Point,  use PointIntersector (derived from LinetIntersector) to "
             "pick a point, suitable for large scale point cloud data\n",
             "mode"_a, "hover"_a = true)
        .def("GetIntersectorMode", &View::GetIntersectorMode,
             "Get intersector mode for picking in scene.")
        .def("GetPickedPointAxes", &View::GetPickedPointAxes, "Get picked point axes handles.")
        .def("ClearPickedPointAxes", &View::ClearPickedPointAxes,
             "Clear picked point axes handles.")
        .def("MultiPicked", &View::MultiPicked,
             "Retieve the multiple picked object's handle, holding Ctrl when picking mode is on.")
        .def("Picked", &View::Picked, "Retieve the latest picked object's handle.")
        .def("PickedPlane", [](View &self) { return self.PickedPlane(); },
             "Retrieve the latest picked position and normal.")
        .def("Load", [](View &v, const std::string &name) { return v.Load(name); }, "fname"_a,
             "Load a model from a file.")
        .def("Load", [](View &v, const std::vector<std::string> &names) { return v.Load(names); },
             "fnames"_a, "Load models from many files.")
        .def("Load",
             [](View &v, const std::string &name, const std::array<float, 3> &pos,
                const std::array<float, 4> &quat) { return v.Load(name, pos, quat); },
             "fname"_a, "pos"_a, "quat"_a, "Load model with a initial transform.")
        .def("Load",
             [](View &v, const std::vector<std::string> &names,
                const std::vector<std::array<float, 3>> &trans,
                const std::vector<std::array<float, 4>> &quats) {
                 return v.Load(names, trans, quats);
             },
             "fnames"_a, "trans"_a, "quats"_a, "Load models with initial transforms.")
        .def("Home", &View::Home, "Go to home position.")
        .def("SetHomePose", &View::SetHomePose, "eye"_a, "point_want_to_look"_a, "upvector"_a,
             "Make the current view camera to look at some point align to an up direction.")
        .def("GetHomePose",
             [](View &self) {
                 std::array<float, 3> eye, point, up;
                 const bool ret = self.GetHomePose(eye, point, up);
                 return std::make_tuple(ret, eye, point, up);
             },
             "Get The view camera's home pose.")
        .def("SetCameraPose", &View::SetCameraPose, "eye"_a, "point_want_to_look"_a, "upvector"_a,
             "Set the current pose of the camera.")
        .def("GetCameraPose",
             [](View &self) {
                 std::array<float, 3> eye, point, up;
                 const bool ret = self.GetCameraPose(eye, point, up);
                 return std::make_tuple(ret, eye, point, up);
             },
             "Get the current pose of the camera.")
        .def("Show", &View::Show, "handle"_a, "Show an object")
        .def("Hide", &View::Hide, "handle"_a, "Hide an object")
        .def("Chain", &View::Chain, "handles"_a, "Chain objects")
        .def("Unchain", &View::Unchain, "handles"_a, "Unchain objects")
        .def("SetTransparency", &View::SetTransparency, "handle"_a, "inv_alpha"_a,
             "Set transparency of an object with inv_alpha value.")
        .def("SetColor", &View::SetColor, "handle"_a, "color"_a, "Set color to an object")
        .def("GetColor", &View::GetColor, "handle"_a,
             "Get color of an object, maybe empty list if there is no color information!")
        .def("Close", &View::Close, "Close window.")
        .def("Delete", py::overload_cast<const Handle &>(&View::Delete), "handle"_a,
             "Delete a created node from the scene.")
        .def("IsClosed", &View::IsClosed, "Check if the view is closed or not.")
        .def("IsAlive", &View::IsAlive, "handle"_a,
             "Check if an object is still alive in the scene.")
        .def("Delete", py::overload_cast<const std::vector<Handle> &>(&View::Delete), "handles"_a,
             "Delete a created node from the scene.")
        .def("Clear", &View::Clear, "Clear the scene")
        .def("Clone", [](View &self, const Handle h) { return self.Clone(h); }, "handle"_a,
             "Clone an object")
        .def("Clone",
             [](View &self, const std::vector<Handle> &handles) { return self.Clone(handles); },
             "Clone list of unchained object")
        .def("Clone",
             [](View &self, const Handle h, const std::array<float, 3> &pos,
                const std::array<float, 4> &quat) { return self.Clone(h, pos, quat); },
             "handle"_a, "pos"_a, "quat"_a, "Clone an object with pos and quat")
        .def("Clone",
             [](View &self, const std::vector<Handle> &handles,
                const std::vector<std::array<float, 3>> &poss,
                const std::vector<std::array<float, 4>> &quats) {
                 return self.Clone(handles, poss, quats);
             },
             "handles"_a, "poss"_a, "quats"_a, "Clone list of unchained object with poss and quats")
        .def("SetText", &View::SetText, "handle"_a, "content"_a, "pos"_a, "font_size"_a = 0.02f,
             "colors"_a = std::vector<float>{0.f, 0.f, 0.f}, "Plot Text.")
        .def("SetTextFont", &View::SetTextFont, "font_name"_a, "Set Text Font.")
        .def("Quad2D", &View::Quad2D, "xys"_a, "colors"_a = std::vector<float>{1.f, 0, 0},
             "mode"_a = 0, "Plot a 2D quad with vertex in secreen position.")
        .def(
            "Text", &View::Text, "content"_a, "pos"_a, "font_size"_a=0.02f,
            "colors"_a = std::vector<float>{0.f, 0.f, 0.f},
            "Plot 3d or 2d text.")
        .def(
            "Point", &View::Point, "xyzs"_a, "ptsize"_a = 1.0f,
            "colors"_a = std::vector<float>{1.f, 0, 0},
            "Plot 3d point or points. Every 3 values is point position.The unit of size is pixels.")
        .def(
            "Point",
            [](View &v, py::array_t<float> xyzs, float ptsize, const std::vector<float> &colors) {
                auto r = xyzs.unchecked<2>();
                std::vector<float> xyzs_vec;
                for (py::ssize_t i = 0; i < r.shape(0); i++)
                    for (py::ssize_t j = 0; j < r.shape(1); j++) xyzs_vec.push_back(r(i, j));
                return v.Point(xyzs_vec, ptsize, colors);
            },
            "xyzs"_a, "ptsize"_a = 1.0f, "colors"_a = std::vector<float>{1.f, 0, 0},
            "Plot 3d point or points. Every 3 values is point position.The unit of size is pixels.")
        .def("Line", &View::Line, "lines"_a, "size"_a = 1.0f,
             "colors"_a = std::vector<float>{1.f, 0, 0},
             "Plot line or lines. lines.size() = line_num * 6. Every 6 values is two end points of "
             "a line.")
        .def("Axes",
             py::overload_cast<const std::array<float, 3> &, const std::array<float, 4> &, float,
                               float>(&View::Axes),
             "pos"_a = std::array<float, 3>{0, 0, 0}, "quat"_a = std::array<float, 4>{0, 0, 0, 1},
             "axis_len"_a = 15.f, "axis_size"_a = 3.f,
             "Plot a Axes with translations and quaternions and set asis len and axis size. The "
             "unit of axis length is meters, and the unit of dimension is pixels.")
        .def(
            "Axes",
            py::overload_cast<const std::vector<std::array<float, 3>> &,
                              const std::vector<std::array<float, 4>> &, float, float>(&View::Axes),
            "translations"_a, "quaternions"_a, "axis_len"_a = 15.f, "axis_size"_a = 3.f,
            "Plot many axes.")
        .def("Axes", py::overload_cast<const std::array<float, 16> &, float, float>(&View::Axes),
             "transform"_a, "axis_len"_a = 15.f, "axis_size"_a = 3.f,
             "Plot a Axes with translations and quaternions and set asis len and axis size. The "
             "unit of axis length is meters, and the unit of dimension is pixels.")
        .def("Axes",
             [](View &v, py::array_t<float> transform, float axis_len, float axis_size) {
                 auto r = transform.unchecked<2>();
                 std::array<float, 16> transform_array;
                 for (py::ssize_t i = 0; i < r.shape(0); i++)
                     for (py::ssize_t j = 0; j < r.shape(1); j++)
                         transform_array[4 * i + j] = r(i, j);
                 return v.Axes(transform_array, axis_len, axis_size);
             },
             "transform"_a, "axis_len"_a = 15.f, "axis_size"_a = 3.f,
             "Plot a Axes with translations and quaternions and set asis len and axis size. The "
             "unit of axis length is meters, and the unit of dimension is pixels.")
        .def("Box", &View::Box, "pos"_a, "extents"_a, "colors"_a = std::vector<float>{1.f, 0, 0},
             "Plot a box with center position and extents, the extents are half the length, width "
             "and height.")
        .def("Cylinder", &View::Cylinder, "center"_a, "radius"_a, "height"_a,
             "colors"_a = std::vector<float>{1.f, 0, 0},
             "Plot a cylinder with center position, radius and height.")
        .def("Sphere", &View::Sphere, "center"_a, "radius"_a,
             "color"_a = std::vector<float>{1.f, 0, 0, 1.f},
             "Plot a sphere with center position and radius.")
        .def("Spheres", &View::Spheres, "centers"_a, "radii"_a,
             "colors"_a = std::vector<float>{1.f, 0, 0, 1.f},
             "Plot a sphere with center position and radius.")
        .def("Cone", &View::Cone, "center"_a, "radius"_a, "height"_a,
             "colors"_a = std::vector<float>{1.f, 0, 0},
             "Plot a cone with center, radius and hight.")
        .def("Arrow", &View::Arrow, "tail"_a, "head"_a, "radius"_a,
             "colors"_a = std::vector<float>{1.f, 0, 0},
             "Plot a arrow with tail position, head position and radius.")
        .def("Plane",
             py::overload_cast<float, float, int, int, const std::vector<float> &>(&View::Plane),
             "xlength"_a, "ylength"_a, "half_x_num_cells"_a = 8, "half_y_num_cells"_a = 8,
             "color"_a = std::vector<float>{1.f, 0, 0}, "Plot plane.")
        .def("Plane",
             py::overload_cast<float, float, int, int, const std::array<float, 3> &,
                               const std::array<float, 4> &, const std::vector<float> &>(
                 &View::Plane),
             "xlength"_a, "ylength"_a, "half_x_num_cells"_a, "half_y_num_cells"_a, "trans"_a,
             "quat"_a, "color"_a = std::vector<float>{1.f, 0, 0}, "Plot plane.")
        .def("Ground", &View::Ground, "halfcells"_a, "cellsize"_a, "color"_a, "Draw a ground plane")
        .def("Mesh", &View::Mesh, "vertices"_a, "indices"_a,
             "colors"_a = std::vector<float>{1.f, 0, 0}, "Plot Mesh.")
        .def("SetPosition", &View::SetPosition, "handle"_a, "pos"_a, "Set position of an object.")
        .def("SetRotation", &View::SetRotation, "handle"_a, "quat"_a, "Set rotatoin of an object.")
        .def("SetTransform", &View::SetTransform, "handle"_a, "pos"_a, "quat"_a,
             "Set Transform of an object.")
        .def("SetTransforms",
             py::overload_cast<const std::vector<Handle> &,
                               const std::vector<std::array<float, 3>> &,
                               const std::vector<std::array<float, 4>> &>(&View::SetTransforms),
             "handles"_a, "trans"_a, "quats"_a, "Set transforms of objects.")
        .def("SetTransforms",
             py::overload_cast<const Handle &, const std::vector<std::array<float, 3>> &,
                               const std::vector<std::array<float, 4>> &>(&View::SetTransforms),
             "handle"_a, "trans"_a, "quats"_a, "Set transform of an object.")

        .def("GetTransform",
             [](View &v, const Handle &nh) {
                 std::array<float, 4> quat;
                 std::array<float, 3> trans;
                 const bool ret = v.GetTransform(nh, trans, quat);
                 return std::make_tuple(ret, trans, quat);
             },
             "Get transform of an object, return [res, trans, quat]")
        .def("GetPosition",
             [](View &v, const Handle &nh) {
                 std::array<float, 3> pos;
                 const bool ret = v.GetPosition(nh, pos);
                 return std::make_tuple(ret, pos);
             },
             "Get position of an object, return [res, pos].")
        .def("GetRotation",
             [](View &v, const Handle &nh) {
                 std::array<float, 4> quat;
                 const bool ret = v.GetRotation(nh, quat);
                 return std::make_tuple(ret, quat);
             },
             "Get rotation of an object, return quat(x,y,z,w).")
        .def("SetObjectAnimation", &View::SetObjectAnimation, "handle"_a, "enable"_a,
             "time"_a = 6.0f, "loop"_a = 1, "pos"_a = std::vector<float>{},
             "quat"_a = std::vector<float>{}, "Set the animation path of an object.")
        .def("SetCameraAnimation",
             py::overload_cast<bool, float, int, const std::vector<float> &,
                               const std::vector<float> &>(&View::SetCameraAnimation),
             "enable"_a, "time"_a = 6.0f, "loop"_a = 1, "pos"_a = std::vector<float>{},
             "quat"_a = std::vector<float>{}, "Set the animation path of an object.")
        .def("SetCameraAnimation",
             py::overload_cast<bool, float, int, const std::vector<float> &,
                               const std::vector<float> &, const std::vector<float> &>(
                 &View::SetCameraAnimation),
             "enable"_a, "time"_a, "loop"_a, "eye"_a, "look"_a,
             "up"_a
             "Set the animation path of an object.")
        .def("EnableGizmo", &View::EnableGizmo, "handle"_a, "gizmotype"_a, "Enable Gizmo")
        .def("DisableGizmo", &View::DisableGizmo, "Disable Gizmo")
        .def("SetGizmoType", &View::SetGizmoType, "gizmotype"_a, "Set Gizmo Type")
        .def("SetGizmoDrawMask", &View::SetGizmoDrawMask, "gizmotype"_a, "mask"_a,
             "Set Gizmo DrawMask")
        .def("SetGizmoDisplayScale", &View::SetGizmoDisplayScale,
             "scale"_a
             "Set Gizmo Display Scale")
        .def("SetGizmoDetectionRange", &View::SetGizmoDetectionRange, "range"_a,
             "Set Gizmo Detection Range");
}

/***********************************************************************
 **
 ** Copyright (c) 2012-2021 Scott Chacon and others
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

#include <stdint.h>
#include <array>
#include <list>
#include <memory>
#include <string>
#include <vector>

namespace Vis {

struct Handle {
    Handle() : type(0), uid(0) {}
    Handle(uint64_t t, uint64_t u) : type(t), uid(u) {}
    inline void Reset() {
        type = 0;
        uid = 0;
    }
    uint64_t type;
    uint64_t uid;
};

inline bool operator==(const Handle a, const Handle b) {
    return a.type == b.type && a.uid == b.uid;
}

struct HandleHasher {
    inline std::size_t operator()(const Handle h) const {
        return std::hash<decltype(h.type)>()(h.type) ^ std::hash<decltype(h.uid)>()(h.uid);
    }
};

void SetLogLevel(const std::string &levelname);
const char *GetVersion();

struct ViewConfig {
    ViewConfig()
        : name("Vis3d")
        , x(0)
        , y(0)
        , width(800)
        , height(600)
        , bgcolor({1.0, 1.0, 1.0, 1.0})
        , use_decoration(true) {
        screen_num = -1;
    }

    ViewConfig(const std::string &name_, int x_, int y_, int width_, int height_,
               const std::array<float, 4> &bgcolor_, bool use_decoration_)
        : name(name_)
        , x(x_)
        , y(y_)
        , width(width_)
        , height(height_)
        , bgcolor(bgcolor_)
        , use_decoration(use_decoration_) {
        screen_num = -1;
    }

    std::string name;
    int x;
    int y;
    int width;
    int height;
    std::array<float, 4> bgcolor;
    bool use_decoration;
    int screen_num;  // FIXME(Hui): screen_num is not working well
};

// clang-format off
enum IntersectorMode {
    IntersectorMode_Disable = 0,  // Disable picking mode
    IntersectorMode_Polytope,     // used to pick object (including point, line, etc.), cann't calculate intersection point
    IntersectorMode_LineSegment,  // used to pick a point and object in scene, hard to pick point, line
    IntersectorMode_Point,        // used to pick a point from a point cloud (drawn with viewer.Point(...))
    IntersectorMode_Line,         // used to pick a line node in scene (drawn with viewer.Line(...))
};
// clang-format on

struct View {
    /**
     * Create a view window to the Scene.
     * @param viewname the name displayed on the window.
     * @param shared to control share the same scene or not.
     */
    explicit View(const std::string &viewname, bool shared = true);

    /**
     * @brief Create a view window by using a config
     *
     * @param cfg ViewConfig instance
     */

    explicit View(const ViewConfig &cfg, bool shared = true);

    /**
     * This function will simply call Close function.
     */
    virtual ~View();

    /**
     * Enable of disable the decoration of window.
     */
    void WindowSetDecoration(bool enable);

    /**
     * Set the window position and size on screen
     */
    void WindowSetRectangle(int x, int y, int width, int height);

    /**
     * Get the window position and size on screen
     */
    void WindowGetRectangle(int &x, int &y, int &width, int &height);

    /**
     * Raiw the window to the top
     */
    void WindowRaise();

    /**
     * Hide the window.Return the rectangle of the window.
     * @NOTE: maintaining the rect of the window will make the implementation complicated.
     * return it could reduce the number of states we need to track.
     */
    std::array<int, 4> WindowHide();

    /**
     * Show the at certain position with certain size.
     * @code
     *   r = v.WindowHide();
     *   v.WindowShow(r);
     * @code
     */
    void WindowShow(const std::array<int, 4> &r);

    /**
     * Close the window.
     * Return true if succeed else false.
     */
    bool Close();  // Close the View

    /**
     * @brief Set/Get camera's pose by using eye/center/up vectors
     *
     * @return bool True if succeed, else False
     */
    bool SetCameraPose(const std::array<float, 3> &eye,
                       const std::array<float, 3> &point_want_to_look,
                       const std::array<float, 3> &upvector);
    bool GetCameraPose(std::array<float, 3> &eye, std::array<float, 3> &point_want_to_look,
                       std::array<float, 3> &upvector);

    /**
     * @brief Set/Get The view camera's home pose.
     * The home pose is the default pose when you press space key.
     *
     * @return bool True if succeed, else False
     */
    bool SetHomePose(const std::array<float, 3> &eye,
                     const std::array<float, 3> &point_want_to_look,
                     const std::array<float, 3> &upvector);
    bool GetHomePose(std::array<float, 3> &eye, std::array<float, 3> &point_want_to_look,
                     std::array<float, 3> &upvector);

    /**
     * Delete an node from the scene.
     * Return true if succeed else false.
     */
    bool Delete(const Handle &nh);  // Delete node

    bool IsClosed() const;

    bool IsAlive(const Handle &nh) const;

    /**
     * Delete nodes from the scene
     * Return true if all deleted else false and top at the break point.
     * NOTE: This function simply calls Delet(handle), so it will be slow since
     * it's not operating in batch mode.
     */
    bool Delete(const std::vector<Handle> &handles);  // delete list of handles

    /**
     * Clear all nodes in the scene.
     */
    bool Clear();  // Delete all nodes

    /**
     * Go to home position
     */
    bool Home();

    /**
     * Switch on the node to make it rendered and visible.
     */
    bool Show(const Handle &nh);

    /**
     * Switch off the node to make it invisible.
     */
    bool Hide(const Handle &nh);

    /**
     * Chain a list of links together: links[0] -> links[1] ...
     * The first link will be the parent of the second link.
     * All children links (start from the second) will be removed from their parents first, then
     * link together. Note: the same links should not be passed into this function, since it will
     * form a circle, which will break the graph. Return true of succeed else false.
     */
    bool Chain(const std::vector<Handle> &links);

    /**
     * Unchain a set of linked objects.
     */
    bool Unchain(const std::vector<Handle> &links);

    /**
     * Set transforms for each object indepedently. If there're links between objects, the
     * transformation will be accumulated.
     */
    bool SetTransforms(const std::vector<Handle> &hs,
                       const std::vector<std::array<float, 3>> &trans,
                       const std::vector<std::array<float, 4>> &quats);

    /**
     * Set the transformations from a base node and it's decendents. In this case, each parent
     * should only have single decedent.
     */
    bool SetTransforms(const Handle &h, const std::vector<std::array<float, 3>> &trans,
                       const std::vector<std::array<float, 4>> &quats);

    /**
     * Set the transparency level of an object.
     * @param inv_alpha is 1 - alpha.
     */
    bool SetTransparency(const Handle &nh, float inv_alpha);

    // bool SetName(const Handle &nh, const std::string &name);

    /**
     * Clone a node by handle. Return a new to handle to the cloned object.
     */
    Handle Clone(const Handle h);

    Handle Clone(const Handle h, const std::array<float, 3> &pos, const std::array<float, 4> &quat);

    // NOTE: the input handles should not be chained together !!
    std::vector<Handle> Clone(const std::vector<Handle> &handles);

    std::vector<Handle> Clone(const std::vector<Handle> &handles,
                              const std::vector<std::array<float, 3>> &poss,
                              const std::vector<std::array<float, 4>> &quats);

    Handle Axes(const std::array<float, 3> &translations = {0, 0, 0},
                const std::array<float, 4> &quaternions = {0, 0, 0, 1}, float axis_len = 15.f,
                float axis_size = 3.f);
    Handle Axes(const std::array<float, 16> &transform, float axis_len = 15.f,
                float axis_size = 3.f);
    std::vector<Handle> Axes(const std::vector<std::array<float, 3>> &trans,
                             const std::vector<std::array<float, 4>> &quat, float axis_len = 15.f,
                             float axis_size = 3.f);
    std::vector<Handle> Axes(const std::vector<std::array<float, 16>> &transform,
                             float axis_len = 15.f, float axis_size = 3.f);

    bool GetViewSize(int &width, int &height);

    /**
     * @brief Get the Picked point handles
     *
     * @return std::vector<Handle>&
     */
    std::vector<Handle> &GetPickedPointAxes();

    void ClearPickedPointAxes();

    /**
     * SetIntersectorMode
     *
     * Set intersector mode for picking in scene.
     *
     * @code
     * v.SetIntersectorMode(IntersectorMode_LineSegment, false);
     * @endcode
     * @param mode IntersectorMode:
     * IntersectorMode_Disable, Disable picking mode
     * IntersectorMode_Polytope, use PolytopeIntersector to pick object (including point, line,
     * etc.), cann't calculate intersection point IntersectorMode_LineSegment, use
     * LineSegmentIntersector to pick a node (except line,point) IntersectorMode_Line, use
     * LineIntersector (derived from LineSegmentIntersector) to pick a line (drawn with
     * viewer.Line(...)), could be replaced with IntersectorMode_Polytope IntersectorMode_Point, use
     * PointIntersector (derived from LinetIntersector) to pick a point, suitable for large scale
     * point cloud data
     * @param hover intersect object while mouse move when hover is true
     */
    void SetIntersectorMode(enum IntersectorMode mode, bool hover = true);

    enum IntersectorMode GetIntersectorMode() const;

    /**
     * @brief  Return the latest picked object's handles.
     * @note   Use Ctrl + left click to pick multiple object.
     * @retval list of Handle
     */
    std::vector<Handle> MultiPicked() const;

    /**
     * Return the latest picked object's handle.
     */
    Handle Picked();

    /**
     * Return the latest picked position and normal
     */
    std::array<float, 6> PickedPlane();

    /**
     * Load model
     */
    Handle Load(const std::string &fname);
    std::vector<Handle> Load(const std::vector<std::string> &fnames);

    Handle Load(const std::string &fname, const std::array<float, 3> &pos,
                const std::array<float, 4> &quat);
    std::vector<Handle> Load(const std::vector<std::string> &fnames,
                             const std::vector<std::array<float, 3>> &trans,
                             const std::vector<std::array<float, 4>> &quats);

    /**
     * @brief Set 2D Text on Screen
     *
     * @param h if h is empty, set a new 2D Text and set the handle.
     * if h is not empty, refesh the 2D text with given param.
     * @param content 2D text content
     * @param pos  2D text position
     * @param font_size 2D text font size
     * @param colors 2D text color
     * @return true
     * @return false
     */
    bool SetText(Handle &h, const std::string &content, const std::vector<float> &pos,
                 float font_size = 0.0f, const std::vector<float> &colors = {});

    /**
     * @brief Set 2D Text Font
     *
     * @param font_name path to find ttf file
     * @return true Set Font file successfuly
     * @return false Using default font
     */
    bool SetTextFont(const std::string &font_name);

    /**
     * @brief Set 2D Quad on Screen
     *
     * * @code
     * bool b = v.Quad2D({100, 100, 200, 200});
     *
     * @param xys Each vertex pixel of the 2D Quad
     * @param color color of this Quad
     * @param mode 0 for QUADS 1 for QUAD_STRIP
     * @return Handle
     */
    Handle Quad2D(const std::vector<float> &xys, const std::vector<float> &color = {1.0f, 0, 0},
                  int mode = 0);
    Handle Point(const std::vector<float> &xyzs, float ptsize = 1.0f,
                 const std::vector<float> &colors = {1.f, 0.f, 0.f});

    /** Plot line or lines.
     * lines.size() = line_num * 6. Every 6 values is two end points of a line.
     */
    Handle Line(const std::vector<float> &lines, float size = 1.f,
                const std::vector<float> &colors = {1.f, 0, 0});

    Handle Box(const std::array<float, 3> &pos, const std::array<float, 3> &extents,
               const std::vector<float> &color = {1.f, 0, 0});
    Handle Sphere(const std::array<float, 3> &center, float radius,
                  const std::vector<float> &color = {1.f, 0, 0});
    Handle Spheres(const std::vector<float> &centers, std::vector<float> &radii,
                   const std::vector<float> &colors = {1.f, 0, 0, 1.f});
    Handle Cone(const std::array<float, 3> &center, float radius, float height,
                const std::vector<float> &color = {1.f, 0, 0});
    Handle Cylinder(const std::array<float, 3> &center, float radius, float height,
                    const std::vector<float> &color = {1.f, 0, 0});
    /**
     * @brief  Plot a Arrow or Arrows
     * @note   Arrows` matrix is default, a Arrow`s matrix is set to its own.
     * @param  tails: Each tail vertex of Arrow
     * @param  heads: Each head vertex of Arrow
     * @param  radius: radius of all Arrow
     * @param  colors: Each color of Arrow
     * @retval Handle
     */
    Handle Arrow(const std::vector<float> &tails, const std::vector<float> &heads, float radius,
                 const std::vector<float> &colors = {1.0f, 0, 0});
    Handle Mesh(const std::vector<float> &vertices, const std::vector<unsigned int> &indices,
                const std::vector<float> &colors = {1.f, 0, 0});

    /**
     * Plot a plane
     *
     * We first plot the plane with the center at the world origin, then transfrom it
     * to the target place with provided T
     *
     * @code
     * h = v.Plane(1, 1);
     * v.SetTransform(h);
     * @endcode
     * @param xlength length of the plane along x-axis
     * @param ylength length of the plane along y-axis
     * @param half_x_num_cells half number of cells along x-axis
     * @param half_y_num_cells half number of cells along y-axis
     * @param color color of the plane
     * @return Handle
     */
    Handle Plane(float xlength, float ylength, int half_x_num_cells = 8, int half_y_num_cells = 8,
                 const std::vector<float> &color = {0.5, 0.5, 0.5});

    Handle Ground(int halfcells, float cellsize, const std::vector<float> &color);

    /**
     * Plane
     *
     * We plot the plane with the center at the world origin and the quaternion.
     *
     * @code
     * h = v.Plane(2, 2, 1, 1, (1, 1, 1), (0, 0, 0, 1))
     * @endcode
     * @param xlength length of the plane along x-axis
     * @param ylength length of the plane along y-axis
     * @param half_x_num_cells half number of cells along x-axis
     * @param half_y_num_cells half number of cells along y-axis
     * @param trans center of the plane
     * @param quat  Unit quaternion to rotate the plane with (x, y, z, w)
     * @param color color of the plane
     * @return Handle
     */
    Handle Plane(float xlength, float ylength, int half_x_num_cells, int half_y_num_cells,
                 const std::array<float, 3> &trans, const std::array<float, 4> &quat,
                 const std::vector<float> &color = {0.5, 0.5, 0.5});

    bool SetColor(const Handle &nh, const std::vector<float> &color);

    std::vector<float> GetColor(const Handle &nh) const;

    bool SetPosition(const Handle &nh, const std::array<float, 3> &pos);
    bool SetRotation(const Handle &nh, const std::array<float, 4> &quat);
    bool SetTransform(const Handle &nh, const std::array<float, 3> &pos,
                      const std::array<float, 4> &quat);

    bool GetPosition(const Handle &nh, std::array<float, 3> &pos);
    bool GetRotation(const Handle &nh, std::array<float, 4> &quat);
    bool GetTransform(const Handle &nh, std::array<float, 3> &pos, std::array<float, 4> &quat);

    /** Set Object Animation path.
     * @param h Object Handle
     * @param enable true for enable Animation, false for disable Animation
     * @param time Animation time for one loop
     * @param loop 0 - SWING 1 - LOOP 2 - NO_LOOP
     * @param pos every postion for Animation path
     * @param quat every quaternion for Animation path
     * @return true or false
     */
    bool SetObjectAnimation(const Handle &h, bool enable, float time = 6.0f, int loop = 1,
                            const std::vector<float> &pos = {},
                            const std::vector<float> &quat = {});
    /** Set Object Animation path.
     * @param enable true for enable Animation, false for disable Animation
     * @param time Animation time for one loop
     * @param loop 0 - SWING 1 - LOOP 2 - NO_LOOP
     * @param pos every camera postion for Animation path
     * @param quat every camera quaternion for Animation path
     * @return true or false
     */
    bool SetCameraAnimation(bool enable, float time = 6.0f, int loop = 1,
                            const std::vector<float> &pos = {},
                            const std::vector<float> &quat = {});
    /** Set Object Animation path.
     * @param h Object Handle
     * @param enable true for enable Animation, false for disable Animation
     * @param time Animation time for one loop
     * @param loop 0 - SWING 1 - LOOP 2 - NO_LOOP
     * @param eye every eye vector for Animation path
     * @param look every look vector for Animation path
     * @param up every up vector for Animation path
     * @return true or false
     */
    bool SetCameraAnimation(bool enable, float time, int loop, const std::vector<float> &eye,
                            const std::vector<float> &look, const std::vector<float> &up);

    /**
     * EnableGizmo
     *
     * disable default manipulator and add auxiliary points, lines and plane to
     * move, rotate and scale the model. Only one model enable at a time.
     *
     * @code
     * bool b = v.ShowGizmo(h, MOVE);
     * @endcode
     * @param h model`s handle.
     * @param gizmotype operation type, MOVE/ROTATE/SCALE/MOVE&ROTATE (1/2/3/4)
     * @return true if success
     */
    bool EnableGizmo(const Handle &h, int gizmotype);

    /**
     * SetGizmoType
     *
     * set Gizmo control type
     *
     * @code
     * bool b = v.SetGizmoType(1);
     * @endcode
     * @param gizmo operation type, MOVE/ROTATE/SCALE (1/2/3)
     * @return true if success
     */
    bool SetGizmoType(int gizmotype);

    /**
     * SetGizmoDrawMask
     *
     * set Gizmo to draw specific axis
     *
     * @code
     * bool b = v.SetGizmoDrawMask(1);
     * @endcode
     * @param optype operation type, MOVE/ROTATE/SCALE (1/2/3)
     * @param draw mask
     * MOVE : X/Y/Z/YZ/XZ/XY/ALL (1/2/4/8/16/32/63)
     * ROTATE : X/Y/Z/TRACKBALL/SCREEN/ALL (1/2/4/8/16/31)
     * SCALE : X/Y/Z/YZ/XZ/XY/ALL (1/2/4/8/16/32/63)
     * @return true if success
     */
    bool SetGizmoDrawMask(int gizmotype, unsigned int mask);

    /**
     * SetGizmoDisplayScale
     *
     * set Gizmo display scale ratio
     *
     * @code
     * bool b = v.SetGizmoScale(1);
     * @endcode
     * @param scale ratio, from 0.0 to MAX_FLOAT
     * default is 1.0
     * @return true if success
     */
    bool SetGizmoDisplayScale(float scale);

    /**
     * SetGizmoDetectionRange
     *
     * set Gizmo detection range
     *
     * @code
     * bool b = v.SetGizmoDetectionRange(1);
     * @endcode
     * @param detection range, from 0.0 to 1.0f
     * default is 0.1f
     * @return true if success
     */
    bool SetGizmoDetectionRange(float range);

    /**
     * DisableGizmo
     *
     * disable gizmo
     *
     * @code
     * bool b = v.DisableGizmo();
     * @endcode
     * @return true if success
     */
    bool DisableGizmo();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Vis

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

#if defined(__INTEL_COMPILER)
#pragma warning(disable : 1682)  // implicit conversion of a 64-bit integral type to a smaller
                                 // integral type (potential portability problem)
#elif defined(__GNUG__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wdeprecated"
#endif
#if __GNUC__ >= 7
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#endif

#include <IGizmo.h>
#include <Vis/Vis.h>
#include <osg/AnimationPath>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Math>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgFX/AnisotropicLighting>
#include <osgFX/BumpMapping>
#include <osgFX/Cartoon>
#include <osgFX/Outline>
#include <osgFX/Scribe>
#include <osgFX/SpecularHighlights>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgUtil/ShaderGen>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/TriStripVisitor>
#include <osgViewer/CompositeViewer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/config/SingleWindow>

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "GzimoDrawable.h"
#include "Intersector/PointIntersector.h"
#include "Manipulator/TouchballManipulator.h"
// #include "Utils.h"

#define VIS_VERSION "1.1.0"

// #define VIS_LOGGER_SHORT 1
#ifndef VIS_DISABLE_LOGGER

#define SPDLOG_STATIC_LIB 1
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#ifndef __FUNC_NAME__
#ifdef WIN32  // WINDOWS
#define __FUNC_NAME__ __FUNCTION__
#else  //*NIX
#define __FUNC_NAME__ __func__
#endif
#endif

namespace Vis {

inline void Sleep(uint64_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline uint64_t GetTimeMillisecond() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

static std::shared_ptr<spdlog::logger> GetOrCreateLogger(
    const char *name, const enum spdlog::level::level_enum level = spdlog::level::err) {
    auto lp = spdlog::get(name);
    if (!lp) {
        lp = spdlog::stdout_color_mt(name);
        lp->set_level(level);
        lp->set_pattern("[%n][%^%L%$]:%v");
    }
    return lp;
}

}  // namespace Vis

static auto sg_vis_logger = Vis::GetOrCreateLogger("Vis");

#if defined(VIS_LOGGER_SHORT)

#define VIS_INFO(...) sg_vis_logger->info("{0}:{1}", __FUNC_NAME__, fmt::format(__VA_ARGS__))
#define VIS_DEBUG(...) sg_vis_logger->debug("{0}:{1}", __FUNC_NAME__, fmt::format(__VA_ARGS__))
#define VIS_WARN(...) sg_vis_logger->warn("{0}:{1}", __FUNC_NAME__, fmt::format(__VA_ARGS__))
#define VIS_ERROR(...) sg_vis_logger->error("{0}:{1}", __FUNC_NAME__, fmt::format(__VA_ARGS__))

#else

#define VIS_INFO(...)                                                         \
    sg_vis_logger->info("{0}:{1}:{2}:{3}", __FILE__, __LINE__, __FUNC_NAME__, \
                        fmt::format(__VA_ARGS__))
#define VIS_DEBUG(...)                                                         \
    sg_vis_logger->debug("{0}:{1}:{2}:{3}", __FILE__, __LINE__, __FUNC_NAME__, \
                         fmt::format(__VA_ARGS__))
#define VIS_WARN(...)                                                         \
    sg_vis_logger->warn("{0}:{1}:{2}:{3}", __FILE__, __LINE__, __FUNC_NAME__, \
                        fmt::format(__VA_ARGS__))
#define VIS_ERROR(...)                                                         \
    sg_vis_logger->error("{0}:{1}:{2}:{3}", __FILE__, __LINE__, __FUNC_NAME__, \
                         fmt::format(__VA_ARGS__))

#endif

#define VIS_ENABLE_BT(n) sg_vis_logger->enable_backtrace(n)
#define VIS_DUMP_BT() sg_vis_logger->dump_backtrace()
#define VIS_DISABLE_BT() sg_vis_logger->disable_backtrace()

#else

#define VIS_SET_LEVEL(level)
#define VIS_INFO(...)
#define VIS_DEBUG(...)
#define VIS_WARN(...)
#define VIS_ERROR(...)

#define VIS_ENABLE_BT(n)
#define VIS_DUMP_BT()
#define VIS_DISABLE_BT()

#endif

using Vis::Handle;
using Vis::ViewConfig;

enum CommandType {
    CommandType_None = 0,
    CommandType_Home,
    CommandType_CreateView,
    CommandType_IsViewClosed,
    CommandType_CloseView,
    CommandType_GetViewSize,
    CommandType_ClearNodes,
    CommandType_ShowNode,
    CommandType_HideNode,
    CommandType_Chain,
    CommandType_Unchain,
    CommandType_SetTransparency,
    CommandType_DeleteNode,
    CommandType_DeleteNodes,
    CommandType_HasNode,
    CommandType_Clone,
    CommandType_CloneMultiple,

    CommandType_PlotAxes,
    CommandType_LoadModel,
    CommandType_PlotText,
    CommandType_SetText,
    CommandType_SetTextFont,
    CommandType_Plot2DQuad,
    CommandType_PlotPoint,
    CommandType_PlotLine,
    CommandType_PlotBox,
    CommandType_PlotCylinder,
    CommandType_PlotSphere,
    CommandType_PlotSpheres,
    CommandType_PlotCone,
    CommandType_PlotArrow,
    CommandType_PlotMesh,
    CommandType_PlotGround,

    CommandType_SetShow,
    CommandType_SetPosition,
    CommandType_SetRotation,   // Quaternion
    CommandType_SetTransform,  // Quaternion + Translation
    CommandType_SetTransforms,
    CommandType_SetColor,
    // CommandType_GetPosition,  these two are implemented by using GetTransform
    // CommandType_GetRotattion,
    CommandType_GetTransform,
    CommandType_SetHomePose,
    CommandType_GetHomePose,
    CommandType_SetCameraPose,
    CommandType_GetCameraPose,
    CommandType_SetObjectAnimation,
    CommandType_SetCameraAnimation,

    CommandType_EnableGizmo,
    CommandType_DisableGizmo,
    CommandType_SetGizmoType,
    CommandType_SetGizmoDrawMask,
    CommandType_SetGizmoDisplayScale,
    CommandType_SetGizmoDetectionRange,
};

enum LoadModelType {
    LoadModelType_ByName = 0,
    LoadModelType_ByNames,
    LoadModelType_ByNameAndTransform,
    LoadModelType_ByNamesAndTransforms,
};

enum ViewObjectType {
    ViewObjectType_None = 0,
    ViewObjectType_View,
    ViewObjectType_Model,
    ViewObjectType_Point,
    ViewObjectType_Line,
    ViewObjectType_Arrow,
    ViewObjectType_Mesh,
    ViewObjectType_Axes,
    ViewObjectType_Box,
    ViewObjectType_Plane,
    ViewObjectType_Sphere,
    ViewObjectType_Spheres,
    ViewObjectType_Cone,
    ViewObjectType_Cylinder,
    ViewObjectType_2D,
    ViewObjectType_Text,
    ViewObjectType_Gzimo,
};

struct Command {
    Command() {
        type = CommandType_None;
        success = false;
        data1 = nullptr;
        data2 = nullptr;
        data3 = nullptr;
        size1 = 0;
        size2 = 0;
        size3 = 0;
    }

    int type;
    bool success;

    struct {
        bool b{false};
        int i{0};
        Handle h;
        std::vector<float> data;
    } ret;  // Used to store command return

    ViewConfig viewcfg;

    Handle who;

    std::string name;
    std::vector<std::string> names;
    osg::Matrixf transform;
    std::array<float, 3> extents;
    std::array<float, 3> pos;
    std::array<float, 3> upvector;

    std::vector<float> buffer;  // color.size could be 3 or 4
    std::vector<Handle> handles;
    std::vector<osg::Matrixf> transforms;

    struct {
        bool b{0};
        int i{0};
        unsigned int u{0};
        float f{0};
        double d{0};
    } val;  // Used to store single value

    const float *data1;
    unsigned int size1;
    const float *data2;
    unsigned int size2;
    const unsigned int *data3;
    unsigned int size3;
};

struct VisGizmo {
    int capture{0};
    bool view_manipulation{false};
    Handle handle;
    Handle refHandle;
    float matrix[16];
};

struct Vis3d {
    Command *pcmd;  // Pointer to an external command object from view

    bool has_cmd{false};  // Has command to process

    bool is_inited{false};

    VisGizmo gizmo;  // gizmo param

    std::atomic<uint64_t> uid{0};  // current UID
    std::atomic<uint64_t> objid{0};

    std::atomic_bool done{false};
    std::atomic_bool entered_loop{false};
    std::atomic_bool exited_loop{false};

    std::mutex mtx;  // Used to prevent multiple thread changing Vis3d

    std::unique_ptr<std::thread> rendering_thread;
    std::mutex view_mtx;  // used to protect the viewer data
    std::condition_variable view_cv;

    enum Vis::IntersectorMode insector_mode{Vis::IntersectorMode_Disable};
    bool insector_hover{false};
    std::list<Handle> picked_handles;
    std::vector<Handle> picked_point_handles;

    std::unordered_map<Handle, std::vector<float>, Vis::HandleHasher> node_colors;

    Handle picked;
    std::array<float, 6> pointnorm;
    std::unordered_map<Handle, osg::ref_ptr<osgViewer::View>, Vis::HandleHasher> viewmap;
    std::unordered_map<Handle, osg::ref_ptr<osg::MatrixTransform>, Vis::HandleHasher> nodemap;
    osg::ref_ptr<osg::Group> scene_root;
    osg::ref_ptr<osg::Switch> node_switch;  // switch to control the group of loaded models
    osg::ref_ptr<osgViewer::CompositeViewer> compviewer;  // THE composite viewer
    // for 2d Text
    osg::ref_ptr<osg::Camera> camera_2d{nullptr};
    osg::ref_ptr<osg::Switch> switch_2d{nullptr};
    std::unordered_map<Handle, osg::ref_ptr<osg::Drawable>, Vis::HandleHasher> map_2d;
    osg::ref_ptr<osgText::Font> font;

    bool has_internal_cmd{false};
    Command internal_cmd;
};

static Vis3d sg_vis3d_shared;  ///< This is only being used as the shared instance

static bool Vis3d_IsInited(const Vis3d *pv3);
static bool Vis3d_Init(Vis3d *pv3);      // Create the system
static void Vis3d_Shutdown(Vis3d *pv3);  // Wait and make sure the vis3d is shutdown
static void Vis3d_ShutdownShared();      // wait and make sure the shared vis3d is shutdown at exit
static void Vis3d_Callback(void *);
static void Vis3d_Update(Vis3d *pv3);   // Process user command
static void Vis3d_Cleanup(Vis3d *pv3);  // Cleanup to make sure the thread could be re-entered

static bool Vis3d__HasView(const Vis3d *pv3, const Handle &vh);
static bool Vis3d__HasNode(const Vis3d *pv3, const Handle &nh);
static void Vis3d__RealizeWindows(Vis3d *pv3);

// These functions will only be used in rendering thread
static bool Vis3d_Command_Execute(Vis3d *pv3, Command *pcmd);  // Execute command
static void Vis3d_Command_Home(Vis3d *pv3, Command *pcmd);
static void Vis3d_Command_CreateView(Vis3d *pv3, Command *pc);
static void Vis3d_Command_CloseView(Vis3d *pv3, Command *pc);
static void Vis3d_Command_GetViewSize(Vis3d *pv3, Command *pc);
static void Vis3d_Command_IsViewClosed(const Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetShow(Vis3d *pv3, Command *pc);
static void Vis3d_Command_Chain(Vis3d *pv3, Command *pc);
static void Vis3d_Command_Unchain(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetTransforms(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetColor(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetTransparency(Vis3d *pv3, Command *pc);
static bool Vis3d_Command_ClearNodes(Vis3d *pv3, Command *pc);
static void Vis3d_Command_LoadModel(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotAxes(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotText(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetText(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetTextFont(Vis3d *pv3, Command *pc);
static void Vis3d_Command_Plot2DQuad(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotPoint(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetTransform(Vis3d *pv3, Command *pc, int flag = 0);
static void Vis3d_Command_GetTransform(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotBox(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotCylinder(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotSphere(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotSpheres(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotCone(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotArrow(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotMesh(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotGround(Vis3d *pv3, Command *pc);
static void Vis3d_Command_PlotLine(Vis3d *pv3, Command *pc);
static void Vis3d_Command_DeleteNode(Vis3d *pv3, Command *pc);
static void Vis3d_Command_DeleteNodes(Vis3d *pv3, Command *pc);
static void Vis3d_Command_HasNode(const Vis3d *pv3, Command *pc);
static void Vis3d_Command_Clone(Vis3d *pv3, Command *pc);
static void Vis3d_Command_CloneMultiple(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetHomePose(Vis3d *pv3, Command *pc);
static void Vis3d_Command_GetHomePose(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetCameraPose(Vis3d *pv3, Command *pc);
static void Vis3d_Command_GetCameraPose(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetObjectAnimation(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetCameraAnimation(Vis3d *pv3, Command *pc);
static void Vis3d_Command_EnableGizmo(Vis3d *pv3, Command *pc);
static void Vis3d_Command_DisableGizmo(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetGizmoType(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetGizmoDrawMask(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetGizmoDisplayScale(Vis3d *pv3, Command *pc);
static void Vis3d_Command_SetGizmoDetectionRange(Vis3d *pv3, Command *pc);

class PickHandler : public osgGA::GUIEventHandler {
private:
    Vis3d *m_pv3 = nullptr;

    osg::ref_ptr<osgFX::Outline> m_effect_hovered = new osgFX::Outline();

    osg::ref_ptr<osgFX::Outline> m_effect_picked = new osgFX::Outline();

    osg::ref_ptr<osg::Group> m_nodes_hovered = new osg::Group();
    osg::ref_ptr<osg::Group> m_nodes_picked = new osg::Group();

    std::vector<float> m_hovered_color{1.0, 0.5, 0.5, 1.0};
    std::vector<float> m_picked_color{1.0, 1.0, 0.5, 1.0};

    float m_picking_piont_marker_size = 0.3;

    Handle m_intersection_axes;
    Handle m_hovered_node;

    bool m_multi_selection = false;
    bool m_has_insection_point = false;

    void SetColor(const Handle &nh, const std::vector<float> &color) {
        if (color.size() == 0 || nh.type == ViewObjectType_Axes || nh.type == ViewObjectType_Line ||
            nh.type == ViewObjectType_Point || nh.type == ViewObjectType_Text ||
            nh.type == ViewObjectType_2D || nh.type == ViewObjectType_Gzimo ||
            nh.type == ViewObjectType_None) {
            return;
        }
        Command c;
        c.type = CommandType_SetColor;
        c.who = nh;
        c.buffer = color;
        Vis3d_Command_SetColor(m_pv3, &c);
    }

    bool IsPicked(const Handle &h) {
        // temporarily we won't pick many nodes, so picked_handles is vector, otherwise it should be
        // set or map
        return std::find(m_pv3->picked_handles.begin(), m_pv3->picked_handles.end(), h) !=
               m_pv3->picked_handles.end();
    }

    std::vector<float> GetColor(const Handle &h) {
        return m_pv3->node_colors.find(h) != m_pv3->node_colors.end() ? m_pv3->node_colors[h]
                                                                      : std::vector<float>();
    }

    static osg::Vec3 Cross(const osg::Vec3 &a, const osg::Vec3 &b) {
        osg::Vec3 r;
        r[0] = a[1] * b[2] - a[2] * b[1];
        r[1] = a[2] * b[0] - a[0] * b[2];
        r[2] = a[0] * b[1] - a[1] * b[0];
        return r;
    }

    Handle DrawPickedPointAxes() {
        Command c;
        c.type = CommandType_PlotAxes;
        osg::Matrixf mat;
        mat.setRotate(osg::Quat());
        mat.setTrans(osg::Vec3f());
        c.transforms.push_back(mat);
        c.buffer.resize(2);
        c.buffer[0] = m_picking_piont_marker_size;
        c.buffer[1] = m_picking_piont_marker_size * 5;
        Vis3d_Command_PlotAxes(m_pv3, &c);
        return c.ret.h;
    }

    void DeleteObject(Handle &h) {
        if (h.uid != 0) {
            Command c;
            c.type = CommandType_DeleteNode;
            c.who = h;
            Vis3d_Command_DeleteNode(m_pv3, &c);
            if (c.ret.b) {
                h = Handle();
            }
        }
    }

    void UpdatePickedPointAxesPosition(const Handle &h) {
        osg::ref_ptr<osg::MatrixTransform> mt = m_pv3->nodemap[h];
        if (mt) {
            const auto &trans = m_pv3->pointnorm;
            osg::Vec3 zdir(trans[3], trans[4], trans[5]);
            if (zdir.normalize() <= 1e-5) {
                zdir.z() = 1.0;
            }
            osg::Vec3 xdir(1., 0., 0.);
            if (xdir == zdir) {
                xdir = osg::Vec3(0., 1., 0.);
            }
            osg::Vec3 ydir = Cross(zdir, xdir);
            ydir.normalize();
            xdir = Cross(ydir, zdir);
            osg::Matrix mat(xdir[0], xdir[1], xdir[2], 0., ydir[0], ydir[1], ydir[2], 0., zdir[0],
                            zdir[1], zdir[2], 0., trans[0], trans[1], trans[2], 1);
            mt->setMatrix(mat);
            VIS_DEBUG("set picked point position to {}, {}, {}\n", trans[0], trans[1], trans[2]);
        }
    }

public:
    PickHandler(Vis3d *pv3) : m_pv3{pv3} {
        m_effect_hovered->setColor(osg::Vec4(0.05, 0.98, 0.404, 1.));
        m_effect_hovered->setWidth(4);
        m_effect_hovered->addChild(m_nodes_hovered);

        m_effect_picked->setColor(osg::Vec4(0.05, 0.404, 0.98, 1.));
        m_effect_picked->setWidth(10);
        m_effect_picked->addChild(m_nodes_picked);

        m_pv3->node_switch->addChild(m_effect_hovered, true);
        m_pv3->node_switch->addChild(m_effect_picked, true);
    }

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) final {
        if (m_pv3 == nullptr) {
            return false;
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::RESIZE) {
            if (m_pv3->camera_2d && m_pv3->switch_2d) {
                HandleResize(m_pv3->camera_2d, m_pv3->switch_2d, ea.getWindowWidth(),
                             ea.getWindowHeight());
            }
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN &&
            ea.getUnmodifiedKey() == osgGA::GUIEventAdapter::KEY_Control_L) {
            m_multi_selection = true;
        }

        // press '+' (Shift + =) to enlarge the axes size
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN &&
            ea.getKey() == osgGA::GUIEventAdapter::KEY_Plus) {
            m_picking_piont_marker_size += 0.01;
        }

        // press '-' to shrink the axes size
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN &&
            ea.getKey() == osgGA::GUIEventAdapter::KEY_Minus) {
            m_picking_piont_marker_size -= 0.01;
            m_picking_piont_marker_size =
                m_picking_piont_marker_size < 0.0001 ? 0.0001 : m_picking_piont_marker_size;
        }

        // press 'Ctrl' to multi-pick points on a mesh
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP &&
            (ea.getUnmodifiedKey() == osgGA::GUIEventAdapter::KEY_Control_L ||
             ea.getUnmodifiedKey() == osgGA::GUIEventAdapter::KEY_Control_R)) {
            m_multi_selection = false;
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE &&
            ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
            if (m_pv3->insector_mode != Vis::IntersectorMode_Disable) {
                osgViewer::View *view = dynamic_cast<osgViewer::View *>(&aa);
                if (view) {
                    Handle h = HandlePicker(view, ea);
                    osg::ref_ptr<osg::MatrixTransform> mt = m_pv3->nodemap[h];
                    if (mt.valid()) {                             // if a valid picking
                        if (!m_nodes_picked->containsNode(mt)) {  // node not picked yet
                            if (!m_multi_selection) {             // single picking mode
                                m_nodes_picked->removeChildren(0, m_nodes_picked->getNumChildren());
                                for (auto h : m_pv3->picked_handles) {
                                    SetColor(h, GetColor(h));
                                }
                                m_pv3->picked_handles.clear();
                                for (auto h : m_pv3->picked_point_handles) {
                                    DeleteObject(h);
                                }
                                m_pv3->picked_point_handles.clear();
                            }
                            m_nodes_picked->addChild(mt);
                            m_pv3->picked_handles.push_back(h);
                            SetColor(h, m_picked_color);
                        } else if (!m_multi_selection) {  // node already picked, unpick it
                            m_nodes_picked->removeChild(mt);
                            m_pv3->picked_handles.remove(h);
                            SetColor(h, GetColor(h));
                        }

                        // ctrl+click to pick a point
                        if (m_has_insection_point && m_multi_selection) {
                            auto axes_h = DrawPickedPointAxes();
                            UpdatePickedPointAxesPosition(axes_h);
                            m_pv3->picked_point_handles.push_back(axes_h);
                        }
                    } else {  // if not valid picking
                        if (!m_multi_selection) {
                            m_nodes_picked->removeChildren(0, m_nodes_picked->getNumChildren());
                            for (auto h : m_pv3->picked_handles) {
                                SetColor(h, GetColor(h));
                            }
                            m_pv3->picked_handles.clear();
                            for (auto h : m_pv3->picked_point_handles) {
                                DeleteObject(h);
                            }
                            m_pv3->picked_point_handles.clear();
                        }
                    }
                    m_pv3->picked = h;
                }
            }
        }

        // mouse moving, effect of hovering on a object
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE) {
            if (m_pv3->insector_hover && m_pv3->insector_mode != Vis::IntersectorMode_Disable) {
                osgViewer::View *view = dynamic_cast<osgViewer::View *>(&aa);
                if (view) {
                    Handle h = HandlePicker(view, ea);
                    osg::ref_ptr<osg::MatrixTransform> mt = m_pv3->nodemap[h];
                    if (mt.valid()) {
                        if (!m_nodes_hovered->containsNode(mt)) {  // hovered over a new object
                            m_nodes_hovered->removeChildren(0, m_nodes_hovered->getNumChildren());
                            if (!IsPicked(m_hovered_node)) {
                                SetColor(m_hovered_node, GetColor(m_hovered_node));
                            }

                            m_nodes_hovered->addChild(mt);
                            if (!IsPicked(h)) {
                                SetColor(h, m_hovered_color);
                            }
                            m_hovered_node = h;
                        }
                    } else {  // hovered on empty area
                        m_nodes_hovered->removeChildren(0, m_nodes_hovered->getNumChildren());
                        if (!IsPicked(m_hovered_node)) {
                            SetColor(m_hovered_node, GetColor(m_hovered_node));
                        }
                        m_hovered_node = Handle();
                    }

                    if (m_has_insection_point && h.uid != 0) {
                        if (m_intersection_axes.uid == 0) {
                            m_intersection_axes = DrawPickedPointAxes();
                        }
                        UpdatePickedPointAxesPosition(m_intersection_axes);
                    } else {
                        DeleteObject(m_intersection_axes);
                    }
                }
            }
        }
        return false;
    }

private:
    Handle FindPickedHandle(const osg::NodePath &nodePath) {
        const osg::Node *node = nullptr;
        for (auto it = nodePath.crbegin(); it != nodePath.crend(); ++it) {
            if ((*it)->asTransform()) {
                node = *it;
                break;
            }
        }
        for (const auto &kv : m_pv3->nodemap) {
            if (kv.second.get() == node) {
                const Handle h = kv.first;
                VIS_DEBUG("Picking object: (type: {0}, uid: {1})\n", h.type, h.uid);
                return h;
            }
        }
        return Handle();
    }

    Handle PickWithPolytopeIntersector(osg::ref_ptr<osgViewer::View> view,
                                       const osgGA::GUIEventAdapter &ea) {
        double x = ea.getXnormalized(), y = ea.getYnormalized();
        double w{0.05}, h{0.05};
        osg::ref_ptr<osgUtil::PolytopeIntersector> intersector = new osgUtil::PolytopeIntersector(
            osgUtil::PolytopeIntersector::PROJECTION, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(intersector.get());
        view->getCamera()->accept(iv);
        if (intersector->containsIntersections()) {
            const osgUtil::PolytopeIntersector::Intersection &result =
                intersector->getFirstIntersection();
            m_pv3->pointnorm = {0, 0, 0, 0, 0, 1};
            return FindPickedHandle(result.nodePath);
        }
        return Handle();
    }

    Handle HandlePicker(osg::ref_ptr<osgViewer::View> view, const osgGA::GUIEventAdapter &ea) {
        double x = ea.getX(), y = ea.getY();
        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;

        switch (m_pv3->insector_mode) {
        case Vis::IntersectorMode_Polytope: {
            m_has_insection_point = false;
            return PickWithPolytopeIntersector(view, ea);
        } break;
        case Vis::IntersectorMode_LineSegment: {
            m_has_insection_point = true;
            intersector = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
        } break;
        case Vis::IntersectorMode_Point: {
            m_has_insection_point = true;
            intersector = new PointIntersector(osgUtil::Intersector::WINDOW, x, y);
        } break;
        case Vis::IntersectorMode_Line: {
            m_has_insection_point = false;
            intersector = new LineIntersector(osgUtil::Intersector::WINDOW, x, y);
        } break;
        default:
            return Handle();
        }

        osgUtil::IntersectionVisitor iv(intersector.get());
        view->getCamera()->accept(iv);
        if (intersector->containsIntersections()) {
            const osgUtil::LineSegmentIntersector::Intersection &result =
                intersector->getFirstIntersection();
            osg::Vec3 trans = result.getWorldIntersectPoint();
            osg::Vec3 norm = result.getWorldIntersectNormal();
            m_pv3->pointnorm = {trans[0], trans[1], trans[2], norm[0], norm[1], norm[2]};
            return FindPickedHandle(result.nodePath);
        }
        return Handle();
    }

    bool HandleResize(osg::ref_ptr<osg::Camera> pcam, osg::ref_ptr<osg::Switch> pswitch,
                      float window_width, float window_height) {
        double left, right, bottom, top, zNear, zFar;
        pcam->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
        pcam->setProjectionMatrixAsOrtho2D(0, window_width, 0, window_height);

        for (unsigned int i = 0; i < pswitch->getNumChildren(); i++) {
            osg::Node *node = pswitch->getChild(i);
            const std::string name = node->getName();
            if (name.find("Text") != std::string::npos) {
                osg::ref_ptr<osgText::Text> text = (osgText::Text *)node;
                auto pos = text->getPosition();
                pos = {pos.x(), window_height - ((float)top - pos.y()), 0};
                text->setPosition(pos);
            } else if (name.find("Geometry") != std::string::npos) {
                osg::ref_ptr<osg::Geometry> geometry = node->asGeometry();
                osg::ref_ptr<osg::Vec2Array> vertices =
                    dynamic_cast<osg::Vec2Array *>(geometry->getVertexArray());
                for (unsigned int i = 0; i < vertices->size(); i++) {
                    float old_height = (*vertices)[i][1];
                    (*vertices)[i][1] = window_height - ((float)top - old_height);
                }
                geometry->setVertexArray(vertices);
            }
        }
        return true;
    }
};

namespace Vis {

static bool Stricmp(const std::string &a, const std::string &b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(),
                      [](char a, char b) { return tolower(a) == tolower(b); });
}

void SetLogLevel(const std::string &name) {
    if (Stricmp(name, "debug") == 0) {
        sg_vis_logger->set_level(spdlog::level::debug);
    } else if (Stricmp(name, "info") == 0) {
        sg_vis_logger->set_level(spdlog::level::info);
    } else if (Stricmp(name, "warn") == 0) {
        sg_vis_logger->set_level(spdlog::level::warn);
    } else if (Stricmp(name, "error") == 0) {
        sg_vis_logger->set_level(spdlog::level::err);
    } else if (Stricmp(name, "off") == 0) {
        sg_vis_logger->set_level(spdlog::level::off);

    } else {
        VIS_ERROR("Wrong name of log level: {0}", name);
    }
}

const char *GetVersion() {
    return VIS_VERSION;
}

struct View::Impl {
    Handle handle;
    Command cmd;
    bool shared = false;
    Vis3d *pv3 = nullptr;

    ~Impl() {
        /// When vis3d is not shared, we need to shutdown the system when the instance exits.
        /// When vis3d is shared, we need to keep it running, and it will only be shutdown when
        /// the program exit.
        if (!shared) {
            delete pv3;
        }
    }
};

View::View(const ViewConfig &cfg, bool shared) {
    m_impl = std::make_unique<View::Impl>();
    m_impl->shared = shared;

    if (shared) {
        m_impl->pv3 = &sg_vis3d_shared;
        if (!Vis3d_IsInited(m_impl->pv3)) {
            if (!Vis3d_Init(m_impl->pv3)) {
                return;
            } else {
                std::atexit(&Vis3d_ShutdownShared);  // Only register at the first time
            }
        }
    } else {
        m_impl->pv3 = new Vis3d;
        if (!Vis3d_Init(m_impl->pv3)) {
            return;
        }
    }

    if (!Vis3d_IsInited(m_impl->pv3)) {
        if (!Vis3d_Init(m_impl->pv3)) {
            return;
        }
    }

    m_impl->cmd.viewcfg = cfg;
    m_impl->cmd.type = CommandType_CreateView;
    m_impl->handle = Handle();
    if (Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd))) {
        m_impl->handle = m_impl->cmd.ret.h;
    }
}

View::View(const std::string &name, bool shared) {
    m_impl = std::make_unique<View::Impl>();
    m_impl->shared = shared;

    if (shared) {
        m_impl->pv3 = &sg_vis3d_shared;
        if (!Vis3d_IsInited(m_impl->pv3)) {
            if (!Vis3d_Init(m_impl->pv3)) {
                return;
            } else {
                std::atexit(&Vis3d_ShutdownShared);  // Only register at the first time
            }
        }
    } else {
        m_impl->pv3 = new Vis3d;
        if (!Vis3d_Init(m_impl->pv3)) {
            return;
        }
    }

    if (!Vis3d_IsInited(m_impl->pv3)) {
        if (!Vis3d_Init(m_impl->pv3)) {
            return;
        }
    }

    m_impl->cmd.viewcfg.name = name;
    m_impl->cmd.type = CommandType_CreateView;
    m_impl->handle = Handle();
    if (Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd))) {
        m_impl->handle = m_impl->cmd.ret.h;
    }
}

View::~View() {
    Close();
}

void View::WindowSetDecoration(bool enable) {
    if (!m_impl || !m_impl->pv3)
        return;

    osgViewer::CompositeViewer::Windows windows;
    {
        m_impl->pv3->compviewer->getWindows(windows);
        for (const auto &it : windows) {
            if (it->isRealizedImplementation()) {
                it->setWindowDecoration(enable);
            }
        }
    }
}

void View::WindowSetRectangle(int x, int y, int width, int height) {
    if (!m_impl || !m_impl->pv3)
        return;

    if (width <= 0 || height <= 0) {
        VIS_ERROR("Window size should be bigger than 0!");
        return;
    }

    osgViewer::CompositeViewer::Windows windows;
    {
        m_impl->pv3->compviewer->getWindows(windows);
        for (const auto &it : windows) {
            if (it->isRealizedImplementation()) {
                it->setWindowRectangle(x, y, width, height);
            }
        }
    }
}

void View::WindowGetRectangle(int &x, int &y, int &width, int &height) {
    if (!m_impl || !m_impl->pv3)
        return;

    osgViewer::CompositeViewer::Windows windows;
    {
        m_impl->pv3->compviewer->getWindows(windows);
        for (const auto &it : windows) {
            if (it->isRealizedImplementation()) {
                it->getWindowRectangle(x, y, width, height);
                break;  // Only get the size of the first window
            }
        }
    }
}

void View::WindowRaise() {
    if (!m_impl || !m_impl->pv3)
        return;

    osgViewer::CompositeViewer::Windows windows;
    {
        m_impl->pv3->compviewer->getWindows(windows);
        for (const auto &it : windows) {
            if (it->isRealizedImplementation()) {
                it->raiseWindow();
            }
        }
    }
}

std::array<int, 4> View::WindowHide() {
    std::array<int, 4> r;
    WindowGetRectangle(r[0], r[1], r[2], r[3]);
    WindowSetRectangle(-1, -1, 1, 1);
    return r;
}

void View::WindowShow(const std::array<int, 4> &rect) {
    WindowSetRectangle(rect[0], rect[1], rect[2], rect[3]);
}

bool View::SetHomePose(const std::array<float, 3> &eye,
                       const std::array<float, 3> &point_want_to_look,
                       const std::array<float, 3> &upvector) {
    if (m_impl) {
        m_impl->cmd.type = CommandType_SetHomePose;
        m_impl->cmd.who = m_impl->handle;

        m_impl->cmd.pos = point_want_to_look;
        m_impl->cmd.upvector = upvector;
        m_impl->cmd.extents = eye;  // Use extents to represent the position of the camera

        return Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd));
    }
    return false;
}

bool View::GetHomePose(std::array<float, 3> &eye, std::array<float, 3> &point_want_to_look,
                       std::array<float, 3> &upvector) {
    if (m_impl) {
        m_impl->cmd.type = CommandType_GetHomePose;
        m_impl->cmd.who = m_impl->handle;
        if (Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd))) {
            if (m_impl->cmd.ret.data.size() == 9) {
                const auto &d = m_impl->cmd.ret.data;
                eye[0] = d[0];
                eye[1] = d[1];
                eye[2] = d[2];
                point_want_to_look[0] = d[3];
                point_want_to_look[1] = d[4];
                point_want_to_look[2] = d[5];
                upvector[0] = d[6];
                upvector[1] = d[7];
                upvector[2] = d[8];
                return true;
            }
        }
    }
    return false;
}

bool View::SetCameraPose(const std::array<float, 3> &eye,
                         const std::array<float, 3> &point_want_to_look,
                         const std::array<float, 3> &upvector) {
    if (m_impl) {
        m_impl->cmd.type = CommandType_SetCameraPose;
        m_impl->cmd.who = m_impl->handle;

        m_impl->cmd.pos = point_want_to_look;
        m_impl->cmd.upvector = upvector;
        m_impl->cmd.extents = eye;  // Use extents to represent the position of the camera

        return Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd));
    }
    return false;
}

bool View::GetCameraPose(std::array<float, 3> &eye, std::array<float, 3> &point_want_to_look,
                         std::array<float, 3> &upvector) {
    if (m_impl) {
        m_impl->cmd.type = CommandType_GetCameraPose;
        m_impl->cmd.who = m_impl->handle;
        if (Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd))) {
            if (m_impl->cmd.ret.data.size() == 9) {
                const auto &d = m_impl->cmd.ret.data;
                eye[0] = d[0];
                eye[1] = d[1];
                eye[2] = d[2];
                point_want_to_look[0] = d[3];
                point_want_to_look[1] = d[4];
                point_want_to_look[2] = d[5];
                upvector[0] = d[6];
                upvector[1] = d[7];
                upvector[2] = d[8];
                return true;
            }
        }
    }
    return false;
}

bool View::SetObjectAnimation(const Handle &h, bool enable, float time, int loop,
                              const std::vector<float> &pos, const std::vector<float> &quat) {
    const size_t pos_size = pos.size() / 3;
    const size_t quat_size = quat.size() / 4;
    if (enable) {
        if (time < 0) {
            VIS_ERROR(u8"Animation time must > 0, current is {}.", time);
            return false;
        }
        if (pos_size != quat_size) {
            VIS_ERROR(u8"pos size [{}] must match quat size [{}].", pos_size, quat_size);
            return false;
        }
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetObjectAnimation;
    c.who = h;
    c.val.f = time;
    c.val.b = enable;
    c.val.i = loop;
    c.data1 = pos.data();
    c.size1 = pos_size;
    c.data2 = quat.data();
    c.size2 = quat_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetCameraAnimation(bool enable, float time, int loop, const std::vector<float> &eye,
                              const std::vector<float> &look, const std::vector<float> &up) {
    const size_t eye_size = eye.size() / 3;
    const size_t look_size = look.size() / 3;
    const size_t up_size = up.size() / 3;
    std::vector<float> pos, quat;
    int path_size = 0;
    if (enable) {
        if (time < 0) {
            VIS_ERROR(u8"Animation time must > 0, current is {}.", time);
            return false;
        }

        if (eye_size != look_size || look_size != up_size) {
            VIS_ERROR(u8"eye size [{}] must match look size [{}] and up size [{}].", eye_size,
                      look_size, up_size);
            return false;
        }

        path_size = eye_size;
        for (int i = 0; i < path_size; i++) {
            osg::Matrix m;
            m.makeLookAt({eye[i * 3 + 0], eye[i * 3 + 1], eye[i * 3 + 2]},
                         {look[i * 3 + 0], look[i * 3 + 1], look[i * 3 + 2]},
                         {up[i * 3 + 0], up[i * 3 + 1], up[i * 3 + 2]});
            auto trans = m.getTrans();
            auto rotate = m.getRotate();
            pos.insert(pos.end(), {(float)trans[0], (float)trans[1], (float)trans[2]});
            quat.insert(quat.end(),
                        {(float)rotate[0], (float)rotate[1], (float)rotate[2], (float)rotate[3]});
        }
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetCameraAnimation;
    c.who = m_impl->handle;
    c.val.f = time;
    c.val.b = enable;
    c.val.i = loop;
    c.data1 = pos.data();
    c.size1 = path_size;
    c.data2 = quat.data();
    c.size2 = path_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetCameraAnimation(bool enable, float time, int loop, const std::vector<float> &pos,
                              const std::vector<float> &quat) {
    const size_t pos_size = pos.size() / 3;
    const size_t quat_size = quat.size() / 4;
    if (enable) {
        if (time < 0) {
            VIS_ERROR(u8"Animation time must > 0, current is {}.", time);
            return false;
        }
        if (pos_size != quat_size) {
            VIS_ERROR(u8"pos size [{}] must match quat size [{}].", pos_size, quat_size);
            return false;
        }
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetCameraAnimation;
    c.who = m_impl->handle;
    c.val.f = time;
    c.val.b = enable;
    c.val.i = loop;
    c.data1 = pos.data();
    c.size1 = pos_size;
    c.data2 = quat.data();
    c.size2 = quat_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::EnableGizmo(const Handle &h, int gizmotype) {
    if (gizmotype < 1 || gizmotype > 4) {
        VIS_ERROR("operation type should be 1 to 3, current is {}.", gizmotype);
        return false;
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_EnableGizmo;
    c.who = h;
    c.val.i = gizmotype;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::DisableGizmo() {
    Command &c = m_impl->cmd;
    c.type = CommandType_DisableGizmo;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetGizmoType(int gizmotype) {
    if (gizmotype < 1 || gizmotype > 3) {
        VIS_ERROR("operation type should be 1 to 3, current is {}.", gizmotype);
        return false;
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetGizmoType;
    c.val.i = gizmotype;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetGizmoDrawMask(int gizmotype, unsigned int mask) {
    if (gizmotype < 1 || gizmotype > 3) {
        VIS_ERROR("operation type should be 1 to 3, current is {}.", gizmotype);
        return false;
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetGizmoDrawMask;
    c.val.i = gizmotype;
    c.val.u = mask;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetGizmoDisplayScale(float scale) {
    if (scale <= 0.0f) {
        VIS_ERROR("scale is out of range. current is [{}]", scale);
        return false;
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetGizmoDisplayScale;
    c.val.f = scale;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::SetGizmoDetectionRange(float range) {
    if (range <= 0.0f || range >= 1.0f) {
        VIS_ERROR("detection range is out of range. current is [{}]", range);
        return false;
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetGizmoDetectionRange;
    c.val.f = range;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

bool View::Close() {
    if (m_impl) {
        m_impl->cmd.type = CommandType_CloseView;
        m_impl->cmd.who = m_impl->handle;

        if (m_impl->shared) {
            return Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd));
        } else {
            Vis3d_Command_Execute(m_impl->pv3, &(m_impl->cmd));
            Vis3d_Shutdown(m_impl->pv3);
        }
    }
    return false;
}

bool View::Delete(const Handle &nh) {
    Command &c = m_impl->cmd;
    c.type = CommandType_DeleteNode;
    c.who = nh;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::IsClosed() const {
    if (!m_impl)
        return false;

    Command &c = m_impl->cmd;
    c.who = m_impl->handle;
    c.type = CommandType_IsViewClosed;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::IsAlive(const Handle &nh) const {
    Command &c = m_impl->cmd;
    c.type = CommandType_HasNode;
    c.who = nh;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Delete(const std::vector<Handle> &handles) {
    Command &c = m_impl->cmd;
    c.type = CommandType_DeleteNodes;
    c.handles = handles;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Clear() {
    Command &c = m_impl->cmd;
    c.type = CommandType_ClearNodes;
    c.who = m_impl->handle;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Home() {
    Command &c = m_impl->cmd;
    c.type = CommandType_Home;
    c.who = m_impl->handle;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Show(const Handle &nh) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetShow;
    c.who = nh;
    c.val.b = true;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Hide(const Handle &nh) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetShow;
    c.who = nh;
    c.val.b = false;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Chain(const std::vector<Handle> &links) {
    if (links.size() < 1) {
        VIS_ERROR("size of links should be more than 1: {0}", links.size());
        return false;
    }

    // check if we have same links
    {
        std::unordered_set<Handle, HandleHasher> s;
        for (const auto h : links) {
            if (s.find(h) != s.end()) {
                VIS_ERROR("Same links passed into Chain, which is ilegal!");
                return false;
            }
        }
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_Chain;
    c.handles = links;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::Unchain(const std::vector<Handle> &links) {
    if (links.size() < 1) {
        VIS_ERROR("links.size() should be more than 1: {0}", links.size());
        return false;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_Unchain;
    c.handles = links;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::SetTransforms(const std::vector<Handle> &hs,
                         const std::vector<std::array<float, 3>> &trans,
                         const std::vector<std::array<float, 4>> &quats) {
    const int hs_size = hs.size();
    const int trans_size = trans.size();
    const int quats_size = quats.size();

    if (hs_size < 1 || (trans_size != hs_size || quats_size != hs_size)) {
        VIS_ERROR("Invalid parameters: hs.size: {0}, trans.size: {1}, quats.size: {2}", hs_size,
                  trans_size, quats_size);
        return false;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_SetTransforms;
    c.handles = hs;
    c.transforms.resize(trans_size);
    for (int i = 0; i < trans_size; ++i) {
        c.transforms[i].setRotate(osg::Quat(quats[i][0], quats[i][1], quats[i][2], quats[i][3]));
        c.transforms[i].setTrans(osg::Vec3f(trans[i][0], trans[i][1], trans[i][2]));
    }
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::SetTransforms(const Handle &h, const std::vector<std::array<float, 3>> &trans,
                         const std::vector<std::array<float, 4>> &quats) {
    const int trans_size = trans.size();
    const int quats_size = quats.size();

    if (trans_size != quats_size) {
        VIS_ERROR("Invalid parameters: trans.size: {0}, quats.size: {1}", trans_size, quats_size);
        return false;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_SetTransforms;
    c.handles.clear();  // clear the handle to indicate using the single link mode
    c.who = h;
    c.transforms.resize(trans_size);
    for (int i = 0; i < trans_size; ++i) {
        c.transforms[i].setRotate(osg::Quat(quats[i][0], quats[i][1], quats[i][2], quats[i][3]));
        c.transforms[i].setTrans(osg::Vec3f(trans[i][0], trans[i][1], trans[i][2]));
    }
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

// TODO(Hui): In OSG, different objects has different ways of setting
// transparency, which needs some time to implement and test. For now,
// we only support Model, Box, Sphere, Cylinder, Cone here.
// https://blog.csdn.net/wang15061955806/article/details/49466337
bool View::SetTransparency(const Handle &nh, float inv_alpha) {
    if (nh.type != ViewObjectType_Model && nh.type != ViewObjectType_Mesh &&
        nh.type != ViewObjectType_Box && nh.type != ViewObjectType_Sphere &&
        nh.type != ViewObjectType_Cylinder && nh.type != ViewObjectType_Cone) {
        VIS_ERROR("Currently only Model, Mesh, Box, Sphere, Cylinder type is supported.");
        return false;
    }

    if (inv_alpha < 0.f || inv_alpha > 1.f) {
        VIS_ERROR("Invalid alpha value: {0}, should be in range [0, 1]", inv_alpha);
        return false;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_SetTransparency;
    c.who = nh;
    c.val.f = 1.f - inv_alpha;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::SetColor(const Handle &nh, const std::vector<float> &color) {
    if (color.size() == 0) {
        VIS_ERROR("Invalud color size: {0}, should be 3 or 4.", color.size());
        return false;
    }
    if (color.size() % 3 != 0 && color.size() % 4 != 0) {
        VIS_ERROR("Invalud color size: {0}, should be 3 or 4.", color.size());
        return false;
    }
    for (auto clr : color) {
        if (clr > 1.f || clr < 0.f) {
            VIS_ERROR("Invalid color value, should be in range [0, 1].", clr);
            return false;
        }
    }
    Command &c = m_impl->cmd;
    c.type = CommandType_SetColor;
    c.who = nh;
    c.buffer = color;
    Vis3d_Command_Execute(m_impl->pv3, &c);
    if (c.ret.b) {
        m_impl->pv3->node_colors[nh] = color;
    }
    return c.ret.b;
}

std::vector<float> View::GetColor(const Handle &nh) const {
    return m_impl->pv3->node_colors[nh];
}

bool View::SetPosition(const Handle &nh, const std::array<float, 3> &trans) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetPosition;
    c.who = nh;
    c.transform.setTrans(osg::Vec3f(trans[0], trans[1], trans[2]));
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::SetRotation(const Handle &nh, const std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetRotation;
    c.who = nh;
    c.transform.setRotate(osg::Quat(quat[0], quat[1], quat[2], quat[3]));
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::SetTransform(const Handle &nh, const std::array<float, 3> &trans,
                        const std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetTransform;
    c.who = nh;

    const osg::Quat q(quat[0], quat[1], quat[2], quat[3]);
    const osg::Vec3f t(trans[0], trans[1], trans[2]);
    c.transform.setRotate(q);
    c.transform.setTrans(t);
    Vis3d_Command_Execute(m_impl->pv3, &c);
    return c.ret.b;
}

bool View::GetPosition(const Handle &nh, std::array<float, 3> &pos) {
    Command &c = m_impl->cmd;
    c.type = CommandType_GetTransform;
    c.who = nh;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        const osg::Vec3f trans = c.transform.getTrans();
        pos[0] = trans.x();
        pos[1] = trans.y();
        pos[2] = trans.z();
        return c.ret.b;
    }
    return false;
}

bool View::GetRotation(const Handle &nh, std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_GetTransform;
    c.who = nh;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        const osg::Quat q = c.transform.getRotate();
        quat[0] = q.x();
        quat[1] = q.y();
        quat[2] = q.z();
        quat[3] = q.w();
        return c.ret.b;
    }
    return false;
}

bool View::GetTransform(const Handle &nh, std::array<float, 3> &pos, std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_GetTransform;
    c.who = nh;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        const osg::Quat q = c.transform.getRotate();
        const osg::Vec3f trans = c.transform.getTrans();
        quat[0] = q.x();
        quat[1] = q.y();
        quat[2] = q.z();
        quat[3] = q.w();
        pos[0] = trans.x();
        pos[1] = trans.y();
        pos[2] = trans.z();
        return true;
    }
    return false;
}

Handle View::Clone(const Handle h) {
    std::array<float, 3> pos;
    GetPosition(h, pos);
    std::array<float, 4> quat;
    GetRotation(h, quat);
    return Clone(h, pos, quat);
}

Handle View::Clone(const Handle h, const std::array<float, 3> &pos,
                   const std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_Clone;
    c.who = h;
    c.transform.setRotate(osg::Quat(quat[0], quat[1], quat[2], quat[3]));
    c.transform.setTrans(osg::Vec3f(pos[0], pos[1], pos[2]));

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

std::vector<Handle> View::Clone(const std::vector<Handle> &handles) {
    std::vector<std::array<float, 3>> poss;
    std::vector<std::array<float, 4>> quats;
    for (auto &h : handles) {
        std::array<float, 3> pos;
        GetPosition(h, pos);
        std::array<float, 4> quat;
        GetRotation(h, quat);
        poss.push_back(pos);
        quats.push_back(quat);
    }
    return Clone(handles, poss, quats);
}

std::vector<Handle> View::Clone(const std::vector<Handle> &handles,
                                const std::vector<std::array<float, 3>> &poss,
                                const std::vector<std::array<float, 4>> &quats) {
    const int hs_size = handles.size();
    const int trans_size = poss.size();
    const int quats_size = quats.size();

    if (hs_size < 1 || (trans_size != hs_size || quats_size != hs_size)) {
        VIS_ERROR("Invalid parameters: hs.size: {0}, trans.size: {1}, quats.size: {2}", hs_size,
                  trans_size, quats_size);
        return {};
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_CloneMultiple;
    c.handles = handles;
    c.transforms.resize(handles.size());
    for (size_t i = 0; i < handles.size(); ++i) {
        c.transforms[i].setRotate(osg::Quat(quats[i][0], quats[i][1], quats[i][2], quats[i][3]));
        c.transforms[i].setTrans(osg::Vec3f(poss[i][0], poss[i][1], poss[i][2]));
    }
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles;
    }
    return {};
}

bool View::GetViewSize(int &width, int &height) {
    Command &c = m_impl->cmd;
    c.type = CommandType_GetViewSize;
    c.who = m_impl->handle;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        if (c.ret.b) {
            width = (int)c.ret.data[0];
            height = (int)c.ret.data[1];
        }
        return c.ret.b;
    }
    return false;
}

void View::SetIntersectorMode(enum Vis::IntersectorMode mode, bool hover) {
    m_impl->pv3->insector_mode = mode;
    m_impl->pv3->insector_hover = hover;
}

enum Vis::IntersectorMode View::GetIntersectorMode() const {
    return m_impl->pv3->insector_mode;
}

std::vector<Handle> View::MultiPicked() const {
    return std::vector<Handle>(std::make_move_iterator(std::begin(m_impl->pv3->picked_handles)),
                               std::make_move_iterator(std::end(m_impl->pv3->picked_handles)));
}

std::vector<Handle> &View::GetPickedPointAxes() {
    return m_impl->pv3->picked_point_handles;
}

void View::ClearPickedPointAxes() {
    Delete(m_impl->pv3->picked_point_handles);
    m_impl->pv3->picked_point_handles.clear();
}

Handle View::Picked() {
    return m_impl->pv3->picked;
}

std::array<float, 6> View::PickedPlane() {
    return m_impl->pv3->pointnorm;
}

Handle View::Load(const std::string &fname) {
    Command &c = m_impl->cmd;
    c.type = CommandType_LoadModel;
    c.val.i = LoadModelType_ByName;
    c.name = fname;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

Handle View::Load(const std::string &fname, const std::array<float, 3> &pos,
                  const std::array<float, 4> &quat) {
    Command &c = m_impl->cmd;
    c.type = CommandType_LoadModel;
    c.val.i = LoadModelType_ByNameAndTransform;
    c.name = fname;

    c.transform.setRotate(osg::Quat(quat[0], quat[1], quat[2], quat[3]));
    c.transform.setTrans(osg::Vec3f(pos[0], pos[1], pos[2]));

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

std::vector<Handle> View::Load(const std::vector<std::string> &fnames) {
    Command &c = m_impl->cmd;
    c.type = CommandType_LoadModel;
    c.val.i = LoadModelType_ByNames;
    c.names = fnames;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles;
    }
    return {};
}

std::vector<Handle> View::Load(const std::vector<std::string> &fnames,
                               const std::vector<std::array<float, 3>> &poss,
                               const std::vector<std::array<float, 4>> &quats) {
    Command &c = m_impl->cmd;
    c.type = CommandType_LoadModel;
    c.val.i = LoadModelType_ByNamesAndTransforms;
    c.names = fnames;
    c.transforms.resize(fnames.size());

    for (size_t i = 0; i < fnames.size(); ++i) {
        c.transforms[i].setRotate(osg::Quat(quats[i][0], quats[i][1], quats[i][2], quats[i][3]));
        c.transforms[i].setTrans(osg::Vec3f(poss[i][0], poss[i][1], poss[i][2]));
    }

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles;
    }
    return {};
}

std::vector<Handle> View::Axes(const std::vector<std::array<float, 3>> &translations,
                               const std::vector<std::array<float, 4>> &quaternions, float axis_len,
                               float axis_size) {
    std::vector<Handle> hs;
    if (axis_len <= 0 || axis_size <= 0) {
        VIS_ERROR("Invalid axis len or size: {0}, {1}", axis_len, axis_size);
        return hs;
    }
    size_t n = translations.size();
    size_t m = quaternions.size();
    n = std::min<size_t>(n, m);
    Command &c = m_impl->cmd;
    c.transforms.clear();
    c.transforms.shrink_to_fit();
    c.handles.clear();
    c.handles.shrink_to_fit();
    c.type = CommandType_PlotAxes;
    for (size_t i = 0; i < n; ++i) {
        const auto &trans = translations[i];
        const auto &quat = quaternions[i];
        osg::Matrixf mat;
        mat.setRotate(osg::Quat(quat[0], quat[1], quat[2], quat[3]));
        mat.setTrans(osg::Vec3f(trans[0], trans[1], trans[2]));
        c.transforms.push_back(mat);
    }
    c.buffer.resize(2);
    c.buffer[0] = axis_len;
    c.buffer[1] = axis_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles;
    }
    return hs;
}

std::vector<Handle> View::Axes(const std::vector<std::array<float, 16>> &transforms, float axis_len,
                               float axis_size) {
    std::vector<Handle> hs;
    if (axis_len <= 0 || axis_size <= 0) {
        VIS_ERROR("Invalid axis len or size: {0}, {1}", axis_len, axis_size);
        return hs;
    }
    size_t n = transforms.size();
    Command &c = m_impl->cmd;
    c.type = CommandType_PlotAxes;
    c.transforms.clear();
    c.transforms.shrink_to_fit();
    c.handles.clear();
    c.handles.shrink_to_fit();
#define AT(i, j) transform[i + j * 4]
    for (const auto &transform : transforms) {
        osg::Matrixf mat;
        mat.set(AT(0, 0), AT(0, 1), AT(0, 2), AT(0, 3), AT(1, 0), AT(1, 1), AT(1, 2), AT(1, 3),
                AT(2, 0), AT(2, 1), AT(2, 2), AT(2, 3), AT(3, 0), AT(3, 1), AT(3, 2), AT(3, 3));
        c.transforms.push_back(mat);
    }
#undef AT
    c.buffer.resize(2);
    c.buffer[0] = axis_len;
    c.buffer[1] = axis_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles;
    }
    return hs;
}

Handle View::Axes(const std::array<float, 3> &trans, const std::array<float, 4> &quat,
                  float axis_len, float axis_size) {
    Handle h;
    if (axis_len <= 0 || axis_size <= 0) {
        VIS_ERROR("Invalid axis len or size: {0}, {1}", axis_len, axis_size);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotAxes;
    c.transforms.clear();
    c.transforms.shrink_to_fit();
    c.handles.clear();
    c.handles.shrink_to_fit();
    osg::Matrixf mat;
    mat.setRotate(osg::Quat(quat[0], quat[1], quat[2], quat[3]));
    mat.setTrans(osg::Vec3f(trans[0], trans[1], trans[2]));
    c.transforms.push_back(mat);
    c.buffer.resize(2);
    c.buffer[0] = axis_len;
    c.buffer[1] = axis_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles[0];
    }
    return h;
}

Handle View::Axes(const std::array<float, 16> &transform, float axis_len, float axis_size) {
    Handle h;
    if (axis_len <= 0 || axis_size <= 0) {
        VIS_WARN("Invalid axis len or size: {0}, {1}", axis_len, axis_size);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotAxes;
    c.transforms.clear();
    c.transforms.shrink_to_fit();
    c.handles.clear();
    c.handles.shrink_to_fit();

    c.buffer.resize(2);
    c.buffer[0] = axis_len;
    c.buffer[1] = axis_size;

    osg::Matrixf mat;
#define AT(i, j) transform[i + j * 4]
    mat.set(AT(0, 0), AT(0, 1), AT(0, 2), AT(0, 3), AT(1, 0), AT(1, 1), AT(1, 2), AT(1, 3),
            AT(2, 0), AT(2, 1), AT(2, 2), AT(2, 3), AT(3, 0), AT(3, 1), AT(3, 2), AT(3, 3));
#undef AT

    c.transforms.push_back(mat);
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.handles[0];
    }
    return h;
}

bool View::SetText(Handle &h, const std::string &content, const std::vector<float> &pos,
                   float font_size, const std::vector<float> &colors) {
    const size_t colors_size = colors.size();
    const size_t pos_size = pos.size();
    bool check_content = content.empty();
    bool check_color = colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0);
    bool check_font = font_size < 0.0f;
    bool check_pos = pos_size != 2 && pos_size != 3;
    Command &c = m_impl->cmd;

    if (h.uid == 0) {
        if (check_content) {
            VIS_WARN("Text is empty.");
            return false;
        }
        if (check_pos) {
            VIS_WARN("pos size is wrong! {0}", pos_size);
            return false;
        }
        if (check_font) {
            VIS_WARN("font size is wrong! {0}", font_size);
            return false;
        }
        c.type = CommandType_PlotText;
    } else {
        if (check_content && check_pos && check_font && check_color) {
            VIS_WARN("noting change.");
            return false;
        }
        c.type = CommandType_SetText;
        c.who = h;
    }

    c.name = content;
    c.val.f = font_size;
    c.data1 = check_pos ? nullptr : pos.data();
    c.size1 = pos_size;
    c.data2 = check_color ? nullptr : colors.data();
    c.size2 = colors_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        if (c.type == CommandType_PlotText)
            h = c.ret.h;
        return c.ret.b;
    }
    return false;
}

bool View::SetTextFont(const std::string &font_name) {
    Command &c = m_impl->cmd;
    c.type = CommandType_SetTextFont;
    c.name = font_name;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.b;
    }
    return false;
}

Handle View::Quad2D(const std::vector<float> &xys, const ::std::vector<float> &colors, int mode) {
    Handle h;

    const size_t xys_size = (int)xys.size();
    const size_t numvert = xys_size / 2;
    const size_t colors_size = (int)colors.size();

    if (xys_size == 0 || xys_size % 2 != 0) {
        VIS_WARN("xyzs.size() is wrong! {0}", xys_size);
        return h;
    }

    if (colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return h;
    }

    size_t color_channels = 0;
    if (colors_size % 3 == 0 && colors_size % 4 == 0) {
        if (colors_size / 3 == numvert) {
            color_channels = 3;
        } else if (colors_size / 4 == numvert) {
            color_channels = 4;
        } else {
            VIS_WARN("colors.size [{}] not match vertex size [{}].", colors_size, numvert);
            return h;
        }
    } else {
        color_channels = colors_size % 3 == 0 ? 3 : 4;
    }

    const size_t numcl = colors_size / color_channels;

    if (numcl != 1 && numcl != numvert) {
        VIS_ERROR("color size [{}] not match vertex size [{}].", numcl, numvert);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_Plot2DQuad;
    c.val.i = mode;
    c.val.u = color_channels;
    c.data1 = xys.data();
    c.size1 = xys_size;
    c.data2 = colors.data();
    c.size2 = colors_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

Handle View::Point(const std::vector<float> &xyzs, float ptsize, const std::vector<float> &colors) {
    Handle h;
    const size_t xyzs_size = (int)xyzs.size();
    const size_t numpt = xyzs_size / 3;
    const size_t colors_size = (int)colors.size();
    if (xyzs_size == 0 || xyzs_size % 3 != 0) {
        VIS_WARN("xyzs.size() is wrong! {0}", xyzs_size);
        return h;
    }

    if (ptsize <= 0) {
        VIS_WARN("point size is wrong! {0}", ptsize);
        return h;
    }

    if (colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return h;
    }

    size_t color_channels = 0;
    if (colors_size % 3 == 0 && colors_size % 4 == 0) {
        if (colors_size / 3 == numpt) {
            color_channels = 3;
        } else if (colors_size / 4 == numpt) {
            color_channels = 4;
        } else {
            VIS_WARN("colors.size [{}] not match point size [{}].", colors_size, numpt);
            return h;
        }
    } else {
        color_channels = colors_size % 3 == 0 ? 3 : 4;
    }

    const size_t numcl = colors_size / color_channels;

    if (numcl != 1 && numcl != numpt) {
        VIS_ERROR("color size [{}] not match point size [{}].", numcl, numpt);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotPoint;
    c.val.f = ptsize;
    c.data1 = xyzs.data();
    c.size1 = xyzs_size;
    c.data2 = colors.data();
    c.size2 = colors_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

Handle View::Line(const std::vector<float> &lines, float size, const std::vector<float> &colors) {
    Command &c = m_impl->cmd;
    const int lines_size = (int)lines.size();
    const int colors_size = (int)colors.size();

    if (lines_size == 0 || lines_size % 6 != 0) {
        VIS_WARN("lines.size() is wrong! {0}", lines_size);
        return Handle();
    }

    if (size <= 0) {
        VIS_WARN("line size is wrong! {0}", size);
        return Handle();
    }

    if (colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return Handle();
    }

    const int color_channels = colors_size % 3 == 0 ? 3 : 4;
    const int numcolors = colors_size / color_channels;
    if (numcolors != 1 && numcolors != (lines_size / 6)) {
        VIS_WARN("Color number should be 1 or the same with line number!");
        return Handle();
    }

    std::vector<float> vert_colors;
    if (numcolors == 1) {
        vert_colors = colors;
    } else {
        vert_colors.resize(numcolors * 2);
        for (int i = 0; i < numcolors; ++i) {
            const auto &c = colors[i];
            vert_colors[i * 2 + 0] = c;
            vert_colors[i * 2 + 1] = c;
        }
    }

    c.type = CommandType_PlotLine;
    c.val.f = size;
    c.data1 = lines.data();
    c.size1 = lines_size;
    c.data2 = vert_colors.data();
    c.size2 = vert_colors.size();

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = colors;
        return c.ret.h;
    }
    return Handle();
}

static bool GenerateGridMesh(float xlenth, float ylenth, int half_x_num_cells, int half_y_num_cells,
                             std::vector<float> &vertices, std::vector<unsigned int> &indices) {
    if (xlenth <= 0 || ylenth <= 0 || half_x_num_cells < 1 || half_y_num_cells < 1) {
        VIS_ERROR("Invalid parameters! All parameters should be positive. {}, {}, {}, {}", xlenth,
                  ylenth, half_x_num_cells, half_y_num_cells);
        return false;
    }

    const int64_t max_cell_num = 40000000;
    int64_t all_num_cells = (int64_t)half_x_num_cells * (int64_t)half_y_num_cells * 4;

    if (all_num_cells > max_cell_num) {
        VIS_ERROR("To much cells! Can not handle {} cells.", all_num_cells);
        return false;
    }

    vertices.clear();
    indices.clear();

    int x_vertices_num = half_x_num_cells * 2 + 1;
    int y_vertices_num = half_y_num_cells * 2 + 1;
    int vertices_num = x_vertices_num * y_vertices_num;
    vertices.resize(vertices_num * 3);
    indices.resize(2 * half_x_num_cells * 2 * half_y_num_cells * 2 * 3);

    size_t pi = 0;
    float x_cell_size = xlenth / (half_x_num_cells * 2);
    float y_cell_size = ylenth / (half_y_num_cells * 2);

    // generate vertices
    for (int y = -half_y_num_cells; y <= half_y_num_cells; ++y) {
        for (int x = -half_x_num_cells; x <= half_x_num_cells; ++x) {
            vertices[pi * 3 + 0] = x * x_cell_size;
            vertices[pi * 3 + 1] = y * y_cell_size;
            vertices[pi * 3 + 2] = 0;
            ++pi;
        }
    }

    // assign indices in anti-clockwise order
    pi = 0;
    for (int j = 0; j < y_vertices_num - 1; j++) {
        for (int i = 0; i < x_vertices_num - 1; ++i) {
            int row1 = j * x_vertices_num;
            int row2 = (j + 1) * x_vertices_num;
            indices[pi++] = row1 + i;
            indices[pi++] = row2 + i + 1;
            indices[pi++] = row2 + i;
            indices[pi++] = row1 + i;
            indices[pi++] = row1 + i + 1;
            indices[pi++] = row2 + i + 1;
        }
    }
    return true;
}

static void TransformVertices(const std::array<float, 3> &trans,
                              const std::array<float, 9> &rotation, std::vector<float> &pts) {
    size_t npts = pts.size() / 3;
    for (size_t i = 0; i < npts; ++i) {
        const float x = pts[i * 3 + 0];
        const float y = pts[i * 3 + 1];
        const float z = pts[i * 3 + 2];
        pts[i * 3 + 0] = rotation[0] * x + rotation[1] * y + rotation[2] * z + trans[0];
        pts[i * 3 + 1] = rotation[3] * x + rotation[4] * y + rotation[5] * z + trans[1];
        pts[i * 3 + 2] = rotation[6] * x + rotation[7] * y + rotation[8] * z + trans[2];
    }
}

static void Quaternion2Matrix(const std::array<float, 4> &quat, std::array<float, 9> &R) {
    float x = quat[0];
    float y = quat[1];
    float z = quat[2];
    float w = quat[3];

    float invNorm;
    invNorm = 1.0f / (float)std::sqrt(w * w + x * x + y * y + z * z);

    w *= invNorm;
    x *= invNorm;
    y *= invNorm;
    z *= invNorm;

    R[0] = 1 - 2 * y * y - 2 * z * z;
    R[1] = 2 * x * y - 2 * w * z;
    R[2] = 2 * x * z + 2 * w * y;
    R[3] = 2 * x * y + 2 * w * z;
    R[4] = 1 - 2 * x * x - 2 * z * z;
    R[5] = 2 * y * z - 2 * w * x;
    R[6] = 2 * x * z - 2 * w * y;
    R[7] = 2 * y * z + 2 * w * x;
    R[8] = 1 - 2 * x * x - 2 * y * y;
}

Handle View::Plane(float xlength, float ylength, int half_x_num_cells, int half_y_num_cells,
                   const std::vector<float> &color) {
    return Plane(xlength, ylength, half_x_num_cells, half_y_num_cells, {0, 0, 0}, {0, 0, 0, 1},
                 color);
}

Handle View::Plane(float xlength, float ylength, int half_x_num_cells, int half_y_num_cells,
                   const std::array<float, 3> &trans, const std::array<float, 4> &quat,
                   const std::vector<float> &color) {
    Handle ret;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    std::array<float, 9> rotation;

    if (color.size() != 3 && color.size() != 4) {
        VIS_WARN("color.size() should be 3 or 4!");
        return ret;
    }

    if (quat[0] == 0 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
        VIS_WARN("quanternion (0, 0, 0, 0) is not valid!");
        return ret;
    }

    Quaternion2Matrix(quat, rotation);

    if (!GenerateGridMesh(xlength, ylength, half_x_num_cells, half_y_num_cells, vertices,
                          indices)) {
        return ret;
    }

    TransformVertices(trans, rotation, vertices);

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotMesh;
    c.data1 = vertices.data();
    c.size1 = vertices.size();
    c.data2 = color.data();
    c.size2 = color.size();
    c.data3 = indices.data();
    c.size3 = indices.size();

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = color;
        return c.ret.h;
    }

    return ret;
}

Handle View::Ground(int halfcells, float cellsize, const std::vector<float> &color) {
    Command &c = m_impl->cmd;
    c.type = CommandType_PlotGround;
    c.buffer = color;
    c.size1 = halfcells;
    c.val.f = cellsize;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        return c.ret.h;
    }
    return Handle();
}

Handle View::Box(const std::array<float, 3> &pos, const std::array<float, 3> &extents,
                 const std::vector<float> &color) {
    if (color.size() != 3 && color.size() != 4) {
        VIS_WARN("color.size() should be 3 or 4!");
        return Handle();
    }

    for (const auto e : extents) {
        if (e <= 0) {
            VIS_WARN("Invalid extent for box: {0}.", e);
            return Handle();
        }
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotBox;
    c.pos = pos;
    c.extents = extents;
    c.buffer = color;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = color;
        return c.ret.h;
    }
    return Handle();
}

Handle View::Cylinder(const std::array<float, 3> &center, float radius, float height,
                      const std::vector<float> &color) {
    if (radius <= 0 || height <= 0) {
        VIS_WARN("Invalid radius or height: {0}, {1}.", radius, height);
        return Handle();
    }

    if (color.size() != 3 && color.size() != 4) {
        VIS_WARN("color.size() should be 3 or 4!");
        return Handle();
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotCylinder;
    c.pos = center;
    c.extents[0] = radius;
    c.extents[1] = height;
    c.buffer = color;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = color;
        return c.ret.h;
    }
    return Handle();
}

Handle View::Sphere(const std::array<float, 3> &center, float radius,
                    const std::vector<float> &color) {
    if (radius <= 0) {
        VIS_WARN("radius should be positive {0}.", radius);
        return Handle();
    }

    if (color.size() != 3 && color.size() != 4) {
        VIS_WARN("color.size() should be 3 or 4!");
        return Handle();
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotSphere;
    c.pos = center;
    c.extents[0] = radius;
    c.buffer = color;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = color;
        return c.ret.h;
    }
    return Handle();
}

Handle View::Spheres(const std::vector<float> &centers, std::vector<float> &radii,
                     const std::vector<float> &colors) {
    Handle h;
    const int centers_size = (int)centers.size();
    const int radii_size = (int)radii.size();
    const int colors_size = (int)colors.size();

    if (centers_size == 0 || centers_size % 3 != 0) {
        VIS_WARN("centers.size() is wrong! {0}", centers_size);
        return h;
    }

    const int num_spheres = centers_size / 3;

    if (radii_size == 0 || (radii_size != 1 && radii_size != num_spheres)) {
        VIS_WARN("radii.size() is wrong! {0}", radii_size);
        return h;
    }

    for (auto &radius : radii) {
        if (radius <= 0) {
            VIS_WARN("radius is wrong! {0}", radius);
            return h;
        }
    }

    if (colors_size == 0 || colors_size % 4 != 0 ||
        (colors_size / 4 != num_spheres && colors_size != 4)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotSpheres;
    c.data1 = centers.data();
    c.size1 = centers_size;
    c.buffer = radii;
    c.data2 = colors.data();
    c.size2 = colors_size;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = colors;
        return c.ret.h;
    }
    return h;
}

Handle View::Cone(const std::array<float, 3> &center, float radius, float height,
                  const std::vector<float> &color) {
    /// NOTE: Center of cone is at the 1/4 height from bottom place inside the cone
    if (radius <= 0 || height <= 0) {
        VIS_WARN("Invalid radius or height: {0}, {1}.", radius, height);
        return Handle();
    }

    if (color.size() != 3 && color.size() != 4) {
        VIS_WARN("color.size() should be 3 or 4!");
        return Handle();
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotCone;
    c.pos = center;
    c.extents[0] = radius;
    c.extents[1] = height;
    c.buffer = color;

    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = color;
        return c.ret.h;
    }

    return Handle();
}

/** Plot an arrow. The tail is the bottom, the head is the sharp side.
 *
 * We use a cylinder and a cone to compose an arrow.
 */
Handle View::Arrow(const std::vector<float> &tails, const std::vector<float> &heads, float radius,
                   const std::vector<float> &colors) {
    Handle h;
    const size_t num_arrow = tails.size() / 3;
    const size_t colors_size = colors.size();

    if (radius <= 0) {
        VIS_WARN("Invalid radius: {0}.", radius);
        return h;
    }

    if (tails.size() == 0 || tails.size() % 3 != 0) {
        VIS_WARN("Invalid tail size! {0}", tails.size());
        return h;
    }

    if (heads.size() == 0 || heads.size() % 3 != 0) {
        VIS_WARN("Invalid head size! {0}", heads.size());
        return h;
    }

    if (tails.size() != heads.size()) {
        VIS_WARN("tail size {0} and head size {0} is not match!", tails.size(), heads.size());
        return h;
    }

    if (colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return h;
    }

    size_t color_channels = 0;
    if (colors_size % 3 == 0 && colors_size % 4 == 0) {
        if (colors_size / 3 == num_arrow) {
            color_channels = 3;
        } else if (colors_size / 4 == num_arrow) {
            color_channels = 4;
        } else {
            VIS_WARN("colors.size [{}] not match arrow size [{}].", colors_size, num_arrow);
            return h;
        }
    } else {
        color_channels = colors_size % 3 == 0 ? 3 : 4;
    }

    const size_t numcl = colors_size / color_channels;

    if (numcl != 1 && numcl != num_arrow) {
        VIS_ERROR("colors.size [{}] not match arrow size [{}].", numcl, num_arrow);
        return h;
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotArrow;
    c.val.f = radius;
    c.data1 = tails.data();
    c.size1 = tails.size();
    c.data2 = heads.data();
    c.size2 = heads.size();
    if (numcl == 1) {
        c.buffer.resize(num_arrow * color_channels);
        for (int i = 0; i < num_arrow; i++) {
            for (int j = 0; j < color_channels; j++) {
                c.buffer[i * color_channels + j] = colors[j];
            }
        }
    } else {
        c.buffer = colors;
    }
    c.val.i = color_channels;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = colors;
        return c.ret.h;
    }
    return h;
}

Handle View::Mesh(const std::vector<float> &vertices, const std::vector<unsigned int> &indices,
                  const std::vector<float> &colors) {
    const int vertices_size = (int)vertices.size();
    const int indices_size = (int)indices.size();
    const int colors_size = (int)colors.size();
    if (vertices_size == 0 || vertices_size % 3 != 0) {
        VIS_WARN("vertices.size() is wrong! {0}", vertices_size);
        return Handle();
    }
    if (indices_size == 0 || indices_size % 3 != 0) {
        VIS_WARN("indices.size() is wrong! {0}", indices_size);
        return Handle();
    }
    if (colors_size == 0 || (colors_size % 3 != 0 && colors_size % 4 != 0)) {
        VIS_WARN("colors.size is wrong! {0}", colors_size);
        return Handle();
    }
    const int color_channels = colors_size % 3 == 0 ? 3 : 4;
    const int numcolors = colors_size / color_channels;
    if (numcolors != 1 && numcolors != (vertices_size / 3)) {
        VIS_WARN("Color number should be 1 or the same with vertices number!");
        return Handle();
    }

    Command &c = m_impl->cmd;
    c.type = CommandType_PlotMesh;
    c.data1 = vertices.data();
    c.size1 = vertices_size;
    c.data2 = colors.data();
    c.size2 = colors_size;
    c.data3 = indices.data();
    c.size3 = indices_size;
    if (Vis3d_Command_Execute(m_impl->pv3, &c)) {
        m_impl->pv3->node_colors[c.ret.h] = colors;
        return c.ret.h;
    }
    return Handle();
}

}  // namespace Vis

static inline uint64_t NextHandleID(Vis3d *pv3) {
    return ++pv3->uid;  // valid from one
}

static inline uint64_t NextObjectID(Vis3d *pv3) {
    return ++pv3->uid;  // valid from one
}

bool Vis3d_IsInited(const Vis3d *pv3) {
    return pv3->is_inited;
}

bool Vis3d_Init(Vis3d *pv3) {
    // create the thread and make sure it's running
    std::lock_guard<std::mutex> lk(pv3->mtx);

    if (pv3->is_inited) {
        VIS_WARN("Vis3d is aready initialized!");
        return true;
    }

    pv3->done.store(false);
    pv3->rendering_thread = std::make_unique<std::thread>(&Vis3d_Callback, pv3);
    pv3->rendering_thread->detach();
    while (!pv3->entered_loop.load()) {
        Vis::Sleep(5);
    }
    pv3->is_inited = true;
    pv3->picked = Handle();
    pv3->pointnorm = {0};
    return true;
}

void Vis3d_Shutdown(Vis3d *pv3) {
    std::lock_guard<std::mutex> lk(pv3->mtx);
    if (!pv3->is_inited) {
        VIS_WARN("Shutdown not initialized Vis3d.");
        return;
    }

    if (pv3->entered_loop.load() == true && pv3->exited_loop.load() == false) {
        pv3->done.store(true);
        if (pv3->rendering_thread->joinable()) {
            pv3->rendering_thread->join();
        }

        // Close all views and set the viewer to be done.
        for (const auto &hv : pv3->viewmap) {
            pv3->compviewer->removeView(hv.second);
        }
        pv3->viewmap.clear();
        pv3->compviewer->setDone(true);
    } else if (pv3->entered_loop.load() == true && pv3->exited_loop.load() == true) {
        VIS_DEBUG("Rendering loop exited.");
    }

    pv3->entered_loop.store(false);
    pv3->exited_loop.store(false);
    pv3->is_inited = false;
}

void Vis3d_ShutdownShared() {
    Vis3d_Shutdown(&sg_vis3d_shared);
}

void Vis3d_Update(Vis3d *pv3) {
    // Sometimes we have to deal with command emit from internal guys,
    // like window is closed by user clicking the 'X', or UI event which will
    // be added later.In these cases, only the viewer thread could handle it
    // properly. Thus we have this internal update section, no need to acquire lock.
    if (pv3->has_internal_cmd) {
        Command *pc = &(pv3->internal_cmd);
        switch (pc->type) {
        case CommandType_CreateView:
            Vis3d_Command_CreateView(pv3, pc);
            break;
        case CommandType_CloseView:
            Vis3d_Command_CloseView(pv3, pc);
            break;
        default:
            VIS_WARN("Not implemented yet!");
            break;
        }
        pv3->has_internal_cmd = false;
    }

    std::lock_guard<std::mutex> lg(pv3->view_mtx);
    if (pv3->has_cmd) {
        Command *pc = pv3->pcmd;
        if (!pc) {
            VIS_ERROR("pcmd == NULL!");
            return;
        }
        switch (pc->type) {
        case CommandType_Home:
            Vis3d_Command_Home(pv3, pc);
            break;
        case CommandType_CreateView:
            Vis3d_Command_CreateView(pv3, pc);
            break;
        case CommandType_CloseView:
            Vis3d_Command_CloseView(pv3, pc);
            break;
        case CommandType_IsViewClosed:
            Vis3d_Command_IsViewClosed(pv3, pc);
            break;
        case CommandType_GetViewSize:
            Vis3d_Command_GetViewSize(pv3, pc);
            break;
        case CommandType_LoadModel:
            Vis3d_Command_LoadModel(pv3, pc);
            break;
        case CommandType_PlotText:
            Vis3d_Command_PlotText(pv3, pc);
            break;
        case CommandType_SetText:
            Vis3d_Command_SetText(pv3, pc);
            break;
        case CommandType_SetTextFont:
            Vis3d_Command_SetTextFont(pv3, pc);
            break;
        case CommandType_Plot2DQuad:
            Vis3d_Command_Plot2DQuad(pv3, pc);
            break;
        case CommandType_PlotPoint:
            Vis3d_Command_PlotPoint(pv3, pc);
            break;
        case CommandType_PlotLine:
            Vis3d_Command_PlotLine(pv3, pc);
            break;
        case CommandType_SetTransparency:
            Vis3d_Command_SetTransparency(pv3, pc);
            break;
        case CommandType_PlotBox:
            Vis3d_Command_PlotBox(pv3, pc);
            break;
        case CommandType_PlotCylinder:
            Vis3d_Command_PlotCylinder(pv3, pc);
            break;
        case CommandType_PlotSphere:
            Vis3d_Command_PlotSphere(pv3, pc);
            break;
        case CommandType_PlotSpheres:
            Vis3d_Command_PlotSpheres(pv3, pc);
            break;
        case CommandType_PlotCone:
            Vis3d_Command_PlotCone(pv3, pc);
            break;
        case CommandType_PlotArrow:
            Vis3d_Command_PlotArrow(pv3, pc);
            break;
        case CommandType_PlotMesh:
            Vis3d_Command_PlotMesh(pv3, pc);
            break;
        case CommandType_PlotGround:
            Vis3d_Command_PlotGround(pv3, pc);
            break;
        case CommandType_PlotAxes:
            Vis3d_Command_PlotAxes(pv3, pc);
            break;
        case CommandType_SetTransform:
            Vis3d_Command_SetTransform(pv3, pc, 0);
            break;
        case CommandType_SetRotation:
            Vis3d_Command_SetTransform(pv3, pc, 1);
            break;
        case CommandType_SetPosition:
            Vis3d_Command_SetTransform(pv3, pc, 2);
            break;
        case CommandType_GetTransform:
            Vis3d_Command_GetTransform(pv3, pc);
            break;
        case CommandType_ClearNodes:
            Vis3d_Command_ClearNodes(pv3, pc);
            break;
        case CommandType_SetShow:
            Vis3d_Command_SetShow(pv3, pc);
            break;
        case CommandType_DeleteNode:
            Vis3d_Command_DeleteNode(pv3, pc);
            break;
        case CommandType_DeleteNodes:
            Vis3d_Command_DeleteNodes(pv3, pc);
            break;
        case CommandType_HasNode:
            Vis3d_Command_HasNode(pv3, pc);
            break;
        case CommandType_Clone:
            Vis3d_Command_Clone(pv3, pc);
            break;
        case CommandType_CloneMultiple:
            Vis3d_Command_CloneMultiple(pv3, pc);
            break;
        case CommandType_Chain:
            Vis3d_Command_Chain(pv3, pc);
            break;
        case CommandType_Unchain:
            Vis3d_Command_Unchain(pv3, pc);
            break;
        case CommandType_SetTransforms:
            Vis3d_Command_SetTransforms(pv3, pc);
            break;
        case CommandType_SetColor:
            Vis3d_Command_SetColor(pv3, pc);
            break;
        case CommandType_SetHomePose:
            Vis3d_Command_SetHomePose(pv3, pc);
            break;
        case CommandType_GetHomePose:
            Vis3d_Command_GetHomePose(pv3, pc);
            break;
        case CommandType_SetCameraPose:
            Vis3d_Command_SetCameraPose(pv3, pc);
            break;
        case CommandType_GetCameraPose:
            Vis3d_Command_GetCameraPose(pv3, pc);
            break;
        case CommandType_SetObjectAnimation:
            Vis3d_Command_SetObjectAnimation(pv3, pc);
            break;
        case CommandType_SetCameraAnimation:
            Vis3d_Command_SetCameraAnimation(pv3, pc);
            break;
        case CommandType_EnableGizmo:
            Vis3d_Command_EnableGizmo(pv3, pc);
            break;
        case CommandType_DisableGizmo:
            Vis3d_Command_DisableGizmo(pv3, pc);
            break;
        case CommandType_SetGizmoDetectionRange:
            Vis3d_Command_SetGizmoDetectionRange(pv3, pc);
            break;
        case CommandType_SetGizmoType:
            Vis3d_Command_SetGizmoType(pv3, pc);
            break;
        case CommandType_SetGizmoDisplayScale:
            Vis3d_Command_SetGizmoDisplayScale(pv3, pc);
            break;
        case CommandType_SetGizmoDrawMask:
            Vis3d_Command_SetGizmoDrawMask(pv3, pc);
            break;
        default:
            VIS_WARN("Not implemented yet!");
            break;
        }
        pv3->has_cmd = false;
    }
    pv3->view_cv.notify_one();
}

void Vis3d_Cleanup(Vis3d *pv3) {
    (void)pv3;
}

bool Vis3d_Command_Execute(Vis3d *pv3, Command *pcmd) {
    if (!pv3)
        return false;
    if (pv3->is_inited) {
        std::unique_lock<std::mutex> ul(pv3->view_mtx);
        pv3->pcmd = pcmd;
        pv3->has_cmd = true;
        pv3->view_cv.wait(ul, [&]() { return !pv3->has_cmd; });
        return pcmd->success;
    } else {
        VIS_WARN("Vis is not initialized! Maybe you have called SystemShutdown.");
        return false;
    }
}

void Vis3d__RealizeWindows(Vis3d *pv3) {
    if (!pv3)
        return;

    osgViewer::CompositeViewer::Windows windows;
    pv3->compviewer->stopThreading();
    {
        pv3->compviewer->getWindows(windows);
        for (const auto &it : windows) {
            if (!it->isRealizedImplementation()) {
                it->realizeImplementation();
            }
        }
    }
    if (windows.size() > 0) {
        pv3->compviewer->setDone(false);
    }
    pv3->compviewer->startThreading();
}

class View3DEventHandler : public osgGA::GUIEventHandler {
public:
    explicit View3DEventHandler(Vis3d *pv3, const Handle vh) : m_pv3(pv3), m_vh(vh) {}

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &) {
        if (!m_pv3)
            return false;

        switch (ea.getEventType()) {
        case osgGA::GUIEventAdapter::CLOSE_WINDOW: {
            VIS_WARN("User clicked the X to close the window.");
            // Let the internal update to handle
            m_pv3->internal_cmd.type = CommandType_CloseView;
            m_pv3->internal_cmd.who = m_vh;
            m_pv3->has_internal_cmd = true;
            return true;
        }
        default:
            break;
        }
        return false;
    }

private:
    Vis3d *m_pv3;
    Handle m_vh;
};

void Vis3d_Command_Home(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
    } else {
        pv3->viewmap[who]->home();
        pc->success = true;
        pc->ret.b = true;
    }
}

void Vis3d_Command_CreateView(Vis3d *pv3, Command *pc) {
    Handle h;
    osg::GraphicsContext::WindowingSystemInterface *wsi =
        osg::GraphicsContext::getWindowingSystemInterface();

    if (!wsi) {
        VIS_ERROR("Error, no WindowSystemInterface available, cannot create windows.");
        pc->success = false;
        pc->ret.h = Handle();
        return;
    }

    ViewConfig &cfg = pc->viewcfg;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = cfg.name;
    traits->x = cfg.x;
    traits->y = cfg.y;
    traits->width = cfg.width;
    traits->height = cfg.height;
    traits->windowDecoration = cfg.use_decoration;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    traits->screenNum = cfg.screen_num;
    traits->readDISPLAY();

    traits->setUndefinedScreenDetailsToDefaultScreen();

    osg::ref_ptr<osg::GraphicsContext> gc =
        osg::GraphicsContext::createGraphicsContext(traits.get());
    if (gc.valid()) {
        gc->setClearColor(osg::Vec4f{0, 0, 0, 1});
        gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    } else {
        VIS_ERROR("GraphicsWindow failed to create.");
        pc->success = false;
        pc->ret.h = Handle();
        return;
    }

    // view one
    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    view->setName(cfg.name);
    osg::ref_ptr<osg::Camera> pcam = view->getCamera();
    pcam->setName(cfg.name);

    pcam->setClearColor({cfg.bgcolor[0], cfg.bgcolor[1], cfg.bgcolor[2], cfg.bgcolor[3]});
    pcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    pcam->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    pcam->setCullingMode(pcam->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
    pcam->setGraphicsContext(gc.get());
    view->setCameraManipulator(
        new TouchballManipulator(&(pv3->gizmo.capture), &(pv3->gizmo.view_manipulation)));

    // Add the state manipulator
    osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;
    statesetManipulator->setStateSet(view->getCamera()->getOrCreateStateSet());

    // add GUI Event handler
    view->addEventHandler(statesetManipulator.get());
    view->addEventHandler(new osgViewer::StatsHandler);
    view->addEventHandler(new osgViewer::HelpHandler);
    view->addEventHandler(new osgViewer::WindowSizeHandler);
    view->addEventHandler(new PickHandler(pv3));

    h.type = ViewObjectType_View;
    h.uid = NextHandleID(pv3);

    view->setName(std::to_string(NextObjectID(pv3)));

    view->addEventHandler(new View3DEventHandler(pv3, h));
    view->setSceneData(pv3->scene_root);

    pc->success = true;
    pc->ret.h = h;

    pv3->compviewer->addView(view);
    pv3->viewmap.insert({h, view});

    if (pv3->compviewer->done()) {
        Vis3d__RealizeWindows(pv3);
    }

    if (pv3->camera_2d != nullptr) {
        pv3->camera_2d->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        pv3->camera_2d->setProjectionMatrixAsOrtho2D(0, traits->width, 0, traits->height);
        pv3->camera_2d->setViewMatrix(osg::Matrix::identity());
        pv3->camera_2d->setClearMask(GL_DEPTH_BUFFER_BIT);
        pv3->camera_2d->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }

    return;
}

bool Vis3d__HasNode(const Vis3d *pv3, const Handle &nh) {
    return pv3->nodemap.find(nh) != pv3->nodemap.end();
}
bool Vis3d__Has2DDrawable(const Vis3d *pv3, const Handle &nh) {
    return pv3->map_2d.find(nh) != pv3->map_2d.end();
}

bool Vis3d__HasView(const Vis3d *pv3, const Handle &vh) {
    return pv3->viewmap.find(vh) != pv3->viewmap.end();
}

void Vis3d_Command_Chain(Vis3d *pv3, Command *pc) {
    const std::vector<Handle> &refhs = pc->handles;
    for (const auto &h : refhs) {
        if (!Vis3d__HasNode(pv3, h)) {
            VIS_ERROR("One of the object could not be found!");
            return;
        }
    }

    for (int i = 1; i < (int)refhs.size(); ++i) {
        const auto mt = pv3->nodemap[refhs[i]];
        // Unchain from its parents
        const int np = mt->getNumParents();
        for (int j = 0; j < np; ++j) {
            mt->getParent(j)->asGroup()->removeChild(mt);
        }
        // Chain
        pv3->nodemap[refhs[i - 1]]->addChild(mt);
    }

    pc->ret.b = true;
    pc->success = true;
}

void Vis3d_Command_Unchain(Vis3d *pv3, Command *pc) {
    const std::vector<Handle> &refhs = pc->handles;
    for (const auto &h : refhs) {
        if (!Vis3d__HasNode(pv3, h)) {
            VIS_ERROR("One of the object could not by find!");
            return;
        }
    }

    for (int i = 1; i < (int)refhs.size(); ++i) {
        auto &prev = pv3->nodemap[refhs[i - 1]];
        auto &curr = pv3->nodemap[refhs[i]];
        if (!prev->removeChild(curr)) {
            VIS_ERROR("No link between links[{0}] and links[{1}].", i - 1, i);
        } else {
            // Here we should not lose any nodes, so after unchain, if a node is orphan, we need to
            // give it a parent...
            if (curr->getNumParents() == 0) {
                pv3->node_switch->addChild(curr);
            }
        }
    }

    pc->ret.b = true;
    pc->success = true;
}

void Vis3d_Command_SetTransforms(Vis3d *pv3, Command *pc) {
    pc->ret.b = false;
    pc->success = false;

    if (pc->handles.size() == 0) {  // single mode
        // we need to check the base link has the right structure to perform this operations:
        // 1. base link has enough children and each parent has only one kid following the line.
        const Handle h = pc->who;
        const int tsz = pc->transforms.size();
        if (!Vis3d__HasNode(pv3, h)) {
            VIS_ERROR("Could not find object: type: {0}, uid: {1}", h.type, h.uid);
            return;
        }

        // For the following, check:
        auto mt = pv3->nodemap[h];
        int i = 1;
        for (; i < tsz; ++i) {
            // each MatrixTransform will at least have its own child.
            // It another object is attached to it. It will have a child with type
            // osg::MatrixTransform, thus we the number of children should be 2
            if (mt->getNumChildren() == 2) {
                mt = dynamic_cast<osg::MatrixTransform *>(mt->getChild(1));
                if (!mt) {
                    VIS_ERROR("Invalid node!!!");
                    return;
                }
            } else {
                break;
            }
        }

        if (i < tsz) {
            VIS_ERROR("Object doesn't have enough single descendant(s)!");
            return;
        }

        // Set the transform
        mt = pv3->nodemap[h];
        for (i = 0; i < tsz - 1; ++i) {
            mt->setMatrix(pc->transforms[i]);
            mt = dynamic_cast<osg::MatrixTransform *>(mt->getChild(1));
            if (!mt) {
                VIS_ERROR("Invalid node!!!");
                return;
            }
        }
        mt->setMatrix(pc->transforms[i]);
        if (h == pv3->gizmo.refHandle) {
            for (int i = 0; i < 16; ++i) {
                pv3->gizmo.matrix[i] = *(pc->transforms[0].ptr() + i);
            }
        }

        pc->ret.b = true;
        pc->success = true;
    } else {
        const int hsz = pc->handles.size();
        if (hsz != (int)pc->transforms.size()) {
            VIS_ERROR(
                "handles.size() != transforms.size(). We should've checked before, shouldn't reach "
                "here!");
            return;
        }

        for (int i = 0; i < hsz; ++i) {
            const Handle h = pc->handles[i];
            if (!Vis3d__HasNode(pv3, h)) {
                VIS_WARN("Could not find object: type: {0}, uid: {1}", h.type, h.uid);
            } else {
                pv3->nodemap[h]->setMatrix(pc->transforms[i]);
                if (h == pv3->gizmo.refHandle) {
                    for (int i = 0; i < 16; ++i) {
                        pv3->gizmo.matrix[i] = *(pc->transforms[0].ptr() + i);
                    }
                }
            }
        }

        pc->ret.b = true;
        pc->success = true;
    }
}

void Vis3d_Command_SetShow(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasNode(pv3, who)) {
        VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
    } else {
        pv3->node_switch->setChildValue(pv3->nodemap[who], pc->val.b);
        pc->ret.b = true;
        pc->success = true;
    }
}

void Vis3d_Command_SetTransparency(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasNode(pv3, who)) {
        VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
        return;
    }

    const auto mt = pv3->nodemap[who];
    const float alpha = pc->val.f;
    if (who.type == ViewObjectType_Model) {
        osg::Node *node = mt->getChild(0);
        if (node == nullptr) {
            VIS_ERROR("No child for the node, which should be impossible! ({0}, {1})", who.type,
                      who.uid);
            pc->ret.b = false;
            pc->success = false;
            return;
        }
        auto ss = node->getOrCreateStateSet();
        ss->setMode(GL_BLEND, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
        osg::Material *material = (osg::Material *)ss->getAttribute(osg::StateAttribute::MATERIAL);
        if (!material) {
            material = new osg::Material;
        }

        material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);

        if (alpha >= 1.0f) {
            ss->setRenderingHint(osg::StateSet::OPAQUE_BIN);
            ss->setAttributeAndModes(material,
                                     osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
        } else {
            ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            ss->setAttributeAndModes(material,
                                     osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
        }

        pc->ret.b = true;
        pc->success = true;
    } else if (who.type == ViewObjectType_Mesh) {  // Geometry
        osg::Geode *gnode = static_cast<osg::Geode *>(mt->getChild(0));
        if (gnode == nullptr) {
            VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})", who.type,
                      who.uid);
            pc->ret.b = false;
            pc->success = false;
        } else {
            osg::Geometry *geom = static_cast<osg::Geometry *>(gnode->getDrawable(0));
            if (geom == nullptr) {
                VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})",
                          who.type, who.uid);
                pc->ret.b = false;
                pc->success = false;
            } else {
                auto temp = geom->getColorArray();
                auto data_type = temp->getType();
                if (std::abs(alpha - 1) < std::numeric_limits<double>::epsilon()) {
                    osg::Vec3Array *colours = new osg::Vec3Array(temp->getNumElements());
                    if (data_type == osg::Array::Vec3ArrayType) {
                        colours = dynamic_cast<osg::Vec3Array *>(temp);
                    } else {
                        osg::Vec4Array *colours_old = dynamic_cast<osg::Vec4Array *>(temp);
                        for (size_t i = 0; i < colours->size(); i++) {
                            (*colours)[i] = osg::Vec3((*colours_old)[i].r(), (*colours_old)[i].g(),
                                                      (*colours_old)[i].b());
                        }
                    }
                    auto state = geom->getOrCreateStateSet();
                    state->setMode(GL_BLEND, osg::StateAttribute::OFF);
                    state->setMode(GL_LIGHTING,
                                   osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
                    geom->setColorArray(colours);

                } else {
                    osg::Vec4Array *colours = new osg::Vec4Array(temp->getNumElements());
                    if (data_type == osg::Array::Vec3ArrayType) {
                        osg::Vec3Array *colours_old = dynamic_cast<osg::Vec3Array *>(temp);
                        for (size_t i = 0; i < colours->size(); i++) {
                            (*colours)[i] = osg::Vec4((*colours_old)[i].x(), (*colours_old)[i].y(),
                                                      (*colours_old)[i].z(), alpha);
                        }
                    } else {
                        osg::Vec4Array *colours_old = dynamic_cast<osg::Vec4Array *>(temp);
                        for (size_t i = 0; i < colours->size(); i++) {
                            (*colours)[i] = (*colours_old)[i];
                            (*colours)[i].a() = alpha;
                        }
                    }
                    // geom->setColorArray(colours, osg::Array::BIND_PER_VERTEX);
                    auto state = geom->getOrCreateStateSet();
                    state->setMode(GL_LIGHTING,
                                   osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
                    state->setMode(GL_BLEND,
                                   osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
                    // state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
                    auto material =
                        (osg::Material *)state->getAttribute(osg::StateAttribute::MATERIAL);
                    if (!material) {
                        material = new osg::Material;
                    }
                    material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
                    material->setDiffuse(osg::Material::FRONT_AND_BACK, (*colours)[0]);
                    material->setAmbient(osg::Material::FRONT_AND_BACK, (*colours)[0]);
                    state->setAttributeAndModes(
                        material, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
                    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                    geom->setColorArray(colours);
                }
                pc->ret.b = true;
                pc->success = true;
            }
        }
    } else {
        // ShapeDrawables
        osg::Geode *gnode = static_cast<osg::Geode *>(mt->getChild(0));
        pc->ret.b = false;
        pc->success = false;

        if (gnode == nullptr) {
            VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})", who.type,
                      who.uid);
            return;
        } else {
            osg::ShapeDrawable *sd = static_cast<osg::ShapeDrawable *>(gnode->getDrawable(0));
            if (sd == nullptr) {
                VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})",
                          who.type, who.uid);
                return;
            } else {
                sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                osg::Vec4 color = sd->getColor();
                color[3] = alpha;
                sd->setColor(color);
            }
        }
        pc->ret.b = true;
        pc->success = true;
    }
}

void Vis3d_Command_SetColor(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasNode(pv3, who)) {
        VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
        return;
    }

    const std::vector<float> color = pc->buffer;
    const int color_size = color.size();
    if (color_size != 3 && color_size != 4) {
        VIS_ERROR("Color should size 3 or 4, color.size() == {0}!", color_size);
        return;
    }

    if (who.type == ViewObjectType_Model) {
        const osg::ref_ptr<osg::MatrixTransform> mt = pv3->nodemap[who];
        osg::Node *node = mt->getChild(0);
        if (node == nullptr)
            return;

        auto ss = node->getOrCreateStateSet();
        ss->setMode(GL_BLEND, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
        osg::Material *material = (osg::Material *)ss->getAttribute(osg::StateAttribute::MATERIAL);
        if (!material) {
            material = new osg::Material;
        }

        osg::Vec4d orig_color = material->getDiffuse(osg::Material::FRONT_AND_BACK);

        for (int i = 0; i < color_size; ++i) {
            orig_color[i] = color[i];
        }

        material->setDiffuse(osg::Material::FRONT_AND_BACK, orig_color);

        const float alpha = orig_color[3];

        if (alpha >= 1.0f) {
            ss->setRenderingHint(osg::StateSet::OPAQUE_BIN);
            ss->setAttributeAndModes(material,
                                     osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
        } else {
            ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            ss->setAttributeAndModes(material,
                                     osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
        }
        pc->ret.b = true;
        pc->success = true;
    } else if (who.type == ViewObjectType_Mesh) {
        /// Geometry
        const osg::ref_ptr<osg::MatrixTransform> mt = pv3->nodemap[who];
        osg::Geode *gnode = static_cast<osg::Geode *>(mt->getChild(0));
        if (gnode == nullptr) {
            VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})", who.type,
                      who.uid);
            pc->ret.b = false;
            pc->success = false;
        } else {
            osg::Geometry *geom = static_cast<osg::Geometry *>(gnode->getDrawable(0));
            if (geom == nullptr) {
                VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})",
                          who.type, who.uid);
                pc->ret.b = false;
                pc->success = false;
            } else {
                geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                osg::Vec4Array *colours = new osg::Vec4Array(1);
                if (color_size == 3) {
                    (*colours)[0].set(color[0], color[1], color[2], 1.0f);
                } else {
                    (*colours)[0].set(color[0], color[1], color[2], color[3]);
                }
                geom->setColorArray(colours, osg::Array::BIND_OVERALL);
                pc->ret.b = true;
                pc->success = true;
            }
        }
    } else if (who.type == ViewObjectType_Axes) {
        VIS_WARN("Setting color to axes type will be ignored!");
        pc->ret.b = true;
        pc->success = true;
        return;
    } else {
        /// ShapeDrawables
        const osg::ref_ptr<osg::MatrixTransform> mt = pv3->nodemap[who];
        osg::Geode *gnode = static_cast<osg::Geode *>(mt->getChild(0));
        if (gnode == nullptr) {
            VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})", who.type,
                      who.uid);
            pc->ret.b = false;
            pc->success = false;
        } else {
            osg::ShapeDrawable *sd = static_cast<osg::ShapeDrawable *>(gnode->getDrawable(0));
            if (sd == nullptr) {
                VIS_ERROR("No child for the geode, which should be impossible! ({0}, {1})",
                          who.type, who.uid);
                pc->ret.b = false;
                pc->success = false;
            } else {
                sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                osg::Vec4d orig_color = sd->getColor();
                for (int i = 0; i < color_size; ++i) {
                    orig_color[i] = color[i];
                }

                sd->setColor(orig_color);
                pc->ret.b = true;
                pc->success = true;
            }
        }
    }
}

void Vis3d_Command_DeleteNode(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (who.type != ViewObjectType_2D) {
        if (!Vis3d__HasNode(pv3, who)) {
            VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
            pc->ret.b = false;
        } else {
            pv3->node_switch->removeChild(pv3->nodemap[who]);
            pv3->nodemap.erase(who);
            pv3->node_colors.erase(who);
            pc->ret.b = true;
        }
    } else {
        if (!Vis3d__Has2DDrawable(pv3, who)) {
            VIS_ERROR("Can not find 2D Drawable: type: {0}, uid: {1}.", who.type, who.uid);
            pc->ret.b = false;
        } else {
            pv3->switch_2d->removeChild(pv3->map_2d[who]);
            pv3->map_2d.erase(who);
            pc->ret.b = true;
        }
    }
}

void Vis3d_Command_DeleteNodes(Vis3d *pv3, Command *pc) {
    for (auto who : pc->handles) {
        if (who.type != ViewObjectType_2D) {
            if (!Vis3d__HasNode(pv3, who)) {
                VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
                pc->ret.b = false;
            } else {
                pv3->node_switch->removeChild(pv3->nodemap[who]);
                pv3->nodemap.erase(who);
                pv3->node_colors.erase(who);
                pc->ret.b = true;
            }
        } else {
            if (!Vis3d__Has2DDrawable(pv3, who)) {
                VIS_ERROR("Can not find 2D Drawable: type: {0}, uid: {1}.", who.type, who.uid);
                pc->ret.b = false;
            } else {
                pv3->switch_2d->removeChild(pv3->map_2d[who]);
                pv3->map_2d.erase(who);
                pc->ret.b = true;
            }
        }
    }
}

void Vis3d_Command_HasNode(const Vis3d *pv3, Command *pc) {
    pc->success = true;
    pc->ret.b = Vis3d__HasNode(pv3, pc->who);
}

void Vis3d_Command_Clone(Vis3d *pv3, Command *pc) {
    // Clone mt pointing to the same object
    if (pc == nullptr) {
        VIS_ERROR("pc is nullptr!");
        pc->success = false;
        pc->ret.b = false;
        return;
    }

    const Handle h = pc->who;
    osg::ref_ptr<osg::MatrixTransform> mt =
        dynamic_cast<osg::MatrixTransform *>(pv3->nodemap[h]->clone(osg::CopyOp::DEEP_COPY_ALL));
    mt->setMatrix(pc->transform);
    Handle dst;
    dst.type = h.type;
    dst.uid = NextHandleID(pv3);
    mt->setName(std::string("mt") + std::to_string(NextObjectID(pv3)));
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({dst, mt});
    pc->success = true;
    pc->ret.b = true;
    pc->ret.h = dst;
}

void Vis3d_Command_CloneMultiple(Vis3d *pv3, Command *pc) {
    // Clone mt pointing to the same object
    if (pc == nullptr) {
        VIS_ERROR("pc is nullptr!");
        pc->success = false;
        pc->ret.b = false;
        return;
    }

    std::vector<Handle> newhs;

    for (int i = 0; pc->handles.size(); ++i) {
        auto &h = pc->handles[i];
        osg::ref_ptr<osg::MatrixTransform> mt = dynamic_cast<osg::MatrixTransform *>(
            pv3->nodemap[h]->clone(osg::CopyOp::DEEP_COPY_ALL));
        mt->setMatrix(pc->transforms[i]);
        Handle dst;
        dst.type = h.type;
        dst.uid = NextHandleID(pv3);
        mt->setName(std::string("mt") + std::to_string(NextObjectID(pv3)));
        pv3->node_switch->addChild(mt);
        pv3->nodemap.insert({dst, mt});
        newhs.push_back(dst);
    }

    pc->success = true;
    pc->ret.b = true;
    pc->handles = newhs;
}

void Vis3d_Command_SetHomePose(Vis3d *pv3, Command *pc) {
    pc->success = true;
    pc->ret.b = false;

    const Handle who = pc->who;
    const auto &eye = pc->extents;
    const auto &point = pc->pos;
    const auto &upvec = pc->upvector;

    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}", who.type, who.uid);
        // FIXME(Hui): originally, I designed this *success* to indicate the status of
        // this command function, ret to be the real return value, but now, it's always
        // confusing me. Need to better way ?
        pc->success = false;
        pc->ret.b = false;
    } else {
        pv3->viewmap[who]->getCameraManipulator()->setHomePosition(
            {eye[0], eye[1], eye[2]}, {point[0], point[1], point[2]},
            {upvec[0], upvec[1], upvec[2]}, false);

        pc->success = true;
        pc->ret.b = true;
    }
}

void Vis3d_Command_GetHomePose(Vis3d *pv3, Command *pc) {
    pc->success = true;
    pc->ret.b = false;

    const Handle who = pc->who;

    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}", who.type, who.uid);
        pc->success = false;
        pc->ret.b = false;
    } else {
        osg::Vec3d eye, point, up;
        pv3->viewmap[who]->getCameraManipulator()->getHomePosition(eye, point, up);

        pc->ret.data.resize(9);
        pc->ret.data[0] = eye[0];
        pc->ret.data[1] = eye[1];
        pc->ret.data[2] = eye[2];
        pc->ret.data[3] = point[0];
        pc->ret.data[4] = point[1];
        pc->ret.data[5] = point[2];
        pc->ret.data[6] = up[0];
        pc->ret.data[7] = up[1];
        pc->ret.data[8] = up[2];
        pc->success = true;
        pc->ret.b = true;
    }
}

void Vis3d_Command_SetCameraPose(Vis3d *pv3, Command *pc) {
    pc->success = true;
    pc->ret.b = false;

    const Handle who = pc->who;
    const auto &eye = pc->extents;
    const auto &point = pc->pos;
    const auto &upvec = pc->upvector;

    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}", who.type, who.uid);
        pc->success = false;
        pc->ret.b = false;
    } else {
        osg::Matrixd vm;
        vm.makeLookAt({eye[0], eye[1], eye[2]}, {point[0], point[1], point[2]},
                      {upvec[0], upvec[1], upvec[2]});
        pv3->viewmap[who]->getCameraManipulator()->setByInverseMatrix(vm);
        pc->success = true;
        pc->ret.b = true;
    }
}

void Vis3d_Command_GetCameraPose(Vis3d *pv3, Command *pc) {
    pc->success = true;
    pc->ret.b = false;

    const Handle who = pc->who;

    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}", who.type, who.uid);
        pc->success = false;
        pc->ret.b = false;
    } else {
        osg::Vec3d eye, point, up;

        osg::Matrixd vm = pv3->viewmap[who]->getCameraManipulator()->getInverseMatrix();
        vm.getLookAt(eye, point, up, 1.0);

        pc->ret.data.resize(9);
        pc->ret.data[0] = eye[0];
        pc->ret.data[1] = eye[1];
        pc->ret.data[2] = eye[2];
        pc->ret.data[3] = point[0];
        pc->ret.data[4] = point[1];
        pc->ret.data[5] = point[2];
        pc->ret.data[6] = up[0];
        pc->ret.data[7] = up[1];
        pc->ret.data[8] = up[2];
        pc->success = true;
        pc->ret.b = true;
    }
}

void Vis3d_Command_SetObjectAnimation(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasNode(pv3, who)) {
        VIS_ERROR("Can not find node: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
        return;
    }

    auto mt = pv3->nodemap[who];
    mt->setUpdateCallback(NULL);
    if (pc->val.b) {
        osg::ref_ptr<osg::AnimationPathCallback> apcb = new osg::AnimationPathCallback;
        osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
        osg::AnimationPath::LoopMode loop = (osg::AnimationPath::LoopMode)pc->val.i;
        float time = pc->val.f;
        unsigned int pos_size = pc->size1;
        unsigned int quat_size = pc->size2;
        if (pos_size == 0 || quat_size == 0) {
            const osg::Vec3 center = -mt->getBound().center();
            osg::Matrix m = mt->getMatrix();
            const unsigned int numSamples = 32;
            float delta_yaw = 2.0f * osg::PI / ((float)numSamples - 1.0f);
            float delta_time = time / (float)numSamples;
            for (unsigned int i = 0; i < numSamples; ++i) {
                float yaw = delta_yaw * (float)i;
                osg::Quat rot(-yaw, osg::Z_AXIS);
                auto real_rotate = rot * m.getRotate().inverse();
                auto real_matrix = m * osg::Matrix::translate(-center) *
                                   osg::Matrix::rotate(real_rotate) *
                                   osg::Matrix::translate(center);
                path->insert(delta_time * (float)i,
                             osg::AnimationPath::ControlPoint(real_matrix.getTrans(),
                                                              real_matrix.getRotate()));
            }
        } else {
            const osg::Vec3 *pos_list = (const osg::Vec3 *)(pc->data1);
            const osg::Vec4 *quat_list = (const osg::Vec4 *)(pc->data2);
            const unsigned int num_samples = pos_size;
            const float delta_time = time / num_samples;
            for (unsigned int i = 0; i < num_samples; i++) {
                osg::Quat rot = {quat_list[i][0], quat_list[i][1], quat_list[i][2],
                                 quat_list[i][3]};
                path->insert(delta_time * i, osg::AnimationPath::ControlPoint(pos_list[i], rot));
            }
        }

        path->setLoopMode(loop);
        apcb->setAnimationPath(path);
        mt->setUpdateCallback(apcb);
    }

    pc->ret.b = true;
    pc->success = true;
}

void Vis3d_Command_SetCameraAnimation(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
        return;
    }

    auto view = pv3->viewmap[who];

    if (pc->val.b) {
        osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
        osg::AnimationPath::LoopMode loop = (osg::AnimationPath::LoopMode)pc->val.i;
        path->setLoopMode(loop);

        float time = pc->val.f;
        unsigned int pos_size = pc->size1;
        unsigned int quat_size = pc->size2;

        if (pos_size == 0 || quat_size == 0) {
            osg::Vec3 pos;
            osg::Quat rot;
            osg::Matrix m;
            auto cam = view->getCameraManipulator();
            osg::Matrix vm_inv = cam->getInverseMatrix();
            std::vector<osg::Vec3> line1 = {
                {1.0000, 0.0000, 0.0000},   {0.7071, 0.0000, -0.7071}, {0.0000, 0.0000, -1.0000},
                {-0.7071, 0.0000, -0.7071}, {-1.0000, 0.0000, 0.0000}, {-0.7071, 0.0000, 0.7071},
                {0.0000, 0.0000, 1.0000},   {0.7071, 0.0000, 0.7071},  {1.0000, 0.0000, 0.0000}};
            std::vector<osg::Vec3> line2 = {
                {0.0000, 0.0000, -1.0000}, {-0.7071, 0.0000, -0.7071}, {-1.0000, 0.0000, 0.0000},
                {-0.7071, 0.0000, 0.7071}, {0.0000, 0.0000, 1.0000},   {0.7071, 0.0000, 0.7071},
                {1.0000, 0.0000, 0.0000},  {0.7071, 0.0000, -0.7071},  {0.0000, 0.0000, -1.0000}};

            const unsigned int numSamples = 9;
            double delta_time = time / numSamples;
            for (unsigned int i = 0; i < numSamples; ++i) {
                for (int j = 0; j < 3; j++) {
                    vm_inv(0, j) = line1[i][j];
                    vm_inv(1, j) = line2[i][j];
                }
                m.invert(vm_inv);
                path->insert(delta_time * i,
                             osg::AnimationPath::ControlPoint(m.getTrans(), m.getRotate()));
            }
        } else {
            const osg::Vec3 *pos_list = (const osg::Vec3 *)(pc->data1);
            const osg::Vec4 *quat_list = (const osg::Vec4 *)(pc->data2);
            const unsigned int num_samples = pos_size;
            const double delta_time = time / num_samples;
            for (unsigned int i = 0; i < num_samples; i++) {
                osg::Matrix m(quat_list[i]);
                m.setTrans(pos_list[i]);
                m.invert(m);
                path->insert(delta_time * i,
                             osg::AnimationPath::ControlPoint(m.getTrans(), m.getRotate()));
            }
        }

        osg::ref_ptr<osgGA::AnimationPathManipulator> apm =
            new osgGA::AnimationPathManipulator(path.get());
        apm->home(view->getFrameStamp()->getReferenceTime());
        view->setCameraManipulator(apm);
    } else {
        view->setCameraManipulator(new osgGA::TrackballManipulator);
    }

    pc->ret.b = true;
    pc->success = true;
}

bool Vis3d_Command_ClearNodes(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!Vis3d__HasView(pv3, who)) {
        VIS_ERROR("Can not find view: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        pc->success = false;
    } else {
        const int num = pv3->node_switch->getNumChildren();
        const int num_2d = pv3->switch_2d->getNumChildren();
        pv3->node_switch->removeChildren(0, num);
        pv3->nodemap.clear();
        pv3->node_colors.clear();
        pv3->switch_2d->removeChildren(0, num_2d);
        pv3->map_2d.clear();
        pc->ret.b = true;
        pc->success = true;
    }
    return pc->success;
}

void Vis3d_Command_CloseView(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (!pv3->viewmap.empty()) {
        if (!Vis3d__HasView(pv3, who)) {
            VIS_ERROR("Can not find view: type: {0}, uid: {1}.", who.type, who.uid);
            pc->success = false;
        } else {
            pv3->compviewer->removeView(pv3->viewmap[who]);
            pv3->viewmap.erase(who);
            pc->success = true;
        }
    }
}

void Vis3d_Command_IsViewClosed(const Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (Vis3d__HasView(pv3, who)) {
        pc->ret.b = false;
    } else {
        pc->ret.b = true;
    }
}

void Vis3d_Command_GetViewSize(Vis3d *pv3, Command *pc) {
    const Handle who = pc->who;
    if (Vis3d__HasView(pv3, who)) {
        pc->ret.b = false;
    }
    auto vp = pv3->viewmap[who]->getCamera()->getViewport();
    pc->ret.data.resize(2);
    pc->ret.data[0] = vp->width();
    pc->ret.data[1] = vp->height();
    pc->ret.b = true;
}

// flag == 0 : update transform
// flag == 1 : update rotation
// flag == 2 : update translation
void Vis3d_Command_SetTransform(Vis3d *pv3, Command *pc, int flag) {
    pc->success = false;
    pc->ret.b = true;

    if (!Vis3d__HasNode(pv3, pc->who)) {
        VIS_ERROR("Can not find the node!");
        pc->ret.b = false;
        return;
    }

    switch (flag) {
    case 0:
        pv3->nodemap[pc->who]->setMatrix(pc->transform);
        if (pc->who == pv3->gizmo.refHandle) {
            for (int i = 0; i < 16; ++i) {
                pv3->gizmo.matrix[i] = *(pc->transform.ptr() + i);
            }
        }
        break;
    case 1: {
        osg::Matrixf m = pv3->nodemap[pc->who]->getMatrix();
        m.setRotate(pc->transform.getRotate());
        pv3->nodemap[pc->who]->setMatrix(m);
        if (pc->who == pv3->gizmo.refHandle) {
            for (int i = 0; i < 16; ++i) {
                pv3->gizmo.matrix[i] = *(pc->transform.ptr() + i);
            }
        }
        break;
    }
    case 2: {
        osg::Matrixf m = pv3->nodemap[pc->who]->getMatrix();
        m.setTrans(pc->transform.getTrans());
        pv3->nodemap[pc->who]->setMatrix(m);
        if (pc->who == pv3->gizmo.refHandle) {
            for (int i = 0; i < 16; ++i) {
                pv3->gizmo.matrix[i] = *(pc->transform.ptr() + i);
            }
        }
        break;
    }
    default:
        pc->ret.b = false;
        VIS_ERROR("Invalid flag {0}!", flag);
        return;
    }

    pc->success = true;
}

void Vis3d_Command_GetTransform(Vis3d *pv3, Command *pc) {
    pc->success = false;
    if (!Vis3d__HasNode(pv3, pc->who)) {
        VIS_ERROR("Can not find the node!");
        return;
    }
    pc->transform = pv3->nodemap[pc->who]->getMatrix();
    pc->success = true;
}

void Vis3d_Command_EnableGizmo(Vis3d *pv3, Command *pc) {
    pc->success = false;
    pc->ret.b = false;

    if (!Vis3d__HasNode(pv3, pc->who)) {
        VIS_ERROR("Can not find the node!");
        return;
    }

    int gizmotype = pc->val.i;

    if (pv3->gizmo.handle.uid != 0) {
        pv3->gizmo.refHandle = Handle();
        pv3->gizmo.capture = false;
        pv3->node_switch->removeChild(pv3->nodemap[pv3->gizmo.handle]);
        pv3->nodemap.erase(pv3->gizmo.handle);
        pv3->gizmo.handle = Handle();
    }

    auto mt = pv3->nodemap[pc->who];
    const osg::Matrix &matrix = mt->getMatrix();
    for (int i = 0; i < 16; ++i) {
        pv3->gizmo.matrix[i] = *(matrix.ptr() + i);
    }
    auto vp = pv3->compviewer->getView(0)->getCamera()->getViewport();

    osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    root->addChild(geode.get());
    geode->setCullingActive(false);  // allow gizmo to always display
    geode->getOrCreateStateSet()->setRenderingHint(
        osg::StateSet::TRANSPARENT_BIN);  // always show at last

    if (gizmotype == 4) {
        osg::ref_ptr<GizmoDrawable> gizmo = new GizmoDrawable;
        gizmo->setCaptureFlag(&pv3->gizmo.capture);
        gizmo->setTransform(mt.get(), pv3->gizmo.matrix);
        gizmo->setGizmoMode(GizmoDrawable::MOVE_GIZMO);
        gizmo->setScreenSize(vp->width(), vp->height());
        geode->addDrawable(gizmo.get());

        gizmo = new GizmoDrawable;
        gizmo->setCaptureFlag(&pv3->gizmo.capture);
        gizmo->setTransform(mt.get(), pv3->gizmo.matrix);
        gizmo->setGizmoMode(GizmoDrawable::ROTATE_GIZMO);
        gizmo->setScreenSize(vp->width(), vp->height());
        geode->addDrawable(gizmo.get());
    } else {
        osg::ref_ptr<GizmoDrawable> gizmo = new GizmoDrawable;
        gizmo->setCaptureFlag(&pv3->gizmo.capture);
        gizmo->setTransform(mt.get(), pv3->gizmo.matrix);
        gizmo->setGizmoMode((GizmoDrawable::Mode)gizmotype);
        gizmo->setScreenSize(vp->width(), vp->height());
        geode->addDrawable(gizmo.get());
    }

    pv3->gizmo.refHandle = pc->who;
    pv3->gizmo.handle.uid = NextHandleID(pv3);
    pv3->gizmo.handle.type = ViewObjectType_Gzimo;
    pv3->gizmo.capture = false;

    pv3->node_switch->insertChild(0, root);
    pv3->nodemap[pv3->gizmo.handle] = root;

    pc->success = true;
    pc->ret.b = true;
}

void Vis3d_Command_DisableGizmo(Vis3d *pv3, Command *pc) {
    if (pv3->gizmo.handle.uid != 0) {
        pv3->gizmo.refHandle = Handle();
        pv3->gizmo.capture = false;
        pv3->node_switch->removeChild(pv3->nodemap[pv3->gizmo.handle]);
        pv3->nodemap.erase(pv3->gizmo.handle);
        pv3->gizmo.handle = Handle();
    }
    pc->success = true;
    pc->ret.b = true;
}

void Vis3d_Command_SetGizmoType(Vis3d *pv3, Command *pc) {
    pc->success = false;
    pc->ret.b = false;

    if (!(pv3->gizmo.handle.uid)) {
        VIS_ERROR("Not enable Gizmo!");
        return;
    }

    int gizmotype = pc->val.i;

    osg::Geode *geode = pv3->nodemap[pv3->gizmo.handle]->getChild(0)->asGeode();
    if (geode->getNumDrawables() > 1) {
        VIS_WARN("No need change type in all mode.");
        return;
    }
    GizmoDrawable *gizmo = dynamic_cast<GizmoDrawable *>(geode->getDrawable(0));
    GizmoDrawable::Mode m = (GizmoDrawable::Mode)gizmotype;

    gizmo->setGizmoMode(m);

    pc->success = true;
    pc->ret.b = true;
}

void Vis3d_Command_SetGizmoDrawMask(Vis3d *pv3, Command *pc) {
    pc->success = false;
    pc->ret.b = false;

    if (!(pv3->gizmo.handle.uid)) {
        VIS_ERROR("Not enable Gizmo!");
        return;
    }

    int gizmotype = pc->val.i;
    unsigned int mask = pc->val.u;
    osg::Geode *geode = pv3->nodemap[pv3->gizmo.handle]->getChild(0)->asGeode();
    for (unsigned int i = 0; i < geode->getNumDrawables(); i++) {
        GizmoDrawable *gizmo = dynamic_cast<GizmoDrawable *>(geode->getDrawable(i));
        if (gizmo->getGizmoMode() == gizmotype) {
            gizmo->setDrawMask(mask);
            break;
        }
    }
    pc->success = true;
    pc->ret.b = true;
}

void Vis3d_Command_SetGizmoDisplayScale(Vis3d *pv3, Command *pc) {
    pc->success = false;
    pc->ret.b = false;

    if (!(pv3->gizmo.handle.uid)) {
        VIS_ERROR("Not enable Gizmo!");
        return;
    }

    float scale = pc->val.f;
    osg::Geode *geode = pv3->nodemap[pv3->gizmo.handle]->getChild(0)->asGeode();
    for (unsigned int i = 0; i < geode->getNumDrawables(); i++) {
        GizmoDrawable *gizmo = dynamic_cast<GizmoDrawable *>(geode->getDrawable(i));
        gizmo->setDisplayScale(scale);
    }

    pc->success = true;
    pc->ret.b = true;
}

void Vis3d_Command_SetGizmoDetectionRange(Vis3d *pv3, Command *pc) {
    pc->success = false;
    pc->ret.b = false;

    if (!(pv3->gizmo.handle.uid)) {
        VIS_ERROR("Not enable Gizmo!");
        return;
    }

    float range = pc->val.f;
    osg::Geode *geode = pv3->nodemap[pv3->gizmo.handle]->getChild(0)->asGeode();
    for (unsigned int i = 0; i < geode->getNumDrawables(); i++) {
        GizmoDrawable *gizmo = dynamic_cast<GizmoDrawable *>(geode->getDrawable(i));
        gizmo->setDetectionRange(range);
    }

    pc->success = true;
    pc->ret.b = true;
}

static inline bool IsEqual(const std::string &a, const std::string &b, bool casesenstive = false) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [&](char a, char b) {
        return casesenstive ? a == b : tolower(a) == tolower(b);
    });
}

/// NOTE: Here we assume the filename does have an extension
static inline bool CheckFileType(const std::string &filename, const std::string &ext) {
    return IsEqual(filename.substr(filename.find_last_of(".") + 1), ext, false);
}

void Vis3d_Command_LoadModel(Vis3d *pv3, Command *pc) {
    const enum LoadModelType lmt = (enum LoadModelType)pc->val.i;

    if (lmt == LoadModelType_ByName || lmt == LoadModelType_ByNameAndTransform) {
        Handle h;
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(pc->name.c_str());
        if (!model) {
            VIS_ERROR("Read model {0} failed!", pc->name);
            pc->ret.h = h;
        } else {
            osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;

            /// STL model doesn't have color information
            if (CheckFileType(pc->name, "stl")) {
                osg::ref_ptr<osg::Material> material = new osg::Material;
                material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 0.5, 1.0));
                model->getOrCreateStateSet()->setAttributeAndModes(
                    material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }

            mt->addChild(model);

            mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));
            model->setName(std::to_string(NextObjectID(pv3)));

            if (lmt == LoadModelType_ByNameAndTransform) {
                mt->setMatrix(pc->transform);
            }

            h.type = ViewObjectType_Model;
            h.uid = NextHandleID(pv3);
            pv3->node_switch->addChild(mt);
            pv3->nodemap.insert({h, mt});

            pc->ret.h = h;
            pc->success = true;
            pv3->node_colors[h] = std::vector<float>{0.5, 0.5, 0.5, 1.0};
        }
    } else if (lmt == LoadModelType_ByNames || lmt == LoadModelType_ByNamesAndTransforms) {
        std::vector<Handle> hs;
        pc->handles.clear();
        for (size_t i = 0; i < pc->names.size(); ++i) {
            const auto &name = pc->names[i];

            osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(name.c_str());
            if (!model) {
                VIS_ERROR("Read model {0} failed!", name);
            } else {
                osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
                mt->addChild(model);
                if (lmt == LoadModelType_ByNamesAndTransforms) {
                    mt->setMatrix(pc->transforms[i]);
                }

                mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));
                model->setName(std::to_string(NextObjectID(pv3)));

                Handle h;
                h.type = ViewObjectType_Model;
                h.uid = NextHandleID(pv3);
                pv3->node_switch->addChild(mt);
                pv3->nodemap.insert({h, mt});
                pv3->node_colors[h] = std::vector<float>{0.5, 0.5, 0.5, 1.0};
                pc->handles.push_back(h);
            }
        }
        pc->success = true;
    } else {
        VIS_ERROR("Should not be here!");
    }
}

void Vis3d_Command_PlotAxes(Vis3d *pv3, Command *pc) {
    const float axis_len = pc->buffer[0];
    const float axis_size = pc->buffer[1];
    Handle h;
    for (const auto &transform : pc->transforms) {
        osg::ref_ptr<osg::Geometry> geo = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array(6);
        (*v)[0] = osg::Vec3f(0.0f, 0.0f, 0.0f);
        (*v)[1] = osg::Vec3f(axis_len, 0.0f, 0.0f);
        (*v)[2] = osg::Vec3f(0.0f, 0.0f, 0.0f);
        (*v)[3] = osg::Vec3f(0.0f, axis_len, 0.0f);
        (*v)[4] = osg::Vec3f(0.0f, 0.0f, 0.0f);
        (*v)[5] = osg::Vec3f(0.0f, 0.0f, axis_len);

        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array(6);
        (*c)[0] = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);
        (*c)[1] = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);  // x red
        (*c)[2] = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
        (*c)[3] = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);  // y green
        (*c)[4] = osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);
        (*c)[5] = osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);  // z blue

        geo->setVertexArray(v);
        geo->setColorArray(c);
        geo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));  // X
        geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2));  // Y
        geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2));  // Z
        geo->getOrCreateStateSet()->setAttribute(new osg::LineWidth(axis_size),
                                                 osg::StateAttribute::ON);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geode->addDrawable(geo);

        h.type = ViewObjectType_Axes;
        h.uid = NextHandleID(pv3);

        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
        mt->addChild(geode);

        geode->setName(std::to_string(NextObjectID(pv3)));
        mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

        mt->setMatrix(transform);
        pv3->node_switch->addChild(mt);
        pv3->nodemap.insert({h, mt});
        pc->handles.push_back(h);
    }
    pc->ret.h = h;
    pc->success = true;
}

void Vis3d_Command_PlotText(Vis3d *pv3, Command *pc) {
    Handle h;
    const int color_channels = pc->size2;
    const std::string &content = pc->name;
    float font_size = pc->val.f;
    const int pos_size = pc->size1;

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    osg::Vec4 color;
    if (color_channels == 3) {
        color = {pc->data2[0], pc->data2[1], pc->data2[2], 1.0f};
    } else if (color_channels == 4) {
        color = {pc->data2[0], pc->data2[1], pc->data2[2], pc->data2[3]};
    } else {
        color = {1, 0, 0, 1};
    }

    if (font_size == 0) {
        if (pos_size == 2) {
            font_size = 18.0f;
        } else if (Vis3d__HasView(pv3, pc->who)) {
            float radius = pv3->viewmap[pc->who]->getCamera()->getBound().radius();
            font_size = 0.03f * radius;
        }
    }
    if (font_size == 0) {
        pc->ret.b = false;
        return;
    }
    osg::Vec3 position;
    if (pos_size == 2) {
        double width, height, top, left, znear, zfar;
        pv3->camera_2d->getProjectionMatrixAsOrtho(left, width, top, height, znear, zfar);
        const float window_height = height;
        position = {pc->data1[0], window_height - pc->data1[1] - font_size, 0.0f};
    } else {
        position = {pc->data1[0], pc->data1[1], pc->data1[2]};
    }

    text->setColor(color);
    text->setFont(pv3->font);
    text->setFontResolution(text->getFontWidth() * 2, text->getFontHeight() * 2);
    text->setCharacterSize(font_size);
    text->setPosition(position);
    text->setLayout(osgText::TextBase::LEFT_TO_RIGHT);
    osgText::String stru8(content, osgText::String::Encoding::ENCODING_UTF8);
    text->setText(stru8);
    text->setName(std::string("Text") + std::to_string(NextObjectID(pv3)));

    h.uid = NextHandleID(pv3);
    if (pos_size == 2) {
        h.type = ViewObjectType_2D;
        pv3->switch_2d->addChild(text);
        pv3->map_2d.insert({h, text});
    } else {
        text->setAxisAlignment(osgText::Text::SCREEN);
        h.type = ViewObjectType_Text;
        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(text.get());
        mt->addChild(geode);
        geode->setName(std::to_string(NextObjectID(pv3)));
        mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));
        pv3->node_switch->insertChild(0, mt);
        pv3->nodemap.insert({h, mt});
    }

    pc->ret.b = true;
    pc->ret.h = h;
}

void Vis3d_Command_SetText(Vis3d *pv3, Command *pc) {
    osg::Drawable *drawable;
    const Handle who = pc->who;
    if (Vis3d__Has2DDrawable(pv3, who)) {
        drawable = pv3->map_2d[who];
    } else if (Vis3d__HasNode(pv3, who)) {
        auto mt = pv3->nodemap[who];
        auto geode = mt->getChild(0)->asGeode();
        if (!geode) {
            VIS_ERROR("Can not find text: type: {0}, uid: {1}.", who.type, who.uid);
            pc->ret.b = false;
            return;
        }
        drawable = geode->getChild(0)->asDrawable();
    } else {
        VIS_ERROR("Can not find text: type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        return;
    }

    osg::ref_ptr<osgText::Text> text = (osgText::Text *)drawable;
    if (!text || drawable->getName().find(u8"Text") == std::string::npos) {
        VIS_ERROR(u8"Not a text:type: {0}, uid: {1}.", who.type, who.uid);
        pc->ret.b = false;
        return;
    }

    if (pc->name != "") {
        osgText::String stru8(pc->name, osgText::String::Encoding::ENCODING_UTF8);
        text->setText(stru8);
    }
    if (pc->val.f > 0) {
        text->setCharacterSize(pc->val.f);
    }
    if (pc->data1 != nullptr) {
        const int pos_size = pc->size1;
        if (pos_size == 2) {
            double width, height, top, left, znear, zfar;
            pv3->camera_2d->getProjectionMatrixAsOrtho(left, width, top, height, znear, zfar);
            const float window_height = height;
            const float pos_x = pc->data1[0];
            const float pos_y = pc->data1[1];
            float font_size = text->getCharacterHeight();
            text->setPosition({pos_x, window_height - pos_y - font_size, 0.0f});
        } else {
            text->setPosition({pc->data1[0], pc->data1[1], pc->data1[2]});
        }
    }

    if (pc->data2 != nullptr) {
        const int color_channels = pc->size2;
        osg::Vec4 color;
        if (color_channels == 3) {
            color = {pc->data2[0], pc->data2[1], pc->data2[2], 1.0f};
        } else {
            color = {pc->data2[0], pc->data2[1], pc->data2[2], pc->data2[3]};
        }
        text->setColor(color);
    }

    pc->success = true;
    pc->ret.b = true;
    return;
}

void Vis3d_Command_SetTextFont(Vis3d *pv3, Command *pc) {
    const std::string &font_name = pc->name;
    if (font_name.empty()) {
        pv3->font = nullptr;
    } else {
        pv3->font = osgText::readRefFontFile(font_name);
    }
    pc->ret.b = pv3->font != nullptr;
}

void Vis3d_Command_Plot2DQuad(Vis3d *pv3, Command *pc) {
    Handle h;
    const int color_channels = pc->val.u;
    const int numvert = pc->size1 / 2;
    const int numcl = pc->size2 / color_channels;
    const int draw_mode = pc->val.i;
    double width, height, top, left, znear, zfar;
    pv3->camera_2d->getProjectionMatrixAsOrtho(left, width, top, height, znear, zfar);
    const float window_height = height - top;

    osg::ref_ptr<osg::Vec2Array> vs = new osg::Vec2Array(numvert, (const osg::Vec2 *)(pc->data1));

    osg::Vec2 *ptr = (osg::Vec2 *)vs->getDataPointer();
    for (int i = 0; i < numvert; i++) {
        osg::Vec2 *v = (osg::Vec2 *)(&ptr[i]);
        auto old_height = (*v)[1];
        (*v)[1] = window_height - old_height;
    }

    osg::ref_ptr<osg::Array> cs;
    if (color_channels == 3) {
        cs = new osg::Vec3Array(numcl, (const osg::Vec3 *)(pc->data2));
    } else {
        cs = new osg::Vec4Array(numcl, (const osg::Vec4 *)(pc->data2));
    }

    osg::PrimitiveSet::Mode quad_mode =
        (osg::PrimitiveSet::Mode)(osg::PrimitiveSet::QUADS + draw_mode);
    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    geo->setVertexArray(vs.get());
    geo->setColorArray(cs.get());
    geo->setColorBinding(numcl == 1 ? osg::Geometry::BIND_OVERALL : osg::Geometry::BIND_PER_VERTEX);
    geo->addPrimitiveSet(new osg::DrawArrays(quad_mode, 0, numvert));
    geo->setName(std::string{"Geometry"} + std::to_string(NextObjectID(pv3)));
    h.type = ViewObjectType_2D;
    h.uid = NextHandleID(pv3);

    pv3->switch_2d->addChild(geo.get());
    pv3->map_2d.insert({h, geo});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Command_PlotPoint(Vis3d *pv3, Command *pc) {
    Handle h;
    const int color_channels = pc->size2 % 3 == 0 ? 3 : 4;
    const int numpt = pc->size1 / 3;
    const int numcl = pc->size2 / color_channels;
    const float ptsize = pc->val.f;

    osg::ref_ptr<osg::Vec3Array> vs = new osg::Vec3Array(numpt, (const osg::Vec3 *)(pc->data1));

    osg::ref_ptr<osg::Array> cs;
    if (color_channels == 3) {
        cs = new osg::Vec3Array(numcl, (const osg::Vec3 *)(pc->data2));
    } else {
        cs = new osg::Vec4Array(numcl, (const osg::Vec4 *)(pc->data2));
    }

    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    geo->setVertexArray(vs.get());
    geo->setColorArray(cs.get());
    geo->setColorBinding(numcl == numpt ? osg::Geometry::BIND_PER_VERTEX
                                        : osg::Geometry::BIND_OVERALL);

    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, numpt));
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geo->getOrCreateStateSet()->setAttribute(new osg::Point(ptsize), osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geo.get());
    mt->addChild(geode);
    h.type = ViewObjectType_Point;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Command_PlotLine(Vis3d *pv3, Command *pc) {
    Handle h;
    const int color_channels = pc->size2 % 3 == 0 ? 3 : 4;
    const int numcl = pc->size2 / color_channels;
    const int numlines = pc->size1 / 6;
    const float linesize = pc->val.f;

    osg::ref_ptr<osg::Vec3Array> vs =
        new osg::Vec3Array(numlines * 2, (const osg::Vec3 *)(pc->data1));

    osg::ref_ptr<osg::Array> cs;
    if (color_channels == 3) {
        cs = new osg::Vec3Array(numcl, (const osg::Vec3 *)(pc->data2));
    } else {
        cs = new osg::Vec4Array(numcl, (const osg::Vec4 *)(pc->data2));
    }

    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    geo->setVertexArray(vs.get());
    geo->setColorArray(cs.get());
    geo->setColorBinding(numcl == 1 ? osg::Geometry::BIND_OVERALL : osg::Geometry::BIND_PER_VERTEX);

    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numlines * 2));
    geo->getOrCreateStateSet()->setAttribute(new osg::LineWidth(linesize), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->addDrawable(geo);

    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    geode->addDrawable(geo.get());
    mt->addChild(geode);
    h.type = ViewObjectType_Line;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Command_PlotBox(Vis3d *pv3, Command *pc) {
    Handle h;
    const std::vector<float> &color = pc->buffer;
    const bool transparent = color.size() == 3 ? false : true;

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode{new osg::Geode()};

    osg::ref_ptr<osg::Box> box{new osg::Box()};

    box->setHalfLengths(osg::Vec3(pc->extents[0], pc->extents[1], pc->extents[2]));

    osg::ref_ptr<osg::ShapeDrawable> sd{new osg::ShapeDrawable(box.get())};
    sd->setColor(osg::Vec4(color[0], color[1], color[2], color.size() == 3 ? 1.f : color[3]));

    if (transparent) {
        sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    geode->addDrawable(sd);
    mt->addChild(geode);

    h.type = ViewObjectType_Box;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pc->success = true;
    pc->ret.h = h;

    osg::Matrixf m;
    m.setTrans(pc->pos[0], pc->pos[1], pc->pos[2]);
    mt->setMatrix(m);
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});
}

void Vis3d_Command_PlotCylinder(Vis3d *pv3, Command *pc) {
    Handle h;
    const float radius = pc->extents[0];
    const float height = pc->extents[1];
    const std::vector<float> &color = pc->buffer;
    const bool transparent = color.size() == 3 ? false : true;

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode{new osg::Geode()};

    osg::ref_ptr<osg::Cylinder> cylinder{new osg::Cylinder()};

    cylinder->setRadius(radius);
    cylinder->setHeight(height);

    osg::ref_ptr<osg::ShapeDrawable> sd{new osg::ShapeDrawable(cylinder.get())};
    sd->setColor(osg::Vec4(color[0], color[1], color[2], color.size() == 3 ? 1.f : color[3]));

    if (transparent) {
        sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    geode->addDrawable(sd);
    mt->addChild(geode);

    h.type = ViewObjectType_Cylinder;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pc->success = true;
    pc->ret.h = h;

    osg::Matrixf m;
    m.setTrans(pc->pos[0], pc->pos[1], pc->pos[2]);
    mt->setMatrix(m);
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});
}

void Vis3d_Command_PlotSphere(Vis3d *pv3, Command *pc) {
    Handle h;
    const float radius = pc->extents[0];
    const std::vector<float> &color = pc->buffer;
    const bool transparent = color.size() == 3 ? false : true;

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode{new osg::Geode()};

    osg::ref_ptr<osg::Sphere> sphere{new osg::Sphere()};

    sphere->setRadius(radius);

    osg::ref_ptr<osg::ShapeDrawable> sd{new osg::ShapeDrawable(sphere.get())};
    sd->setColor(osg::Vec4(color[0], color[1], color[2], color.size() == 3 ? 1.f : color[3]));

    if (transparent) {
        sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    geode->addDrawable(sd);
    mt->addChild(geode);

    h.type = ViewObjectType_Sphere;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pc->success = true;
    pc->ret.h = h;

    osg::Matrixf m;
    m.setTrans(pc->pos[0], pc->pos[1], pc->pos[2]);
    mt->setMatrix(m);
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});
}

void Vis3d_Command_PlotSpheres(Vis3d *pv3, Command *pc) {
    Handle h;

    const std::vector<float> radii = pc->buffer;
    const int num_radii = radii.size();

    const int num_spheres = pc->size1 / 3;
    const int colors_size = pc->size2 / 4;

    const float *centers = pc->data1;
    const float *colors = pc->data2;

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode{new osg::Geode()};

    for (int i = 0; i < num_spheres; ++i) {
        osg::ref_ptr<osg::Sphere> sphere{new osg::Sphere()};
        double radius = (num_radii == 1) ? radii[0] : radii[i];
        sphere->setCenter(osg::Vec3f(centers[0 + 3 * i], centers[1 + 3 * i], centers[2 + 3 * i]));
        sphere->setRadius(radius);
        osg::ref_ptr<osg::ShapeDrawable> sd{new osg::ShapeDrawable(sphere.get())};
        int offset_index = (colors_size == 1) ? 0 : i;
        sd->setColor(osg::Vec4(colors[0 + 4 * offset_index], colors[1 + 4 * offset_index],
                               colors[2 + 4 * offset_index], colors[3 + 4 * offset_index]));

        sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        geode->addDrawable(sd);
        mt->addChild(geode);
    }

    h.type = ViewObjectType_Spheres;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pc->success = true;
    pc->ret.h = h;

    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});
}

void Vis3d_Command_PlotCone(Vis3d *pv3, Command *pc) {
    Handle h;
    const float radius = pc->extents[0];
    const float height = pc->extents[1];
    const std::vector<float> &color = pc->buffer;
    const bool transparent = color.size() == 3 ? false : true;

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode{new osg::Geode()};

    osg::ref_ptr<osg::Cone> cone{new osg::Cone()};

    cone->setRadius(radius);
    cone->setHeight(height);

    osg::ref_ptr<osg::ShapeDrawable> sd{new osg::ShapeDrawable(cone.get())};
    sd->setColor(osg::Vec4(color[0], color[1], color[2], color.size() == 3 ? 1.f : color[3]));

    if (transparent) {
        sd->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        sd->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    geode->addDrawable(sd);
    mt->addChild(geode);

    h.type = ViewObjectType_Cone;
    h.uid = NextHandleID(pv3);

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pc->success = true;
    pc->ret.h = h;

    osg::Matrixf m;
    m.setTrans(pc->pos[0], pc->pos[1], pc->pos[2]);
    mt->setMatrix(m);
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});
}

void Vis3d_Command_PlotArrow(Vis3d *pv3, Command *pc) {
    Handle h;
    int num_arrow = pc->size1 / 3;

    osg::ref_ptr<osg::Vec3Array> tail_array =
        new osg::Vec3Array(num_arrow, (const osg::Vec3 *)pc->data1);
    osg::ref_ptr<osg::Vec3Array> head_array =
        new osg::Vec3Array(num_arrow, (const osg::Vec3 *)pc->data2);

    const float radius = pc->val.f;
    const int color_channels = pc->val.i;
    const std::vector<float> &color = pc->buffer;
    const float *ptail = pc->data1;
    const float *phead = pc->data2;

    // Arrow is composed by cone and cylinder
    const float cone_ratio = 0.2f;
    osg::Vec3f trans;
    osg::Quat quat;
    const osg::Vec3f zaxis{0, 0, 1};
    osg::ref_ptr<osg::MatrixTransform> mt_pa{new osg::MatrixTransform};
    mt_pa->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));
    for (int i = 0; i < num_arrow; i++) {
        osg::ref_ptr<osg::Geode> geode_cone{new osg::Geode()};
        osg::ref_ptr<osg::Geode> geode_cylinder{new osg::Geode()};
        const osg::Vec3f tail{ptail[i * 3 + 0], ptail[i * 3 + 1], ptail[i * 3 + 2]};
        const osg::Vec3f head{phead[i * 3 + 0], phead[i * 3 + 1], phead[i * 3 + 2]};
        osg::Vec3f vec = head - tail;
        const float vec_len = vec.normalize();
        const float cone_height = vec_len * cone_ratio;
        const float cylinder_height = vec_len - cone_height;

        osg::ref_ptr<osg::Cone> cone{new osg::Cone()};
        osg::ref_ptr<osg::Cylinder> cylinder{new osg::Cylinder()};

        cylinder->setCenter(osg::Vec3f(0, 0, cylinder_height * 0.5f));
        cylinder->setRadius(radius);
        cylinder->setHeight(cylinder_height);
        cone->setCenter(
            osg::Vec3f(0, 0, cylinder_height + cone_height * 0.25f));  // Move cone to the top
        cone->setRadius(radius);
        cone->setHeight(cone_height);

        osg::ref_ptr<osg::ShapeDrawable> sd_cone{new osg::ShapeDrawable(cone.get())};
        sd_cone->setColor(osg::Vec4(color[i * color_channels + 0], color[i * color_channels + 1],
                                    color[i * color_channels + 2],
                                    color_channels == 3 ? 1.f : color[i * color_channels + 3]));

        osg::ref_ptr<osg::ShapeDrawable> sd_cylinder{new osg::ShapeDrawable(cylinder.get())};
        sd_cylinder->setColor(osg::Vec4(color[i * color_channels + 0],
                                        color[i * color_channels + 1],
                                        color[i * color_channels + 2],
                                        color_channels == 3 ? 1.f : color[i * color_channels + 3]));

        if (color_channels == 4) {
            sd_cylinder->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            sd_cylinder->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            sd_cone->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            sd_cone->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        }

        geode_cone->addDrawable(sd_cone);
        geode_cylinder->addDrawable(sd_cylinder);
        osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
        mt->addChild(geode_cone);
        mt->addChild(geode_cylinder);
        geode_cone->setName(std::to_string(NextObjectID(pv3)));
        geode_cylinder->setName(std::to_string(NextObjectID(pv3)));
        mt->setName(std::string{"sub_mt"} + std::to_string(NextObjectID(pv3)));
        quat.makeRotate(zaxis, vec);
        trans = tail;
        if (num_arrow != 1) {
            osg::Matrixf m;
            m.setTrans(trans);
            m.setRotate(quat);
            mt->setMatrix(m);
        }
        mt_pa->addChild(mt);
    }

    if (num_arrow == 1) {
        osg::Matrixf m;
        m.setTrans(trans);
        m.setRotate(quat);
        mt_pa->setMatrix(m);
    }

    h.type = ViewObjectType_Arrow;
    h.uid = NextHandleID(pv3);

    pv3->node_switch->addChild(mt_pa);
    pv3->nodemap.insert({h, mt_pa});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Command_PlotGround(Vis3d *pv3, Command *pc) {
    Handle h;
    std::vector<float> color = pc->buffer;
    int halfcells = pc->size1;
    float cellsize = pc->val.f;

    osg::Geode *geode = new osg::Geode();
    osg::Geometry *geom = new osg::Geometry();

    osg::Vec3Array *vertices = new osg::Vec3Array();

    for (int i = -halfcells; i <= halfcells; ++i) {
        vertices->push_back(osg::Vec3(halfcells * cellsize, i * cellsize, 0.0f));
        vertices->push_back(osg::Vec3(-halfcells * cellsize, i * cellsize, 0.0f));
        vertices->push_back(osg::Vec3(i * cellsize, halfcells * cellsize, 0.0f));
        vertices->push_back(osg::Vec3(i * cellsize, -halfcells * cellsize, 0.0f));
    }

    geom->setVertexArray(vertices);
    osg::Vec4Array *colors = new osg::Vec4Array();
    float alpha = 1.0f;
    if (color.size() > 3)
        alpha = color[3];
    colors->push_back(osg::Vec4(color[0], color[1], color[2], alpha));
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    osg::Vec3Array *normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4 * (halfcells * 2 + 1)));

    geode->addDrawable(geom);
    osg::StateSet *set = new osg::StateSet();
    set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    set->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f), osg::StateAttribute::ON);
    set->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    geode->setStateSet(set);
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();

    geode->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));
    mt->addChild(geode);
    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Command_PlotMesh(Vis3d *pv3, Command *pc) {
    Handle h;
    const int color_channels = pc->size2 % 3 == 0 ? 3 : 4;
    const int numcl = pc->size2 / color_channels;
    osg::ref_ptr<osg::Array> cs;
    if (color_channels == 3) {
        cs = new osg::Vec3Array(numcl, (const osg::Vec3 *)(pc->data2));
    } else {
        cs = new osg::Vec4Array(numcl, (const osg::Vec4 *)(pc->data2));
    }
    const bool transparent = color_channels == 3 ? false : true;
    osg::ref_ptr<osg::Vec3Array> vertices =
        new osg::Vec3Array(pc->size1 / 3, (const osg::Vec3 *)(pc->data1));
    osg::ref_ptr<osg::DrawElementsUInt> indices =
        new osg::DrawElementsUInt(GL_TRIANGLES, pc->size3, pc->data3);
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->addPrimitiveSet(indices.get());
    geom->setColorArray(cs.get());
    geom->setColorBinding(numcl == 1 ? osg::Geometry::BIND_OVERALL
                                     : osg::Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::MatrixTransform> mt{new osg::MatrixTransform};
    osg::ref_ptr<osg::Geode> geode_mesh{new osg::Geode()};

    if (transparent) {
        geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    geode_mesh->addDrawable(geom.get());
    mt->addChild(geode_mesh);

    osgUtil::SmoothingVisitor smoother;
    // When outlining extruded polygons, only draw a post outline if the angle between the adjoining
    // faces exceeds this value. This has the effect of only outlining corners that are sufficiently
    // “sharp”.
    smoother.setCreaseAngle(osg::PI * 0.5);
    smoother.apply(*geode_mesh);

    h.type = ViewObjectType_Mesh;
    h.uid = NextHandleID(pv3);

    geode_mesh->setName(std::to_string(NextObjectID(pv3)));
    mt->setName(std::string{"mt"} + std::to_string(NextObjectID(pv3)));

    pv3->node_switch->addChild(mt);
    pv3->nodemap.insert({h, mt});

    pc->success = true;
    pc->ret.h = h;
}

void Vis3d_Callback(void *p) {
    Vis3d *pv3 = static_cast<Vis3d *>(p);

    osg::setNotifyLevel(osg::FATAL);
    pv3->compviewer = new osgViewer::CompositeViewer;
    if (!pv3->compviewer) {
        VIS_ERROR("Failed to create viewer");
        return;
    }

    pv3->scene_root = new osg::Group;
    if (!pv3->scene_root) {
        VIS_ERROR("Failed to create scene root");
        return;
    }

    pv3->node_switch = new osg::Switch;
    if (!pv3->node_switch) {
        VIS_ERROR("Failed to create node switch");
        return;
    }

    pv3->camera_2d = new osg::Camera;
    if (!pv3->camera_2d) {
        VIS_ERROR("Failed to create camera 2d");
        return;
    }

    pv3->switch_2d = new osg::Switch;
    if (!pv3->switch_2d) {
        VIS_ERROR("Failed to create switch 2d");
        return;
    }

    pv3->scene_root->addChild(pv3->node_switch);
    pv3->scene_root->addChild(pv3->camera_2d);
    pv3->camera_2d->addChild(pv3->switch_2d);

    pv3->compviewer->setKeyEventSetsDone(0);  // Prevent ESC exiting
    pv3->compviewer->setThreadingModel(
        osgViewer::CompositeViewer::CullThreadPerCameraDrawThreadPerContext);

    const uint64_t delta_ms = 33;  // milli-second per frame

    pv3->entered_loop.store(true);

    while (!pv3->done.load()) {
        const uint64_t start_ms = Vis::GetTimeMillisecond();
        if (pv3->compviewer->done()) {
            if (pv3->compviewer->getNumViews() == 0) {
                Vis::Sleep(delta_ms);
            } else {
                VIS_WARN("Viewer is done, but still has view!");
                break;
            }
        } else {
            pv3->compviewer->frame();
        }

        // Process user event
        Vis3d_Update(pv3);

        /// We put more time on processing user event
        while (Vis::GetTimeMillisecond() < start_ms + delta_ms) {
            Vis3d_Update(pv3);
            Vis::Sleep(5);  // Sleep 5 ms
        }
    }

    pv3->exited_loop.store(true);
    Vis3d_Cleanup(pv3);
}

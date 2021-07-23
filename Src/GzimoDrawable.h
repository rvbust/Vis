#include <IGizmo.h>

#include <osg/Drawable>
#include <osg/MatrixTransform>
#include <osgGA/EventVisitor>
#include <osgGA/TrackballManipulator>

class GizmoDrawable : public osg::Drawable {
public:
    struct GizmoEventCallback : public osg::Drawable::EventCallback {
        virtual void event(osg::NodeVisitor* nv, osg::Drawable* drawable) {
            osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
            GizmoDrawable* gizmoDrawable = dynamic_cast<GizmoDrawable*>(drawable);
            if (!ev || !gizmoDrawable)
                return;

            const osgGA::EventQueue::Events& events = ev->getEvents();
            for (osgGA::EventQueue::Events::const_iterator itr = events.begin();
                 itr != events.end(); ++itr) {
                const osgGA::GUIEventAdapter* ea = (*itr)->asGUIEventAdapter();
                int x = ea->getX(), y = ea->getY();
                if (ea->getMouseYOrientation() == osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS)
                    y = ea->getWindowHeight() - y;
                switch (ea->getEventType()) {
                case osgGA::GUIEventAdapter::PUSH:
                    if (gizmoDrawable->getGizmoObject()) {
                        bool capture = false;
                        if (!(gizmoDrawable->_captureGizmo && *gizmoDrawable->_captureGizmo != 0)) {
                            capture = gizmoDrawable->getGizmoObject()->OnMouseDown(x, y);
                        }
                        if (gizmoDrawable->_captureGizmo && capture) {
                            *gizmoDrawable->_captureGizmo |= 1 << gizmoDrawable->getGizmoMode();
                        }
                    }
                    break;
                case osgGA::GUIEventAdapter::RELEASE:
                    if (gizmoDrawable->getGizmoObject())
                        gizmoDrawable->getGizmoObject()->OnMouseUp(x, y);
                    if (gizmoDrawable->_captureGizmo) {
                        *gizmoDrawable->_captureGizmo = 0;
                    }
                    break;
                case osgGA::GUIEventAdapter::MOVE:
                case osgGA::GUIEventAdapter::DRAG:
                    if (gizmoDrawable->getGizmoObject())
                        gizmoDrawable->getGizmoObject()->OnMouseMove(x, y);
                    break;
                case osgGA::GUIEventAdapter::RESIZE:
                    gizmoDrawable->setScreenSize(ea->getWindowWidth(), ea->getWindowHeight());
                    break;
                case osgGA::GUIEventAdapter::FRAME:
                    gizmoDrawable->applyTransform();
                    break;
                default:
                    break;
                }
            }
        }
    };

    GizmoDrawable() : _gizmo(0), _editMatrix(nullptr), _captureGizmo(nullptr), _mode(NO_GIZMO) {
        _screenSize[0] = 800;
        _screenSize[1] = 600;

        setEventCallback(new GizmoEventCallback);
        setSupportsDisplayList(false);
        getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    }

    GizmoDrawable(const GizmoDrawable& copy, osg::CopyOp op = osg::CopyOp::SHALLOW_COPY)
        : osg::Drawable(copy, op)
        , _transform(copy._transform)
        , _gizmo(copy._gizmo)
        , _editMatrix(copy._editMatrix)
        , _captureGizmo(nullptr)
        , _mode(copy._mode) {
        _screenSize[0] = copy._screenSize[0];
        _screenSize[1] = copy._screenSize[1];
    }

    enum Mode { NO_GIZMO = 0, MOVE_GIZMO, ROTATE_GIZMO, SCALE_GIZMO };
    void setGizmoMode(Mode m, IGizmo::LOCATION loc = IGizmo::LOCATE_LOCAL) {
        _mode = m;
        if (_gizmo)
            delete _gizmo;
        switch (m) {
        case MOVE_GIZMO:
            _gizmo = CreateMoveGizmo();
            break;
        case ROTATE_GIZMO:
            _gizmo = CreateRotateGizmo();
            break;
        case SCALE_GIZMO:
            _gizmo = CreateScaleGizmo();
            break;
        default:
            _gizmo = NULL;
            return;
        }

        if (_gizmo) {
            _gizmo->SetEditMatrix(_editMatrix);
            _gizmo->SetScreenDimension(_screenSize[0], _screenSize[1]);
            _gizmo->SetLocation(loc);
            //_gizmo->SetDisplayScale( 0.5f );
        }
    }

    Mode getGizmoMode() const { return _mode; }
    IGizmo* getGizmoObject() { return _gizmo; }
    const IGizmo* getGizmoObject() const { return _gizmo; }

    void setTransform(osg::MatrixTransform* node, float* matrix) {
        _transform = node;
        _editMatrix = matrix;
    }

    void setCaptureFlag(int* capture) { _captureGizmo = capture; }

    osg::MatrixTransform* getTransform() { return _transform.get(); }
    const osg::MatrixTransform* getTransform() const { return _transform.get(); }

    void setScreenSize(int w, int h) {
        _screenSize[0] = w;
        _screenSize[1] = h;
        if (_gizmo)
            _gizmo->SetScreenDimension(w, h);
    }

    void applyTransform() {
        if (_gizmo && _transform.valid()) {
            _transform->setMatrix(osg::Matrix(_editMatrix));
        }
    }

    void setDrawMask(unsigned int mask) {
        if (_gizmo) {
            _gizmo->SetAxisMask(mask);
        }
    }

    void setDisplayScale(float scale) {
        if (_gizmo) {
            _gizmo->SetDisplayScale(scale);
        }
    }

    void setDetectionRange(float range) {
        if (_gizmo) {
            _gizmo->SetDetectionRange(range);
        }
    }

    META_Object(osg, GizmoDrawable);

    virtual void drawImplementation(osg::RenderInfo& renderInfo) const {
        osg::State* state = renderInfo.getState();
        state->disableAllVertexArrays();
        state->disableTexCoordPointer(0);

        glPushMatrix();
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        if (_gizmo) {
            _gizmo->SetCameraMatrix(osg::Matrixf(state->getModelViewMatrix()).ptr(),
                                    osg::Matrixf(state->getProjectionMatrix()).ptr());
            if (!_captureGizmo || !(*_captureGizmo) || (*_captureGizmo & (1 << _mode))) {
                _gizmo->Draw();
            }
        }
        glPopAttrib();
        glPopMatrix();
    }

protected:
    virtual ~GizmoDrawable() {}

    osg::observer_ptr<osg::MatrixTransform> _transform;
    IGizmo* _gizmo;
    float* _editMatrix;
    int* _captureGizmo;
    int _screenSize[2];
    Mode _mode;
};

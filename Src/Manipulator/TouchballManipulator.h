#include <osgGA/MultiTouchTrackballManipulator>

class TouchballManipulator : public osgGA::MultiTouchTrackballManipulator {
public:
    TouchballManipulator(int* gizmod = nullptr, bool* view_manipulation = nullptr)
        : m_gizmod(gizmod), m_view_manipulation(view_manipulation) {
        setMinimumDistance(-0.5f, true);
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

protected:
    virtual void handleMultiTouchPan(const osgGA::GUIEventAdapter* now,
                                     const osgGA::GUIEventAdapter* last,
                                     const double eventTimeDelta);
    virtual void handleMultiTouchZoom(const osgGA::GUIEventAdapter* now,
                                      const osgGA::GUIEventAdapter* last,
                                      const double eventTimeDelta);

private:
    int* m_gizmod;
    bool* m_view_manipulation;
    double m_4time = 0.0f;
    double m_3time = 0.0f;
    double m_2time = 0.0f;
    double m_1time = 0.0f;
    double m_ztime = 0.0f;
};

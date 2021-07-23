#include "TouchballManipulator.h"

void TouchballManipulator::handleMultiTouchZoom(const osgGA::GUIEventAdapter* now,
                                                const osgGA::GUIEventAdapter* last,
                                                const double eventTimeDelta) {
    const osg::Vec2 pt_1_now(now->getTouchData()->get(0).x, now->getTouchData()->get(0).y);
    const osg::Vec2 pt_2_now(now->getTouchData()->get(1).x, now->getTouchData()->get(1).y);
    const osg::Vec2 pt_1_last(last->getTouchData()->get(0).x, last->getTouchData()->get(0).y);
    const osg::Vec2 pt_2_last(last->getTouchData()->get(1).x, last->getTouchData()->get(1).y);

    const float gap_now((pt_1_now - pt_2_now).length());
    const float gap_last((pt_1_last - pt_2_last).length());

    const float relativeChange = (gap_last - gap_now) / gap_last;

    osg::notify(osg::ALWAYS) << gap_now << " " << gap_last << " " << relativeChange;
    double delta = now->getTime() - m_ztime;
    // zoom gesture
    if (fabs(relativeChange) > 0.02 || delta < 0.1) {
        osg::notify(osg::ALWAYS) << " zoom " << delta << std::endl;
        if (fabs(relativeChange) > 0.02) {
            m_ztime = now->getTime();
        }
        zoomModel(relativeChange, true);
    }
}

void TouchballManipulator::handleMultiTouchPan(const osgGA::GUIEventAdapter* now,
                                               const osgGA::GUIEventAdapter* last,
                                               const double eventTimeDelta) {
    float dx = now->getXnormalized() - last->getXnormalized();
    float dy = now->getYnormalized() - last->getYnormalized();
    const float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
    panModel(dx * scale, dy * scale);
}

bool TouchballManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) {
    if (m_gizmod && (*m_gizmod)) {
        // do not manipulate when capture gizmo
        return false;
    }
    if (m_view_manipulation && (*m_view_manipulation)) {
        // do not manipulate when capture view_manipulation
        return false;
    }

    bool handled(false);

    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::PUSH:
    case osgGA::GUIEventAdapter::DRAG:
    case osgGA::GUIEventAdapter::RELEASE:
        if (ea.isMultiTouchEvent()) {
            double eventTimeDelta = 0.;  // _ga_t0->getTime() - _ga_t1->getTime();
            if (eventTimeDelta < 0.) {
                OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
                eventTimeDelta = 0.;
            }
            osgGA::GUIEventAdapter::TouchData* data = ea.getTouchData();

            // three touches or two taps for home position
            if ((data->getNumTouchPoints() == 4) ||
                ((data->getNumTouchPoints() == 1) && (data->get(0).tapCount >= 2))) {
                flushMouseEventStack();
                _thrown = false;
                home(ea, us);
                handled = true;
                m_4time = ea.getTime();
            }

            else if (data->getNumTouchPoints() == 3) {
                double delta = ea.getTime() - m_4time;
                // OSG_ALWAYS << "Manipulator 2 : delta = " << delta << std::endl;
                if ((delta > 0.1) && (_lastEvent.valid()) &&
                    (_lastEvent->getTouchData()->getNumTouchPoints() == 3)) {
                    handleMultiTouchPan(&ea, _lastEvent.get(), eventTimeDelta);
                    m_3time = ea.getTime();
                }

                handled = true;
            } else if (data->getNumTouchPoints() == 2) {
                double delta = ea.getTime() - m_3time;
                // OSG_ALWAYS << "Manipulator 2 : delta = " << delta << std::endl;
                if ((delta > 0.1) && (_lastEvent.valid()) &&
                    (_lastEvent->getTouchData()->getNumTouchPoints() == 2)) {
                    handleMultiTouchZoom(&ea, _lastEvent.get(), eventTimeDelta);
                    m_2time = ea.getTime();
                }
                handled = true;

            } else if (data->getNumTouchPoints() == 1) {
                double current_time = ea.getTime();
                if (m_1time == 0) {
                    m_1time = current_time;
                }
                double delta4 = current_time - m_4time;
                double delta3 = current_time - m_3time;
                double delta2 = current_time - m_2time;
                double delta1 = current_time - m_1time;
                // OSG_ALWAYS << "Manipulator 1 : delta3 = " << delta3 << "  delta2 = "<< delta2 <<
                // "  delta1 = "<< delta1 << std::endl;
                if (delta4 < 0.1 || delta3 < 0.1 || delta2 < 0.1 || delta1 < 0.1) {
                    handled = true;
                }
            }

            _lastEvent = new osgGA::GUIEventAdapter(ea);

            // check if all touches ended
            unsigned int num_touches_ended(0);
            for (osgGA::GUIEventAdapter::TouchData::iterator i = data->begin(); i != data->end();
                 ++i) {
                if ((*i).phase == osgGA::GUIEventAdapter::TOUCH_ENDED)
                    num_touches_ended++;
            }

            if (num_touches_ended == data->getNumTouchPoints()) {
                m_1time = 0;
                _lastEvent = NULL;
            }
        }
        break;
    default:
        break;
    }

    return handled ? handled : TrackballManipulator::handle(ea, us);
}

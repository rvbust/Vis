#include "LineIntersector.h"
#include <osg/Geometry>

LineIntersector::LineIntersector()
    : osgUtil::LineSegmentIntersector(MODEL, 0.f, 0.f), m_offset(0.05f) {
    m_hitIndices.clear();
}

LineIntersector::LineIntersector(const osg::Vec3 &start, const osg::Vec3 &end)
    : osgUtil::LineSegmentIntersector(start, end), m_offset(0.05f) {
    m_hitIndices.clear();
}

LineIntersector::LineIntersector(osgUtil::Intersector::CoordinateFrame cf, double x, double y)
    : osgUtil::LineSegmentIntersector(cf, x, y), m_offset(0.05f) {
    m_hitIndices.clear();
}

LineIntersector::LineIntersector(osgUtil::Intersector::CoordinateFrame cf, const osg::Vec3d &start,
                                 const osg::Vec3d &end)
    : osgUtil::LineSegmentIntersector(cf, start, end), m_offset(0.05f) {}

void LineIntersector::setOffset(float offset) {
    m_offset = offset;
}

float LineIntersector::getOffset() const {
    return m_offset;
}

void LineIntersector::getHitIndices(int &first, int &last) const {
    if (m_hitIndices.empty()) {
        first = -1;
        last = -1;
    } else {
        first = m_hitIndices.front();
        last = m_hitIndices.back();
    }
}

osgUtil::Intersector *LineIntersector::clone(osgUtil::IntersectionVisitor &iv) {
    if (_coordinateFrame == MODEL && iv.getModelMatrix() == 0) {
        osg::ref_ptr<LineIntersector> cloned = new LineIntersector(_start, _end);
        cloned->_parent = this;
        cloned->m_offset = m_offset;
        return cloned.release();
    }

    osg::Matrix matrix;
    switch (_coordinateFrame) {
    case WINDOW:
        if (iv.getWindowMatrix())
            matrix.preMult(*iv.getWindowMatrix());
        if (iv.getProjectionMatrix())
            matrix.preMult(*iv.getProjectionMatrix());
        if (iv.getViewMatrix())
            matrix.preMult(*iv.getViewMatrix());
        if (iv.getModelMatrix())
            matrix.preMult(*iv.getModelMatrix());
        break;
    case PROJECTION:
        if (iv.getProjectionMatrix())
            matrix.preMult(*iv.getProjectionMatrix());
        if (iv.getViewMatrix())
            matrix.preMult(*iv.getViewMatrix());
        if (iv.getModelMatrix())
            matrix.preMult(*iv.getModelMatrix());
        break;
    case VIEW:
        if (iv.getViewMatrix())
            matrix.preMult(*iv.getViewMatrix());
        if (iv.getModelMatrix())
            matrix.preMult(*iv.getModelMatrix());
        break;
    case MODEL:
        if (iv.getModelMatrix())
            matrix = *iv.getModelMatrix();
        break;
    }

    osg::Matrix inverse = osg::Matrix::inverse(matrix);
    osg::ref_ptr<LineIntersector> cloned = new LineIntersector(_start * inverse, _end * inverse);
    cloned->_parent = this;
    cloned->m_offset = m_offset;
    return cloned.release();
}

void LineIntersector::intersect(osgUtil::IntersectionVisitor &iv, osg::Drawable *drawable) {
    osg::BoundingBox bb = drawable->getBoundingBox();
    bb.xMin() -= m_offset;
    bb.xMax() += m_offset;
    bb.yMin() -= m_offset;
    bb.yMax() += m_offset;
    bb.zMin() -= m_offset;
    bb.zMax() += m_offset;

    osg::Vec3d s(_start), e(_end);
    if (!intersectAndClip(s, e, bb))
        return;
    if (iv.getDoDummyTraversal())
        return;

    osg::Geometry *geometry = drawable->asGeometry();
    if (geometry) {
        if (!this->isRightPrimitive(geometry))
            return;

        osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(geometry->getVertexArray());
        if (!vertices)
            return;

        for (unsigned int i = 0; i < vertices->size(); ++i) {
            int j = (i == 0) ? vertices->size() - 1 : i - 1;
            double distance = getSkewLinesDistance(s, e, (*vertices)[i], (*vertices)[j]);
            if (m_offset < distance)
                continue;

            Intersection hit;
            hit.ratio = distance;
            hit.nodePath = iv.getNodePath();
            hit.drawable = drawable;
            hit.matrix = iv.getModelMatrix();
            hit.localIntersectionPoint = (*vertices)[i];
            m_hitIndices.push_back(i);
            insertIntersection(hit);
        }
    }
}

bool LineIntersector::isVirtualIntersector() const {
    return false;
}

double LineIntersector::getSkewLinesDistance(const osg::Vec3d &r1, const osg::Vec3d &r2,
                                             const osg::Vec3d &v1, const osg::Vec3d &v2) {
    osg::Vec3d u1 = r2 - r1;
    osg::Vec3d u2 = v2 - v1;
    osg::Vec3d u3 = u1 ^ u2;
    osg::Vec3d dir = r1 - v2;
    if (u3.length() == 0)
        return 1;
    return std::fabs((dir * u3) / u3.length());
}

bool LineIntersector::isRightPrimitive(const osg::Geometry *geometry) {
    const osg::Geometry::PrimitiveSetList &primitives = geometry->getPrimitiveSetList();
    for (const auto &p : primitives) {
        if (p->getMode() == GL_LINE_LOOP || p->getMode() == GL_LINES ||
            p->getMode() == GL_LINE_STRIP)
            return true;
    }
    return false;
}

#include "PointIntersector.h"

#include <osg/Geometry>

PointIntersector::PointIntersector() : LineIntersector() {}

PointIntersector::PointIntersector(const osg::Vec3 &start, const osg::Vec3 &end)
    : LineIntersector(start, end) {}

PointIntersector::PointIntersector(osgUtil::Intersector::CoordinateFrame cf, double x, double y)
    : LineIntersector(cf, x, y) {}

PointIntersector::PointIntersector(osgUtil::Intersector::CoordinateFrame cf,
                                   const osg::Vec3d &start, const osg::Vec3d &end)
    : LineIntersector(cf, start, end) {}

osgUtil::Intersector *PointIntersector::clone(osgUtil::IntersectionVisitor &iv) {
    if (_coordinateFrame == MODEL && iv.getModelMatrix() == 0) {
        osg::ref_ptr<PointIntersector> cloned = new PointIntersector(_start, _end);
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
    osg::ref_ptr<PointIntersector> cloned = new PointIntersector(_start * inverse, _end * inverse);
    cloned->_parent = this;
    cloned->m_offset = m_offset;
    return cloned.release();
}

void PointIntersector::intersect(osgUtil::IntersectionVisitor &iv, osg::Drawable *drawable) {
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

        osg::Vec3d dir = e - s;
        double invLength = 1.0 / dir.length();
        for (unsigned int i = 0; i < vertices->size(); ++i) {
            double distance = std::fabs((((*vertices)[i] - s) ^ dir).length());
            distance *= invLength;
            if (m_offset < distance)
                continue;

            Intersection hit;
            hit.ratio = distance;
            hit.nodePath = iv.getNodePath();
            hit.drawable = drawable;
            hit.matrix = iv.getModelMatrix();
            hit.primitiveIndex = i;
            hit.localIntersectionPoint = (*vertices)[i];
            m_hitIndices.push_back(i);
            insertIntersection(hit);
        }
    }
}

bool PointIntersector::isRightPrimitive(const osg::Geometry *geometry) {
    const osg::Geometry::PrimitiveSetList &primitives = geometry->getPrimitiveSetList();
    for (const auto &p : primitives) {
        if (p->getMode() == GL_POINTS)
            return true;
    }
    return false;
}

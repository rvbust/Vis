/*!
 * \brief OSG-based line intersector class.
 * \details This file is a part of osgIntersectors example program. See more details:
 * https://github.com/vicrucann/osg-intersectors-example
 * \autor Victoria Rudakova
 * \date 2016-2017
 * \copyright MIT License
 */

#ifndef LINEINTERSECTOR_H
#define LINEINTERSECTOR_H

#include <osgUtil/LineSegmentIntersector>
#include <vector>

/*! \class LineIntersector
 * \brief A class that allows to catch intersections with line loops and lines OpenGL types.
 * It uses shortest distance between the cast ray and the geometry line which is calculated
 * as a distance between skew lines.
 * In addition, it filters out the geometries whose primitive sets are different than line-types.
 */

class LineIntersector : public osgUtil::LineSegmentIntersector {
public:
    LineIntersector();

    LineIntersector(const osg::Vec3& start, const osg::Vec3& end);
    LineIntersector(CoordinateFrame cf, double x, double y);
    LineIntersector(CoordinateFrame cf, const osg::Vec3d& start, const osg::Vec3d& end);

    void setOffset(float offset);
    float getOffset() const;
    void getHitIndices(int& first, int& last) const;

    virtual Intersector* clone(osgUtil::IntersectionVisitor& iv);
    virtual void intersect(osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable);

    bool isVirtualIntersector() const;

protected:
    double getSkewLinesDistance(const osg::Vec3d& r1, const osg::Vec3d& r2, const osg::Vec3d& v1,
                                const osg::Vec3d& v2);
    virtual bool isRightPrimitive(const osg::Geometry* geometry);

    float m_offset;
    std::vector<unsigned int> m_hitIndices;
};

#endif  // LINEINTERSECTOR_H

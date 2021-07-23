///////////////////////////////////////////////////////////////////////////////////////////////////
// LibGizmo
// File Name :
// Creation : 10/01/2012
// Author : Cedric Guillemet
// Description : LibGizmo
//
/// Copyright (C) 2012 Cedric Guillemet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
/// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef GIZMOTRANSFORMMOVE_H__
#define GIZMOTRANSFORMMOVE_H__

#include "GizmoTransform.h"

class CGizmoTransformMove : public CGizmoTransform {
public:
    CGizmoTransformMove();
    virtual ~CGizmoTransformMove();

    // return true if gizmo transform capture mouse
    virtual bool OnMouseDown(unsigned int x, unsigned int y);
    virtual void OnMouseMove(unsigned int x, unsigned int y);
    virtual void OnMouseUp(unsigned int x, unsigned int y);

    virtual void Draw();
    // snap

    virtual void SetSnap(float snapx, float snapy, float snapz) {
        m_MoveSnap = tvector3(snapx, snapy, snapz);
    }
    virtual void SetSnap(const float snap) {}

    tvector3 GetMoveSnap() { return m_MoveSnap; }

    virtual void ApplyTransform(tvector3& trans, bool bAbsolute);
    virtual void SetAxisMask(unsigned int mask);

protected:
    enum MOVETYPE {
        MOVE_NONE = 0,
        MOVE_X = 1 << 0,
        MOVE_Y = 1 << 1,
        MOVE_Z = 1 << 2,
        MOVE_XY = 1 << 3,
        MOVE_XZ = 1 << 4,
        MOVE_YZ = 1 << 5,
        MOVE_XYZ = 1 << 6,
        MOVE_ALL = (1 << 7) - 1
    };
    MOVETYPE m_MoveType, m_MoveTypePredict;
    // tplane m_plan;
    // tvector3 m_LockVertex;
    int m_AxisDir;
    tvector3 m_MoveSnap;
    unsigned int mDrawMask;

    bool GetOpType(MOVETYPE& type, int &axis_dir, unsigned int x, unsigned int y, bool mousedown = true);
    tvector3 RayTrace(tvector3& rayOrigin, tvector3& rayDir, tvector3& norm);
};

#endif  // !defined(AFX_GIZMOTRANSFORMMOVE_H__8276C568_C663_463C_AE7F_B913E2A712A4__INCLUDED_)

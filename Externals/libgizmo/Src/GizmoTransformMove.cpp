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

#include "GizmoTransformMove.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

IGizmo* CreateMoveGizmo() {
    return new CGizmoTransformMove;
}

tvector3 ptd;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CGizmoTransformMove::CGizmoTransformMove() : CGizmoTransform() {
    m_MoveType = CGizmoTransformMove::MOVETYPE::MOVE_NONE;
    m_MoveTypePredict = CGizmoTransformMove::MOVETYPE::MOVE_NONE;
    mMask = MOVE_X | MOVE_Y | MOVE_Z;
    mDrawMask = mMask;
    m_AxisDir = 0;
}

CGizmoTransformMove::~CGizmoTransformMove() {}

tvector3 CGizmoTransformMove::RayTrace(tvector3& rayOrigin, tvector3& rayDir, tvector3& norm) {
    tvector3 df, inters;
    m_plan = vector4(m_pMatrix->GetTranslation(), norm);
    m_plan.RayInter(inters, rayOrigin, rayDir);
    ptd = inters;
    df = inters - m_pMatrix->GetTranslation();
    df /= GetScreenFactor();
    m_LockVertex = inters;
    return df;
}

bool CGizmoTransformMove::GetOpType(MOVETYPE& type, int& axis_dir, unsigned int x, unsigned int y,
                                    bool mousedown) {
    tvector3 rayOrigin, rayDir, df;
    float absx, absy, absz;
    BuildRay(x, y, rayOrigin, rayDir);
    m_svgMatrix = *m_pMatrix;

    tvector3 trss(GetTransformedVector(0).Length(), GetTransformedVector(1).Length(),
                  GetTransformedVector(2).Length());

    tmatrix mt;
    if (mLocation == LOCATE_LOCAL) {
        mt = *m_pMatrix;
        mt.Inverse();
    } else {
        // world
        mt.Translation(-m_pMatrix->V4.position);
    }
    type = MOVE_NONE;
    axis_dir = 0;
    float minRange = 0.6f;
    float maxRange = 1.0f;
    float planRange = 0.5f;
    float planeMinRange = 0.17677669529f;
    float planeMaxRange = planeMinRange + planRange + minRange;
    // plan 1 : X/Z
    df = RayTrace2(rayOrigin, rayDir, GetTransformedVector(1), mt, trss, false);
    absx = (float)fabs(df.x);
    absz = (float)fabs(df.z);

    if ((mMask & MOVE_X) && (absx >= minRange) && (absx <= maxRange) && (absz < mDetectionRange)) {
        type = MOVE_X;
        axis_dir = fabs(absx - df.x) > FLOAT_EPSILON ? -1 : 1;
    } else if ((mMask & MOVE_Z) &&(absz >= minRange) && (absz <= maxRange) && (absx < mDetectionRange)) {
        type = MOVE_Z;
        axis_dir = fabs(absz - df.z) > FLOAT_EPSILON ? -1 : 1;
    } else if ((mMask & MOVE_XZ) &&(df.x < planeMaxRange) && (df.z < planeMaxRange) && (df.x >= planeMinRange) &&
               (df.z >= planeMinRange)) {
        type = MOVE_XZ;
    } else {
        // plan 2 : X/Y
        df = RayTrace2(rayOrigin, rayDir, GetTransformedVector(2), mt, trss, false);
        absx = (float)fabs(df.x);
        absy = (float)fabs(df.y);

        if ((mMask & MOVE_X) &&(absx >= minRange) && (absx <= maxRange) && (absy < mDetectionRange)) {
            type = MOVE_X;
            axis_dir = fabs(absx - df.x) > FLOAT_EPSILON ? -1 : 1;
        } else if ((mMask & MOVE_Y) &&(absy >= minRange) && (absy <= maxRange) && (absx < mDetectionRange)) {
            type = MOVE_Y;
            axis_dir = fabs(absy - df.y) > FLOAT_EPSILON ? -1 : 1;
        } else if ((mMask & MOVE_XY) &&(df.x < planeMaxRange) && (df.y < planeMaxRange) && (df.x > planeMinRange) &&
                   (df.y > planeMinRange)) {
            type = MOVE_XY;
        } else {
            // plan 3: Y/Z
            df = RayTrace2(rayOrigin, rayDir, GetTransformedVector(0), mt, trss, false);
            absy = (float)fabs(df.y);
            absz = (float)fabs(df.z);

            if ((mMask & MOVE_Y) &&(absy >= minRange) && (absy <= maxRange) && (absz < mDetectionRange)) {
                type = MOVE_Y;
                axis_dir = fabs(absy - df.y) > FLOAT_EPSILON ? -1 : 1;
            } else if ((mMask & MOVE_Z) &&(absz >= minRange) && (absz <= maxRange) && (absy < mDetectionRange)) {
                type = MOVE_Z;
                axis_dir = fabs(absz - df.z) > FLOAT_EPSILON ? -1 : 1;
            } else if ((mMask & MOVE_YZ) &&(df.y < planeMaxRange) && (df.z < planeMaxRange) && (df.y > planeMinRange) &&
                       (df.z > planeMinRange)) {
                type = MOVE_YZ;
            }
        }
    }
    if (mousedown) {
        if (type & mMask) {
            mDrawMask = type;
            return true;
        } else {
            mDrawMask = mMask;
            axis_dir = 0;
            type = MOVE_NONE;
            return false;
        }
    } else {
        return false;
    }
}

bool CGizmoTransformMove::OnMouseDown(unsigned int x, unsigned int y) {
    if (m_pMatrix) {
        return GetOpType(m_MoveType, m_AxisDir, x, y);
    }

    m_MoveType = MOVE_NONE;
    return false;
}

void CGizmoTransformMove::OnMouseMove(unsigned int x, unsigned int y) {
    if (m_MoveType != MOVE_NONE) {
        tvector3 rayOrigin, rayDir, df, inters;

        BuildRay(x, y, rayOrigin, rayDir);
        m_plan.RayInter(inters, rayOrigin, rayDir);

        tvector3 axeX(1, 0, 0), axeY(0, 1, 0), axeZ(0, 0, 1);

        if (mLocation == LOCATE_LOCAL) {
            axeX.TransformVector(*m_pMatrix);
            axeY.TransformVector(*m_pMatrix);
            axeZ.TransformVector(*m_pMatrix);
            axeX.Normalize();
            axeY.Normalize();
            axeZ.Normalize();
        }

        df = inters - m_LockVertex;

        switch (m_MoveType) {
        case MOVE_XZ:
            df = tvector3(df.Dot(axeX), 0, df.Dot(axeZ));
            break;
        case MOVE_X:
            df = tvector3(df.Dot(axeX), 0, 0);
            break;
        case MOVE_Z:
            df = tvector3(0, 0, df.Dot(axeZ));
            break;
        case MOVE_XY:
            df = tvector3(df.Dot(axeX), df.Dot(axeY), 0);
            break;
        case MOVE_YZ:
            df = tvector3(0, df.Dot(axeY), df.Dot(axeZ));
            break;
        case MOVE_Y:
            df = tvector3(0, df.Dot(axeY), 0);
            break;
        }

        tvector3 adf;

        tmatrix mt;
        if (m_bUseSnap) {
            SnapIt(df.x, m_MoveSnap.x);
            SnapIt(df.y, m_MoveSnap.y);
            SnapIt(df.z, m_MoveSnap.z);
        }

        adf = df.x * axeX + df.y * axeY + df.z * axeZ;

        mt.Translation(adf);
        *m_pMatrix = m_svgMatrix;
        m_pMatrix->Multiply(mt);
        // if (mTransform) mTransform->Update();

        if (mEditPos)
            *mEditPos = m_pMatrix->V4.position;
    } else {
        // predict move
        if (m_pMatrix) {
            GetOpType(m_MoveTypePredict, m_AxisDir, x, y, false);
        }
    }
}

void CGizmoTransformMove::OnMouseUp(unsigned int x, unsigned int y) {
    m_MoveType = MOVE_NONE;
    mDrawMask = mMask;
}

void CGizmoTransformMove::Draw() {
    ComputeScreenFactor();

    if (m_pMatrix) {
        // glDisable(GL_DEPTH_TEST);
        tvector3 orig = m_pMatrix->GetTranslation();

        tvector3 axeX(1, 0, 0), axeY(0, 1, 0), axeZ(0, 0, 1);

        tvector3 colorYZ(1, 0, 0), colorXZ(0, 1, 0), colorXY(0, 0, 1);
        tvector4 colorX(1, 0, 0, 1), colorY(0, 1, 0, 1), colorZ(0, 0, 1, 1);
        tvector4 colorHovered(0.5f, 0.5f, 0.5f, 0.5f);

        float maxRange = 1.0f - mDetectionRange;
        float midRange = 0.5f;
        float minRange = mDetectionRange;

        if (mLocation == LOCATE_LOCAL) {
            axeX.TransformVector(*m_pMatrix);
            axeY.TransformVector(*m_pMatrix);
            axeZ.TransformVector(*m_pMatrix);
            axeX.Normalize();
            axeY.Normalize();
            axeZ.Normalize();
        }

        if (mDrawMask & MOVE_YZ) {
            bool hovered = !(m_MoveTypePredict != MOVE_YZ || m_MoveType == MOVE_YZ);
            DrawQuad(orig, GetScreenFactor(), axeY, axeZ, hovered ? colorHovered : colorYZ,
                     midRange);
        }

        if (mDrawMask & MOVE_XZ) {
            bool hovered = !(m_MoveTypePredict != MOVE_XZ || m_MoveType == MOVE_XZ);
            DrawQuad(orig, GetScreenFactor(), axeX, axeZ, hovered ? colorHovered : colorXZ,
                     midRange);
        }

        if (mDrawMask & MOVE_XY) {
            bool hovered = !(m_MoveTypePredict != MOVE_XY || m_MoveType == MOVE_XY);
            DrawQuad(orig, GetScreenFactor(), axeX, axeY, hovered ? colorHovered : colorXY,
                     midRange);
        }

        axeX *= GetScreenFactor();
        axeY *= GetScreenFactor();
        axeZ *= GetScreenFactor();

        if (mDrawMask & MOVE_XYZ) {
            DrawSphere(orig, axeX.Length() * 0.1f, {0, 0, 0, 1});
        }
        bool positve = false;
        bool negative = false;
        if (mDrawMask & MOVE_X) {
            positve = m_MoveTypePredict == MOVE_X && m_MoveType == MOVE_NONE && m_AxisDir == 1;
            negative = m_MoveTypePredict == MOVE_X && m_MoveType == MOVE_NONE && m_AxisDir == -1;
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_X && m_AxisDir == 1)) {
                DrawAxis(orig, axeX, axeY, axeZ, minRange, positve ? colorHovered : colorX);
            }
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_X && m_AxisDir == -1)) {
                DrawAxis(orig, -axeX, axeY, axeZ, minRange, negative ? colorHovered : colorX);
            }
        }

        if (mDrawMask & MOVE_Y) {
            positve = m_MoveTypePredict == MOVE_Y && m_MoveType == MOVE_NONE && m_AxisDir == 1;
            negative = m_MoveTypePredict == MOVE_Y && m_MoveType == MOVE_NONE && m_AxisDir == -1;
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_Y && m_AxisDir == 1)) {
                DrawAxis(orig, axeY, axeX, axeZ, minRange, positve ? colorHovered : colorY);
            }
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_Y && m_AxisDir == -1)) {
                DrawAxis(orig, -axeY, axeX, axeZ, minRange, negative ? colorHovered : colorY);
            }
        }

        if (mDrawMask & MOVE_Z) {
            positve = m_MoveTypePredict == MOVE_Z && m_MoveType == MOVE_NONE && m_AxisDir == 1;
            negative = m_MoveTypePredict == MOVE_Z && m_MoveType == MOVE_NONE && m_AxisDir == -1;
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_Z && m_AxisDir == 1)) {
                DrawAxis(orig, axeZ, axeX, axeY, minRange, positve ? colorHovered : colorZ);
            }
            if (m_MoveType == MOVE_NONE || (m_MoveType == MOVE_Z && m_AxisDir == -1)) {
                DrawAxis(orig, -axeZ, axeX, axeY, minRange, negative ? colorHovered : colorZ);
            }
        }
    }
    // debug
    // glPointSize(20);
    // glBegin(GL_POINTS);
    // glVertex3fv(&ptd.x);
    // glEnd();
    // glEnable(GL_DEPTH_TEST);
}

void CGizmoTransformMove::ApplyTransform(tvector3& trans, bool bAbsolute) {
    if (bAbsolute) {
        m_pMatrix->m16[12] = trans.x;
        m_pMatrix->m16[13] = trans.y;
        m_pMatrix->m16[14] = trans.z;
    } else {
        m_pMatrix->m16[12] += trans.x;
        m_pMatrix->m16[13] += trans.y;
        m_pMatrix->m16[14] += trans.z;
    }
}

void CGizmoTransformMove::SetAxisMask(unsigned int mask) {
    if (mask == 0xffff) {
        mask = MOVE_ALL;
    }
    if (mask > MOVE_ALL || mask <= MOVE_NONE)
        return;
    mMask = mask;
    mDrawMask = mask;
}

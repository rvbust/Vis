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

#include "GizmoTransformRotate.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
extern tvector3 ptd;

IGizmo *CreateRotateGizmo() {
    return new CGizmoTransformRotate;
}

CGizmoTransformRotate::CGizmoTransformRotate() : CGizmoTransform() {
    m_RotateType = ROTATE_NONE;
    m_RotateTypePredict = ROTATE_NONE;
    mMask = AXIS_X | AXIS_Y | AXIS_Z;
    mDrawMask = mMask;
    m_Ng2 = 0;
    m_AngleSnap = 0.f;
}

CGizmoTransformRotate::~CGizmoTransformRotate() {}

bool CGizmoTransformRotate::CheckRotatePlan(tvector3 &vNorm, float factor, const tvector3 &rayOrig,
                                            const tvector3 &rayDir, int id) {
    tvector3 df, inters;
    float minRange = (1.0f - mDetectionRange) * factor;
    float maxRange = (1.0f + mDetectionRange) * factor;
    m_Axis2 = vNorm;
    m_plane = vector4(m_pMatrix->GetTranslation(), vNorm);
    m_plane.RayInter(inters, rayOrig, rayDir);  // ��ƽ������ߵĽ���
    // ptd = inters;
    df = inters - m_pMatrix->GetTranslation();  // ��ý��������ĵľ���
    float len = df.Length();

    if ((len > minRange) && (len < maxRange)) {
        m_svgMatrix = *m_pMatrix;

        m_LockVertex = df;
        m_LockVertex.Normalize();

        m_Vtx = m_LockVertex;
        m_Vty.Cross(m_LockVertex, vNorm);
        m_Vty.Normalize();
        m_Vtx *= factor;
        m_Vty *= factor;
        m_Vtz.Cross(m_Vtx, m_Vty);
        m_Ng2 = 0;
        if (id != -1)
            m_Axis = GetVector(id);

        m_OrigScale.Scaling(GetTransformedVector(0).Length(), GetTransformedVector(1).Length(),
                            GetTransformedVector(2).Length());

        m_InvOrigScale.Inverse(m_OrigScale);

        return true;
    }
    return false;
}

bool CGizmoTransformRotate::GetOpType(ROTATETYPE &type, unsigned int x, unsigned int y) {
    tvector3 rayOrigin, rayDir, axis;
    tvector3 dir = m_pMatrix->GetTranslation() - m_CamSrc;
    dir.Normalize();

    BuildRay(x, y, rayOrigin, rayDir);
    float factor = GetScreenFactor() * 0.5f;

    if (mMask & AXIS_TRACKBALL)
        if (CheckRotatePlan(dir, factor, rayOrigin, rayDir, -1)) {
            tmatrix mt = *m_pMatrix;
            mt.NoTrans();
            // mt.Inverse();
            // m_Axis = m_Axis2 = dir;
            m_Axis.TransformPoint(mt);
            /*
            m_Axis *=tvector3(GetTransformedVector(0).Length(),
            GetTransformedVector(1).Length(),
            GetTransformedVector(2).Length());
            */
            type = ROTATE_TWIN;
            mDrawMask = AXIS_TRACKBALL;
            return true;
        }

    // plan 1 : X/Z
    m_Axis = GetTransformedVector(0) * factor;
    if (mMask & AXIS_X)
        if (CheckRotatePlan(m_Axis, factor, rayOrigin, rayDir, 0)) {
            type = ROTATE_X;
            mDrawMask = AXIS_X;
            return true;
        }
    m_Axis = GetTransformedVector(1) * factor;
    if (mMask & AXIS_Y)
        if (CheckRotatePlan(m_Axis, factor, rayOrigin, rayDir, 1)) {
            type = ROTATE_Y;
            mDrawMask = AXIS_Y;
            return true;
        }
    m_Axis = GetTransformedVector(2) * factor;
    if (mMask & AXIS_Z)
        if (CheckRotatePlan(m_Axis, factor, rayOrigin, rayDir, 2)) {
            type = ROTATE_Z;
            mDrawMask = AXIS_Z;
            return true;
        }

    // m_Axis = GetTransformedVector(dir);
    if (mMask & AXIS_SCREEN)
        if (CheckRotatePlan(dir, 1.2f, rayOrigin, rayDir, -1)) {
            tmatrix mt = *m_pMatrix;
            mt.NoTrans();
            mt.Inverse();
            m_Axis = m_Axis2 = dir;
            m_Axis.TransformPoint(mt);

            m_Axis *= tvector3(GetTransformedVector(0).Length(), GetTransformedVector(1).Length(),
                               GetTransformedVector(2).Length());

            type = ROTATE_SCREEN;
            mDrawMask = AXIS_SCREEN;
            return true;
        }

    type = ROTATE_NONE;
    mDrawMask = mMask;

    return false;
}

bool CGizmoTransformRotate::OnMouseDown(unsigned int x, unsigned int y) {
    if (m_pMatrix) {
        return GetOpType(m_RotateType, x, y);
    }

    m_RotateType = ROTATE_NONE;
    return false;
}

void CGizmoTransformRotate::Rotate1Axe(const tvector3 &rayOrigin, const tvector3 &rayDir) {
    tvector3 inters;
    m_plane = vector4(m_pMatrix->GetTranslation(), m_Axis2);
    m_plane.RayInter(inters, rayOrigin, rayDir);
    ptd = inters;

    tvector3 df = inters - m_pMatrix->GetTranslation();

    df.Normalize();
    m_LockVertex2 = df;

    float acosng = df.Dot(m_LockVertex);
    if ((acosng < -0.99999f) || (acosng > 0.99999f))
        m_Ng2 = 0.f;
    else
        m_Ng2 = (float)acos(acosng);

    if (df.Dot(m_Vty) > 0)
        m_Ng2 = -m_Ng2;

    tmatrix mt, mt2;

    if (m_bUseSnap) {
        m_Ng2 *= (360.0f / ZPI);
        SnapIt(m_Ng2, m_AngleSnap);
        m_Ng2 /= (360.0f / ZPI);
    }

    mt.RotationAxis(m_Axis, m_Ng2);
    mt.Multiply(m_InvOrigScale);
    mt.Multiply(m_svgMatrix);
    mt2 = m_OrigScale;
    mt2.Multiply(mt);
    *m_pMatrix = mt2;

    if (m_Axis == tvector3::ZAxis) {
        if (mEditQT) {
            /*
            Dans le cadre du jeu, et vu les pb avec les quaternions,
            le 1er float du quaternion en stockage est l'angle en radian.
            le stockage reste un quaternion.
            il y a des pbs de conversion quaternion/matrix
            */
#if USE_QUATERNION
            tquaternion gth(*m_pMatrix);

            gth.Normalize();
            gth.UnitInverse();

            tquaternion qtg;
            qtg.RotationAxis(m_Axis, m_Ng2);
            *mEditQT = gth;  // gth+qtg;//tquaternion(mt2);
            mEditQT->Normalize();
#else
            mEditQT->z = m_Ng2;
#endif
        }
    }
}

void CGizmoTransformRotate::OnMouseMove(unsigned int x, unsigned int y) {
    tvector3 rayOrigin, rayDir, axis;

    BuildRay(x, y, rayOrigin, rayDir);

    if (m_RotateType != ROTATE_NONE) {
        if (m_RotateType == ROTATE_TWIN) {
            tvector3 inters;
            tvector3 dir = m_pMatrix->GetTranslation() - m_CamSrc;
            dir.Normalize();

            m_plane = vector4(m_pMatrix->GetTranslation(), dir);
            m_plane.RayInter(inters, rayOrigin, rayDir);
            ptd = inters;
            tvector3 df = inters - m_pMatrix->GetTranslation();
            df /= GetScreenFactor();
            float lng1 = df.Length();
            if (lng1 >= 1.0f)
                lng1 = 0.9f;

            float height = (float)sin(acos(lng1));
            tvector3 m_CamRealUp, camRight;
            camRight.Cross(m_Axis, m_CamUp);
            m_CamRealUp.Cross(camRight, dir);

            tvector3 idt = height * dir;
            idt += df;

            // ptd = idt;
            idt = m_LockVertex - idt;

            tmatrix mt, mt2;

            mt.LookAtLH(tvector3(0, 0, 0), idt, m_CamRealUp);
            mt.Multiply(m_InvOrigScale);
            mt.Multiply(m_svgMatrix);
            mt2 = m_OrigScale;
            mt2.Multiply(mt);
            *m_pMatrix = mt2;
            /*
            if (mEditQT)
            {
            *mEditQT = tquaternion(mt2);
            }
            */
        } else {
            Rotate1Axe(rayOrigin, rayDir);
        }

        // if (mTransform) mTransform->Update();
    } else {
        // predict move
        if (m_pMatrix) {
            GetOpType(m_RotateTypePredict, x, y);
        }
    }
}

void CGizmoTransformRotate::OnMouseUp(unsigned int x, unsigned int y) {
    m_RotateType = ROTATE_NONE;
}
/*
            char tmps[512];
            sprintf(tmps, "%5.2f %5.2f %5.2f %5.2f", plCam.x, plCam.y, plCam.z, plCam.w );
            MessageBoxA(NULL, tmps, tmps, MB_OK);
            */
void CGizmoTransformRotate::Draw() {
    if (m_pMatrix) {
        ComputeScreenFactor();

        tvector3 right, up, frnt, dir;

        tvector4 colorX(1.0f, 0.0f, 0.0f, 1.0f), colorY(0.0f, 1.0f, 0.0f, 1.0f),
            colorZ(0.0f, 0.0f, 1.0f, 1.0f);
        tvector4 colorTB(0.2f, 0.2f, 0.2f, 1), colorTwin(1.0f, 0.3f, 1.0f, 1),
            coloHovered(0.5f, 0.5f, 0.5f, 0.5f);
        tvector4 colorCamem(1.0f, 1.0f, 0.0f, 0.5f);

        // glDisable(GL_DEPTH_TEST);
        tvector3 orig(m_pMatrix->GetTranslation());

        tvector3 plnorm(m_CamSrc - orig);

        plnorm.Normalize();

        tplane plCam = vector4(plnorm, 0);

        dir = orig - m_CamSrc;
        dir.Normalize();

        right.Cross(dir, GetTransformedVector(1));
        right.Normalize();

        up.Cross(dir, right);
        up.Normalize();

        right.Cross(dir, up);
        right.Normalize();

        tvector3 axeX(1, 0, 0), axeY(0, 1, 0), axeZ(0, 0, 1);

        if (mLocation == LOCATE_LOCAL) {
            axeX.TransformVector(*m_pMatrix);
            axeY.TransformVector(*m_pMatrix);
            axeZ.TransformVector(*m_pMatrix);
            axeX.Normalize();
            axeY.Normalize();
            axeZ.Normalize();
        }

        int drawMask = m_RotateType == ROTATE_NONE ? mMask : mDrawMask;
        float scale = mDetectionRange;
        float factor = 0.5f * GetScreenFactor();
        // Twin
        if (drawMask & AXIS_TRACKBALL) {
            bool hovered = !(m_RotateTypePredict != ROTATE_TWIN || m_RotateType == ROTATE_TWIN);
            DrawCircle(orig, right * factor, up * factor, hovered ? coloHovered : colorTB, scale);
        }

        // Screen
        if (drawMask & AXIS_SCREEN) {
            bool hovered = !(m_RotateTypePredict != ROTATE_SCREEN || m_RotateType == ROTATE_SCREEN);
            DrawCircle(orig, up * 1.2f * factor, right * 1.2f * factor,
                       hovered ? coloHovered : colorTwin, scale);
        }

        // X
        right.Cross(dir, axeX);
        right.Normalize();
        frnt.Cross(right, axeX);

        frnt.Normalize();

        if (drawMask & AXIS_X) {
            bool hovered = !(m_RotateTypePredict != ROTATE_X || m_RotateType == ROTATE_X);
            DrawCircle(orig, right * factor, frnt * factor, hovered ? coloHovered : colorX, scale * factor);
        }

        // Y

        right.Cross(dir, axeY);
        right.Normalize();
        frnt.Cross(right, axeY);

        frnt.Normalize();

        if (drawMask & AXIS_Y) {
            bool hovered = !(m_RotateTypePredict != ROTATE_Y || m_RotateType == ROTATE_Y);
            DrawCircle(orig, right * factor, frnt * factor, hovered ? coloHovered : colorY, scale* factor);
        }

        // Z
        right.Cross(dir, axeZ);
        right.Normalize();
        frnt.Cross(right, axeZ);

        frnt.Normalize();

        if (drawMask & AXIS_Z) {
            bool hovered = !(m_RotateTypePredict != ROTATE_Z || m_RotateType == ROTATE_Z);
            DrawCircle(orig, right * factor, frnt * factor, hovered ? coloHovered : colorZ, scale* factor);
        }
        // camembert
        if ((m_RotateType != ROTATE_NONE) && (m_RotateType != ROTATE_TWIN)) {
            DrawCamem(orig, m_Vtx, m_Vty, -m_Ng2, colorCamem);
        }

        //// debug
        // glPointSize(20);
        // glBegin(GL_POINTS);
        // glVertex3fv(&ptd.x);
        // glEnd();
        // glEnable(GL_DEPTH_TEST);

#if 0
#ifdef WIN32
        GDD->GetD3D9Device()->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
        GDD->GetD3D9Device()->SetRenderState(D3DRS_CULLMODE , D3DCULL_NONE );
        GDD->GetD3D9Device()->SetRenderState(D3DRS_ZENABLE , D3DZB_TRUE);
        GDD->GetD3D9Device()->SetRenderState(D3DRS_ALPHATESTENABLE , FALSE);
        GDD->GetD3D9Device()->SetRenderState(D3DRS_ZWRITEENABLE , TRUE);
#endif
        extern RenderingState_t GRenderingState;
        GRenderingState.mAlphaTestEnable = 0;
        GRenderingState.mZWriteEnable = 1;
        GRenderingState.mBlending = 0;
        GRenderingState.mCulling = 0;
        GRenderingState.mZTestType = 1;
#endif
    }
}

void CGizmoTransformRotate::ApplyTransform(tvector3 &trans, bool bAbsolute) {
    tmatrix mt;
    m_OrigScale.Scaling(GetTransformedVector(0).Length(), GetTransformedVector(1).Length(),
                        GetTransformedVector(2).Length());

    if (bAbsolute) {
        tvector3 translation = m_pMatrix->GetTranslation();

        // X
        mt.RotationAxis(GetVector(0), ((trans.x / 360) * ZPI));
        mt.Multiply(m_OrigScale);
        *m_pMatrix = mt;
        // Y
        mt.RotationAxis(GetVector(1), ((trans.y / 360) * ZPI));
        mt.Multiply(m_OrigScale);
        *m_pMatrix = mt;
        // Z
        mt.RotationAxis(GetVector(2), ((trans.z / 360) * ZPI));
        mt.Multiply(m_OrigScale);
        *m_pMatrix = mt;

        // translate
        m_pMatrix->m16[12] = translation.x;
        m_pMatrix->m16[13] = translation.y;
        m_pMatrix->m16[14] = translation.z;
    } else {
        tmatrix mt2;
        m_InvOrigScale.Inverse(m_OrigScale);

        if (trans.x != 0) {
            m_svgMatrix = *m_pMatrix;
            mt.RotationAxis(GetVector(0), ((trans.x / 360) * ZPI));
            mt.Multiply(m_InvOrigScale);
            mt.Multiply(m_svgMatrix);
            mt2 = m_OrigScale;
            mt2.Multiply(mt);
            *m_pMatrix = mt2;
        }
        if (trans.y != 0) {
            m_svgMatrix = *m_pMatrix;
            mt.RotationAxis(GetVector(1), ((trans.y / 360) * ZPI));
            mt.Multiply(m_InvOrigScale);
            mt.Multiply(m_svgMatrix);
            mt2 = m_OrigScale;
            mt2.Multiply(mt);
            *m_pMatrix = mt2;
        }
        if (trans.z != 0) {
            m_svgMatrix = *m_pMatrix;
            mt.RotationAxis(GetVector(2), ((trans.z / 360) * ZPI));
            mt.Multiply(m_InvOrigScale);
            mt.Multiply(m_svgMatrix);
            mt2 = m_OrigScale;
            mt2.Multiply(mt);
            *m_pMatrix = mt2;
        }
    }
}

void CGizmoTransformRotate::SetAxisMask(unsigned int mask) {
    if (mask == 0xffff) {
        mask = AXIS_X | AXIS_Y | AXIS_Z;
    }
    if (mask > AXIS_ALL || mask < AXIS_X)
        return;
    mMask = mask;
    mDrawMask = mask;
}
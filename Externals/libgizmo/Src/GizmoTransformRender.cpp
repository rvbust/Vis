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

#include "GizmoTransformRender.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

static const int slices = 50;

void DrawCustomCone(const tvector3 &top, const tvector3 &bottom, const tvector3 &vtx,
                    const tvector3 &vty, float scale, const tvector4 &col) {
    tvector3 vt;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);

    glColor4fv(&col.x);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3fv(&top.x);
    for (int i = 0; i <= slices; i++) {
        float tp = (2.0f * ZPI) / slices * i;
        vt = vtx * (float)cos(tp) * scale;
        vt += vty * (float)sin(tp) * scale;
        vt += bottom;
        glVertex3f(vt.x, vt.y, vt.z);
    }
    glEnd();
}

void DrawCustomCircle(const tvector3 &orig, const tvector3 &vtx, const tvector3 &vty, float ng,
                      float scale, const tvector4 &col) {
    tvector3 vt;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);

    glColor4fv(&col.x);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3fv(&orig.x);
    for (int i = 0; i <= slices; i++) {
        float tp = ng / slices * i;
        vt = vtx * (float)cos(tp) * scale;
        vt += vty * (float)sin(tp) * scale;
        vt += orig;
        glVertex3f(vt.x, vt.y, vt.z);
    }
    glEnd();
}

void DrawCustomCylinder(const tvector3 &top, const tvector3 &bottom, const tvector3 &vtx,
                        const tvector3 &vty, float scale, const tvector4 &col) {
    tvector3 _top, _bottom;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glColor4fv(&col.x);
    glBegin(GL_QUAD_STRIP);  //连续填充四边形串
    for (int i = 0; i <= slices; i++) {
        float tp = ((2.0f * ZPI) / slices) * i;
        _top = vtx * (float)cos(tp) * scale;
        _top += vty * (float)sin(tp) * scale;
        _bottom = _top + bottom;
        _top += top;
        glVertex3f(_top.x, _top.y, _top.z);
        glVertex3f(_bottom.x, _bottom.y, _bottom.z);
    }
    glEnd();
}

void DrawCustomCylinder(const tvector3 &top, const tvector3 &bottom, const tvector3 &vtx,
                        const tvector3 &vty, const tvector3 &vty2, float scale,
                        const tvector4 &col) {
    tvector3 _top, _bottom;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glColor4fv(&col.x);
    glBegin(GL_QUAD_STRIP);  //连续填充四边形串
    for (int i = 0; i <= slices; i++) {
        float tp = ((2.0f * ZPI) / slices) * i;
        _top = vtx * (float)cos(tp) * scale;
        _top += vty * (float)sin(tp) * scale;
        _top += top;
        _bottom = vtx * (float)cos(tp) * scale;
        _bottom += vty2 * (float)sin(tp) * scale;
        _bottom += bottom;
        glVertex3f(_top.x, _top.y, _top.z);
        glVertex3f(_bottom.x, _bottom.y, _bottom.z);
    }
    glEnd();
}

void CGizmoTransformRender::DrawCircle(const tvector3 &orig, const tvector3 &vtx,
                                       const tvector3 &vty, const tvector4 &col, float scale) {
    tvector3 top, bottom, vtz, vtw, vtw2;
    float half_scale = scale * 0.5f;
    vtz.Cross(vtx, vty);
    vtz.Normalize();
    vtz *= half_scale;

    for (int i = 0; i < slices; i += 1) {
        float tp = (2 * ZPI / slices) * i;
        top = vtx * (float)cos(tp);
        top += vty * (float)sin(tp);
        top += orig;
        vtw = top - orig;
        vtw.Normalize();
        vtw *= half_scale;

        float tp2 = (2 * ZPI / slices) * (i + 1);
        bottom = vtx * (float)cos(tp2);
        bottom += vty * (float)sin(tp2);
        bottom += orig;
        vtw2 = bottom - orig;
        vtw2.Normalize();
        vtw2 *= half_scale;
        DrawCustomCylinder(top, bottom, vtz, vtw, vtw2, 1.0f, col);
    }
}

void CGizmoTransformRender::DrawCircleHalf(const tvector3 &orig, float r, float g, float b,
                                           const tvector3 &vtx, const tvector3 &vty,
                                           tplane &camPlan) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4f(r, g, b, 1);

    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < slices; i++) {
        tvector3 vt;
        float tp = (2 * ZPI / slices) * i;
        vt = vtx * (float)cos(tp);
        vt += vty * (float)sin(tp);
        vt += orig;
        if (camPlan.DotNormal(vt)) {
            glVertex3f(vt.x, vt.y, vt.z);
        }
    }
    glEnd();
}

void CGizmoTransformRender::DrawSphere(const tvector3 &orig, float radius, const tvector4 &col) {
    double step_z = ZPI / slices;
    double step_xy = 2 * ZPI / slices;
    float x[4], y[4], z[4];

    double angle_z = 0.0;
    double angle_xy = 0.0;
    int i = 0, j = 0;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4fv(&col.x);
    glBegin(GL_QUADS);
    for (i = 0; i < slices; i++) {
        angle_z = i * step_z;

        for (j = 0; j < slices; j++) {
            angle_xy = j * step_xy;
            x[0] = (float)(radius * sin(angle_z) * cos(angle_xy));
            y[0] = (float)(radius * sin(angle_z) * sin(angle_xy));
            z[0] = (float)(radius * cos(angle_z));

            x[1] = (float)(radius * sin(angle_z + step_z) * cos(angle_xy));
            y[1] = (float)(radius * sin(angle_z + step_z) * sin(angle_xy));
            z[1] = (float)(radius * cos(angle_z + step_z));

            x[2] = (float)(radius * sin(angle_z + step_z) * cos(angle_xy + step_xy));
            y[2] = (float)(radius * sin(angle_z + step_z) * sin(angle_xy + step_xy));
            z[2] = (float)(radius * cos(angle_z + step_z));

            x[3] = (float)(radius * sin(angle_z) * cos(angle_xy + step_xy));
            y[3] = (float)(radius * sin(angle_z) * sin(angle_xy + step_xy));
            z[3] = (float)(radius * cos(angle_z));
            for (int k = 0; k < 4; k++) {
                glVertex3f(orig.x + x[k], orig.y + y[k], orig.z + z[k]);
            }
        }
    }
    glEnd();
}

void CGizmoTransformRender::DrawAxis(const tvector3 &orig, const tvector3 &axis_norm,
                                     const tvector3 &axis_x, const tvector3 &axis_y, float scale,
                                     const tvector4 &col) {
    float cone_scale = 0.5f;
    auto cylinder_begin = orig + axis_norm * 0.60f;
    auto cylinder_end = orig + axis_norm * 0.80f;
    auto arrow_end = orig + axis_norm;
    DrawCustomCircle(cylinder_begin, axis_x, axis_y, 2 * ZPI, scale * cone_scale, col);
    DrawCustomCylinder(cylinder_begin, cylinder_end, axis_x, axis_y, scale * cone_scale, col);
    DrawCustomCircle(cylinder_end, axis_x, axis_y, 2 * ZPI, scale, col);
    DrawCustomCone(arrow_end, cylinder_end, axis_x, axis_y, scale, col);
}

void CGizmoTransformRender::DrawCamem(const tvector3 &orig, const tvector3 &vtx,
                                      const tvector3 &vty, float ng, const tvector4 &col) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    int i = 0;
    tvector3 vt;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);

    glColor4fv(&col.x);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3fv(&orig.x);
    for (i = 0; i <= slices; i++) {
        float tp = ((ng) / slices) * i;
        vt = vtx * (float)cos(tp);
        vt += vty * (float)sin(tp);
        vt += orig;
        glVertex3f(vt.x, vt.y, vt.z);
    }
    glEnd();

    glDisable(GL_BLEND);

    glColor4f(col.x, col.y, col.z, 1.0f);
    glBegin(GL_LINE);
    // line1
    glVertex3fv(&orig.x);
    vt = vtx * (float)cos(0);
    vt += vty * (float)sin(0);
    vt += orig;
    glVertex3f(vt.x, vt.y, vt.z);

    // line2
    glVertex3fv(&orig.x);
    vt = vtx * (float)cos(ng);
    vt += vty * (float)sin(ng);
    vt += orig;
    glVertex3f(vt.x, vt.y, vt.z);
    glEnd();
}

void CGizmoTransformRender::DrawQuad(const tvector3 &orig, float size, const tvector3 &axisU,
                                     const tvector3 &axisV, const tvector3 &color,
                                     float dec_range) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);

    float det_size = size * dec_range;
    float offset_size = size * SQRT2 * (1.0f - dec_range) * 0.25f;

    tvector3 pts[4];
    pts[0] = orig + (axisU * offset_size) + (axisV * offset_size);
    pts[1] = pts[0] + (axisU * det_size);
    pts[2] = pts[0] + (axisU + axisV) * det_size;
    pts[3] = pts[0] + (axisV * det_size);

    glColor4f(color[0], color[1], color[2], 0.5f);
    glBegin(GL_QUADS);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glVertex3fv(&pts[3].x);
    glEnd();

    glColor4f(color[0], color[1], color[2], 1);
    glBegin(GL_LINE_STRIP);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glVertex3fv(&pts[3].x);
    glEnd();

    glDisable(GL_BLEND);
}

void CGizmoTransformRender::DrawTri(const tvector3 &orig, float size, bool bSelected,
                                    const tvector3 &axisU, const tvector3 &axisV) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);

    tvector3 pts[3];
    pts[0] = orig;

    pts[1] = (axisU);
    pts[2] = (axisV);

    pts[1] *= size;
    pts[2] *= size;
    pts[1] += orig;
    pts[2] += orig;

    if (!bSelected)
        glColor4f(1, 1, 0, 0.5f);
    else
        glColor4f(1, 1, 1, 0.6f);

    glBegin(GL_TRIANGLES);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    // glVertex3fv(&pts[0].x);
    glEnd();

    if (!bSelected)
        glColor4f(1, 1, 0.2f, 1);
    else
        glColor4f(1, 1, 1, 0.6f);

    glBegin(GL_LINE_STRIP);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glEnd();

    glDisable(GL_BLEND);
}

#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright 2021 RVBUST Inc.

from RVBUST import Vis
from IPython import embed

def ShowArrow():
    single_arrow = v.Arrow([1, -1, 1], [3, -3, 3], 0.1, [1, 0, 0])
    multiple_arrow_with_single_color = v.Arrow([1, 1, 1, -2, 2, 2],
                                               [3, 3, 3, -4, 4, 4], 0.1, [1,1,0])
    multiple_arrow_with_multiple_color = v.Arrow([1, 2, 1, -2, 1, 2],
                                                 [3, 4, 3, -4, 3, 4], 0.1,
                                                 [0, 1, 1, 1, 0, 1])


def ShowBox():
    transparent_box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    normal_box = v.Box([0, 2, 0], [0.5, 0.5, 0.5], [1, 0, 0])


def MoveBox():
    transparent_box = v.Box([-2, 0, 0], [0.5, 0.5, 0.5], [1, 0, 0, 0.5])
    v.EnableGizmo(transparent_box, 4)
    v.SetGizmoDisplayScale(2)


def ShowPointAndLine():
    pos = [
        0.7071, 0.0000, -0.0000, -0.7071, -0.6533, 0.2706, 0.2706, 0.6533,
        -0.5000, 0.5000, 0.5000, 0.5000, -0.2706, 0.6533, 0.6533, 0.2706,
        -0.0000, 0.7071, 0.7071, 0.0000, 0.2706, 0.6533, 0.6533, -0.2706,
        0.5000, 0.5000, 0.5000, -0.5000, -0.6533, -0.2706
    ]
    blue_points = v.Point(pos, 2, [0, 0, 1])
    red_lines = v.Line(pos)


def ShowSphere():
    single_sphere = v.Sphere([0, 2, 0], 0.5, [1, 0, 0])
    multiple_spheres = v.Spheres([0, 2, 2, 0, 4, 2], [1, 0.5],
                                 [0, 1, 0, 1, 0, 0, 1, 0.5])


def ShowCone():
    single_cone = v.Cone([2, 0, 0], 0.5, 1, [1, 1, 0])


def ShowCylinder():
    single_cylinder = v.Cylinder([4, 2, 0], 0.5, 1, [0, 1, 1])


def ShowMesh():
    vertices = [
        2, 2, 1, 1.5, 1.5, 0, 2.5, 1.5, 0, 2.5, 2.5, 0, 1.5, 2.5, 0, 2, 2, -1
    ]
    indices = [
        0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 5, 2, 1, 5, 3, 2, 5, 4, 3, 5, 1, 4
    ]
    colors = [1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1]
    diamond_mesh = v.Mesh(vertices, indices, colors)
    single_diamond_mesh = v.Mesh(vertices, indices, [1, 1, 0])
    v.SetPosition(single_diamond_mesh, [0, 2, 0])


def ShowPlane():
    grey_floor = v.Plane(2, 2, color=[0.5, 0.5, 0.5])
    grey_floor_with_transfrom = v.Plane(2, 2, 4, 4, [-2, 0, 0], [0.3, 0, 0, 1],
                                        [0.5, 0.5, 0.5])


def Show2DText():
    b = v.SetTextFont("SourceCodePro-Regular.ttf")
    t1 = v.Text("测试2DText1", [100, 100])

    t2 = v.Text("测试2DText2", [100, 100], 0, [1, 0, 0.5])

    t3 = v.Text("测试2DText3", [300, 300], 10, [])


def Show3DText():
    b = v.SetTextFont("SourceCodePro-Regular.ttf")
    d1 = v.Text("测试3DText1", [0, 4, 0], 0.1)

    d2 = v.Text("测试3DText2", [2, 0, 0], 0.2, [])

    d2 = v.Text("测试3DText3", [0, 0, 2], 0.3, [1, 0, 1, 0.5])


def ShowQuad2D():
    Parallelogram_in_screen = v.Quad2D([0, 0, 50, 0, 100, 50, 50, 50],
                                       [0, 1, 1])
    rectangle_in_screen = v.Quad2D([10, 100, 50, 100, 50, 300, 10, 300],
                                   [0, 1, 1])


def Basic():
    Vis.SetLogLevel("debug")
    axes1 = v.Axes([0, 0, 0], [0, 0, 0, 1], 2, 1)
    ShowBox()
    ShowPointAndLine()
    ShowSphere()
    ShowCone()
    ShowCylinder()
    ShowArrow()
    ShowMesh()
    ShowPlane()

    Show2DText()
    Show3DText()
    ShowQuad2D()

    MoveBox()
    pass


if __name__ == "__main__":
    cfg = Vis.ViewConfig()
    cfg.x = 0
    cfg.y = 40
    cfg.width = 800
    cfg.height = 600
    cfg.name = "Basics"
    v = Vis.View(cfg, False)
    Basic()
    v.Home()
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    

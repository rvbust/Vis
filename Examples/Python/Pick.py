#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright 2021 RVBUST Inc.

'''
Vis Picking例子
Intersector交互模式
- 点击鼠标右键进行选择
- 同时按住Ctrl可以进行多选
- Axes Marker的大小可以通过按键盘上的 “+/-”来快捷调整 
'''

import numpy as np
from RVBUST import Vis

def PickDemo():
    v = Vis.View("PickDemo")

    h1 = v.Axes([0, 0, 0], [0, 0, 0, 1], 0.5, 2)
    h3 = v.Box([1, 0, 0], [0.1, 0.1, 0.1])
    h5 = v.Point(np.random.uniform(-1, 1, 90), 5, [0, 1, 0])
    h6 = v.Line([0, 0, 0, 1, 1, 1], 1)


    v.SetIntersectorMode(Vis.IntersectorMode_LineSegment)
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    hs = v.GetPickedPointAxes()  
    for h in hs:
        print(v.GetPosition(h)[1])
    v.ClearPickedPointAxes()

    v.SetIntersectorMode(Vis.IntersectorMode_Polytope)
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    hs = v.MultiPicked()  
    for h in hs:
        print(h)

    v.SetIntersectorMode(Vis.IntersectorMode_Point)
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    hs = v.GetPickedPointAxes()
    for h in hs:
        print(v.GetPosition(h))
    v.ClearPickedPointAxes()

    v.SetIntersectorMode(Vis.IntersectorMode_Line)
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    hs = v.MultiPicked()  
    for h in hs:
        print(h)

if __name__ == '__main__':
    PickDemo()

#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from RVBUST import Vis
import time


def TransformDemo():
    v = Vis.View("Transform")

    files_path = ["../../Data/Models/Visual/BaseLink.stl", "../../Data/Models/Visual/Link1.stl", "../../Data/Models/Visual/Link2.stl",
                  "../../Data/Models/Visual/Link3.stl", "../../Data/Models/Visual/Link4.stl", "../../Data/Models/Visual/Link5.stl", "../../Data/Models/Visual/Link6.stl"]

    hs = v.Load(files_path)

    res, position = v.GetPosition(hs[0])
    res, rotation = v.GetRotation(hs[0])
    res, position, rotation = v.GetTransform(hs[0])

    v.SetPosition(hs[0], position)
    v.SetRotation(hs[0], rotation)
    v.SetTransform(hs[0], position, rotation)
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())


    state1_pos = [[0.0, 0, 0], [0, 0, 0], [0.05, 0, 0.33], [0.05, 0, 0.66], [
        0.05, 0, 0.695], [0.385, 0, 0.695], [0.385, 0, 0.615]]
    state1_rot = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [
        0, 0, 0, 1], [0, 0.707108, 0, 0.707105], [0, 0.707108, 0, 0.707105]]


    state2_pos = [[0, 0, 0], [0, 0, 0], [0.05, 0, 0.33], [0.335788, 0, 0.495], [
        0.335788, 0, 0.53], [0.670788, 0, 0.53], [0.670788, 0, 0.45]]
    state2_rot = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0.5, 0, 0.866025], [0, 0, 0, 1], [
        0, 0, 0, 1], [0, 0.707107, 0, 0.707107],  [0, 0.707107, 0, 0.707107]]


    state3_pos = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 0.05, 0.33],
                  [0, 0.3357, 0.495], [0, 0.3357, 0.53], [0, 0.6707, 0.53],
                  [0, 0.6707, 0.45]]

    state3_rot = [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.7071, 0.7071],
                  [-0.3535, 0.35355, 0.61237, 0.61237],
                  [0.0, 0.0, 0.7071, 0.7071],
                  [0.0, 0.0, 0.7071, 0.7071],
                  [-0.5, 0.5, 0.5, 0.5],
                  [-0.5, 0.5, 0.5, 0.5]]

    v.SetTransforms(hs, state1_pos, state1_rot)
    time.sleep(0.5)
    v.SetTransforms(hs, state2_pos, state2_rot)
    time.sleep(0.5)
    v.SetTransforms(hs, state3_pos, state3_rot)

    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())

if __name__ == '__main__':
    TransformDemo()

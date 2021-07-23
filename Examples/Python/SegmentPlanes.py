#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright 2021 RVBUST Inc.

from RVBUST import Vis 
import numpy as np
try:
    import open3d as o3d
except:
    print("Open3D is needed for running this example.")
    raise 
import time

def RandColor():
    return np.random.rand(3)

def SegmentPlaneDemo():
    v = Vis.View("SegementPlane")
    pcd = o3d.io.read_point_cloud("../../Data/TLS_kitchen_sample.ply")
    pts = np.array(pcd.points).copy()
    
    v.Point(pts, 1, (1,0,0))
    v.Home()
    
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)

    segment_models={}
    segments={}
    rest=pcd
    d_threshold=0.01
        
    for i in range(10):
        segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        segments[i]=rest.select_by_index(inliers)
        labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*10, min_points=10))
        candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
        best_candidate=int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])
        print("the best candidate is: ", best_candidate)
        rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
        segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
        
        plane_pts = np.array(segments[i].points)
        v.Point(plane_pts, 3, RandColor())
        time.sleep(0.3)
        
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())

if __name__ == '__main__':
    SegmentPlaneDemo()

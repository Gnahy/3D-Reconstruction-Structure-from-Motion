#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 31 14:22:56 2026

@author: gyar0001
"""

import os
import cv2
import numpy as np

def extract_uniform_frames(video_path, out_dir, num_frames=100):
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    indices = np.linspace(0, total - 1, num_frames).astype(int)

    for i, idx in enumerate(indices):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(os.path.join(out_dir, f"frame_{i:03d}.jpg"), frame)

    cap.release()

extract_uniform_frames(video_path="chair.mp4", out_dir="frames", num_frames=100)


#%%

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("points3D.ply")
o3d.visualization.draw_geometries([pcd])

#%%

import os, copy
import numpy as np
import open3d as o3d

def q2r(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ], float)

def read_cam(images_txt, cam_idx=0):
    cams = []
    with open(images_txt) as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    for i in range(0, len(lines), 2):
        parts = lines[i].split()
        q = np.array(list(map(float, parts[1:5])))
        t = np.array(list(map(float, parts[5:8])))
        name = parts[9]
        R = q2r(q)
        C = -R.T @ t
        v = R.T @ np.array([0.0, 0.0, 1.0])
        v = v / np.linalg.norm(v)
        cams.append((name, C, v))

    return cams[cam_idx % len(cams)]

def main(ply="points3D.ply", images_txt="sparse_txt/images.txt",
         out_dir="results", cam_idx=0, i=100, j=500):

    os.makedirs(out_dir, exist_ok=True)

    pcd = o3d.io.read_point_cloud(ply)
    name, C, v = read_cam(images_txt, cam_idx)

    pcd_tr = copy.deepcopy(pcd)
    pcd_tr.translate(5.0 * v, relative=True)
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(v * (-np.deg2rad(60)))
    pcd_tr.rotate(R, center=pcd_tr.get_center())
    o3d.io.write_point_cloud(os.path.join(out_dir, "translated_rotated.ply"), pcd_tr)

    pcd_sc = copy.deepcopy(pcd_tr)
    pcd_sc.scale(5.0, center=pcd_sc.get_center())
    o3d.io.write_point_cloud(os.path.join(out_dir, "translated_rotated_scaled.ply"), pcd_sc)

    P0 = np.asarray(pcd_tr.points)
    P1 = np.asarray(pcd_sc.points)
    d0 = np.linalg.norm(P0[i] - P0[j])
    d1 = np.linalg.norm(P1[i] - P1[j])

    print("Camera:", name)
    print("v:", v)
    print("d_before:", d0)
    print("d_after :", d1)
    print("expected:", d0 * 5.0)

if __name__ == "__main__":
    main()

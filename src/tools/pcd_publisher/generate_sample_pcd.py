import os
import argparse
import open3d as o3d
import numpy as np


def generate_sample_pcd(path, num_points=2000, seed=42):
    np.random.seed(seed)
    points = np.random.randn(num_points, 3).astype(np.float32) * 3.0
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(path, pcd, write_ascii=False)
    print(f"Wrote sample PCD -> {path}  ({num_points} points)")


if __name__ == "__main__":
    resource_dir = os.path.join(os.path.dirname(__file__))
    out = os.path.join(resource_dir, "sample.pcd")
    generate_sample_pcd(out)

#!/usr/bin/env python3
"""
Visualize and inspect PCD/PLY/NPY point clouds.
Generates PNG images from multiple viewpoints. No GUI or GPU required.

Usage:
    python3 visualize_pointcloud.py global_cloud.ply
    python3 visualize_pointcloud.py global_cloud.pcd --voxel 0.1
    python3 visualize_pointcloud.py global_cloud.npy --color height --views all
    python3 visualize_pointcloud.py global_cloud.ply local_map.ply --overlay
"""
import numpy as np
import argparse
import os
import sys


def load_ply(filepath):
    """Load binary little-endian PLY with x,y,z float32 fields."""
    with open(filepath, 'rb') as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if b"end_header" in line:
                break
        # Parse vertex count from header
        n_vertices = 0
        for line in header.decode('ascii').split('\n'):
            if line.startswith('element vertex'):
                n_vertices = int(line.split()[-1])
        data = np.frombuffer(f.read(n_vertices * 12), dtype=np.float32)
    return data.reshape(-1, 3)


def load_pcd(filepath):
    """Load ASCII PCD file."""
    points = []
    data_started = False
    with open(filepath, 'r') as f:
        for line in f:
            if data_started:
                parts = line.strip().split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            elif line.strip() == 'DATA ascii':
                data_started = True
    return np.array(points, dtype=np.float32) if points else np.empty((0, 3))


def load_npy(filepath):
    """Load numpy array."""
    return np.load(filepath).astype(np.float32)


def load_pointcloud(filepath):
    """Auto-detect format and load."""
    ext = os.path.splitext(filepath)[1].lower()
    if ext == '.ply':
        pts = load_ply(filepath)
    elif ext == '.pcd':
        pts = load_pcd(filepath)
    elif ext == '.npy':
        pts = load_npy(filepath)
    else:
        raise ValueError(f"Unsupported format: {ext}")
    print(f"Loaded {filepath}: {pts.shape[0]:,} points")
    return pts


def voxel_downsample(points, voxel_size):
    """Voxel grid downsampling."""
    if voxel_size <= 0 or points.shape[0] == 0:
        return points
    indices = np.floor(points / voxel_size).astype(np.int32)
    _, unique_idx = np.unique(indices, axis=0, return_index=True)
    result = points[unique_idx]
    print(f"Downsampled: {points.shape[0]:,} -> {result.shape[0]:,} points (voxel={voxel_size}m)")
    return result


def compute_colors(points, mode='height'):
    """Compute per-point colors for visualization."""
    if points.shape[0] == 0:
        return np.zeros(0, dtype=np.float64)

    if mode == 'height':
        values = points[:, 2].astype(np.float64)
    elif mode == 'depth':
        values = points[:, 0].astype(np.float64)
    elif mode == 'distance':
        values = np.linalg.norm(points, axis=1).astype(np.float64)
    elif mode == 'intensity':
        values = np.linalg.norm(points[:, :2], axis=1).astype(np.float64)
    else:
        values = points[:, 2].astype(np.float64)

    # Normalize to [0, 1] with outlier clipping
    if len(values) < 2:
        return np.full(len(values), 0.5)
    p5, p95 = np.percentile(values, [2, 98])
    values = np.clip(values, p5, p95)
    if p95 > p5:
        values = (values - p5) / (p95 - p5)
    else:
        values = np.full_like(values, 0.5)
    return values


def print_stats(points, name="Cloud"):
    """Print point cloud statistics."""
    print(f"\n{'='*50}")
    print(f"  {name}")
    print(f"{'='*50}")
    print(f"  Points:     {points.shape[0]:,}")
    print(f"  X range:    [{points[:,0].min():.3f}, {points[:,0].max():.3f}] m  (span: {points[:,0].ptp():.3f})")
    print(f"  Y range:    [{points[:,1].min():.3f}, {points[:,1].max():.3f}] m  (span: {points[:,1].ptp():.3f})")
    print(f"  Z range:    [{points[:,2].min():.3f}, {points[:,2].max():.3f}] m  (span: {points[:,2].ptp():.3f})")
    print(f"  Centroid:   [{points[:,0].mean():.3f}, {points[:,1].mean():.3f}, {points[:,2].mean():.3f}]")
    dists = np.linalg.norm(points - points.mean(axis=0), axis=1)
    print(f"  Mean dist from centroid: {dists.mean():.3f} m")
    print(f"  Max dist from centroid:  {dists.max():.3f} m")
    print(f"{'='*50}\n")


def render_view(ax, points, colors, cmap, elev, azim, title, point_size=0.3, alpha=0.6):
    """Render a single 3D view."""
    if points.shape[0] == 0:
        ax.set_title(title + " (no data)")
        return
    # Ensure colors match points
    if len(colors) != len(points):
        colors = np.full(len(points), 0.5)
    ax.view_init(elev=elev, azim=azim)
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=colors, cmap=cmap, s=point_size, alpha=alpha,
               edgecolors='none', rasterized=True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title, fontsize=11, fontweight='bold')

    # Equal aspect ratio
    max_range = max(points[:, 0].ptp(), points[:, 1].ptp(), points[:, 2].ptp()) / 2
    if max_range == 0:
        max_range = 1.0
    mid = points.mean(axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)


def render_2d_view(ax, points, colors, cmap, axis1, axis2, title, point_size=0.3, alpha=0.6):
    """Render a 2D projection."""
    if points.shape[0] == 0:
        ax.set_title(title + " (no data)")
        return
    if len(colors) != len(points):
        colors = np.full(len(points), 0.5)
    labels = {'x': (0, 'X (m)'), 'y': (1, 'Y (m)'), 'z': (2, 'Z (m)')}
    i1, l1 = labels[axis1]
    i2, l2 = labels[axis2]
    ax.scatter(points[:, i1], points[:, i2],
               c=colors, cmap=cmap, s=point_size, alpha=alpha,
               edgecolors='none', rasterized=True)
    ax.set_xlabel(l1)
    ax.set_ylabel(l2)
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)


def generate_views(points, colors, output_dir, prefix, cmap='viridis', point_size=0.3,
                   views='all', dpi=150):
    """Generate multiple view PNGs."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    os.makedirs(output_dir, exist_ok=True)

    # Subsample for rendering if too many points
    max_render = 500_000
    if points.shape[0] > max_render:
        idx = np.random.choice(points.shape[0], max_render, replace=False)
        pts_render = points[idx]
        col_render = colors[idx]
        print(f"Subsampled to {max_render:,} points for rendering")
    else:
        pts_render = points
        col_render = colors

    generated = []

    # --- 3D views ---
    if views in ('all', '3d'):
        view_configs = [
            (30, -60, "3D Perspective"),
            (90, -90, "Top Down (XY)"),
            (0, -90, "Front (XZ)"),
            (0, 0, "Side (YZ)"),
        ]

        fig = plt.figure(figsize=(20, 16))
        for i, (elev, azim, title) in enumerate(view_configs):
            ax = fig.add_subplot(2, 2, i + 1, projection='3d')
            render_view(ax, pts_render, col_render, cmap, elev, azim, title, point_size)

        fig.suptitle(f'{prefix} — Point Cloud ({points.shape[0]:,} points)', fontsize=14, fontweight='bold')
        plt.tight_layout()
        path = os.path.join(output_dir, f"{prefix}_3d_views.png")
        plt.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close()
        generated.append(path)
        print(f"Saved: {path}")

    # --- 2D projections ---
    if views in ('all', '2d'):
        fig, axes = plt.subplots(1, 3, figsize=(21, 6))

        projections = [
            ('x', 'y', 'Top View (XY)'),
            ('x', 'z', 'Side View (XZ)'),
            ('y', 'z', 'Side View (YZ)'),
        ]
        for ax, (a1, a2, title) in zip(axes, projections):
            render_2d_view(ax, pts_render, col_render, cmap, a1, a2, title, point_size)

        fig.suptitle(f'{prefix} — 2D Projections ({points.shape[0]:,} points)', fontsize=14, fontweight='bold')
        plt.tight_layout()
        path = os.path.join(output_dir, f"{prefix}_2d_projections.png")
        plt.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close()
        generated.append(path)
        print(f"Saved: {path}")

    # --- Density / histogram ---
    if views in ('all', 'stats'):
        fig, axes = plt.subplots(1, 3, figsize=(18, 5))
        for ax, (dim, label) in zip(axes, [(0, 'X'), (1, 'Y'), (2, 'Z')]):
            ax.hist(points[:, dim], bins=100, color='steelblue', alpha=0.7, edgecolor='none')
            ax.set_xlabel(f'{label} (m)')
            ax.set_ylabel('Count')
            ax.set_title(f'{label} Distribution')
            ax.axvline(points[:, dim].mean(), color='red', linestyle='--', label=f'mean={points[:, dim].mean():.2f}')
            ax.legend()

        fig.suptitle(f'{prefix} — Spatial Distribution', fontsize=14, fontweight='bold')
        plt.tight_layout()
        path = os.path.join(output_dir, f"{prefix}_distribution.png")
        plt.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close()
        generated.append(path)
        print(f"Saved: {path}")

    return generated


def overlay_clouds(clouds_list, names, output_dir, dpi=150):
    """Overlay multiple point clouds with different colors."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    cloud_colors = ['#2196F3', '#F44336', '#4CAF50', '#FF9800', '#9C27B0']
    max_render = 300_000

    fig, axes = plt.subplots(1, 3, figsize=(21, 6))
    projections = [
        (0, 1, 'Top View (XY)', 'X (m)', 'Y (m)'),
        (0, 2, 'Side View (XZ)', 'X (m)', 'Z (m)'),
        (1, 2, 'Side View (YZ)', 'Y (m)', 'Z (m)'),
    ]

    for ax, (i1, i2, title, xl, yl) in zip(axes, projections):
        for pts, name, color in zip(clouds_list, names, cloud_colors):
            if pts.shape[0] > max_render:
                idx = np.random.choice(pts.shape[0], max_render, replace=False)
                pts_sub = pts[idx]
            else:
                pts_sub = pts
            ax.scatter(pts_sub[:, i1], pts_sub[:, i2], s=0.2, alpha=0.4,
                       color=color, label=f'{name} ({pts.shape[0]:,})', edgecolors='none')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(markerscale=10, fontsize=9)

    fig.suptitle('Point Cloud Overlay', fontsize=14, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(output_dir, "overlay_comparison.png")
    plt.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close()
    print(f"Saved: {path}")
    return path


def main():
    parser = argparse.ArgumentParser(
        description='Visualize and inspect PCD/PLY/NPY point clouds',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s global_cloud.ply
  %(prog)s global_cloud.pcd --voxel 0.1 --color distance
  %(prog)s global_cloud.ply local_map.ply --overlay
  %(prog)s global_cloud.npy --views 2d --dpi 300 --output ./figures
        """)
    parser.add_argument('files', nargs='+', help='Point cloud files (PLY/PCD/NPY)')
    parser.add_argument('--voxel', type=float, default=0.0,
                        help='Voxel downsample size in meters (0=off, default: 0)')
    parser.add_argument('--color', choices=['height', 'depth', 'distance', 'intensity'],
                        default='height', help='Coloring mode (default: height)')
    parser.add_argument('--cmap', default='viridis',
                        help='Matplotlib colormap (default: viridis)')
    parser.add_argument('--views', choices=['all', '3d', '2d', 'stats'],
                        default='all', help='Which views to generate (default: all)')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory (default: same as input file)')
    parser.add_argument('--dpi', type=int, default=150,
                        help='Output image DPI (default: 150)')
    parser.add_argument('--point-size', type=float, default=0.3,
                        help='Point size for rendering (default: 0.3)')
    parser.add_argument('--overlay', action='store_true',
                        help='Overlay multiple clouds in one figure')
    parser.add_argument('--stats-only', action='store_true',
                        help='Only print statistics, no images')

    args = parser.parse_args()

    # Load all clouds
    clouds = []
    names = []
    for filepath in args.files:
        if not os.path.exists(filepath):
            print(f"ERROR: File not found: {filepath}")
            sys.exit(1)
        pts = load_pointcloud(filepath)
        if args.voxel > 0:
            pts = voxel_downsample(pts, args.voxel)
        clouds.append(pts)
        names.append(os.path.splitext(os.path.basename(filepath))[0])

    # Print stats for all
    for pts, name in zip(clouds, names):
        print_stats(pts, name)

    if args.stats_only:
        return

    output_dir = args.output or os.path.dirname(os.path.abspath(args.files[0]))

    # Overlay mode
    if args.overlay and len(clouds) > 1:
        overlay_clouds(clouds, names, output_dir, args.dpi)

    # Individual views
    for pts, name in zip(clouds, names):
        colors = compute_colors(pts, args.color)
        generate_views(pts, colors, output_dir, name,
                       cmap=args.cmap, point_size=args.point_size,
                       views=args.views, dpi=args.dpi)

    print(f"\nAll outputs saved to: {output_dir}")


if __name__ == '__main__':
    main()
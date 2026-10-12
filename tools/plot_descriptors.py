#!/usr/bin/env python3
"""Plot exported STD, BTC and Scan Context descriptors from one frame."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_triangles(path):
    triangles = []
    with open(path, newline='') as stream:
        for row in csv.DictReader(stream):
            triangles.append(np.array([
                [float(row[f'{key}x']), float(row[f'{key}y'])]
                for key in ('a', 'b', 'c')
            ]))
    return triangles


def read_btc_bits(path):
    with open(path, newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return None
    row = max(rows, key=lambda item: sum(item[f'{key}_bits'].count('1')
                                         for key in ('a', 'b', 'c')))
    bits = [row[f'{key}_bits'] for key in ('a', 'b', 'c')]
    width = max(map(len, bits))
    matrix = np.zeros((3, width), dtype=np.uint8)
    for index, string in enumerate(bits):
        matrix[index, :len(string)] = [int(value) for value in string]
    return matrix


def plot_triangles(ax, triangles, color, max_triangles):
    if not triangles:
        return
    indices = np.linspace(0, len(triangles) - 1,
                          min(max_triangles, len(triangles)), dtype=int)
    for index in indices:
        points = triangles[index]
        closed = np.vstack((points, points[0]))
        ax.plot(closed[:, 0], closed[:, 1], color=color, alpha=0.55, linewidth=0.9)
        ax.scatter(points[:, 0], points[:, 1], s=5, color=color, alpha=0.8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--std-csv', type=Path, required=True)
    parser.add_argument('--btc-csv', type=Path, required=True)
    parser.add_argument('--scancontext-csv', type=Path, required=True)
    parser.add_argument('--points-bin', type=Path,
                        help='Optional KITTI float32 x,y,z,intensity scan')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-triangles', type=int, default=50)
    args = parser.parse_args()

    std = read_triangles(args.std_csv)
    btc = read_triangles(args.btc_csv)
    btc_bits = read_btc_bits(args.btc_csv)
    sc = np.loadtxt(args.scancontext_csv, delimiter=',')
    if sc.shape != (20, 60):
        raise ValueError(f'Expected a 20x60 Scan Context, got {sc.shape}')
    points = None
    if args.points_bin:
        raw = np.fromfile(args.points_bin, dtype=np.float32)
        if raw.size % 4:
            raise ValueError('KITTI bin must contain x,y,z,intensity float32 tuples')
        points = raw.reshape(-1, 4)[::10, :2]

    fig, axes = plt.subplots(1, 4, figsize=(20, 5), constrained_layout=True)
    for ax, descriptors, color, title in zip(
            axes[:2], (std, btc), ('#1e88e5', '#e86b2f'), ('STD', 'BTC')):
        if points is not None:
            ax.scatter(points[:, 0], points[:, 1], s=0.15,
                       color='#767676', alpha=0.35, rasterized=True)
        plot_triangles(ax, descriptors, color, args.max_triangles)
        ax.set_title(f'{title}: {len(descriptors)} descriptors')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=0.15)
    image = axes[2].imshow(sc, origin='lower', aspect='auto', cmap='magma',
                           extent=[0, 360, 0, 80], interpolation='nearest')
    axes[2].set_title('Scan Context: 20 rings × 60 sectors')
    axes[2].set_xlabel('Azimuth (degrees)')
    axes[2].set_ylabel('Radius (m)')
    fig.colorbar(image, ax=axes[2], label='Maximum height (m)')
    if btc_bits is not None:
        axes[3].imshow(btc_bits, aspect='auto', cmap='Greys',
                       vmin=0, vmax=1, interpolation='nearest')
        axes[3].set_yticks([0, 1, 2], ['A', 'B', 'C'])
        axes[3].set_xlabel('Occupancy bit index')
        axes[3].set_ylabel('Triangle vertex')
        axes[3].set_title('BTC binary codes (one triangle)')
    else:
        axes[3].text(0.5, 0.5, 'No BTC descriptors',
                     ha='center', va='center', transform=axes[3].transAxes)
        axes[3].set_axis_off()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == '__main__':
    main()

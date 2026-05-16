#!/usr/bin/env python3
"""
STOP escape event visualizer.
Usage:
  python3 viz.py              # load latest stop_event_*.json
  python3 viz.py <file.json>  # load specific file
"""
import json, sys, glob, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

SECTOR_SIZE     = 10    # degrees (must match jw_won.py)
ROBOT_HW        = 110   # half-width mm
STOP_FWD_MIN    = 100
STOP_FWD_MAX    = 180
STOP_HORIZ_TH   = 110
MIN_PASSAGE     = 260   # mm


def to_xy(angle_deg, dist_mm):
    r = math.radians(angle_deg)
    return dist_mm * math.sin(r), dist_mm * math.cos(r)  # x=lateral, y=fwd


def draw_sector_wedge(ax, center_deg, radius, color, alpha=0.25, zorder=2):
    a0 = math.radians(center_deg - SECTOR_SIZE / 2)
    a1 = math.radians(center_deg + SECTOR_SIZE / 2)
    thetas = np.linspace(a0, a1, 20)
    xs = np.concatenate([[0], radius * np.sin(thetas), [0]])
    ys = np.concatenate([[0], radius * np.cos(thetas), [0]])
    ax.fill(xs, ys, color=color, alpha=alpha, zorder=zorder)


def visualize(path):
    with open(path) as f:
        d = json.load(f)

    scan        = d['scan']           # [[angle, dist], ...]
    heading     = d['heading']        # global heading at STOP trigger (deg)
    target      = d['target']         # chosen escape angle (robot-local deg)
    gap_dist    = d['gap_dist']       # avg LIDAR range in chosen sector (mm)
    sector_info = {float(k): v for k, v in d['sector_info'].items()}

    # ── figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle(
        f'STOP Escape Event  |  Robot heading={heading:.1f}°  '
        f'Chosen escape={target:+.0f}° (robot-local)  '
        f'Avg range in sector={gap_dist:.0f}mm',
        fontsize=12, fontweight='bold'
    )
    ax1 = fig.add_subplot(1, 2, 1)   # top-down view
    ax2 = fig.add_subplot(1, 2, 2)   # sector bar chart

    # ── color scale (avg_dist: red=close/blocked, green=far/open) ─────────────
    all_dists = [v['avg_dist'] for v in sector_info.values()]
    norm  = Normalize(vmin=min(all_dists), vmax=max(all_dists))
    cmap  = plt.cm.RdYlGn

    # ── TOP-DOWN VIEW ─────────────────────────────────────────────────────────
    # 1. sector wedges
    for c, info in sorted(sector_info.items()):
        clr   = cmap(norm(info['avg_dist']))
        alpha = 0.55 if c == target else 0.20
        draw_sector_wedge(ax1, c, min(info['avg_dist'], 1400), clr, alpha=alpha)
        # passage width label on chosen sector
        if c == target:
            tx, ty = to_xy(c, min(info['avg_dist'] * 0.6, 900))
            ax1.text(tx, ty, f"{info['passage']:.0f}mm\npassage",
                     ha='center', va='center', fontsize=8,
                     color='darkgreen', fontweight='bold', zorder=7)

    # 2. LIDAR scan points (color by distance)
    pt_xs, pt_ys, pt_ds = [], [], []
    stop_xs, stop_ys = [], []
    for a, dist in scan:
        if dist <= 0:
            continue
        x, y = to_xy(a, dist)
        horiz = abs(dist * math.sin(math.radians(a)))
        fwd   = dist * math.cos(math.radians(a))
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            stop_xs.append(x); stop_ys.append(y)
        else:
            pt_xs.append(x); pt_ys.append(y); pt_ds.append(dist)

    sc = ax1.scatter(pt_xs, pt_ys, s=6, c=pt_ds, cmap='Blues_r',
                     vmin=0, vmax=1500, alpha=0.7, zorder=3, label='LIDAR points')
    if stop_xs:
        ax1.scatter(stop_xs, stop_ys, s=30, c='red', marker='x',
                    zorder=6, label='STOP trigger points')

    # 3. STOP zone rectangle
    stop_rect = patches.Rectangle(
        (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
        linewidth=2, edgecolor='red', facecolor='red', alpha=0.15,
        label=f'STOP zone (fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm)', zorder=4
    )
    ax1.add_patch(stop_rect)

    # 4. Robot body
    robot_rect = patches.FancyBboxPatch(
        (-ROBOT_HW, -80), ROBOT_HW * 2, 240,
        boxstyle='round,pad=5',
        linewidth=2, edgecolor='#333', facecolor='#888', alpha=0.55, zorder=4
    )
    ax1.add_patch(robot_rect)
    ax1.annotate('', xy=(0, 190), xytext=(0, 80),
                 arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
    ax1.text(0, 210, 'fwd', ha='center', va='bottom', fontsize=8, color='black')

    # 5. Escape direction arrow
    arrow_dist = min(gap_dist * 0.75, 1100)
    tx, ty = to_xy(target, arrow_dist)
    ax1.annotate('', xy=(tx, ty), xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color='limegreen', lw=3.5), zorder=6)
    lbl_x, lbl_y = to_xy(target, arrow_dist + 120)
    ax1.text(lbl_x, lbl_y,
             f'escape {target:+.0f}°\navg dist={gap_dist:.0f}mm\n'
             f'(≠ passage width!)',
             ha='center', fontsize=8, color='darkgreen', fontweight='bold', zorder=7,
             bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7))

    # 6. axes
    plt.colorbar(sc, ax=ax1, shrink=0.45, label='LIDAR range (mm)', pad=0.01)
    ax1.set_xlim(-1500, 1500)
    ax1.set_ylim(-500, 1600)
    ax1.set_aspect('equal')
    ax1.axhline(0, color='gray', lw=0.5, zorder=1)
    ax1.axvline(0, color='gray', lw=0.5, zorder=1)
    ax1.set_xlabel('← Robot Left  |  Lateral (mm)  |  Robot Right →')
    ax1.set_ylabel('Forward (mm) ↑')
    ax1.set_title('Top-down LIDAR view (robot frame)\nGreen wedge = chosen escape sector')
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(True, alpha=0.25, zorder=0)

    # ── SECTOR BAR CHART ──────────────────────────────────────────────────────
    centers  = sorted(sector_info.keys())
    avg_ds   = [sector_info[c]['avg_dist']  for c in centers]
    passages = [sector_info[c]['passage']   for c in centers]
    scores   = [sector_info[c]['score']     for c in centers]
    valids   = [sector_info[c]['valid']     for c in centers]

    bar_colors = [
        'limegreen' if c == target else
        ('#4CAF50' if v else '#EF9A9A')
        for c, v in zip(centers, valids)
    ]

    x = np.array(centers)
    w = SECTOR_SIZE * 0.75

    # avg dist bars (primary)
    bars = ax2.bar(x, avg_ds, width=w, color=bar_colors, alpha=0.75,
                   label='Avg LIDAR dist in sector', zorder=3)

    # passage width overlay (hatched)
    ax2.bar(x, passages, width=w, color='none',
            edgecolor='navy', linewidth=1.2, hatch='//', alpha=0.6,
            label='Passage width (gap_l+gap_r)', zorder=4)

    # score line
    ax2_r = ax2.twinx()
    ax2_r.plot(centers, scores, 'o--', color='purple', lw=1.5,
               markersize=5, label='forward_score', zorder=5)
    ax2_r.set_ylabel('forward_score (dist × cos factor)', color='purple', fontsize=9)
    ax2_r.tick_params(axis='y', labelcolor='purple')

    ax2.axvline(target, color='darkgreen', lw=2.5, linestyle='--',
                label=f'chosen {target:+.0f}°', zorder=5)
    ax2.axvline(0, color='black', lw=1.2, linestyle=':', label='0° forward', zorder=5)
    ax2.axhline(MIN_PASSAGE, color='orange', lw=1.5, linestyle='-.',
                label=f'min passage {MIN_PASSAGE}mm', zorder=5)

    # value labels on bars
    for bar, p in zip(bars, passages):
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, h + 10,
                 f'{p:.0f}', ha='center', va='bottom', fontsize=6, color='navy')

    ax2.set_xlabel('Sector center angle (°)\n← Left (negative) | Right (positive) →')
    ax2.set_ylabel('Distance (mm)')
    ax2.set_title(
        'Sector analysis\n'
        'Bar=avg dist (color: green=valid passage, red=too narrow)\n'
        'Hatch=passage width  Line=forward_score'
    )
    lines1, lbls1 = ax2.get_legend_handles_labels()
    lines2, lbls2 = ax2_r.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, lbls1 + lbls2, fontsize=7, loc='upper right')
    ax2.grid(True, alpha=0.25, zorder=0)
    ax2.set_xticks(range(-90, 91, 10))

    plt.tight_layout()
    out = path.replace('.json', '.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    try:
        plt.show()
    except Exception:
        pass


def load_latest():
    files = sorted(glob.glob('stop_event_*.json'))
    if not files:
        print("No stop_event_*.json files found in current directory.")
        return None
    print(f"Loading: {files[-1]}")
    return files[-1]


if __name__ == '__main__':
    fpath = sys.argv[1] if len(sys.argv) > 1 else load_latest()
    if fpath:
        visualize(fpath)

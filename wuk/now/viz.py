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

SECTOR_SIZE  = 10    # degrees (must match jw_won.py)
ROBOT_HW     = 110   # half-width mm
STOP_FWD_MIN = 100
STOP_FWD_MAX = 180
STOP_HORIZ_TH = 110
MIN_PASSAGE  = 260   # mm


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

    scan        = d['scan']
    heading     = d['heading']
    target      = d['target']
    gap_dist    = d['gap_dist']
    sector_info = {float(k): v for k, v in d['sector_info'].items()}

    chosen_passage = sector_info[target]['passage']
    chosen_gap_l   = sector_info[target]['gap_l']
    chosen_gap_r   = sector_info[target]['gap_r']

    # ── figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle(
        f'STOP Escape Event  |  Robot heading={heading:.1f}°  '
        f'Chosen escape={target:+.0f}° (robot-local)  '
        f'Passage width={chosen_passage:.0f}mm  (gap_L={chosen_gap_l:.0f} + gap_R={chosen_gap_r:.0f})',
        fontsize=12, fontweight='bold'
    )
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)

    # ── color scale: passage width (red=narrow/blocked, green=wide/open) ──────
    passages_all = [v['passage'] for v in sector_info.values()]
    norm = Normalize(vmin=0, vmax=max(passages_all))
    cmap = plt.cm.RdYlGn

    # ── TOP-DOWN VIEW ─────────────────────────────────────────────────────────
    # 1. sector wedges — radius = avg_dist (geometric), color = passage width
    for c, info in sorted(sector_info.items()):
        clr   = cmap(norm(info['passage']))
        alpha = 0.60 if c == target else 0.22
        draw_sector_wedge(ax1, c, min(info['avg_dist'], 1400), clr, alpha=alpha)

    # 2. chosen sector: gap_l / gap_r breakdown label
    ref_x, ref_y = to_xy(target, min(gap_dist * 0.55, 800))
    ax1.text(ref_x, ref_y,
             f'gap_L={chosen_gap_l:.0f}\ngap_R={chosen_gap_r:.0f}\ntotal={chosen_passage:.0f}mm',
             ha='center', va='center', fontsize=8,
             color='darkgreen', fontweight='bold', zorder=7,
             bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.75))

    # 3. LIDAR scan points (color by distance)
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
        ax1.scatter(stop_xs, stop_ys, s=35, c='red', marker='x',
                    zorder=6, label='STOP trigger points')

    # 4. STOP zone rectangle
    stop_rect = patches.Rectangle(
        (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
        linewidth=2, edgecolor='red', facecolor='red', alpha=0.15,
        label=f'STOP zone (fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm)', zorder=4
    )
    ax1.add_patch(stop_rect)

    # 5. Robot body
    robot_rect = patches.FancyBboxPatch(
        (-ROBOT_HW, -80), ROBOT_HW * 2, 240,
        boxstyle='round,pad=5',
        linewidth=2, edgecolor='#333', facecolor='#888', alpha=0.55, zorder=4
    )
    ax1.add_patch(robot_rect)
    ax1.annotate('', xy=(0, 190), xytext=(0, 80),
                 arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
    ax1.text(0, 210, 'fwd', ha='center', va='bottom', fontsize=8, color='black')

    # 6. Escape direction arrow
    arrow_dist = min(gap_dist * 0.75, 1100)
    tx, ty = to_xy(target, arrow_dist)
    ax1.annotate('', xy=(tx, ty), xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color='limegreen', lw=3.5), zorder=6)
    lbl_x, lbl_y = to_xy(target, arrow_dist + 130)
    ax1.text(lbl_x, lbl_y,
             f'escape {target:+.0f}°',
             ha='center', fontsize=9, color='darkgreen', fontweight='bold', zorder=7,
             bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7))

    # 7. colorbar (sector wedge color = passage width)
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    plt.colorbar(sm, ax=ax1, shrink=0.45,
                 label='Passage width gap_L+gap_R (mm)', pad=0.01)

    plt.colorbar(sc, ax=ax1, shrink=0.35,
                 label='LIDAR range (mm)', pad=0.08)

    ax1.set_xlim(-1500, 1500)
    ax1.set_ylim(-500, 1600)
    ax1.set_aspect('equal')
    ax1.axhline(0, color='gray', lw=0.5, zorder=1)
    ax1.axvline(0, color='gray', lw=0.5, zorder=1)
    ax1.set_xlabel('← Robot Left  |  Lateral (mm)  |  Robot Right →')
    ax1.set_ylabel('Forward (mm) ↑')
    ax1.set_title('Top-down LIDAR view (robot frame)\n'
                  'Wedge color = passage width (green=wide, red=narrow)')
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(True, alpha=0.25, zorder=0)

    # ── SECTOR BAR CHART (passage width 기준) ─────────────────────────────────
    centers = sorted(sector_info.keys())
    gap_ls  = [sector_info[c]['gap_l']   for c in centers]
    gap_rs  = [sector_info[c]['gap_r']   for c in centers]
    scores  = [sector_info[c]['score']   for c in centers]
    valids  = [sector_info[c]['valid']   for c in centers]

    bar_colors_l = ['#1a7a1a' if c == target else ('#388E3C' if v else '#E57373')
                    for c, v in zip(centers, valids)]
    bar_colors_r = ['#2ecc2e' if c == target else ('#66BB6A' if v else '#EF9A9A')
                    for c, v in zip(centers, valids)]

    x = np.array(centers)
    w = SECTOR_SIZE * 0.75

    # stacked bar: gap_l (bottom) + gap_r (top)
    ax2.bar(x, gap_ls, width=w, color=bar_colors_l, alpha=0.85,
            label='gap_L (left side to wall)', zorder=3)
    ax2.bar(x, gap_rs, width=w, bottom=gap_ls, color=bar_colors_r,
            alpha=0.85, label='gap_R (right side to wall)', zorder=3)

    # total passage width label on each bar
    for c, gl, gr, v in zip(centers, gap_ls, gap_rs, valids):
        total = gl + gr
        if total > 0:
            ax2.text(c, total + 8, f'{total:.0f}',
                     ha='center', va='bottom', fontsize=6,
                     color='darkgreen' if c == target else ('navy' if v else 'darkred'))

    # passage_score line (right y-axis)
    ax2_r = ax2.twinx()
    ax2_r.plot(centers, scores, 'o--', color='purple', lw=1.8,
               markersize=5, label='passage_score\n(passage × cos factor)', zorder=5)
    ax2_r.set_ylabel('passage_score', color='purple', fontsize=9)
    ax2_r.tick_params(axis='y', labelcolor='purple')

    ax2.axvline(target, color='darkgreen', lw=2.5, linestyle='--',
                label=f'chosen {target:+.0f}°', zorder=5)
    ax2.axvline(0, color='black', lw=1.2, linestyle=':', label='0° forward', zorder=5)
    ax2.axhline(MIN_PASSAGE, color='orange', lw=1.8, linestyle='-.',
                label=f'min passage {MIN_PASSAGE}mm', zorder=5)

    ax2.set_xlabel('Sector center angle (°)\n← Left (negative) | Right (positive) →')
    ax2.set_ylabel('Passage width = gap_L + gap_R (mm)')
    ax2.set_title('Sector passage width analysis\n'
                  'Stack: gap_L (dark) + gap_R (light)  |  Line: passage_score')
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

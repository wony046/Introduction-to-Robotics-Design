#!/usr/bin/env python3
"""
STOP escape event visualizer (FGM version).
JSON format: {heading, target, gap_dist (=chosen width), gap_info, scan}
Usage:
  python3 viz.py              # load latest stop_event_*.json
  python3 viz.py <file.json>  # load specific file
"""
import json, sys, glob, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

ROBOT_HW      = 110
STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 110
MIN_PASSAGE   = 260


def to_xy(angle_deg, dist_mm):
    r = math.radians(angle_deg)
    return dist_mm * math.sin(r), dist_mm * math.cos(r)  # x=lateral, y=fwd


def visualize(path):
    with open(path) as f:
        d = json.load(f)

    scan     = d['scan']
    heading  = d['heading']
    target   = d['target']
    gap_dist = d['gap_dist']          # chosen gap width (mm)
    gaps     = d.get('gap_info', [])

    chosen_gap = next((g for g in gaps if g.get('chosen')), None)

    fig = plt.figure(figsize=(16, 8))
    fig.suptitle(
        f'STOP Escape Event (FGM)  |  Robot heading={heading:.1f}°  '
        f'Chosen escape={target:+.0f}° (robot-local)  '
        f'Gap width={gap_dist:.0f}mm',
        fontsize=12, fontweight='bold'
    )
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)

    # ── TOP-DOWN VIEW ─────────────────────────────────────────────────────────

    # 1. LIDAR scan points
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

    # 2. FGM gaps — edge_a to edge_b 연결선으로 갭 개구부 표시
    for g in gaps:
        ea, eb = g['edge_a'], g['edge_b']
        x1, y1 = to_xy(ea[0], ea[1])
        x2, y2 = to_xy(eb[0], eb[1])
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2

        if g.get('chosen'):
            clr, lw, alpha, zord = 'limegreen', 3.5, 1.0, 7
        elif g.get('passable'):
            clr, lw, alpha, zord = '#4CAF50',   2.0, 0.85, 5
        else:
            clr, lw, alpha, zord = '#E57373',   1.2, 0.55, 4

        # 갭 개구부 선
        ax1.plot([x1, x2], [y1, y2], color=clr, lw=lw, alpha=alpha,
                 solid_capstyle='round', zorder=zord)
        # 엣지 점
        ax1.scatter([x1, x2], [y1, y2], s=22, color=clr, zorder=zord + 1)

        # 폭 라벨 (통과 가능 갭만)
        if g.get('passable') or g.get('chosen'):
            ax1.text(mx, my + 35, f"{g['width']:.0f}mm",
                     ha='center', va='bottom', fontsize=7, zorder=9,
                     color='darkgreen' if g.get('chosen') else 'forestgreen',
                     fontweight='bold')

        # 갭 중심 → 로봇 방향선 (chosen만)
        if g.get('chosen'):
            cx, cy = to_xy(g['center_angle'], min(g['depth'] * 0.55, 950))
            ax1.plot([0, cx], [0, cy], color='limegreen', lw=1.2,
                     linestyle='--', alpha=0.5, zorder=6)

    # 3. 선택 갭 탈출 화살표
    if chosen_gap:
        arrow_r = min(chosen_gap['depth'] * 0.7, 1100)
        tx, ty = to_xy(target, arrow_r)
        ax1.annotate('', xy=(tx, ty), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='->', color='limegreen', lw=3.5),
                     zorder=8)
        ax1.text(tx, ty + 65,
                 f'escape {target:+.0f}°\nwidth={gap_dist:.0f}mm',
                 ha='center', fontsize=8, color='darkgreen', fontweight='bold',
                 zorder=9, bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

    # 4. STOP zone 박스
    ax1.add_patch(patches.Rectangle(
        (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
        linewidth=2, edgecolor='red', facecolor='red', alpha=0.15,
        label='STOP zone', zorder=4
    ))

    # 5. 로봇 본체
    ax1.add_patch(patches.FancyBboxPatch(
        (-ROBOT_HW, -80), ROBOT_HW * 2, 240, boxstyle='round,pad=5',
        linewidth=2, edgecolor='#333', facecolor='#888', alpha=0.55, zorder=4
    ))
    ax1.annotate('', xy=(0, 190), xytext=(0, 80),
                 arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
    ax1.text(0, 212, 'fwd', ha='center', va='bottom', fontsize=8)

    legend_handles = [
        Line2D([0], [0], color='limegreen', lw=3.5, label='chosen gap'),
        Line2D([0], [0], color='#4CAF50',   lw=2.0, label='passable gap'),
        Line2D([0], [0], color='#E57373',   lw=1.2, label='too narrow gap'),
        patches.Patch(fc='red', alpha=0.3,           label='STOP zone'),
    ]
    ax1.legend(handles=legend_handles, fontsize=8, loc='upper right')
    plt.colorbar(sc, ax=ax1, shrink=0.45, label='LIDAR range (mm)', pad=0.01)

    ax1.set_xlim(-1500, 1500)
    ax1.set_ylim(-500, 1600)
    ax1.set_aspect('equal')
    ax1.axhline(0, color='gray', lw=0.5, zorder=1)
    ax1.axvline(0, color='gray', lw=0.5, zorder=1)
    ax1.set_xlabel('← Robot Left  |  Lateral (mm)  |  Robot Right →')
    ax1.set_ylabel('Forward (mm) ↑')
    ax1.set_title('Top-down LIDAR view (robot frame)\n'
                  'Lines = gap openings between obstacle edges (FGM)')
    ax1.grid(True, alpha=0.25, zorder=0)

    # ── FGM GAP ANALYSIS BAR CHART ────────────────────────────────────────────
    if gaps:
        sorted_gaps = sorted(gaps, key=lambda g: g['center_angle'])
        angles  = [g['center_angle'] for g in sorted_gaps]
        widths  = [g['width']        for g in sorted_gaps]
        depths  = [g['depth']        for g in sorted_gaps]
        bar_colors = [
            'limegreen' if g.get('chosen') else ('#4CAF50' if g.get('passable') else '#EF9A9A')
            for g in sorted_gaps
        ]

        x     = np.arange(len(sorted_gaps))
        bar_w = 0.55

        bars = ax2.bar(x, widths, width=bar_w, color=bar_colors, alpha=0.85,
                       zorder=3, label='Gap width (mm)')

        # 폭 수치 라벨
        for bar, w_val in zip(bars, widths):
            ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 8,
                     f'{w_val:.0f}', ha='center', va='bottom', fontsize=7)

        # 깊이 — 보조 축
        ax2_r = ax2.twinx()
        ax2_r.plot(x, depths, 's--', color='steelblue', lw=1.8,
                   markersize=7, label='Depth = max(edge dist)', zorder=5)
        ax2_r.set_ylabel('Gap depth (mm)', color='steelblue', fontsize=9)
        ax2_r.tick_params(axis='y', labelcolor='steelblue')

        # 최소 통과 폭 기준선
        ax2.axhline(MIN_PASSAGE, color='orange', lw=2, linestyle='-.',
                    label=f'min passage {MIN_PASSAGE}mm', zorder=4)

        # x축 라벨: 각도
        ax2.set_xticks(x)
        xlabels = ax2.set_xticklabels(
            [f'{a:+.0f}°' for a in angles], fontsize=8, rotation=45
        )
        # chosen 라벨 강조
        chosen_idx = next((i for i, g in enumerate(sorted_gaps) if g.get('chosen')), None)
        if chosen_idx is not None:
            xlabels[chosen_idx].set_color('darkgreen')
            xlabels[chosen_idx].set_fontweight('bold')

        lines1, lbls1 = ax2.get_legend_handles_labels()
        lines2, lbls2 = ax2_r.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, lbls1 + lbls2, fontsize=8, loc='upper right')

        ax2.set_xlabel('Gap center angle (robot-local)\n← Left (negative) | Right (positive) →')
        ax2.set_ylabel('Gap width = edge-to-edge distance (mm)')
        ax2.set_title('FGM Gap Analysis\n'
                      'Bar=width  Blue=depth  Orange=min passage  Green=chosen')
        ax2.grid(True, alpha=0.25, axis='y', zorder=0)
    else:
        ax2.text(0.5, 0.5, 'No gaps detected in scan',
                 ha='center', va='center', transform=ax2.transAxes, fontsize=14)
        ax2.set_title('FGM Gap Analysis — no gaps found')

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

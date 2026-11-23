#!/usr/bin/env python3
"""
Plot each launch-test scene: the synthetic scan and the command the node returned.

plot_scenes.py <scenes.csv> <out.svg>: the 360-beam scan seen from above (robot at the
centre, forward = up) and the command from CI as an arrow: forward motion = arrow up,
turning = curved arrow left (+) or right (-). Standard library only.
"""
import csv
import math
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
# panel width, plot radius (px, beams beyond 2.5 m drawn on the rim), px per metre
P, R, S = 220, 100, 40
W, H = P * len(rows), 300
el = [f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
      '<defs><marker id="a" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" '
      'markerHeight="7" orient="auto">'
      '<path d="M0 0L10 5L0 10z" fill="#005f73"/></marker></defs>']
for k, r in enumerate(rows):
    cx, cy = k * P + P / 2, 145
    ranges = [float(x) for x in r['ranges'].split()]
    el.append(f'<circle cx="{cx}" cy="{cy}" r="{R}" fill="#f6f8f9" stroke="#e6eaed"/>')
    el.append(f'<circle cx="{cx}" cy="{cy}" r="{0.8 * S:.1f}" fill="none" stroke="#9aa5b1" '
              f'stroke-dasharray="3 3"/>')
    for i, d in enumerate(ranges):  # beam i at bearing i deg (left positive), forward = up
        if d > 3.4:
            continue
        b = math.radians(i)
        rr = min(d * S, R)
        col = '#a2624a' if d < 0.8 else '#6b7785'
        el.append(f'<circle cx="{cx - rr * math.sin(b):.1f}" cy="{cy - rr * math.cos(b):.1f}" '
                  f'r="1.6" fill="{col}"/>')
    el.append(f'<path d="M{cx - 5} {cy + 4}L{cx} {cy - 7}L{cx + 5} {cy + 4}z" fill="#10161e"/>')
    v, w = float(r['v']), float(r['w'])
    if v > 0:
        el.append(f'<line x1="{cx}" y1="{cy - 14}" x2="{cx}" y2="{cy - 14 - v * 200:.1f}" '
                  f'stroke="#005f73" '
                  f'stroke-width="3" marker-end="url(#a)"/>')
    if w:
        sgn = 1 if w > 0 else -1  # + = counter-clockwise = turn left
        x0, x1 = cx + sgn * 62, cx - sgn * 62
        el.append(f'<path d="M{x0:.1f} {cy + 30}Q{cx} {cy - 120} {x1:.1f} {cy + 10}" fill="none" '
                  f'stroke="#005f73" '
                  f'stroke-width="3" marker-end="url(#a)"/>')
    el.append(f'<text x="{cx}" y="22" text-anchor="middle" fill="#10161e" '
              f'font-weight="700">{r["scene"]}</text>')
    turn = 'none' if not w else ('left' if w > 0 else 'right')
    el.append(f'<text x="{cx}" y="268" text-anchor="middle" fill="#005f73">v {v:g} m/s, w {w:+g} '
              f'rad/s ({turn})</text>')
el.append(f'<text x="8" y="{H - 8}" fill="#6b7785">Synthetic scans (red = closer than '
          f'safe_distance 0.8 m, dashed). '
          f'Arrows: the node\'s /cmd_vel in CI.</text>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
       f'font-family="Helvetica, Arial, sans-serif" '
       f'font-size="12">\n' + '\n'.join(el) + '\n</svg>\n')
open(sys.argv[2], 'w').write(svg)
print(f'{sys.argv[2]}: {len(rows)} scenes')

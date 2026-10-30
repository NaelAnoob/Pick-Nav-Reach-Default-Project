"""Build presentation-ready figures and an HTML report from measured trials."""
import csv
import html
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', str(Path('tmp/matplotlib').resolve()))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'results/grasping'
NAMES = {2: 'Soup can', 4: 'Potted meat', 5: 'Banana', 6: 'Strawberry', 14: 'Foam brick'}


def main():
    results = json.loads((OUT/'summary.json').read_text())
    passed = sum(r['success'] for r in results)
    held = sum(r['sustained_hold'] for r in results)
    objects = sorted({r['object_idx'] for r in results})
    fig, ax = plt.subplots(1, 2, figsize=(13.5, 4.5), layout='constrained')
    colors = ['#2563eb', '#0891b2', '#7c3aed', '#db2777', '#059669']
    for x, (obj, color) in enumerate(zip(objects, colors)):
        rows = [r for r in results if r['object_idx'] == obj]
        ax[0].scatter(np.full(len(rows), x)+np.linspace(-.12,.12,len(rows)),
            [100*r['final_clearance_m'] for r in rows], s=85, color=color, edgecolors='white', linewidth=1, zorder=3)
        ax[1].scatter(np.full(len(rows), x)+np.linspace(-.12,.12,len(rows)),
            [r['first_lift_physics_steps'] for r in rows], s=85, color=color, edgecolors='white', linewidth=1, zorder=3)
    for axis in ax:
        axis.set_xticks(range(len(objects)), [NAMES[i] for i in objects], fontsize=10)
        axis.grid(axis='y', alpha=.2)
        axis.spines[['top', 'right']].set_visible(False)
    ax[0].axhline(10, color='#475569', ls='--', lw=1.5, label='10 cm requirement')
    ax[0].set(ylabel='Object bottom above table (cm)', title='Clearance after 500 physics steps of holding', ylim=(0, 23))
    ax[1].axhline(3000, color='#dc2626', ls='--', lw=1.5, label='3,000 physics-step limit')
    ax[1].set(ylabel='Physics steps from scene start', title='Time to lift with contact on both fingers')
    for axis in ax:
        axis.legend(frameon=False, loc='lower right', fontsize=9)
    fig.savefig(OUT/'benchmark.png', dpi=180, facecolor='white')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12.8, 4.4), layout='constrained')
    for obj, color in zip(objects, colors):
        with (OUT/f'object_{obj:02d}_seed_42/trace.csv').open() as f:
            trace = list(csv.DictReader(f))
        trace = [r for r in trace if r['phase'] != 'settle']
        ax.plot([int(r['physics_steps']) for r in trace], [100*float(r['clearance']) for r in trace],
                color=color, lw=2, label=NAMES[obj])
    ax.axhline(10, color='#475569', ls='--', lw=1)
    ax.axvline(3000, color='#dc2626', ls='--', lw=1)
    ax.set(xlabel='Physics steps from scene start', ylabel='Object bottom above table (cm)',
           title='Measured lift trajectories | seed 42', ylim=(-2, 23))
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(alpha=.15)
    ax.legend(frameon=False, ncol=5, loc='upper left')
    fig.savefig(OUT/'lift_traces.png', dpi=180, facecolor='white')
    plt.close(fig)

    with (OUT/'summary.csv').open('w', newline='') as f:
        flat = [{k:v for k,v in r.items() if k != 'grasp'} for r in results]
        writer = csv.DictWriter(f, fieldnames=flat[0].keys())
        writer.writeheader()
        writer.writerows(flat)
    rows_html = ''
    for r in results:
        if r['success']:
            outcome = 'PASS'
        elif r['first_lift_physics_steps'] is None:
            outcome = 'NO 10 CM LIFT'
        elif r['first_lift_physics_steps'] > 3000:
            outcome = 'TIMING MISS'
        else:
            outcome = 'HOLD / CLEARANCE MISS'
        rows_html += (f"<tr><td>{NAMES[r['object_idx']]}</td><td>{r['seed']}</td>"
            f"<td>{100*r['final_clearance_m']:.1f} cm</td><td>{r['first_lift_physics_steps'] or '-'}</td>"
            f"<td>{r['stable_hold_physics_steps']}</td><td>{1000*(r['max_hold_relative_drift_m'] or 0):.2f} mm</td>"
            f"<td class=\"{'pass' if r['success'] else 'miss'}\">{outcome}</td></tr>")
    minimum = min(r['final_clearance_m'] for r in results)*100
    maximum = max(r['final_clearance_m'] for r in results)*100
    drift = max(r['max_hold_relative_drift_m'] or 0 for r in results)*1000
    penetration = max(r['max_robot_obstacle_penetration_m'] for r in results)*1000
    report = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Antipodal grasping | measured results</title><style>
*{{box-sizing:border-box}} body{{margin:0;background:#f1f5f9;color:#17243b;font:16px/1.55 'Segoe UI',sans-serif}}
main{{max-width:1120px;margin:auto;padding:48px 36px}} .eyebrow{{font-size:12px;font-weight:700;letter-spacing:2px;color:#2563eb;text-transform:uppercase}}
h1{{font-size:48px;line-height:1.1;letter-spacing:-2px;margin:14px 0 18px}} h2{{font-size:27px;margin-top:0}} h3{{font-size:17px}}
.lead{{font-size:20px;color:#52627a;max-width:850px}} .cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:18px;margin:30px 0}}
.card,section{{background:white;border:1px solid #e2e8f0;border-radius:16px;padding:28px;margin-bottom:24px}} .card{{margin:0}} .value{{font-size:36px;font-weight:750;letter-spacing:-1px}}
.muted{{color:#64748b;font-size:14px}} .pipeline{{display:grid;grid-template-columns:repeat(4,1fr);gap:20px}} .number{{color:#2563eb;font-size:13px;font-weight:800}}
img{{width:100%;height:auto;display:block;border-radius:8px}} .demo{{max-width:800px;margin:auto}} table{{width:100%;border-collapse:collapse;font-size:14px}}
th,td{{text-align:left;padding:10px 8px;border-bottom:1px solid #e2e8f0}} th{{color:#64748b;font-size:12px}} .pass{{color:#047857;font-weight:700}} .miss{{color:#b45309;font-weight:700}}
code{{background:#eef2ff;padding:3px 6px;border-radius:4px}} a{{color:#2563eb}} .callout{{border-left:4px solid #2563eb;padding-left:20px}} @media(max-width:750px){{main{{padding:24px 16px}}h1{{font-size:36px}}.cards,.pipeline{{grid-template-columns:1fr}}table{{font-size:11px}}}}
@media print{{body{{background:white}}main{{padding:0}}section{{break-inside:avoid}}.demo{{max-width:500px}}}}
</style><main><div class="eyebrow">CS4278 / CS5478 · Pick, Navigation, Reach · 26 October 2026</div>
<h1>From contact geometry<br>to a stable lift.</h1>
<p class="lead">A lecture-based antipodal grasping pipeline, executed with the Fetch robot in PyBullet. Five provided YCB objects, three fixed seeds, fifteen measured trials.</p>
<div class="cards"><div class="card"><div class="value">{passed}/{len(results)}</div>Full evaluation pass<div class="muted">10 cm clearance, lift by 3,000 physics steps, stable hold</div></div>
<div class="card"><div class="value">{held}/{len(results)}</div>Sustained lift and hold<div class="muted">500 physics steps above 10 cm with bilateral finger contact</div></div>
<div class="card"><div class="value">{minimum:.1f}–{maximum:.1f} cm</div>Final table clearance<div class="muted">Measured at the bottom of the object's collision AABB</div></div></div>
<section><h2>How the method follows the lectures</h2><div class="pipeline">
<div><div class="number">01 · PROPOSE</div><h3>Surface contacts</h3><p>Enumerate pairs from 1,024 sampled surface points and outward normals. Reject pairs wider than the 92 mm planning aperture.</p></div>
<div><div class="number">02 · FILTER</div><h3>Both friction cones</h3><p>For d = (q − p)/‖q − p‖, require −nₚ·d and nᵩ·d &gt; 1/√(1+μ²), with a positive margin. Here μ = 0.5.</p></div>
<div><div class="number">03 · PLAN</div><h3>Reachable approach</h3><p>Rank geometric candidates, solve damped Jacobian IK, and check joint-space edges. Use RRT-Connect and shortcutting when direct transit is blocked.</p></div>
<div><div class="number">04 · EXECUTE</div><h3>Close, lift, hold</h3><p>Follow a Cartesian approach, close the fingers at up to 60 N each, lift at least 19 cm with extra clearance for long objects, then measure contact and drift.</p></div></div>
<p class="muted">Lecture references: Week 3 Grasping, pp. 13 and 46; Week 3&amp;4 Grasping, p. 49; Week 5 Motion Planning, pp. 49 and 62–65. Ranking uses a geometric heuristic, not wrench-hull epsilon quality. Two ideal point contacts do not establish arbitrary six-dimensional force closure; finite finger pads provide the simulated contact behavior.</p></section>
<section><h2>Measured benchmark</h2><img src="benchmark.png" alt="Measured clearance and physics-step timing for fifteen trials">
<p>Seeds: 0, 7, 42. The strict pass counts a lift only while both fingers contact the object. Holding requires at least 50 consecutive control steps above 10 cm with less than 20 mm relative translation drift; the experiment records 100 control steps (500 physics steps).</p>
<p class="callout">{passed} of {len(results)} trials meet every measured threshold, including final clearance. {held} trials sustain the lift for the full observation period. Maximum measured hold drift is {drift:.2f} mm. Maximum robot penetration into the table or maze during execution is {penetration:.2f} mm.</p>
<div style="overflow:auto"><table><thead><tr><th>Object</th><th>Seed</th><th>Final clearance</th><th>First lift, physics steps</th><th>Hold, physics steps</th><th>Max hold drift</th><th>Outcome</th></tr></thead><tbody>{rows_html}</tbody></table></div>
<p class="muted">These are development trials on provided objects and selected seeds, not the course's hidden evaluation. Five physics steps run inside each env.step call; both counts are retained in the raw data. Timing begins at scene start and includes settling and base approach.</p></section>
<section><h2>A replay you can present</h2><div class="demo"><img src="../demo/object_05_seed_7/demo.gif" alt="Recorded simulated banana grasp"></div>
<p class="muted">Banana, seed 7. Frames come from the actual physics run. No object attachment, teleportation, or contact constraints are used during execution. Planning queries temporarily change robot joint states and restore the complete simulator state before execution.</p></section>
<section><h2>The lift, over time</h2><img src="lift_traces.png" alt="Actual object clearance trajectories for seed 42"></section>
<section><h2>A short explanation for tomorrow</h2><p>“I implemented antipodal grasp generation from point clouds and surface normals. I use the lecture's friction-cone condition to reject unstable contact pairs, then check whether the Fetch arm can reach and approach the remaining candidates without collisions. The robot closes its fingers, lifts the object, and holds it using simulated contact forces.</p>
<p>Across five provided objects and three seeds, {held} out of {len(results)} trials sustained the lift for the full holding period, and {passed} met the complete timing-and-stability test. The final clearance was {minimum:.1f} to {maximum:.1f} centimetres. I have logged every trial, including failures. The next step is to broaden the grasp approach directions and integrate carrying and navigation.”</p></section>
<section><h2>Reproduce and inspect</h2><p>From the project folder:</p><p><code>.venv\\Scripts\\python.exe controller.py --headless --objects 2 4 5 6 14 --seeds 0 7 42</code></p>
<p>GUI demo: <code>.venv\\Scripts\\python.exe controller.py --objects 5 --seeds 42</code></p>
<p><a href="summary.csv">Summary CSV</a> · <a href="summary.json">Complete JSON metrics</a> · <a href="../environment.txt">Tested dependency versions</a></p>
<p class="muted">Scope: pick-only, top-down grasps, a fixed base staging location, known full surface point clouds, and the original static scene. Collision checking covers the table, maze, and non-adjacent robot links at discrete configurations. Carried-object collision geometry, navigation, noisy partial perception, orientation drift scoring, and a six-dimensional wrench-hull test are future work. PyBullet's default object and pad friction are retained; μ=0.5 is the proposal-filter assumption, not an identified physical friction coefficient. The pass field is a local measurement of lift, timing, and hold; final end-effector pose errors are reported separately.</p></section>
</main></html>'''
    (OUT/'report.html').write_text(report, encoding='utf-8')
    print(f'Report: {passed}/{len(results)} full passes; {held}/{len(results)} held; clearance {minimum:.1f}-{maximum:.1f} cm')


if __name__ == '__main__':
    main()

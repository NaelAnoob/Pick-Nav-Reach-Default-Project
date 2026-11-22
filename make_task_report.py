"""Build a standalone HTML report from measured full-task benchmark results."""
import argparse
import html
import json
from pathlib import Path


def make_report(directory):
    directory = Path(directory)
    results = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    rows, scenes = [], []
    for result in results:
        obj, seed = result['object_idx'], result['seed']
        trial = f'object_{obj:02d}_seed_{seed}'
        status = 'PASS' if result['success'] else 'FAIL'
        rows.append(f'<tr><td>{obj}</td><td>{seed}</td><td>{status}</td>'
                    f'<td>{"yes" if result["pick_success"] else "no"}</td>'
                    f'<td>{result["distance_to_goal_m"]*100:.2f}</td>'
                    f'<td>{result["physics_steps"]}</td><td>{result["max_carry_drift_m"]*100:.2f}</td>'
                    f'<td>{html.escape(result["reason"])}</td>'
                    f'<td><a href="{trial}/task_result.json">JSON</a></td></tr>')
        if result.get('route'):
            walls = ''.join(f'<rect x="{x-.5}" y="{-y-.5}" width="1" height="1"/>'
                            for x, y in result.get('cube_positions', []))
            route = ' '.join(f'{x},{-y}' for x, y in result['route'])
            goal = result.get('goal_pos', [17.5, 0, .9])
            gif = f'<img src="{trial}/demo.gif" alt="Simulation replay">' if (directory/trial/'demo.gif').exists() else ''
            scenes.append(f'<article><h3>Object {obj}, seed {seed}: {status}</h3>'
                          f'<svg viewBox="-5 -7 24 14" role="img" aria-label="Maze and planned base route">'
                          f'<g fill="#c2ccd8">{walls}</g>'
                          f'<polyline points="{route}" fill="none" stroke="#087fbe" stroke-width=".1"/>'
                          f'<circle cx="{goal[0]}" cy="{-goal[1]}" r=".16" fill="#159447"/>'
                          f'</svg>{gif}</article>')
    passed = sum(r['success'] for r in results)
    picks = sum(r['pick_success'] for r in results)
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Pick–Nav–Reach results</title>
<style>body{{font:16px/1.5 system-ui;margin:40px auto;max-width:1200px;padding:0 20px;color:#192b40;background:#f8fafc}}
h1,h2,h3{{line-height:1.2}}table{{border-collapse:collapse;width:100%;background:white;font-size:14px}}
td,th{{padding:10px;text-align:left;border-bottom:1px solid #d8e1ea}}.table{{overflow:auto}}
article{{background:white;padding:20px;margin:20px 0;border:1px solid #d8e1ea;border-radius:12px}}svg{{width:100%}}img{{max-width:100%}}</style>
<h1>Pick–Nav–Reach</h1><p>Measured simulation results: <strong>{passed}/{len(results)} deliveries</strong> and
<strong>{picks}/{len(results)} picks meeting the local rubric checks</strong>.</p>
<h2>Method</h2><p>Antipodal proposals use observed surface points and normals. Damped Jacobian IK,
sampled collision checks and bidirectional arm planning produce a pick trajectory. The controller
closes the fingers and lifts through ordinary motor actions. It rotates and retracts the grasp
for transport, then plans a fixed-heading holonomic base route with A* on a 10 cm lattice.
Maze walls and the table are inflated by the robot and carried-object footprint. Exact segment
tests reject corner cutting; a second check validates the full articulated geometry and payload.
Bounded-acceleration commands stop at corners. Final IK corrections use the observed object
offset, followed by 60 control steps at the goal.</p>
<h2>Measurements</h2><p>Delivery requires the object to remain within 10 cm of the goal throughout
the final hold and completion within 15,000 physics steps. One control step advances five physics
steps. Pick success additionally requires a first lift by 3,000 physics steps and at least 50
consecutive stable hold steps. Carry drift measures translation relative to the gripper; more than
9 cm or an object height below 45 cm aborts execution. Failures remain in the denominator.</p>
<div class="table"><table><thead><tr><th>Object</th><th>Seed</th><th>Delivery</th><th>Pick</th>
<th>Goal error (cm)</th><th>Physics steps</th><th>Carry drift (cm)</th><th>Outcome</th><th>Data</th>
</tr></thead><tbody>{''.join(rows)}</tbody></table></div>
<h2>Planned routes</h2><p>Blue lines show planned base paths; green marks the object goal.
A planned route alone does not demonstrate successful execution.</p>{''.join(scenes)}
<h2>Limits</h2><p>These are development seeds, not hidden-test scores. The fixed-heading planner
can reject routes feasible with base rotation. Carry geometry is rigid only during planning queries;
execution relies on contact forces and can slip. Joint collision checks are discrete. The nominal
friction coefficient used by the proposal filter is not a measured surface property. The supplied
environment and planner assume one active PyBullet client per process. No fixed constraint attaches
the object to the robot, and execution never teleports the robot or object.</p></html>'''
    target = directory/'report.html'
    target.write_text(document, encoding='utf-8')
    print(target.resolve())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=Path('results/full_task'))
    make_report(parser.parse_args().input)

"""Create an evidence report from the saved pick, floor, and delivery trials."""
import csv
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
OUT = ROOT/'results/final'
NAMES = {2: 'Soup can', 4: 'Meat can', 5: 'Banana', 6: 'Strawberry', 14: 'Foam brick'}


def read_summary(folder, filename='task_result.json'):
    path = ROOT/folder
    if (path/'summary.json').exists():
        return json.loads((path/'summary.json').read_text(encoding='utf-8'))
    return [json.loads(f.read_text(encoding='utf-8')) for f in sorted(path.glob('object_*/'+filename))]


def rate(rows, field='success'):
    return f'{sum(bool(r.get(field)) for r in rows)}/{len(rows)}'


def make_plots(baseline, development, validation, floor):
    plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout='constrained')
    groups = [baseline, development, validation]
    x = np.arange(3)
    for offset, field, label, color in [(-.18, 'pick_success', 'Pick', '#137fba'), (.18, 'success', 'Delivery', '#13a18e')]:
        values = [100*sum(bool(r.get(field)) for r in rows)/max(len(rows), 1) for rows in groups]
        axes[0].bar(x+offset, values, width=.35, label=label, color=color)
        for i, (value, rows) in enumerate(zip(values, groups)):
            axes[0].text(i+offset, value+2, rate(rows, field), ha='center', fontsize=10)
    axes[0].set(xticks=x, xticklabels=['Original\n15 development', 'Updated\n15 development', 'Updated\n25 fresh'],
                ylim=(0, 115), ylabel='Successful trials (%)', title='Picks improved; delivery remains unreliable')
    axes[0].legend(loc='upper center', bbox_to_anchor=(.5, -.22), ncol=2, frameon=False)
    values, labels = [], []
    for obj in NAMES:
        rows = [r for r in floor if r['object_idx'] == obj]
        values.append(100*sum(r['success'] for r in rows)/max(len(rows), 1))
        labels.append(NAMES[obj])
    axes[1].bar(np.arange(5), values, color='#137fba')
    for i, obj in enumerate(NAMES):
        axes[1].text(i, values[i]+2, rate([r for r in floor if r['object_idx'] == obj]), ha='center')
    axes[1].set(xticks=np.arange(5), xticklabels=labels, ylim=(0, 115), title='Floor pickup after an initial fall', ylabel='Successful pickups (%)')
    axes[1].tick_params(axis='x', labelrotation=25)
    fig.savefig(OUT/'benchmark.png', dpi=180)
    plt.close(fig)

    source = ROOT/'results/retention/rotate/object_02_seed_0/trace.csv'
    if not source.exists():
        return
    with source.open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['phase'] in ('hold', 'tuck', 'retreat', 'navigate')]
    t = np.array([float(r['physics_steps'])/240 for r in rows])
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, layout='constrained')
    for finger, color in [(1, '#137fba'), (2, '#e18d27')]:
        force = np.array([float(r[f'finger_{finger}_force_n']) for r in rows])
        tangent = np.array([float(r[f'finger_{finger}_tangent_n']) for r in rows])
        axes[0].plot(t, force, color=color, label=f'Finger {finger}')
        axes[2].plot(t, tangent/np.maximum(.25*force, .001), color=color)
    axes[0].set(ylabel='Normal force (N)', title='Reproduced soup-can failure: original controller, seed 0')
    axes[0].legend(frameon=False, ncol=2)
    translation, = axes[1].plot(t, [100*float(r['relative_drift']) for r in rows], label='Translation', color='#137fba')
    rotation_axis = axes[1].twinx()
    rotation_axis.spines['right'].set_visible(True)
    rotation, = rotation_axis.plot(t, [np.rad2deg(float(r['relative_rotation_rad'])) for r in rows], label='Rotation', color='#bb4350')
    axes[1].set(ylabel='Translation drift (cm)')
    rotation_axis.set(ylabel='Rotation drift (degrees)')
    axes[1].legend(handles=[translation, rotation], frameon=False, ncol=2, loc='upper left')
    axes[2].axhline(1, color='#6c7884', linestyle='--', linewidth=1)
    axes[2].set(ylabel='Tangential / (mu x normal)', xlabel='Physics time since scene start (s)', ylim=(0, 1.5))
    previous = None
    for time, row in zip(t, rows):
        if row['phase'] != previous:
            for ax in axes:
                ax.axvline(time, color='#b6c1ca', linewidth=.7)
            axes[0].text(time+.05, axes[0].get_ylim()[1]*.94, row['phase'], fontsize=9, va='top')
            previous = row['phase']
    fig.savefig(OUT/'failure_diagnosis.png', dpi=180)
    plt.close(fig)


def table(rows, folder, floor=False):
    lines = []
    for r in sorted(rows, key=lambda x: (x['object_idx'], x['seed'])):
        obj, seed = r['object_idx'], r['seed']
        filename = 'floor_result.json' if floor else 'task_result.json'
        link = Path(folder)/f'object_{obj:02d}_seed_{seed}'/filename
        # Report lives at results/final, one level below results.
        href = '../'+str(link)[len('results/'):].replace('\\', '/')
        steps = r.get('duration_physics_steps', r.get('physics_steps'))
        recoveries = r.get('recoveries', [])
        recovered = sum(x['success'] for x in recoveries)
        lines.append(f'<tr><td>{NAMES.get(obj, obj)}</td><td>{seed}</td><td>{"PASS" if r["success"] else "FAIL"}</td>'
                     f'<td>{steps}</td><td>{recovered if recoveries else "-"}</td>'
                     f'<td>{html.escape(r["reason"])}</td><td><a href="{href}">data</a></td></tr>')
    return '<table><tr><th>Object</th><th>Seed</th><th>Outcome</th><th>Physics steps</th><th>Recovered</th><th>Reason</th><th>Evidence</th></tr>'+''.join(lines)+'</table>'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    baseline = read_summary('results/full_task')
    development = read_summary('results/final/development')
    validation = read_summary('results/final/validation')
    floor = read_summary('results/floor_pick/final_benchmark', 'floor_result.json')
    comparisons = {
        'Standard, no recovery': [r for r in development if r['seed'] == 0],
        'Standard, recovery': read_summary('results/recovery/comparison/standard_recovery'),
        'Faster, no recovery': read_summary('results/recovery/comparison/fast_no_recovery'),
        'Faster, recovery': read_summary('results/recovery/comparison/fast_recovery')}
    make_plots(baseline, development, validation, floor)
    inventory = []
    for directory in sorted((ROOT/'results/retention').iterdir()):
        if directory.is_dir():
            rows = read_summary(directory.relative_to(ROOT))
            if rows:
                inventory.append(dict(experiment=directory.name, trials=len(rows),
                                      picks=sum(r.get('pick_success', False) for r in rows), deliveries=sum(r['success'] for r in rows)))
    (OUT/'experiment_inventory.json').write_text(json.dumps(inventory, indent=2), encoding='utf-8')
    evidence = dict(original=dict(picks=rate(baseline, 'pick_success'), deliveries=rate(baseline)),
                    development=dict(picks=rate(development, 'pick_success'), deliveries=rate(development)),
                    fresh=dict(picks=rate(validation, 'pick_success'), deliveries=rate(validation)),
                    isolated_floor_pick=rate(floor), comparison={k: rate(v) for k, v in comparisons.items()})
    (OUT/'evidence.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    comparison_rows = ''.join(f'<tr><td>{name}</td><td>{rate(rows)}</td><td>{sum(len(r.get("recoveries", [])) for r in rows)}</td>'
                              f'<td>{sum(x["success"] for r in rows for x in r.get("recoveries", []))}</td></tr>' for name, rows in comparisons.items())
    sections = ''.join(f'<h3>{html.escape(name)}</h3>'+table(rows, 'results/recovery/comparison/'+{
        'Standard, recovery':'standard_recovery', 'Faster, no recovery':'fast_no_recovery',
        'Faster, recovery':'fast_recovery'}.get(name, '' )) for name, rows in comparisons.items() if name != 'Standard, no recovery')
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pick–Nav–Reach investigation and floor recovery</title><style>
body{{font:16px/1.55 system-ui;max-width:1140px;margin:40px auto;padding:0 24px;color:#183047;background:#f6f9fc}}
h1,h2,h3{{line-height:1.2}}h2{{margin-top:40px}}img{{max-width:100%;background:white;border-radius:8px}}
table{{border-collapse:collapse;background:white;width:100%;font-size:14px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #dce5ed}}
section{{overflow:auto}}.status{{padding:18px;border-left:5px solid #137fba;background:#e8f3fc}}a{{color:#0866a2}}code{{background:#e5edf4;padding:2px 5px}}
</style><h1>Pick–Nav–Reach: measured investigation and floor recovery</h1>
<p>Team 47 · CS4278 · 22 November 2026</p>
<p class="status"><strong>Picking improved; transport remains an open problem.</strong>
Development picks: {rate(baseline, 'pick_success')} → {rate(development, 'pick_success')}; deliveries: {rate(baseline)} → {rate(development)}.
Fresh-seed results: {rate(validation, 'pick_success')} picks and {rate(validation)} deliveries.
Floor recovery is implemented and tested separately; it is not a guarantee that a faster full delivery will succeed.</p>
<img src="benchmark.png" alt="Measured pick, delivery, and floor-pick success counts">
<h2>What the failure tests established</h2>
<p>The loaded finger and object friction coefficients are both 0.5, so their combined coefficient is 0.25.
The original proposal filter assumed 0.5. Bullet combines these coefficients by multiplication
(<a href="https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp">official source</a>).
The original soup-can failure was reproduced at physics step 6,285. Both fingers already produced approximately
60 N during the static hold. Rotation and translation relative to the gripper grew before contact was lost.
The tangential-load diagnostic was already close to its nominal friction limit during holding.</p>
<p>This evidence points to contact geometry, internal shear, and inadequate resistance to transport disturbances.
It does not identify a single universal cause. Changing carry orientation, reducing squeeze, and increasing solver
iterations did not fix the selected can failures. Gentler acceleration retained some grasps longer but exceeded the
15,000-step task limit. The supplied robot, masses, friction, force limits, and constraints remain unchanged.</p>
<img src="failure_diagnosis.png" alt="Normal finger forces and relative object motion before the reproduced drop">
<h2>Implemented improvements</h2>
<p>The controller moves the empty arm faster and finishes closure after sustained measured loading. It still closes
the fingers fully, lifts physically, and checks the hold. An optional finite-pad wrench model evaluates gravity and
four lateral acceleration loads using the measured friction and 60 N per-jaw limit. The default applies this model
to elongated clouds only, because applying it to every shape degraded compact-object results.</p>
<p>The model is approximate: it uses a point-cloud estimate of the center of mass, sampled pad contacts, and an
inscribed friction pyramid. Its mass-capacity value is a ranking signal, not a success certificate.</p>
<h2>Floor recovery</h2>
<p>After an observed slip, the optional recovery controller brakes, opens the hand, and waits for a sufficiently slow
floor contact. It searches nearby robot stances and antipodal grasps, checks the approach and a complete lift,
executes through ordinary motor actions, and requires 50 stable hold observations. Floor collision checks cover
the arm and fingers; existing floor contact is permitted only at the start of lifting. Recovery never resets or
attaches the task object. All recovery actions count toward the original 15,000-physics-step deadline.</p>
<p>Floor trajectories respect individual joint velocity limits. A fixed step size shared by prismatic and revolute
joints caused a large hand-position error in an early foam-brick test; respecting the joint limits corrected that
isolated pickup. Approaches are checked against their actual final position before closing.</p>
<p>Successful pickups after real transport slips were observed for both cans. The recovered object can still slip
again during navigation. Recovery therefore resumes in the lifted pose and replans the route using the newly
observed payload. Rolling objects, inaccessible stances, and low-clearance IK remain limitations.</p>
<p><a href="../floor_pick/demo/object_02_seed_0/floor_pick.gif">Floor-pick replay</a> ·
<a href="../floor_pick/demo/object_02_seed_0/floor_pick.png">Final floor-pick image</a></p>
<h2>Does recovery justify going faster?</h2>
<p>The controlled comparison uses the same five objects and seed 0. Standard transport requests 0.9 m/s with
0.8 m/s² acceleration; faster transport requests 1.8 m/s with 1.6 m/s² acceleration. These are commanded limits,
not a claim that the robot tracks every requested speed exactly. A successful floor pickup is counted separately
from a successful delivery. Failed, blocked, and timed-out trials remain in every denominator.</p>
<section><table><tr><th>Policy</th><th>Deliveries</th><th>Recovery attempts</th><th>Successful floor pickups</th></tr>{comparison_rows}</table></section>
<p><strong>The comparison does not support increasing speed.</strong> Standard-speed recovery picked up all three
dropped objects, but they slipped again later. The faster profile reduced deliveries from 2/5 to 1/5, with or
without recovery. Keep the standard profile as the default. This five-case development comparison is small;
it is evidence against assuming free recovery, not a general speed guarantee.</p>
<h2>Complete development results</h2><section>{table(development, 'results/final/development')}</section>
<h2>Fresh seeds</h2><p>Seeds 1, 11, 23, 83, and 101 were held out from controller selection.</p><section>{table(validation, 'results/final/validation')}</section>
<h2>Isolated fallen-object floor tests</h2><p>Objects start above the floor with a sideways orientation before the first
physics step. These are initialization fixtures, not recovered full-task trials. They are never added to the delivery rate.</p>
<section>{table(floor, 'results/floor_pick/final_benchmark', floor=True)}</section>
<h2>Speed comparison trial details</h2><section>{sections}</section>
<h2>Reproduction and limits</h2><p>See the repository README for commands. Trial JSON, contact diagnostics, and CSV
traces are retained. The <a href="experiment_inventory.json">experiment inventory</a> includes unsuccessful
development studies, and <a href="../retention/experimental_source.zip">archived research source</a> preserves
discarded variants. These results do not establish hidden-test performance or reliable recovery in every maze.
The project remains a tested research prototype with unresolved transport reliability.</p></html>'''
    (OUT/'report.html').write_text(document, encoding='utf-8')
    print(json.dumps(evidence, indent=2))
    print(OUT/'report.html')


if __name__ == '__main__':
    main()

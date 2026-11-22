# Pick - Navigate - Reach (Team 47)

Anoob Hakim Nael, A0284633Y - CS4278 final project.

This project picks an object from a point cloud, carries it through the supplied maze, and holds it at the goal. It includes contact diagnostics and optional physical floor recovery. Robot assets, masses, friction, force limits, and physics engine settings are unchanged.

## Run

Python 3.10 or newer is required (tested with Python 3.12).

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe full_task.py --objects 5 --seeds 7
.\.venv\Scripts\python.exe full_task.py --headless --objects 2 4 5 6 14 --seeds 0 1 7 11 23 42 83 101 --output results/my_benchmark
.\.venv\Scripts\python.exe -m unittest discover -s tests
```

Add `--record` for replay images and GIFs. Add `--recover-slips` for one floor recovery attempt; all recovery actions count toward the same task deadline. `--max-recoveries` controls the limit. For isolated floor initial conditions:

```powershell
.\.venv\Scripts\python.exe floor_pick.py --fallen --objects 2 4 5 6 14 --seeds 0 7 42
```

The isolated runner sets the object's initial pose before the first simulation step. Production recovery never resets, attaches, or teleports the object.

## Control and evaluation

Grasp proposals use outward normals and antipodal friction-cone tests. Arm motion uses damped Jacobian inverse kinematics, direct edges or bidirectional RRT, and Cartesian approach/lift. Navigation uses a 10 cm A* grid with measured robot/payload footprint inflation and full geometry validation.

The controller interpolates each 5/240-second reference interval into five motor targets at 240 Hz. It sets the environment's action grouping to one physics tick and converts its action counter so elapsed physics time is preserved. The simulator's physics timestep remains 1/240 second. `--legacy-rate` restores five ticks per held target. This control-rate change matters: held position targets produced repeated acceleration/braking pulses hidden by end-of-interval observations.

Default requested navigation speed is 1.8 m/s with 0.8 m/s^2 acceleration. A time-budget estimate can raise these to 2.2 m/s and 1.1 m/s^2 for long routes. Goal correction can search wider wrist orientations and a small base adjustment. Experimental grasp/carry options remain available; the default uses geometric grasps and rotated carry.

A pick requires first bilateral-contact lift by 3,000 physics steps, at least 10 cm table clearance and 50 stable reference observations. Delivery requires 60 final observations within 10 cm of the goal and completion within 15,000 physics steps. Each reference observation spans five physics ticks. Failed trials stay in the denominator. Floor recovery does not erase drops or reset the deadline.

## Evidence and known limitations

The last complete 40-case development batch passed 40/40 picks and 35/40 deliveries. It combines `results/controller_work/smooth_balanced` and `smooth_balanced_development`. These are development results, not a held-out generalization claim. The final revision completed 29/40 planned trials before the requested cutoff, delivering 28/29 completed cases; 11 remain incomplete. The banana slipped in the failed case. All 22 automated checks passed. Frozen results and source hashes are in `results/release`.

Fallen-object fixtures passed 14/15: nine can/foam cases in `smooth_floor_tolerances` plus five fruit cases in `smooth_floor_settle`. One strawberry case lost stable contact. Floor pickup alone is not proof of successful full-task recovery and delivery. Banana retention, limited time on long mazes, fixed-heading footprint conservatism, and final arm reach remain limitations.

`task_result.json` contains full delivery outcomes; `result.json` contains pick outcomes. Traces and contact diagnostics retain forces, relative pose, clearance, and phase timing. Only one PyBullet client per process is supported.

## Submission

Project repository: https://github.com/NaelAnoob/Pick-Nav-Reach-Default-Project

The local final code must be uploaded to the repository and access granted to the teaching team before submission; remote publication/access has not been verified here. The final code archive is an overlay for this repository, including its existing `assets` folder. Older report artifacts under other output paths describe earlier experiments and are superseded by the final PDF in `output/pdf`.

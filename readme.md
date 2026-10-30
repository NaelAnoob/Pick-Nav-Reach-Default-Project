# Default-Course-Project
Default Course Project of CS4278/CS5478 Intelligent Robots: Algorithms and Systems

## Implemented grasping demo

The pick-only implementation is in `grasping.py` and `controller.py`. It generates
antipodal proposals from the observed point cloud and normals, solves arm/torso
IK with the mobile base fixed, plans a collision-checked transit (direct edge or
RRT-Connect), approaches in Cartesian space, closes the fingers, lifts, and holds.
The lift distance accounts for object size so a long object can rotate below the
fingers without losing table clearance. Navigation is not implemented here.

Install the dependencies in a Python virtual environment (Python 3.10 or newer;
the recorded experiments used Python 3.12). PyBullet may build from source if no
wheel is available for the selected Python version.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Run a visible demo, or collect the full five-object, three-seed benchmark:

```powershell
.\.venv\Scripts\python.exe controller.py --objects 5 --seeds 7
.\.venv\Scripts\python.exe controller.py --headless --objects 2 4 5 6 14 --seeds 0 7 42
.\.venv\Scripts\python.exe controller.py --headless --record --objects 5 --seeds 7 --output results/demo
.\.venv\Scripts\python.exe make_grasp_report.py
.\.venv\Scripts\python.exe -m unittest discover -s tests
```

Open `results/grasping/report.html` in a browser for the measured results, charts,
an animated replay, a short presentation explanation, and limitations. Each trial
has `result.json` and `trace.csv`; `summary.csv` combines the metrics. The replay
is `results/demo/object_05_seed_7/demo.gif`. Tested versions are saved in
`results/environment.txt`. Headless mode runs without deliberate wall-clock
delays; GUI mode runs at the environment's intended physics rate.

Lecture correspondence:

- **Week 3 Grasping, pp. 13, 46; Week 3&4 Grasping, p. 49:** Coulomb friction cones
  and the antipodal contact-line condition. With outward normals and
  `d = (q - p) / ||q - p||`, both `-n_p dot d` and `n_q dot d` must exceed
  `1 / sqrt(1 + mu^2)`. The proposal filter uses `mu=0.5`, with an extra positive
  margin, and rejects openings beyond 92 mm. This is a nominal assumption;
  PyBullet's original contact friction settings remain in place.
- **Week 5 Motion Planning, pp. 49, 62-65:** collision-free configurations and
  discretized local edges, bidirectional RRT extension, and path shortcutting.
  Transit uses the direct edge when clear, and RRT-Connect when needed.
- Candidate ranking combines cone margin, contact centrality, and contact-line
  horizontality. It is a heuristic, not a computed wrench-hull epsilon quality.
  The lecture's planar antipodal criterion is not a proof of arbitrary 6-D force
  closure for two ideal point contacts; the simulation uses finite finger pads.

Evaluation conventions:

- One controller action invokes `env.step`, which advances **five physics steps**.
  Both counts are logged. First lift must occur by 3,000 **physics** steps,
  counting from scene start, including settling and base approach.
- A lift counts only with positive normal contact force on both fingers and the
  bottom of the object's collision AABB at least 10 cm above the table.
- A local pass requires at least 50 consecutive control steps of holding above
  that height, less than 20 mm translation drift relative to the gripper, and
  final clearance above 10 cm. The runner observes 100 control steps (500 physics
  steps) and separately reports whether the entire hold stayed stable.
- Final position/orientation errors describe the gripper versus the planned
  **lifted** grasp pose. They are reported separately from the local pass field.
  These development results do not claim a hidden-test score or full task success.
- Execution uses ordinary motor targets and contact forces. No object attachment,
  teleportation, or fixed grasp constraint is used. Kinematic planning queries
  restore the simulator state before executing through `env.step`.
- Collision queries cover the table, maze, and non-adjacent robot links. They use
  discrete joint samples, not continuous collision detection. The current vertical
  lift does not explicitly propagate carried-object geometry through the planner.
  The planner and controller assume one active simulation client.

The environment now also supports `gui=False`, optional `realtime` and `verbose`
flags, finite-action validation, seeded point-cloud sampling, `close()`, and a
temporary maze file so experiments do not overwrite the checked-in maze asset.
The original keyboard-controlled entry point remains available.

In this project, the task is to:

1. Generate antipodal grasp proposals, solve IK, and pick up an object on the table.
2. Carry the object and navigate the robot through the maze. 
3. Bring the object to the target position (visualized as the green sphere). 

We provide an environment code for the robot in `pick_nav_reach_env.py`.

![Scene](imgs/scene.png)


# Installation

1. Create a conda environment using `Python 3.10`.

```
conda create -n pnr python==3.10
conda activate pnr
```

2. Our environment is build on [PyBullet](https://pybullet.org/wordpress/index.php/forum-2/). Install it with pip:

```
pip3 install pybullet numpy matplotlib trimesh
```

3. Clone the project repo:

```
git clone https://github.com/NUS-LinS-Lab/Pick-Nav-Reach-Default-Project.git
```

# Run the Environment 

`python pick_nav_reach_env.py`

You should replace the `keyboard_controller` with a customized module designed by yourselves and then use `env.step()` to controll the robot to complete the task.

`env.step()` takes in a `numpy.ndarray` of shape (15,) as **action**, where 15 is the number of DoFs of the Fetch robot. The **observation** and **extra information** will be returned in python dictionaries as follows:

```python
# key_name: value_type, value_size

# observation dict
qpos: <class 'numpy.ndarray'>, (15,) # (num_dofs,)
qvel: <class 'numpy.ndarray'>, (15,) # (num_dofs,)
object_pos: <class 'numpy.ndarray'>, (3,) # position
object_xyzw: <class 'numpy.ndarray'>, (4,) # quaternion xyzw
goal_pos: <class 'numpy.ndarray'>, (3,) # position
object_pc: <class 'trimesh.caching.TrackedArray'>, (1024, 3) # (num_points, position)
object_normals: <class 'numpy.ndarray'>, (1024, 3) # (num_points, direction)
cube_positions: <class 'numpy.ndarray'>, (140, 2) # (num_cubes, xy_position)

# info dict
dist_to_goal: <class 'numpy.float64'>, () # < 0.1 means success
success: <class 'numpy.bool'>, () # success or not

```

# Requirement

You should implement the grasp generation, motion planning, and navigation algorithms in your customized module by yourselves to accomplish the task.

# Rubrics

We evaluate your algorithms in the following two aspects with 5 Unprovided seeds:

- Pick: 5 test objects (within the provided objects).

- Navigation and Reach: 5 test mazes.

Unless otherwise stated: placement tolerance = **10 cm**

## Pick (64%)

### Grasp Genration 

Use the **object point cloud and surface normals** to generate grasps.

- Function for Grasp Generation - 20 pts

### Motion Planning and Grasping

Use motion planning to execute the generated grasps.

- Function for Motion Planning - 20 pts

For each trial, award the following (sum; then average across trials):

- Lift within 3000 sim steps — 6 pts

- Lift height ≥ 10 cm above table — 6 pts

- Hold stability for ≥ 50 steps — 6 pts

- Final pose close to target grasp — 6 pts

![Scene](imgs/success_grasp.png)

## Navigation and Reach (36%)

Use the maze configuration to navigate and reach the goal.

- Function for Navigation - 20 pts

For each trial, award the following (sum; then average across trials):

- reach within 10 cm of the goal sphere — 6 pts

- reach within 15000 sim steps - 6 pts

- Carry Stability — 4 pts

    no drop that requires re-grasp during navigation → 2 pts each (max 8).

![Scene](imgs/success_navigation.png)


# References

- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3).

# Acknowledgments

- YCB object models are chosen and adapted from [here](https://www.ycbbenchmarks.com/).

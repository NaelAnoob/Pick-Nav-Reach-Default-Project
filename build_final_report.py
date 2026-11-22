"""Generate the final academic report from immutable release evidence."""
import json
from pathlib import Path
from xml.sax.saxutils import escape
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak

ROOT=Path(__file__).resolve().parent
DATA=ROOT/'results/release'
D=json.loads((DATA/'frozen_evidence.json').read_text())
OUT=ROOT/'output/pdf/Team47_Final_Project_Report.pdf'
OUT.parent.mkdir(parents=True,exist_ok=True)
W=484
styles={
 'body':ParagraphStyle('body',fontName='Times-Roman',fontSize=10.5,leading=13,spaceAfter=8,alignment=TA_JUSTIFY),
 'h1':ParagraphStyle('h1',fontName='Helvetica-Bold',fontSize=14,leading=18,spaceBefore=8,spaceAfter=10),
 'h2':ParagraphStyle('h2',fontName='Helvetica-Bold',fontSize=11,leading=15,spaceBefore=5,spaceAfter=7),
 'small':ParagraphStyle('small',fontName='Times-Roman',fontSize=9,leading=11.5,spaceAfter=7),
 'caption':ParagraphStyle('caption',fontName='Times-Italic',fontSize=9,leading=12,spaceAfter=10),
 'title':ParagraphStyle('title',fontName='Helvetica-Bold',fontSize=20,leading=25,alignment=TA_CENTER,spaceAfter=14),
 'author':ParagraphStyle('author',fontName='Times-Roman',fontSize=11,leading=15,alignment=TA_CENTER,spaceAfter=15),
 'code':ParagraphStyle('code',fontName='Courier',fontSize=8,leading=11,spaceAfter=7)}
story=[]
def para(t,kind='body'):story.append(Paragraph(t,styles[kind]))
def heading(t):para(t,'h1')
def sub(t):para(t,'h2')
def table(rows,widths):
 wrapped=[[Paragraph(str(v),styles['small']) for v in r] for r in rows]
 t=Table(wrapped,colWidths=widths,repeatRows=1,hAlign='LEFT')
 t.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'TOP'),('LINEABOVE',(0,0),(-1,0),.8,colors.black),('LINEBELOW',(0,0),(-1,0),.5,colors.black),('LINEBELOW',(0,-1),(-1,-1),.8,colors.black),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#edf1f3')),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),3)]))
 story.append(t);story.append(Spacer(1,9))
def page():story.append(PageBreak())
def fig(name,height,caption):
 story.append(Image(str(DATA/name),width=W,height=height));para(caption,'caption')

para('Pick, Navigate, Reach:<br/>Diagnosing Grasp Failures and Improving<br/>Contact-Based Mobile Manipulation','title')
para('Anoob Hakim Nael (A0284633Y)<br/>Team 47 | CS4278 Intelligent Robots: Algorithms and Systems<br/>22 November 2026','author')
sub('Abstract')
para('This project implements a complete simulated mobile manipulation pipeline: antipodal grasp generation, collision-aware arm motion, payload-aware maze navigation, and final object positioning. The presentation [6] identified grasp retention as the central problem: 13 of 15 picks passed, but only 5 deliveries succeeded. We investigate contact forces, object motion relative to the hand, and motor-target timing without modifying the supplied robot model. A key observed mechanism is repeated acceleration and braking when each position target is held for five physics ticks. Per-tick interpolation removes these speed pulses while preserving the physics timestep and elapsed-step budget. The last complete 40-case development batch achieved 40 picks and 35 deliveries. Subsequent reach and timing fixes were followed by a deadline-limited final rerun: 28 of 29 completed trials delivered; 11 planned trials were unfinished at the freeze. Isolated fallen-object recovery achieved 14 of 15 pickups. These results improve the working pipeline substantially, but do not establish universal delivery or recovery reliability.')
heading('1  Introduction')
para('Picking, carrying, and reaching are coupled tasks. A grasp that survives a static lift can rotate or slide when the wrist changes orientation or the mobile base accelerates. A collision-free arm posture can also leave the object unreachable at the final marker. Our task is to pick an object from the table, transport it through an observed maze, and keep it within 10 cm of the goal. Execution must use simulated contact forces and remain within the original task time budget.')
para('The work extends the presentation in four directions: phase-specific failure instrumentation; transport-oriented grasp experiments; smoother motion commands; and physical floor recovery. We retain the original assets, masses, contact friction, force limits, and physics engine settings. The contribution is an integrated controller and an evidence-based diagnosis, rather than a new robot design or a claim of globally optimal grasp quality.')
heading('2  Related Work')
para('Antipodal grasps connect opposing contact normals with friction-cone reasoning [1]. Ferrari and Canny formulate grasp quality in wrench space [2]; this motivates our transport scoring experiments, but our default geometric score is not their epsilon metric. Arm transit uses a bidirectional RRT approach inspired by RRT-Connect [3]. Navigation applies A* graph search [4] after obstacle inflation. PyBullet provides the motor and contact interface [5]. Our emphasis is the interaction between these conventional components under a finite execution budget.')
page()
heading('3  Method: Perception, Grasping, and Arm Motion')
sub('3.1  Observations and execution contract')
para('The simulator supplies joint state, object point cloud and outward surface normals, object pose, maze block positions, and goal position. This is a state-informed simulation pipeline, not a camera-only perception system. It assumes one active PyBullet client per process. The base has independent translation joints, allowing fixed-heading holonomic maze motion. Success in this setting should not be interpreted as hardware validation.')
table([['Stage','Representation and decision'],['Perceive','Point cloud, outward normals, joint state, maze blocks'],['Grasp','Antipodal pairs; aperture, surface and table filters'],['Plan arm','Damped Jacobian IK; checked joint edges; bidirectional RRT'],['Transport','Measured payload transform; conservative footprint; A*'],['Finish / recover','Observed object correction and hold; optional floor regrasp']],[80,404])
sub('3.2  Antipodal contact proposals')
para('For contact points p1 and p2, define the normalized segment d = (p2 - p1) / ||p2 - p1||. With outward unit normals n1 and n2, a candidate requires min(-n1 dot d, n2 dot d) &gt; 1 / sqrt(1 + mu squared). The nominal geometric filter uses mu = 0.5 and a positive margin. This sign convention is essential: reversing the outward normals would accept the wrong surface relation. Degenerate pairs and invalid normals are rejected.')
para('The generator restricts contact opening to below 92 mm and favors a nearly horizontal contact segment with a top-down approach that clears the table. Candidates are ranked using cone margin, centrality, and contact-line geometry. These heuristics are inexpensive and support the full task, but do not guarantee resistance to gravity torque after reorientation. Finite finger pads, uncertain contact locations, and object shape remain relevant even for an accepted antipodal pair.')
sub('3.3  Inverse kinematics and collision checks')
para('Damped Jacobian inverse kinematics solves arm and torso targets with the base held fixed during normal picking. The final default tolerances are 0.5 mm position and 0.002 rad orientation. A clear direct joint edge is preferred; otherwise bidirectional tree search and path shortcutting provide a transit. Cartesian waypoints define the last approach and the vertical lift. Lift distance accounts for the object extent so a long object can hang below the fingers without losing table clearance.')
para('Collision queries cover the table, maze, selected nonadjacent robot links, and arm/finger floor clearance. During carrying, the measured object-to-gripper transform propagates a hypothetical payload through each checked configuration. Planning uses temporary kinematic states inside save/restore queries; these queries do not advance execution or attach the object. Returned paths execute through normal motor targets. Joint samples approximate continuous clearance, so they cannot certify the absence of every between-sample contact.')
para('Finger closure ramps toward zero opening and monitors bilateral normal force. The faster pick policy can stop the ramp after sustained loading, then keeps closure commanded during lift and transport. Static hold requires both contact and limited object drift. Closing the fingers once is therefore supplemented by observation and explicit drop detection; it is not a calibrated force-feedback regulator.')
page()
heading('4  Failure Investigation and Control Improvement')
sub('4.1  Instrumentation and hypotheses')
para('The presentation reported that every failed delivery in its 15-case benchmark was a detected slip. We therefore logged finger normal and tangential forces, joint tracking, object translation and rotation relative to the hand, gripper rotation, base speed, clearance, and phase timing. Relative pose distinguishes an object rolling inside the jaws from the whole hand moving. Contact loss and rising drift provide evidence about retention before the object reaches the floor.')
para('The nominal friction coefficient used to filter proposals is not the simulator contact coefficient. In this setup, the unchanged finger and object friction values imply an effective coefficient of 0.25, while the geometric filter uses 0.5. Force traces near friction saturation support friction and torque limitations in some drops. This mismatch is a plausible contributing mechanism; it does not by itself explain every failure or justify increasing the simulator friction.')
sub('4.2  A hidden speed disturbance')
para('A second mechanism appears at the command interface. The original environment holds a position target for five 1/240-second physics ticks. Repeated position changes cause an initial velocity pulse followed by braking toward the fixed target. Reading speed only after the fifth tick hides most of the within-interval disturbance. A constant requested translation speed therefore does not imply constant instantaneous speed of the hand and payload.')
fig('control_pulses.png',166,'Figure 1. Measured per-physics-tick base velocity in a steady window of a 0.3 m/s translation. Both policies execute 240 physics ticks in the diagnostic. Held targets span approximately 0.130-0.537 m/s; interpolated targets remain approximately 0.300 m/s.')
para('The improvement splits every reference interval into five linearly interpolated position targets, one per physics tick. The reference duration remains 5/240 seconds. The controller changes action grouping to one tick and converts the action counter so elapsed physics time is preserved. Neither the physics timestep nor total simulated time is reduced. This is a higher command rate and a controller change; it is not an unchanged action interface or a robot-model modification.')
sub('4.3  Alternatives and interpretation')
para('We also tested finite-pad transport wrench scoring, different carrying orientations, tighter IK, and alternate contact approaches. The wrench model discretizes friction and estimates gravity/acceleration capacity with a linear program; it helped some shapes but did not consistently outperform geometric selection. Research options remain available, while the final default uses the simpler geometric policy. The pulse diagnostic directly establishes a timing disturbance; aggregate delivery gains combine several controller changes and should not be attributed to interpolation alone.')
page()
heading('5  Navigation, Final Reach, and Floor Recovery')
sub('5.1  Payload-aware navigation')
para('After lifting, the arm retracts into a carrying posture and the base retreats from the table. The measured object-to-hand transform is refreshed after retreat because the object can shift in the grasp. We form a conservative planar footprint from the union of collision AABBs of the robot and payload. Walls and the table are inflated by this asymmetric footprint plus 25 mm. This accounts for the object protruding beyond the base.')
para('A* searches a 10 cm lattice [4]. The lattice is aligned with corridor centers after asymmetric inflation so a narrow feasible lane is not missed simply because of the start location. Exact segment-rectangle tests reject blocked edges and shortcuts, including diagonal corner cuts. The resulting route is checked again using articulated robot and payload geometry. Nearby staging positions can be considered when the exact final base offset is blocked, provided the object can still reach the goal.')
para('Base targets use trapezoidal velocity profiles and stop at corners. The final requested cruising profile is 1.8 m/s and 0.8 m/s squared. For a route estimated to exceed the remaining time, a bounded budget check can raise the speed to 2.2 m/s and acceleration to 1.1 m/s squared. This does not increase the deadline. It is a time-risk tradeoff: greater acceleration can worsen retention, and a higher speed cap helps little on short segments that never reach it.')
sub('5.2  Reach and delivery verification')
para('The desired final base position accounts for the observed object offset rather than placing the gripper at the object goal. After transport, up to three corrections update this offset. Straight Cartesian correction is preferred. If the arm encounters a singularity or workspace limit, the final controller can search a small base adjustment and alternate wrist orientations, then validate a collision-free joint path. Targeted tests demonstrate that wider wrist rotations can solve cases where a straight downward correction is blocked.')
para('A delivery passes only when all 60 final reference observations remain within 10 cm of the goal and execution finishes within 15,000 physics ticks. Sixty observations span 300 ticks, or 1.25 seconds. A trial that arrives near the marker with insufficient time for this hold remains a failure. The object is never attached to the gripper or snapped to the target.')
sub('5.3  Physical recovery from a drop')
para('With recovery enabled, object height below 0.45 m or translation drift above 9 cm triggers a stop. The robot brakes, opens the fingers, and waits for supported floor contact with low linear and angular velocity. The maximum wait was extended from 100 to 300 reference intervals because the banana and strawberry were still rocking after the shorter wait. Recovery then searches nearby stances and antipodal grasps, validates approach and lift, closes with both fingers, and lifts to a useful carrying height.')
para('Floor motion respects joint velocity limits and checks the arm and fingers against the floor. Near workspace boundaries, floor IK uses 3 mm and 0.025 rad tolerances, followed by an executed approach-position check below 8 mm. A successful recovery requires 50 stable hold observations, more than 10 cm floor clearance, bilateral contact, and drift below 2 cm. Every action uses the original time budget. A successful floor pickup is measured separately from subsequent delivery, and a drop permanently removes carry-stability credit.')
page()
heading('6  Experiments and Results')
sub('6.1  Protocol and evaluation boundaries')
para('We evaluate five provided objects: soup can (2), meat can (4), banana (5), strawberry (6), and foam brick (14). The complete development batch uses eight seeds per object: 0, 1, 7, 11, 23, 42, 83, and 101. A pick requires bilateral contact and 10 cm table clearance by 3,000 physics ticks, at least 50 consecutive stable hold observations, and final clearance. Failure cases remain in the denominator. Seeds examined during iteration are development cases, including seeds initially described as fresh; they are not a final held-out test set.')
table([['Controller / evaluation','Pick','Delivery'],['Presentation baseline, seeds 0, 7, 42','13/15','5/15'],['Complete improved batch, same 15 cases','15/15','14/15'],['Complete improved batch, additional 25 cases','25/25','21/25'],['Complete improved batch, all cases','40/40','35/40'],['Final revision, completed before time freeze','29/29','28/29']],[304,85,95])
para('The complete improved batch uses interpolated commands, geometric grasps, a 1.8 m/s cruising cap, 0.8 m/s squared acceleration, and the earlier reach search. The final revision adds wider wrist search and the deadline-aware navigation adjustment. Its final rerun was stopped at the user-requested cutoff: 29 of 40 planned trials completed, one failed by banana slip, and 11 were incomplete. The 28/29 ratio conditions on completion; it is not a 40-case success rate. Against all 40 planned cases, only 28 deliveries were verified at the freeze. Targeted retests are supporting development evidence, not substitutes for a complete final batch.')
fig('by_object.png',172,'Figure 2. Delivery counts for the last complete 40-case development batch. Failures were one soup-can deadline, two banana cases (slip and reach), one strawberry reach, and one foam-brick reach.')
para('Successful deliveries in that complete batch took 11,010-14,330 physics ticks; maximum final object error was 3.10 cm. Maximum logged robot-obstacle penetration was approximately 0.50 mm, so the evidence supports collision-aware execution rather than a claim of mathematically zero contact. All 40 pick tests passed. The final revision also passed 22 automated checks, including physical floor pickup, action validation, contact geometry, query restoration, timing conversion, and preservation of dynamics and constraints.')
page()
heading('7  Discussion, Limitations, and Reproducibility')
sub('7.1  What the investigation resolved')
para('The main practical improvement is to treat execution timing as part of grasp stability. A static antipodal test and a clean planned route did not reveal the repeated acceleration impulses produced by held motor targets. Measuring velocity at the physics rate exposed this disturbance, and interpolating targets removed it in the diagnostic. Contact-based telemetry also prevented successful static holds from being mistaken for reliable delivery. The complete improved batch increased matched development deliveries from 5/15 to 14/15 while retaining the supplied physical model.')
para('Goal reach and time accounting then became visible bottlenecks. A high carry posture can be collision-free throughout the maze yet impossible to lower with its current wrist orientation. Wider orientation search solved the targeted banana and strawberry reach cases, and the bounded timing adjustment solved the targeted soup-can deadline case. These are measured local improvements. The deadline-limited final batch provides useful confirmation, but leaves 11 planned cases untested at the cutoff.')
sub('7.2  Recovery and remaining limitations')
para('Isolated fallen-object fixtures use a side-fallen initial orientation before the first physics tick, three seeds (0, 7, 42), and the unchanged robot and object properties. Recovery passed 14/15 fixtures: all three cases for each can, banana, and foam brick, and two of three strawberry cases. Extending settling time accounts for the fruit improvement over the earlier 9/15 fixture result. The remaining strawberry grasp failed the stability check. These controlled fixtures differ from drops inside a narrow maze, where the object may roll under an obstacle or leave no collision-free stance.')
para('No reliable speed benefit from recovery itself has been established. Earlier transport-slip experiments could retrieve an object but then lose it again. The final complete batch uses no recovery, and the deadline-limited rerun does not measure full-task recovery success. The faster cruising policy is supported by smoother-command delivery experiments, not by an assumption that every dropped object can be retrieved. The known banana slip remains unresolved.')
para('Other limitations are conservative fixed-heading footprints, finite collision sampling, state-informed perception, simplified transport wrench estimation, and development reuse of seeds. Robot trajectories are not dynamically optimized, and the controller does not estimate a calibrated safe grip force. Reported results cover these provided objects and test seeds only. A broader independent benchmark and real hardware experiments would be required to establish general robustness.')
sub('7.3  Reproduction and code availability')
para('The final project code is available in the repository at <link href="https://github.com/NaelAnoob/Pick-Nav-Reach-Default-Project" color="#155a7b">github.com/NaelAnoob/Pick-Nav-Reach-Default-Project</link>. The repository contains the project code and running instructions. The accompanying submission archive also preserves the source, tests, and frozen experimental evidence. To reproduce the project, clone the repository with its assets, install requirements.txt in a virtual environment, and run full_task.py; Appendix A gives exact commands.')
heading('8  Conclusion')
para('The project delivers an implemented and tested pick-navigate-reach pipeline, a concrete control-timing diagnosis, improved delivery performance, and physical floor regrasping. The last complete batch achieved 35/40 deliveries and the final deadline-limited revision verified 28 deliveries from 29 completed trials. We finalize the measured result with an explicit banana-retention limitation, rather than claiming the unfinished benchmark or arbitrary drop recovery succeeded.')
page()
heading('References')
refs=[
'[1] R. M. Murray, Z. Li, and S. S. Sastry. <i>A Mathematical Introduction to Robotic Manipulation.</i> CRC Press, 1994. <link href="https://www.cds.caltech.edu/~murray/books/MLS/">Book archive</link>.',
'[2] C. Ferrari and J. Canny. Planning optimal grasps. <i>IEEE International Conference on Robotics and Automation (ICRA)</i>, 1992. <link href="https://users.cs.duke.edu/~tomasi/public/ReadingGroup/Ferrari%20and%20Canny%20ICRA%201992.pdf">Paper</link>.',
'[3] J. J. Kuffner Jr. and S. M. LaValle. RRT-Connect: An efficient approach to single-query path planning. <i>ICRA</i>, vol. 2, pp. 995-1001, 2000. <link href="https://www.clear.rice.edu/comp450/papers/kuffner_lavalle_00.pdf">Paper</link>.',
'[4] P. E. Hart, N. J. Nilsson, and B. Raphael. A formal basis for the heuristic determination of minimum cost paths. <i>IEEE Transactions on Systems Science and Cybernetics</i>, 4(2):100-107, 1968. <link href="https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf">Paper</link>.',
'[5] Bullet Physics contributors. <i>PyBullet Quickstart Guide.</i> Online documentation, accessed 22 November 2026. <link href="https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html">Official source</link>.',
'[6] Team 47. <i>Pick - Navigate - Reach: Final Presentation.</i> CS4278 project presentation, supplied by the author. Internal project artifact. The baseline counts and follow-up objectives in this report refer to its slides 6-10.'
]
for r in refs:para(r)
sub('Evidence provenance')
para('All experimental numbers in this report are computed from local trial outputs, not from the external references. The immutable snapshot is results/release/frozen_evidence.json, frozen at 13:54:22 UTC on 22 November 2026. It includes the complete development batch, completed final reruns, isolated floor fixtures, targeted tests, and source-file hashes. The pulse measurements are in results/release/control_velocities.csv. No in-progress trial was treated as a completed success.')
para('The supplied presentation proposed instrumentation, transport robustness scoring, reduced disturbance, and recovery. Sections 3-7 explain how each was investigated. No additional oral presentation feedback was supplied, so the report does not invent reviewer comments or claim to have addressed feedback that was unavailable.')
page()
heading('Appendix A  Running and Auditing the Project')
para('Clone the project repository linked in Section 7.3, including its assets, and run the following commands from the project root. The accompanying source archive is an optional copy of the code and evidence; robot meshes are provided in the repository. Use one simulation client per process. Graphical mode omits --headless. Recording is optional and increases wall-clock time.')
for cmd in ['python -m venv .venv',r'.\.venv\Scripts\python.exe -m pip install -r requirements.txt',r'.\.venv\Scripts\python.exe full_task.py --headless --objects 5 --seeds 7',r'.\.venv\Scripts\python.exe -m unittest discover -s tests']:
 para(escape(cmd),'code')
para('Full benchmark: add --objects 2 4 5 6 14 --seeds 0 1 7 11 23 42 83 101 and an --output directory. Add --recover-slips for optional recovery; --max-recoveries defaults to one. Floor fixtures use floor_pick.py --fallen with the same object and seed arguments. The fixture sets initial pose before physics execution; production recovery does not reset the object.')
sub('A.1  Trial outputs')
table([['File','Meaning'],['result.json','Pick timing, clearance, hold drift, grasp, and contact status'],['task_result.json','Delivery, reason, time, drops, recoveries, route and goal error'],['trace.csv / task_trace.csv','Per-reference contact, pose, speed, phase and goal-distance observations'],['floor_result.json','Isolated recovery outcome, floor clearance, hold and duration'],['frozen_evidence.json','Immutable release snapshot; includes source hashes and trial paths']],[142,342])
sub('A.2  Frozen final batch, by object')
rows=[['Object','Completed / planned','Delivered / completed']]
for o,name in [(2,'Soup can'),(4,'Meat can'),(5,'Banana'),(6,'Strawberry'),(14,'Foam brick')]:
 a=[x for x in D['final_batch'] if x['object_idx']==o];rows.append([name,f'{len(a)}/8',f'{sum(x["success"] for x in a)}/{len(a)}'])
table(rows,[160,162,162])
para('The 11 missing final results are interrupted or unstarted cases at the time freeze. They are incomplete, not observed failures. The full earlier 40-case batch is retained separately and must not be silently combined with the final revision to report a final 39/40 or 40/40 score. The completed final banana failure is object 5, seed 1, detected as a slip.')

def footer(c,doc):
 c.setFont('Helvetica',8);c.setFillColor(colors.HexColor('#555555'))
 c.drawString(64,32,'Team 47 | Anoob Hakim Nael | CS4278 Final Project')
 c.drawRightString(548,32,str(doc.page))
 c.setStrokeColor(colors.HexColor('#bbbbbb'));c.line(64,44,548,44)

doc=SimpleDocTemplate(str(OUT),pagesize=letter,rightMargin=64,leftMargin=64,topMargin=48,bottomMargin=58,title='Pick, Navigate, Reach: Final Project Report',author='Anoob Hakim Nael')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(OUT)

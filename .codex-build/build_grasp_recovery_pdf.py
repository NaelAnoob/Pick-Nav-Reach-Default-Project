"""Build a compact review copy of the measured project evidence."""
import json
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT/'results/final'
OUT = ROOT/'output/pdf/Pick_Nav_Reach_Grasp_Recovery_Report.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
evidence = json.loads((DATA/'evidence.json').read_text())
floor = json.loads((ROOT/'results/floor_pick/final_benchmark/summary.json').read_text())
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='TitleCustom', fontName='Helvetica-Bold', fontSize=25, leading=29, textColor=colors.HexColor('#15324b'), spaceAfter=14))
styles.add(ParagraphStyle(name='SubtitleCustom', fontName='Helvetica', fontSize=12, leading=17, textColor=colors.HexColor('#47647a'), spaceAfter=18))
styles.add(ParagraphStyle(name='BodyCustom', fontName='Helvetica', fontSize=10.5, leading=15, spaceAfter=10, alignment=TA_LEFT))
styles.add(ParagraphStyle(name='SmallCustom', fontName='Helvetica', fontSize=8.5, leading=12, textColor=colors.HexColor('#47647a'), spaceAfter=8))
styles['Heading1'].textColor = colors.HexColor('#15324b')
styles['Heading1'].fontSize = 19
styles['Heading1'].leading = 23
styles['Heading1'].spaceAfter = 14
styles['Heading2'].textColor = colors.HexColor('#137fba')
styles['Heading2'].spaceBefore = 10


def p(text, small=False):
    return Paragraph(text, styles['SmallCustom' if small else 'BodyCustom'])


def table(rows, widths):
    wrapped = [[p(str(cell), small=True) for cell in row] for row in rows]
    item = Table(wrapped, colWidths=widths, hAlign='LEFT', repeatRows=1)
    item.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e5f1fa')),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#f4f7fa')]),
        ('LINEBELOW', (0, 0), (-1, 0), .8, colors.HexColor('#91b6cf')),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'), ('LEFTPADDING', (0, 0), (-1, -1), 9),
        ('RIGHTPADDING', (0, 0), (-1, -1), 9), ('TOPPADDING', (0, 0), (-1, -1), 8),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6)]))
    return item


def footer(canvas, doc):
    canvas.setStrokeColor(colors.HexColor('#d5e0e8'))
    canvas.line(38, 36, A4[0]-38, 36)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(colors.HexColor('#47647a'))
    canvas.drawString(38, 23, 'TEAM 47  |  CS4278  |  Measured simulation evidence')
    canvas.drawRightString(A4[0]-38, 23, str(doc.page))


story = [Paragraph('Pick - Navigate - Reach', styles['TitleCustom']),
         Paragraph('Grasp failure investigation and floor recovery<br/>22 November 2026', styles['SubtitleCustom']),
         p('<b>Status:</b> picking improved, and floor recovery is implemented. Transport retention remains unreliable; this is a tested research prototype, not a completed reliable delivery solution.'),
         table([['Benchmark', 'Picks', 'Deliveries'],
                ['Original development cases', '13/15', '5/15'],
                ['Updated, same development cases', '15/15', '6/15'],
                ['Updated, five fresh seeds per object', '25/25', '4/25']], [303, 105, 111]),
         Spacer(1, 15), Image(str(DATA/'benchmark.png'), width=519, height=226.5),
         p('All 15 isolated fallen-object tests are separate from the delivery benchmark. Floor pickup passed in 9/15 cases: all soup-can, meat-can, and foam-brick cases; the fruit cases did not meet the settling limits.'),
         p('<b>The robot model is unchanged.</b> No masses, contact friction, actuator force limits, or robot constraints were modified. The object is not attached to the hand. Recovery executes through the same motor-target interface and counts toward the original 15,000-physics-step deadline.'),
         PageBreak(), Paragraph('What the failure tests show', styles['Heading1']),
         p('The original soup-can failure was reproduced at physics step 6,285. Relative rotation grew during the carrying transition, then increased sharply before both finger contacts were lost. Normal forces were already approximately 60 N during the static hold.'),
         Image(str(DATA/'failure_diagnosis.png'), width=510, height=357),
         Spacer(1, 9),
         p('<b>Confirmed mismatch:</b> the loaded finger and object friction values are each 0.5. Their combined coefficient is 0.25, while the original proposal filter assumed 0.5. Tangential contact loading was already near its nominal limit during some static holds.'),
         p('This supports a contact-geometry and transport-wrench explanation, including internal shear and poor torque resistance. It does not prove one universal cause. Carry-orientation changes, reduced squeeze, and higher solver iterations did not fix the selected can failures. Gentler acceleration kept some grasps longer but exhausted the time budget.'),
         p('The updated controller uses faster empty-arm motion and measured closure loading. An approximate finite-pad wrench score is applied to elongated point clouds only; using it for compact objects degraded results. The score is a model-based ranking signal, not a success guarantee.'),
         p('Friction rule: Bullet official source, btManifoldResult::calculateCombinedFriction. '
           '<link href="https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp" color="#137fba">View source</link>. '
           'Raw evidence: results/retention/rotate/object_02_seed_0/trace.csv.', small=True),
         PageBreak(), Paragraph('Floor recovery: implementation and tests', styles['Heading1']),
         p('<b>1.</b> Brake after the observed slip, open the hand, and wait for sufficiently slow floor contact.<br/>'
           '<b>2.</b> Search nearby base stances and grasps from the observed point cloud and surface normals.<br/>'
           '<b>3.</b> Check the base motion, arm transit, approach, and a complete lift against obstacles, the floor, and robot geometry.<br/>'
           '<b>4.</b> Respect individual joint velocity limits and verify the actual hand reaches the floor grasp.<br/>'
           '<b>5.</b> Close, lift to transport height, and require 50 stable control observations.<br/>'
           '<b>6.</b> Replan using the newly measured payload. A previous drop remains in the carry-stability record.'),
         Paragraph('Isolated fallen-object results', styles['Heading2']),
         table([['Object', 'Successful pickups', 'Main limitation when unsuccessful']]+[
             [name, f"{sum(r['success'] for r in floor if r['object_idx']==obj)}/3",
              'Did not settle within limits' if obj in (5, 6) else '-']
             for obj, name in [(2, 'Soup can'), (4, 'Meat can'), (5, 'Banana'), (6, 'Strawberry'), (14, 'Foam brick')]], [160, 125, 234]),
         Spacer(1, 12),
         Paragraph('A verified execution issue', styles['Heading2']),
         p('Using the same position increment for rotary and linear joints caused a large hand-position error in an early floor pickup. Timing motion against each joint velocity limit corrected the isolated foam-brick case. The robot physics were identical in both trials.'),
         table([['Foam-brick floor test', 'Stable hold', 'Final hand error', 'Physics steps'],
                ['Shared position increment', '0 steps', '190.1 mm', '9,760'],
                ['Individual velocity limits', '60 steps', '0.15 mm', '4,170']], [210, 90, 115, 104]),
         Spacer(1, 12),
         p('<b>19 automated tests pass.</b> Checks include a physical floor pickup, unchanged object and finger properties, unchanged constraint count, floor clearance, and a recovery that cannot reset or exceed the task deadline.'),
         p('Floor fixtures set their initial pose before the first physics step. Full-task recovery never resets the object. Rolling objects, blocked stances, and lift reachability remain explicit failure modes.', small=True),
         PageBreak(), Paragraph('Does recovery make faster transport viable?', styles['Heading1']),
         table([['Matched five-case policy', 'Deliveries', 'Successful floor pickups'],
                ['Standard, no recovery', '2/5', '-'], ['Standard, with recovery', '2/5', '3/3 attempts'],
                ['Faster, no recovery', '1/5', '-'], ['Faster, with recovery', '1/5', '2/4 attempts']], [285, 90, 144]),
         Spacer(1, 12),
         p('Standard commands use 0.9 m/s and 0.8 m/s2; faster commands use 1.8 m/s and 1.6 m/s2. These are requested limits, not guaranteed measured speeds. All five objects use seed 0 and the same 15,000-step deadline.'),
         p('<b>The comparison does not support going faster.</b> Standard-speed recovery picked up all three dropped objects, but they slipped again later. Those recoveries cost 4,400-5,135 physics steps (18.3-21.4 seconds), leaving substantially less time for delivery.'),
         p('Keep standard transport as the default and floor recovery optional. The remaining work is reducing repeat slips and handling moving objects; a successful floor pickup alone does not establish successful or timely delivery.'),
         Paragraph('Review and reproduce', styles['Heading2']),
         p('Full report and links to every trial: <b>results/final/report.html</b><br/>'
           'Floor-pick replay: <b>results/floor_pick/demo/object_02_seed_0/floor_pick.gif</b><br/>'
           'Controller and recovery: <b>full_task.py</b> and <b>floor_recovery.py</b><br/>'
           'Isolated floor tests: <b>floor_pick.py</b><br/>'
           'Usage and benchmark commands: <b>readme.md</b>'),
         p('The source data includes unsuccessful trials, contact diagnostics, CSV traces, and discarded development variants. Fresh-seed performance is reported separately. These measurements do not establish hidden-test performance.', small=True)]

doc = SimpleDocTemplate(str(OUT), pagesize=A4, rightMargin=38, leftMargin=38,
                        topMargin=38, bottomMargin=49, title='Pick-Nav-Reach Grasp Investigation and Floor Recovery', author='Team 47')
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(OUT)

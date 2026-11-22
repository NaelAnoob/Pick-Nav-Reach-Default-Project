import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const workspaceDir = "D:/CLASSES NUS/Y4S1/CS4278/Project/Pick-Nav-Reach-Default-Project";
const buildDir = path.join(workspaceDir, ".codex-build");
const previewDir = path.join(buildDir, "previews");
const outputDir = path.join(workspaceDir, "presentation-output");
const candidatePath = path.join(buildDir, "pick_nav_reach_final_candidate.pptx");
const finalPath = path.join(outputDir, "Pick_Nav_Reach_Final_Presentation_v3.pptx");
const skillDir = "C:/Users/naela/.codex/plugins/cache/openai-primary-runtime/presentations/26.1111.10934/skills/presentations";
const runtimePython = "C:/Program Files/WindowsApps/OpenAI.CodexPrimaryRuntime.v26-1111-574-0_26.1111.574.0_x64__3k8sg7r9htsxt/dependencies/python/python.exe";

const C = {
  navy: "#0B132B",
  ink: "#182033",
  blue: "#2563EB",
  cyan: "#0EA5A8",
  green: "#159447",
  amber: "#F59E0B",
  red: "#DC3545",
  pale: "#F4F7FB",
  line: "#D7DFEA",
  muted: "#5D687B",
  white: "#FFFFFF",
};
const FONT = "Arial";
const images = {
  scene: path.join(workspaceDir, "imgs", "scene.png"),
  grasp: path.join(workspaceDir, "results", "demo", "object_05_seed_7", "final.png"),
  graspGif: path.join(workspaceDir, "results", "demo", "object_05_seed_7", "demo.gif"),
  benchmark: path.join(workspaceDir, "results", "grasping", "benchmark.png"),
  lift: path.join(workspaceDir, "results", "grasping", "lift_traces.png"),
  navigation: path.join(workspaceDir, "imgs", "success_navigation.png"),
};

const imageData = {};
for (const [key, imagePath] of Object.entries(images)) {
  imageData[key] = new Uint8Array(await fs.readFile(imagePath));
}
function imageSource(key, contentType = "image/png") {
  return { blob: imageData[key], contentType };
}

await fs.mkdir(previewDir, { recursive: true });
await fs.mkdir(outputDir, { recursive: true });

const deck = Presentation.create({ slideSize: { width: 1280, height: 720 } });

function addText(slide, text, pos, style = {}, name) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    name,
    position: pos,
    fill: "none",
    line: { style: "solid", fill: "none", width: 0 },
  });
  shape.text = text;
  shape.text.style = {
    fontFamily: FONT,
    fontSize: style.fontSize ?? 24,
    bold: style.bold ?? false,
    color: style.color ?? C.ink,
    alignment: style.alignment ?? "left",
    verticalAlignment: style.verticalAlignment ?? "top",
  };
  return shape;
}

function addBox(slide, pos, fill, radius = 18, line = "none", name) {
  return slide.shapes.add({
    geometry: radius ? "roundRect" : "rect",
    name,
    position: pos,
    fill,
    line: { style: "solid", fill: line, width: line === "none" ? 0 : 1 },
    ...(radius ? { borderRadius: radius } : {}),
  });
}

function addRule(slide, x, y, w, color = C.blue, h = 5) {
  return addBox(slide, { left: x, top: y, width: w, height: h }, color, 0);
}

function addHeader(slide, title, section, index, titleSize = 42) {
  addText(slide, section.toUpperCase(), { left: 66, top: 38, width: 300, height: 25 }, { fontSize: 15, bold: true, color: C.blue });
  addText(slide, title, { left: 66, top: 73, width: 1100, height: 62 }, { fontSize: titleSize, bold: true, color: C.navy });
  addText(slide, String(index).padStart(2, "0"), { left: 1180, top: 42, width: 44, height: 28 }, { fontSize: 16, bold: true, color: C.muted, alignment: "right" });
  addRule(slide, 66, 139, 90, C.cyan, 5);
}

function addFooter(slide, text = "CS4278 Intelligent Robots") {
  addText(slide, text, { left: 66, top: 686, width: 520, height: 18 }, { fontSize: 12, color: "#7C8799" });
}

function addBullet(slide, text, x, y, w, accent = C.blue, fontSize = 24) {
  addBox(slide, { left: x, top: y + 10, width: 10, height: 10 }, accent, 5);
  addText(slide, text, { left: x + 26, top: y, width: w - 26, height: 62 }, { fontSize, color: C.ink });
}

function setNotes(slide, timing, body, source) {
  slide.speakerNotes.text = `${timing}\n\n${body}\n\nSource: ${source}`;
  slide.speakerNotes.setVisible(true);
}

// 1. Cover
{
  const s = deck.slides.add();
  s.background.fill = C.navy;
  s.images.add({ ...imageSource("grasp"), alt: "Fetch robot holding a banana above the table", fit: "cover", position: { left: 720, top: 0, width: 560, height: 720 } });
  addBox(s, { left: 676, top: 0, width: 180, height: 720 }, { color: C.navy, transparency: 18 }, 0);
  addText(s, "PICK · NAVIGATE · REACH", { left: 70, top: 82, width: 520, height: 32 }, { fontSize: 17, bold: true, color: "#69D4D7" });
  addText(s, "A complete mobile\nmanipulation pipeline", { left: 70, top: 145, width: 620, height: 170 }, { fontSize: 58, bold: true, color: C.white });
  addRule(s, 70, 342, 105, C.amber, 7);
  addText(s, "Antipodal grasping, collision-aware motion planning,\nand A* navigation in PyBullet", { left: 70, top: 375, width: 600, height: 82 }, { fontSize: 25, color: "#DCE5F2" });
  addText(s, "TEAM [NUMBER]", { left: 70, top: 560, width: 240, height: 26 }, { fontSize: 17, bold: true, color: "#69D4D7" });
  addText(s, "[Member names]", { left: 70, top: 595, width: 560, height: 32 }, { fontSize: 22, color: C.white });
  addText(s, "CS4278 Final Project", { left: 70, top: 659, width: 300, height: 20 }, { fontSize: 14, color: "#AAB6C8" });
  setNotes(s, "0:00–0:35", "Introduce the goal: one controller picks an unknown provided object, carries it through an observed maze, and places it within 10 cm of the target. State the key result up front: the full pipeline runs, but grasp stability during transport remains the main failure mode.", "Project repository: readme.md and results/full_task/summary.json");
}

// 2. Problem and motivation
{
  const s = deck.slides.add(); s.background.fill = C.white; addHeader(s, "The task combines manipulation and mobility", "Problem and motivation", 2);
  s.images.add({ ...imageSource("scene"), alt: "Top view of the robot, object, maze and object goal", fit: "contain", position: { left: 66, top: 175, width: 660, height: 410 } });
  addText(s, "One trial must satisfy both", { left: 780, top: 185, width: 420, height: 42 }, { fontSize: 28, bold: true, color: C.navy });
  addBullet(s, "Pick the object from its point cloud and surface normals", 780, 252, 420, C.blue, 23);
  addBullet(s, "Carry it through a maze without collision or dropping it", 780, 340, 420, C.cyan, 23);
  addBullet(s, "Hold the object within 10 cm of the goal", 780, 428, 420, C.green, 23);
  addBox(s, { left: 780, top: 530, width: 400, height: 105 }, "#FFF7E6", 14, "#F5D08A");
  addText(s, "Why it matters", { left: 802, top: 547, width: 145, height: 22 }, { fontSize: 17, bold: true, color: "#A15C00" });
  addText(s, "Errors compound across grasping, planning, and transport.", { left: 802, top: 577, width: 350, height: 52 }, { fontSize: 20, color: C.ink });
  addFooter(s);
  setNotes(s, "0:35–1:25", "Explain that this is harder than a pick-only or navigation-only task. A grasp that looks stable while stationary may fail after arm reorientation or base motion. The final score therefore depends on the weakest stage in the chain.", "Project assignment in readme.md; imgs/scene.png");
}

// 3. System architecture
{
  const s = deck.slides.add(); s.background.fill = C.pale; addHeader(s, "System architecture", "Approach and implementation", 3);
  const stages = [
    ["1", "Perceive", "Point cloud, normals, maze blocks"],
    ["2", "Generate grasp", "Antipodal contact pairs"],
    ["3", "Plan arm", "IK, collision checks, RRT-Connect"],
    ["4", "Carry and navigate", "Footprint-aware A* route"],
    ["5", "Correct and hold", "Observed object offset, 60 checks"],
  ];
  const xs = [66, 300, 534, 768, 1002];
  for (let i = 0; i < stages.length; i++) {
    if (i < stages.length - 1) addRule(s, xs[i] + 194, 315, 40, C.line, 3);
    const circle = s.shapes.add({ geometry: "ellipse", position: { left: xs[i], top: 213, width: 58, height: 58 }, fill: i === 4 ? C.green : C.blue, line: { style: "solid", fill: "none", width: 0 } });
    circle.text = stages[i][0]; circle.text.style = { fontFamily: FONT, fontSize: 27, bold: true, color: C.white, alignment: "center", verticalAlignment: "middle" };
    addText(s, stages[i][1], { left: xs[i], top: 294, width: 194, height: 35 }, { fontSize: 23, bold: true, color: C.navy });
    addText(s, stages[i][2], { left: xs[i], top: 342, width: 194, height: 82 }, { fontSize: 19, color: C.muted });
  }
  addBox(s, { left: 90, top: 500, width: 1100, height: 105 }, C.white, 16, C.line);
  addText(s, "Implementation principle", { left: 120, top: 523, width: 230, height: 27 }, { fontSize: 20, bold: true, color: C.cyan });
  addText(s, "Every motion executes through env.step() and simulated contact forces. Planning queries restore simulator state, so the object is never teleported or attached by a fixed constraint.", { left: 365, top: 515, width: 785, height: 70 }, { fontSize: 22, color: C.ink });
  addFooter(s);
  setNotes(s, "1:25–2:20", "Walk left to right. Emphasize the separation between geometric planning and physical execution. The planner can propose a collision-free sequence, but the grasp must still survive contact dynamics during carrying.", "grasping.py, controller.py, navigation.py, and full_task.py");
}

// 4. Grasping approach
{
  const s = deck.slides.add(); s.background.fill = C.white; addHeader(s, "Antipodal grasp generation and arm planning", "Approach and implementation", 4, 40);
  s.images.add({ ...imageSource("grasp"), alt: "Fetch robot holding a banana after a planned lift", fit: "cover", position: { left: 66, top: 174, width: 565, height: 410 }, geometry: "roundRect", borderRadius: 16 });
  addText(s, "Candidate generation", { left: 690, top: 185, width: 430, height: 34 }, { fontSize: 27, bold: true, color: C.navy });
  addBullet(s, "Sample opposing surface points whose normals satisfy a friction-cone test", 690, 238, 500, C.blue, 21);
  addBullet(s, "Reject openings above 92 mm and rank by cone margin, centrality, and horizontal contact line", 690, 322, 500, C.blue, 21);
  addText(s, "Execution planning", { left: 690, top: 433, width: 430, height: 34 }, { fontSize: 27, bold: true, color: C.navy });
  addBullet(s, "Solve damped Jacobian IK with the base fixed", 690, 486, 500, C.cyan, 21);
  addBullet(s, "Use direct joint-space transit when clear, otherwise RRT-Connect, then approach, close, lift, and hold", 690, 558, 500, C.cyan, 21);
  addFooter(s);
  setNotes(s, "2:20–3:30", "Describe the lecture connection. The contact pair must lie inside both friction cones under the nominal coefficient μ = 0.5. The method then checks reachability and collision-free arm motion. Clarify that the ranking is a heuristic, not a full six-dimensional force-closure metric.", "readme.md lecture correspondence; grasping.py and controller.py");
}

// 5. Navigation
{
  const s = deck.slides.add(); s.background.fill = C.pale; addHeader(s, "Navigation accounts for the carried object", "Approach and implementation", 5);
  s.images.add({ ...imageSource("navigation"), alt: "Fetch robot carrying an object through a narrow maze corridor", fit: "cover", position: { left: 728, top: 166, width: 486, height: 468 }, geometry: "roundRect", borderRadius: 18 });
  addText(s, "10 cm lattice A*", { left: 66, top: 184, width: 500, height: 40 }, { fontSize: 31, bold: true, color: C.navy });
  addBullet(s, "Inflate walls and table by the union of robot and payload footprints, plus 25 mm clearance", 66, 250, 600, C.blue, 23);
  addBullet(s, "Use exact segment–rectangle tests so diagonal shortcuts cannot cut corners", 66, 354, 600, C.cyan, 23);
  addBullet(s, "Validate the full route against articulated robot geometry, self-collision, and payload geometry", 66, 458, 600, C.green, 23);
  addBox(s, { left: 66, top: 575, width: 600, height: 62 }, "#EAF7F4", 12, "#B6DED4");
  addText(s, "Base commands use a trapezoidal velocity profile and stop at route corners.", { left: 88, top: 592, width: 560, height: 30 }, { fontSize: 20, color: "#0D6B57" });
  addFooter(s);
  setNotes(s, "3:30–4:35", "Explain the key implementation choice: the robot keeps a fixed heading, so the footprint becomes conservative but easy to validate. The planner includes the object, not only the base. Point out that narrow routes may be rejected even if a rotating robot could pass.", "navigation.py and full_task.py; imgs/success_navigation.png");
}

// 6. Pick-only results
{
  const s = deck.slides.add(); s.background.fill = C.white; addHeader(s, "The isolated pick usually succeeds", "Results and demonstration", 6);
  s.images.add({ ...imageSource("benchmark"), alt: "Measured clearance and lift timing for five objects across three seeds", fit: "contain", position: { left: 66, top: 164, width: 1148, height: 410 } });
  addText(s, "13/15", { left: 86, top: 585, width: 150, height: 52 }, { fontSize: 43, bold: true, color: C.blue });
  addText(s, "met timing and stability checks", { left: 235, top: 595, width: 330, height: 32 }, { fontSize: 21, color: C.ink });
  addText(s, "15/15", { left: 625, top: 585, width: 150, height: 52 }, { fontSize: 43, bold: true, color: C.green });
  addText(s, "sustained the full 500-step hold", { left: 775, top: 595, width: 360, height: 32 }, { fontSize: 21, color: C.ink });
  addFooter(s, "Development benchmark: 5 provided objects × 3 seeds; not hidden-test performance");
  setNotes(s, "4:35–5:40", "Report the pick-only benchmark. All objects cleared the 10 cm requirement and held for 500 physics steps. Two trials missed the 3,000-step timing limit, so 13 of 15 passed the complete local test. This result shows that the static lift behavior is strong, but it does not test transport disturbances.", "results/grasping/summary.json, summary.csv, and benchmark.png");
}

// 7. Full task results
{
  const s = deck.slides.add(); s.background.fill = C.pale; addHeader(s, "Transport exposes the grasping failure", "Results and demonstration", 7);
  addText(s, "Successful trials out of 15", { left: 66, top: 175, width: 400, height: 34 }, { fontSize: 27, bold: true, color: C.navy });
  const chart = s.charts.add("bar", {
    position: { left: 66, top: 226, width: 650, height: 330 },
    categories: ["Pick test", "End-to-end delivery"],
    series: [{ name: "Successful trials", values: [13, 5], fill: C.blue }],
    hasLegend: false,
    dataLabels: { showValue: true, position: "outEnd" },
    xAxis: { min: 0, max: 15, majorUnit: 5, majorGridlines: { style: "solid", fill: C.line, width: 1 }, title: { text: "Successful trials out of 15" } },
    yAxis: { reverseOrder: true },
  });
  chart.title = undefined;
  addBox(s, { left: 775, top: 195, width: 420, height: 140 }, "#FFF0F0", 16, "#F3B8BE");
  addText(s, "10/15 delivery failures", { left: 808, top: 222, width: 350, height: 42 }, { fontSize: 31, bold: true, color: C.red });
  addText(s, "Every failure was detected as an object slip.", { left: 808, top: 278, width: 345, height: 35 }, { fontSize: 22, color: C.ink });
  addText(s, "What worked", { left: 775, top: 390, width: 180, height: 32 }, { fontSize: 25, bold: true, color: C.green });
  addBullet(s, "The complete pipeline planned and executed successful deliveries in 5 trials", 775, 438, 420, C.green, 21);
  addBullet(s, "Successful final errors ranged from 0.09 to 4.89 cm", 775, 515, 420, C.green, 21);
  addText(s, "The dominant bottleneck is grasp retention during carrying, not route planning.", { left: 66, top: 607, width: 1120, height: 40 }, { fontSize: 27, bold: true, color: C.navy, alignment: "center" });
  addFooter(s, "Development benchmark: objects 2, 4, 5, 6, 14; seeds 0, 7, 42");
  setNotes(s, "5:40–6:45", "State the result plainly. Only 5 of 15 trials met the full final-hold and time criteria. All ten failures were object slips. This is why I conclude that the current grasping approach did not work reliably for the complete task, even though the isolated pick often passed.", "results/full_task/summary.json and summary.csv; measured counts summarized in readme.md");
}

// 8. Demo
{
  const s = deck.slides.add(); s.background.fill = C.navy; addText(s, "RESULTS AND DEMONSTRATION", { left: 66, top: 40, width: 350, height: 24 }, { fontSize: 15, bold: true, color: "#69D4D7" });
  addText(s, "A successful pick sequence", { left: 66, top: 78, width: 630, height: 55 }, { fontSize: 43, bold: true, color: C.white });
  s.images.add({ ...imageSource("graspGif", "image/gif"), alt: "Animated banana grasp, lift, and hold for object 5 seed 7", fit: "cover", position: { left: 66, top: 160, width: 760, height: 475 }, geometry: "roundRect", borderRadius: 16 });
  addText(s, "Banana · seed 7", { left: 880, top: 188, width: 300, height: 35 }, { fontSize: 27, bold: true, color: C.white });
  addText(s, "2,565", { left: 880, top: 270, width: 180, height: 60 }, { fontSize: 52, bold: true, color: "#69D4D7" });
  addText(s, "physics steps to first lift", { left: 880, top: 329, width: 280, height: 34 }, { fontSize: 20, color: "#CFD9E8" });
  addText(s, "21.8 cm", { left: 880, top: 410, width: 230, height: 60 }, { fontSize: 47, bold: true, color: "#8FE1B6" });
  addText(s, "final table clearance", { left: 880, top: 469, width: 260, height: 34 }, { fontSize: 20, color: "#CFD9E8" });
  addText(s, "1.21 mm", { left: 880, top: 550, width: 230, height: 52 }, { fontSize: 42, bold: true, color: "#FCD17A" });
  addText(s, "maximum hold drift", { left: 880, top: 603, width: 260, height: 30 }, { fontSize: 20, color: "#CFD9E8" });
  addText(s, "08", { left: 1180, top: 42, width: 44, height: 28 }, { fontSize: 16, bold: true, color: "#AAB6C8", alignment: "right" });
  setNotes(s, "6:45–7:30", "Let the embedded animation run for about 20 seconds. Narrate only the four phases: approach, close, lift, hold. Then contrast this controlled stationary hold with the later reorientation and base acceleration that destabilize the grasp.", "results/demo/object_05_seed_7/demo.gif and result.json");
}

// 9. Failure analysis
{
  const s = deck.slides.add(); s.background.fill = C.white; addHeader(s, "Why the grasp fails during transport", "Discussion and conclusions", 9);
  addText(s, "Observed evidence", { left: 66, top: 181, width: 280, height: 38 }, { fontSize: 28, bold: true, color: C.red });
  addBox(s, { left: 66, top: 238, width: 350, height: 320 }, "#FFF5F5", 18, "#F1C5C9");
  addText(s, "Static hold", { left: 95, top: 266, width: 160, height: 32 }, { fontSize: 23, bold: true, color: C.navy });
  addText(s, "15/15 sustained 500 physics steps", { left: 95, top: 308, width: 285, height: 52 }, { fontSize: 22, color: C.ink });
  addText(s, "During carrying", { left: 95, top: 390, width: 240, height: 32 }, { fontSize: 23, bold: true, color: C.navy });
  addText(s, "10/15 slipped after reorientation, retreat, or base motion", { left: 95, top: 440, width: 285, height: 92 }, { fontSize: 21, color: C.ink });
  addText(s, "Likely causes", { left: 485, top: 181, width: 280, height: 38 }, { fontSize: 28, bold: true, color: C.navy });
  const causes = [
    ["Grasp metric mismatch", "Antipodal geometry ranks static contacts but does not model transport wrench resistance."],
    ["Orientation disturbance", "Rotating into the carry pose changes gravity torque and can roll the object between the pads."],
    ["Friction uncertainty", "The μ = 0.5 filter is nominal and does not identify the simulator’s effective pad–object friction."],
    ["Open-loop retention", "The controller closes once, then carries without adjusting grip force or recovering from incipient slip."],
  ];
  for (let i = 0; i < causes.length; i++) {
    const y = 238 + i * 96;
    addText(s, causes[i][0], { left: 485, top: y, width: 260, height: 28 }, { fontSize: 21, bold: true, color: i === 0 ? C.red : C.blue });
    addText(s, causes[i][1], { left: 750, top: y - 2, width: 450, height: 70 }, { fontSize: 19, color: C.ink });
    if (i < causes.length - 1) addRule(s, 485, y + 77, 715, C.line, 2);
  }
  addFooter(s);
  setNotes(s, "7:30–8:45", "Separate evidence from hypotheses. The evidence is the sharp gap between stationary hold and transport. The likely reasons come from the controller design: static geometric ranking, a large carry reorientation, nominal friction, and no closed-loop slip response. Avoid claiming that any single cause has already been proven.", "Measured evidence: results/grasping and results/full_task. Mechanism analysis: grasping.py, controller.py, and full_task.py");
}

// 10. Plan and conclusion
{
  const s = deck.slides.add(); s.background.fill = C.pale; addHeader(s, "Plan before final submission", "Discussion and conclusions", 10);
  const items = [
    ["1", "Instrument the failure", "Log finger forces, object rotation, and relative pose before and after each carry phase."],
    ["2", "Score transport robustness", "Evaluate candidate grasps under gravity in the intended carry orientation, not only at the table."],
    ["3", "Reduce disturbances", "Compare top-down and support carry modes, slower reorientation, and gentler base acceleration."],
    ["4", "Add recovery", "Detect slip early, increase closure within safe limits, or stop and regrasp before entering the maze."],
  ];
  for (let i = 0; i < items.length; i++) {
    const y = 178 + i * 104;
    const n = s.shapes.add({ geometry: "ellipse", position: { left: 78, top: y, width: 56, height: 56 }, fill: i === 3 ? C.green : C.blue, line: { style: "solid", fill: "none", width: 0 } });
    n.text = items[i][0]; n.text.style = { fontFamily: FONT, fontSize: 26, bold: true, color: C.white, alignment: "center", verticalAlignment: "middle" };
    addText(s, items[i][1], { left: 166, top: y - 1, width: 300, height: 35 }, { fontSize: 25, bold: true, color: C.navy });
    addText(s, items[i][2], { left: 470, top: y - 2, width: 735, height: 64 }, { fontSize: 21, color: C.ink });
  }
  addBox(s, { left: 78, top: 596, width: 1126, height: 80 }, C.navy, 14);
  addText(s, "Takeaway: the pipeline is complete. The final improvement target is grasp stability under transport disturbances.", { left: 120, top: 612, width: 1040, height: 52 }, { fontSize: 20, bold: true, color: C.white, alignment: "center" });
  addFooter(s);
  setNotes(s, "8:45–10:00", "Close with the plan in priority order. First collect evidence that distinguishes rotation slip from base-motion slip. Then change the grasp score and carry motion, and finally add recovery. End with the takeaway: the system integration is complete, but reliable grasp retention is the remaining blocker before submission.", "Planned work based on measured failure modes and current controller structure");
}

// Export previews and candidate.
for (let i = 0; i < deck.slides.items.length; i++) {
  const blob = await deck.export({ slide: deck.slides.items[i], format: "png", scale: 1 });
  await fs.writeFile(path.join(previewDir, `slide-${String(i + 1).padStart(2, "0")}.png`), new Uint8Array(await blob.arrayBuffer()));
}
const montage = await deck.export({ format: "webp", montage: true, scale: 0.45 });
await fs.writeFile(path.join(buildDir, "montage.webp"), new Uint8Array(await montage.arrayBuffer()));
const pptx = await PresentationFile.exportPptx(deck);
await pptx.save(candidatePath);

const { finalizePresentation } = await import(pathToFileURL(path.join(skillDir, "container_tools", "artifact_tool_utils.mjs")).href);
const result = await finalizePresentation({
  workspaceDir,
  candidatePath,
  finalPath,
  explicitTotalSlideCount: 10,
  requiredNativeTableOwnerSlides: [],
  requiredNativeChartOwnerSlides: [7],
  materializeLiteralChartWorkbooks: true,
  pythonExecutable: runtimePython,
  integrityValidatorPath: path.join(skillDir, "container_tools", "inspect_presentation_package_integrity.py"),
  layoutValidatorPath: path.join(skillDir, "container_tools", "inspect_presentation_layout_geometry.py"),
  layoutArgs: ["--expected-slide-size-emu", "12192000,6858000", "--validate-heading-fit"],
  fontPolicy: { basis: "design", families: [FONT] },
  verifyArtifactToolImport: true,
  receiptPath: path.join(buildDir, "Pick_Nav_Reach_Final_Presentation_v3.validation.json"),
});

console.log(JSON.stringify({ finalPath, previewDir, montage: path.join(buildDir, "montage.webp"), result }, null, 2));

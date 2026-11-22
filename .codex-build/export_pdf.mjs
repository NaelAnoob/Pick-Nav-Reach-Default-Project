import fs from "node:fs/promises";
import path from "node:path";
import { FileBlob, PresentationFile } from "@oai/artifact-tool";

const workspace = "D:/CLASSES NUS/Y4S1/CS4278/Project/Pick-Nav-Reach-Default-Project";
const input = path.join(workspace, "presentation-output", "Pick_Nav_Reach_Final_Presentation_v3.pptx");
const outputDir = path.join(workspace, "output", "pdf");
const output = path.join(outputDir, "Pick_Nav_Reach_Final_Presentation.pdf");

await fs.mkdir(outputDir, { recursive: true });
const presentation = await PresentationFile.importPptx(await FileBlob.load(input));
const pdf = await presentation.export({ format: "pdf" });
await fs.writeFile(output, new Uint8Array(await pdf.arrayBuffer()));
console.log(output);

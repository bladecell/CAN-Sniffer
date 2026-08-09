import fs from "fs";
import path from "path";

// Usage: bun src/scripts/add-icon.ts <path-to-svg> <name>
const svgPath = process.argv[2];
const iconName = process.argv[3];

if (!svgPath || !iconName) {
  console.error("Usage: bun src/scripts/add-icon.ts <path-to-svg> <name>");
  process.exit(1);
}

const fullSvgPath = path.resolve(svgPath);

if (!fs.existsSync(fullSvgPath)) {
  console.error(`Error: File not found at ${fullSvgPath}`);
  process.exit(1);
}

// 1. Extract viewBox and path from the SVG
const content = fs.readFileSync(fullSvgPath, "utf-8");

const viewBoxMatch = content.match(/viewBox="([^"]+)"/);
const viewBox = viewBoxMatch ? viewBoxMatch[1] : "0 0 512 512";

let pathData = "";
const pathRegex = /<path[^>]+d="([^"]+)"/g;
let match;
while ((match = pathRegex.exec(content)) !== null) {
  pathData += match[1] + " ";
}
pathData = pathData.trim();

if (!pathData) {
  console.error("Error: Could not find a <path d=\"...\"> in the SVG file.");
  process.exit(1);
}

// 2. Format the new icon entry
const safeName =
  iconName.includes("-") || iconName === "import" || iconName === "export"
    ? `"${iconName}"`
    : iconName;

const newEntry = `  ${safeName}: {\n    viewBox: "${viewBox}",\n    path: "${pathData}",\n  },\n`;

// 3. Inject it into src/lib/icons.ts
const iconsFile = path.join(process.cwd(), "src/lib/icons.ts");
let iconsContent = fs.readFileSync(iconsFile, "utf-8");

// Find the end of the iconData object
// We look for "};" that closes the export const iconData = { ... };
const injectionIndex = iconsContent.lastIndexOf("};");

if (injectionIndex === -1) {
  console.error("Error: Could not find the end of the iconData object in src/lib/icons.ts");
  process.exit(1);
}

iconsContent = 
  iconsContent.slice(0, injectionIndex) + 
  newEntry + 
  iconsContent.slice(injectionIndex);

fs.writeFileSync(iconsFile, iconsContent, "utf-8");
console.log(`✅ Successfully added '${iconName}' to src/lib/icons.ts!`);

// fm-render: tldraw-rebrand sha256:0659c9359ce325a315e13dc6baac429f6ae0f54b7da6b64d802dc93d82849292 — rendered by the First Motive render plane — edit the upstream source, not this file
// Rebrand one tldraw SVG export for render.sh: tldraw colours become First
// Motive palette colours, tldraw fonts become an embedded Geist Mono, and the
// stamp line goes first. Exits non-zero, writing nothing, on a #rrggbb or
// rgb() colour outside the palette (the two forms tldraw-cli draws in; its
// rgba() values are transparent CSS defaults) or an export with no <defs> for
// the font.
//
//   node tldraw-rebrand.cjs <tldraw-palette.json> <font.ttf> <export.svg> <stamp>
//
// .cjs, not .js, so it loads as CommonJS in a repo whose package.json says
// "type": "module".
const fs = require("fs");

function fail(message) {
  console.error(message);
  process.exit(1);
}

const [palettePath, fontPath, svgPath, stamp] = process.argv.slice(2);
if (!stamp) fail("usage: tldraw-rebrand.cjs <palette.json> <font.ttf> <export.svg> <stamp>");

const toBrand = new Map();
for (const [brand, hexes] of Object.entries(JSON.parse(fs.readFileSync(palettePath, "utf8")).remap)) {
  for (const hex of hexes) toBrand.set(hex.toLowerCase(), brand);
}

const byte = (n) => Number(n).toString(16).padStart(2, "0");
const outside = new Set();
let svg = fs
  .readFileSync(svgPath, "utf8")
  .replace(/rgb\((\d+),\s*(\d+),\s*(\d+)\)/g, (_, r, g, b) => `#${byte(r)}${byte(g)}${byte(b)}`)
  .replace(/#[0-9a-fA-F]{6}\b/g, (hex) => {
    const brand = toBrand.get(hex.toLowerCase());
    if (!brand) outside.add(hex);
    return brand ?? hex;
  })
  .replace(/tldraw_(draw|sans|serif|mono), (sans-serif|serif|monospace)/g, "'Geist Mono', monospace");

if (outside.size) {
  fail(`colours outside the palette: ${[...outside].join(" ")} — draw in black, violet, yellow, or light-violet`);
}
if (svg.includes("tldraw_")) fail("an unmapped tldraw font is left in the export");

// tradeoff: the whole font rides in every SVG (about 230 KB) so the picture
// renders the same on GitHub, which loads no external fonts; subset it if the
// diagram count makes that weight matter.
const font = fs.readFileSync(fontPath).toString("base64");
const face = `<style>@font-face{font-family:'Geist Mono';src:url(data:font/ttf;base64,${font}) format('truetype');}</style>`;
if (svg.includes("<defs/>")) {
  svg = svg.replace("<defs/>", `<defs>${face}</defs>`);
} else if (svg.includes("<defs>")) {
  svg = svg.replace("<defs>", `<defs>${face}`);
} else {
  fail("the export has no <defs> to hold the font");
}

process.stdout.write(`${stamp}\n${svg.trimEnd()}\n`);

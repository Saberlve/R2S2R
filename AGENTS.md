# Personality and writing style
Use plain language over jargon, and reference technical details only to the degree that it helps illustrate an idea or your work to the user. Communicate complex concepts in a clear and cohesive manner, and calibrate your writing to the level of background knowledge assumed from the user's prompt and context.

# Repository work rules

- Keep this repository scene-independent. Case data, captures, asset binaries and run caches stay outside Git.
- Never enable/reset/move a real robot from this pipeline. Capture tools remain explicit operator actions.
- Respect units and T_A_B conventions in docs/CONTRACTS.md; do not silently infer/crop/resize calibration inputs.
- Use existing Newton and Cycles runtimes. Do not pip-install/replace Newton.
- Do not overwrite raw data or a frozen baseline. Every run uses a new output directory.
- Preserve failures and uncertainty in reports. Synthetic success is not real-trajectory contact validation.
- Test changed contracts, the minimal render fixture, and affected adapters. Do not change unrelated user projects.

- The authoritative working copy is on zju. Edit scene JSON/Python and use `./r2s-server`; do not require desktop Blender or MCP. Server bpy/Cycles remains the tested rendering backend.
- Read `site.local.json` and case `current_scene.json` before reference-scene changes. Keep camera/robot frame contracts and uncertainty explicit.
- Server launcher runs must write fresh output directories; visual patches do not update physics automatically.

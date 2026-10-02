---
name: local-imagegen-api
description: Generate project-local raster figures through the configured OpenAI-compatible image API gateway.
---

# Local Image Generation Skill

This skill is used for project-bound raster assets in the opening-defense
presentation. It is intentionally separate from the system image-generation
skill and does not modify that skill or its scripts.

## Environment

The generator reads:

```text
IMAGE_GEN_API_KEY
GATEWAY
```

`GATEWAY` defaults to
`https://api.xcdw.us/v1/images/generations`. The API key must remain in the
local environment and must never be written into prompts, source files,
metadata, logs, or the presentation.

## Execution

Run:

```bash
python3 tools/opening_defense_ppt/imagegen_skill/scripts/generate_image.py \
  --prompt-file tools/opening_defense_ppt/imagegen_skill/prompts/cover_robot.txt \
  --out tools/opening_defense_ppt/output/assets/cover_robot.png \
  --model gpt-image-2 \
  --size 1536x1024 \
  --quality high
```

The script calls the configured OpenAI-compatible image-generation endpoint.
It accepts either a base64 image response or a returned image URL and writes
only the requested raster output.

## Prompt constraints

- Use generated images for scene context or visual atmosphere, not for exact
  equations, sensor labels, factor-graph semantics, or quantitative plots.
- Do not request text inside generated images unless it is strictly
  decorative and non-essential.
- Keep the robot geometry generic unless the prompt is explicitly grounded in
  a supplied reference image.
- Do not fabricate measurements, experimental results, maps, or performance
  curves.
- Inspect every generated image before embedding it in the deck.

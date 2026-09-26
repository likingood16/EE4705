# Task 3.iii: Gemini vs Qwen scene description

10 scenes captured in Gazebo by `run_vlm_comparison.py capture` (each room's
waypoint heading, then rooms 1-4 turned around), images in
`evaluation/scenes/comparison/`. Expected objects were estimated from the world
file and map, then corrected by looking at each image (scene-001 also shows the
human figure and the bookshelf; scene-005 shows only the figure's legs). Both
models got the same prompt (`scene_describer.DESCRIPTION_PROMPT`), temperature 0.
20 trials, one row each in `vlm_scene_trials.csv`.

Scoring: an object counts as reported only if the description names it (or a
clear synonym: "trash bin", "shelving unit", "humanoid figure"). Every keyword
flag was checked against the image; all 11 hallucination flags were false
("bin" inside "cabinet", table "legs", hedges such as "possibly a shelf" about
real wooden surfaces), so none remain.

| | Gemini 3.5 Flash-Lite | Qwen3-VL-Plus |
| --- | --- | --- |
| Expected objects (10 scenes) | 9 | 9 |
| Correctly reported | 7 (78%) | 5 (56%) |
| Missed | 2: figure legs called "white poles"; cinder block | 4: distant figure called a "silver pole", figure legs called "pipes"; table and cinder block described but not named |
| Hallucinated objects | 0 | 0 |
| Mean latency | 18.3 s (median 3.4 s, max 57.3 s) | 1.9 s (median 1.9 s, max 2.7 s) |
| Failed calls in this run | 0 | 0 |
| Tokens per call | 1116 in / 39 out | 359 in / 62 out |
| Cost per call | US$0 (Google AI Studio free tier) | US$0 (Alibaba Model Studio free quota) |

Both models missed the same two hard cases (only the legs of the figure in
scene-005; the small, dark cinder block 3.9 m away in scene-006). Qwen used a
third of the input tokens per image.

## Choice for the final system: Qwen3-VL-Plus

- **Latency.** Qwen answered in 1.4-2.7 s every time. Gemini's median was
  3.4 s, but 3 of 10 calls took 43-57 s. On the same days Gemini also returned
  503 "high demand" and 504 deadline errors, and a 30 s grounding call timed out
  (`task4_evidence/preflight/attempt-6ripgcm0`). In a spoken-style dialogue a
  40 s pause is a failure.
- **Grounding.** Task 4 needs bounding boxes. Qwen-VL is trained for grounding
  and its boxes were tight and correct in every approach test; the brief
  recommends it for this.
- **Accuracy trade-off.** Gemini named two more objects out of nine. Neither
  model hallucinated, and the two it got right that Qwen missed were described
  by Qwen, just not named.
- **Parser.** On the 20 Task 2 utterances Qwen (qwen-plus) scored 20/20 in
  0.74 s mean (`command_parser_trials_qwen.csv`), matching Gemini's 20/20 in
  0.73 s, so the chat parser also defaults to Qwen, with Gemini as fallback.

Environment variables switch any role back to Gemini: `VISION_PROVIDER`,
`GROUNDING_PROVIDER`, `PARSER_PROVIDER`.

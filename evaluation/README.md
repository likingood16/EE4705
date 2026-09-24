# Evaluation records

Use these CSV templates while testing instead of creating results from memory at
the end. Add one row per trial and preserve failed runs.

- `command_parser_trials.csv` - at least 20 utterances
- `vlm_scene_trials.csv` - at least 2 models across 10 scenes
- `object_approach_trials.csv` - at least 10 trials
- `end_to_end_trials.csv` - at least 20 randomized trials

Define success criteria before running the final evaluation. Record latency, cost,
and failure reasons as required by the brief.

## Task 3 VLM comparison (Gemini vs Qwen, 10 scenes)

`run_vlm_comparison.py` produces the 2 models x 10 scenes = 20 trials. Put
`GEMINI_API_KEY` and `QWEN_API_KEY` in `.env` (see `.env.example`), then from the
project root after `source scripts/activate_ubuntu.sh`:

```bash
# 1. With the simulation running: drive to the rooms and save 10 camera frames.
python evaluation/run_vlm_comparison.py capture
# Check evaluation/scenes/comparison/scenes.csv: expected_objects is estimated
# from the world file and map; correct it against the images if needed.

# 2. No simulation needed: send every scene to both models (resumable).
python evaluation/run_vlm_comparison.py evaluate

# 3. Fill correct/missed/hallucinated objects and print the summary table.
python evaluation/run_vlm_comparison.py score
```

Results go to `vlm_scene_trials.csv`. Keyword scoring flags possible
hallucinations; confirm them against the images before quoting numbers.

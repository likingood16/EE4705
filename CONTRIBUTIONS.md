# Contribution record

| Member | Matriculation number | Main responsibilities | Main files | Tests performed |
|---|---|---|---|---|
| Charansagar Ramanujam | A0301658E | Task 2: SLAM map, room waypoints, room navigation, multi-turn chat and command parser | `maps/`, `config/room_waypoints.yaml`, `goto_room.py`, `terminal_chat.py` | 20-utterance command-parser evaluation (`evaluation/command_parser_trials.csv`) |
| Alexander Likin | A0254488N | Task 3: VLM client, scene description, camera capture, result logging, Gemini vs Qwen comparison; simulation bring-up and Nav2 fixes; Task 4 live validation and trial runs; Task 5 end-to-end trials | `vlm_client.py`, `scene_describer.py`, `camera_snapshot.py`, `cli.py`, `result_logger.py`, `evaluation/run_vlm_comparison.py` | 10-scene VLM comparison (`evaluation/vlm_comparison.md`, `vlm_scene_trials.csv`); `test_perception.py`, `test_qwen_vlm.py`, `test_vlm_retries.py` |
| Marie | | Task 4: object grounding, approach geometry, policy, velocity limits, search tracking, trial logging | `object_grounder.py`, `approach_*.py`, `search_tracker.py`, `docs/task4_integration_guide.md` | Offline approach pipeline and safety tests (`test_approach_*.py`, `test_object_grounder.py`, `test_search_tracker.py`) |

Every source file under `ros2_ws/src`, `evaluation` and `scripts` starts with a
two-line header naming its task and the contributors recorded in its git history
(`git log --follow`).

Commits authored 'Claude' were made by an AI coding assistant in Alexander's session (declared in the report).

## Shared work

| Area | Activity | Done by | Evidence / result |
|---|---|---|---|
| Task 1 | Simulation bringup, Nav2 configuration, setup scripts | Alexander Likin | `ros2_ws/src/ee4705_bringup/`, `scripts/`, `docs/results_summary.md` section 6 |
| Task 4 | Integration of the approach controller with the chat, 12 approach trials (3 runs) | Alexander Likin (on Marie's modules) | `evaluation/object_approach_trials.csv`, `evaluation/task4_evidence/` |
| Task 5 | 20 randomized end-to-end trials, demo script | Alexander Likin | `evaluation/end_to_end_trials.csv`, `evaluation/e2e_evidence/`, `docs/demo_script.md` |
| Report | Results summary and labelled floor plan | Alexander Likin | `docs/results_summary.md`, `docs/diagrams/floor_plan_labelled.png` |

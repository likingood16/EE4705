# Contribution record

| Member | Matriculation number | Main responsibilities | Main files | Tests performed |
|---|---|---|---|---|
| Charansagar Ramanujam | A0301658E | Task 2: SLAM map, room waypoints, room navigation, multi-turn chat and command parser; Task 4: integration of navigation, VLM and object approach in the chat; custom world launch; README | `maps/`, `config/room_waypoints.yaml`, `goto_room.py`, `terminal_chat.py`, `approach_controller.py`, `launch/custom_house.launch.py` | 20-utterance command-parser evaluation (`evaluation/command_parser_trials.csv`) |
| Alexander Likin | A0254488N | Task 3: VLM client, scene description, camera capture, result logging, Gemini vs Qwen comparison; Task 4 live validation, approach runtime and trial runs; Task 5 end-to-end trials; simulation setup scripts | `vlm_client.py`, `scene_describer.py`, `camera_snapshot.py`, `cli.py`, `result_logger.py`, `approach_robot.py`, `approach_runtime.py`, `back_away.py`, `evaluation/run_vlm_comparison.py`, `evaluation/run_approach_trials.py`, `evaluation/run_end_to_end_trials.py` | 10-scene VLM comparison (`evaluation/vlm_comparison.md`, `vlm_scene_trials.csv`); `test_perception.py`, `test_qwen_vlm.py`, `test_vlm_retries.py`, `test_approach_robot_fake_ros.py`, `test_approach_runtime.py`, `test_approach_step.py` |
| Marie | | Task 4: object grounding, approach geometry, policy, velocity limits, search tracking, trial logging | `object_grounder.py`, `approach_geometry.py`, `approach_policy.py`, `approach_velocity.py`, `approach_session.py`, `search_tracker.py`, `approach_result_logger.py`, `docs/task4_integration_guide.md` | Offline approach pipeline and safety tests (`test_approach_*.py`, `test_object_grounder.py`, `test_search_tracker.py`) |

Every source file under `ros2_ws/src`, `evaluation` and `scripts` starts with a
two-line header naming its task and the contributors recorded in its git history
(`git log --follow`). Commits authored `likingood16` are from Alexander Likin's account.

Commits authored 'Claude' were made by an AI coding assistant in Alexander's session (declared in the report).

## Shared work

| Area | Activity | Done by | Evidence / result |
|---|---|---|---|
| Task 1 | Simulation setup, system check, setup scripts | Alexander Likin; custom world launch by Charansagar Ramanujam | `ros2_ws/src/ee4705_bringup/`, `scripts/`, `launch/custom_house.launch.py` |
| Tasks 2 and 4 | Integrating navigation, scene description and the approach controller into one chat | Charansagar Ramanujam | `terminal_chat.py`, `approach_controller.py` |
| Task 4 | Approach runtime on Marie's modules, 12 approach trials (3 runs), grounding prompt test | Alexander Likin (on Marie's modules) | `evaluation/object_approach_trials.csv`, `evaluation/task4_evidence/`, `evaluation/grounding_prompt_test.csv` |
| Task 5 | 20 randomized end-to-end trials, demo script | Alexander Likin | `evaluation/end_to_end_trials.csv`, `evaluation/e2e_evidence/`, `docs/demo_script.md` |
| Report | Results summary | Alexander Likin | `docs/results_summary.md` |

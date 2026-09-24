# Task 4 evidence

Every approach attempt writes a folder `attempt-*` with each camera frame sent
to the VLM (`frame-NNN.jpg`), the box drawn on it with the bearing and range
the controller used (`box-NNN.jpg`), the raw model answer and decision
(`frame-NNN.json`), the outcome (`outcome.json`) and a one-row `trial.csv`.

- `preflight/`: live-simulation checks while bringing the controller up
  (Phase A), in order:
  - `attempt-0pialekz`: observation only, Qwen, fire hydrant from room 2.
    Box correct; bearing -5.3 deg vs -5.0 deg from Gazebo ground truth.
  - `attempt-6ripgcm0`: observation only, Gemini: VLM timeout (Gemini was
    returning 503/504 overload errors at the time). Kept as a failure.
  - `attempt-2idrzdq7`: motion, fire hydrant from room 2: arrived in 5 calls,
    final laser range 0.53 m, 0.68 m centre-to-centre (Gazebo).
  - `attempt-kmtjx4it`: motion, car wheel. Nav2 had failed to reach room 4
    (the global costmap TF freeze, fixed in commit 29fa8c0), so the target
    was out of view; the search then aborted on stale sensor data. Kept as a
    failure.
  - `attempt-b4osireg`: motion, car wheel from room 4: arrived in 3 calls,
    0.64 m centre-to-centre.
  - `attempt-ig0fe25j`: motion, fire hydrant from room 2 facing away: found
    after three 45 deg search turns, arrived in 9 calls, 0.70 m.
  - `attempt-j9t4gixl`: motion, "blue ball" (not in the world): full turn,
    8 views, "Sorry, I could not find the blue ball after turning a full circle."
- `chat/`: approaches started from `terminal_chat` ("move to the ...").
- `trials/`: the 12 evaluation trials (`evaluation/run_approach_trials.py`),
  three runs. Run 1 (folders directly in `trials/`) used the first controller
  and grounding prompt; `run2/` after the prompt and controller fixes; `run3/`
  adds laser-confirmed close-range arrival and backing off before a blocked
  turn. Each frame JSON also records the odometry pose it was taken from
  (runs 2-3).

# Task 3 vision work

## First milestone

Run one saved JPEG through `vision_demo` in mock mode, then through one real VLM.
Do not begin with the ROS2 camera connection.

## Final Task 3 requirements

1. Describe the robot's current view concisely.
2. Answer follow-up visual questions about the current view.
3. Compare at least two VLM services on at least ten Gazebo scenes.
4. Record correct, missed, and hallucinated objects, latency, and cost.
5. Explain which model was selected for the integrated system and why.

## Evaluation procedure

For every final scene:

1. Save the source camera image with a stable name such as `room01_view01.jpg`.
2. Write the ground-truth visible objects before reading either model response.
3. Use the same prompt and image for both models.
4. Save both raw responses and latency values.
5. Manually score correct, missed, and hallucinated objects.
6. Add notes explaining ambiguous objects or model failures.

Use `evaluation/vlm_scene_trials.csv` as the shared results table.

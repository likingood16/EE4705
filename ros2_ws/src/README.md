# ROS2 source packages

Place the group's ROS2 packages in this directory. A possible package split is:

- `ee4705_manager` - multi-turn chat and JSON command dispatch
- `ee4705_navigation` - room waypoint loading and Nav2 goals
- `ee4705_perception` - camera capture, VLM calls, and visual Q&A
- `ee4705_approach` - grounding, search, motion control, and laser safety
- `ee4705_bringup` - launch files and full-system configuration

Create only the packages the group actually needs. Keep shared messages and APIs
small so members can test their components independently.

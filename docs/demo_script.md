# Demo video script (Task 5.iii)

One uncut session, about 3.5-4.5 minutes: two rooms, two object approaches,
the second with the object behind the robot (not initially visible), plus a
follow-up question that relies on the chat history. Dry-run on a fresh
simulation: 197 s of robot time, every step succeeded.

## Before recording

```bash
cd ~/EE4705
source scripts/activate_ubuntu.sh
cd ros2_ws && python3 /usr/bin/colcon build --symlink-install && cd ..
ros2 daemon stop
```

Close any previous simulation (never use Gazebo's Reset World). Arrange the
screen: Gazebo on one side (camera following the robot helps), the chat
terminal on the other; RViz optional.

## Terminal 1: simulation

```bash
source scripts/activate_ubuntu.sh
bash scripts/start_simulation.sh 2>&1 | tee ~/sim.log
```

Wait for `Initial AMCL pose was set successfully` (about 30 s), then start
recording.

## Terminal 2: chat (start recording before this)

```bash
source scripts/activate_ubuntu.sh
ros2 run ee4705_perception terminal_chat
```

Type, pressing ENTER after each and waiting for the robot's reply:

1. `Go to Room 4 and tell me what you see.`
   Navigates (~50 s), then describes the black wheel against the wall.
2. `Move to the car wheel.`
   Visible target: turns ~17 deg, drives in two steps, "I am now next to the
   car wheel." (~25 s, 3 VLM calls)
3. `Now go to Room 1.`
   Backs away from the wheel, navigates across the house (~85 s), describes
   the corridor (brick wall, shelves, the white figure). The fire hydrant is
   behind the robot.
4. `Move to the fire hydrant.`
   Not initially visible: turns in 45 deg steps until it sees the hydrant,
   then approaches, "I am now next to the fire hydrant." (~30 s, ~8 VLM calls)
5. `What colour is it?`
   Follow-up resolved from the chat history: "red".
6. `exit`

Each reply is followed by the model used and the stage latencies, which is
worth leaving on screen. If Gazebo is slow on the recording machine, the
navigation legs take longer but the script is the same.

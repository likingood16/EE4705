# Group collaboration guide

## One-time Git setup

Each member should configure Git on their own Ubuntu installation:

```bash
git config --global user.name "Your Name"
git config --global user.email "your-github-email@example.com"
git clone https://github.com/likingood16/EE4705.git
cd EE4705
```

The repository owner must add the other members under GitHub repository
**Settings -> Collaborators**. Each member uses their own GitHub account.

## Starting a piece of work

Always update `main` before creating a branch:

```bash
git switch main
git pull origin main
git switch -c feature/short-description
```

Suggested branch names include:

- `feature/room-waypoints`
- `feature/command-parser`
- `feature/scene-description`
- `feature/object-approach`
- `test/vlm-comparison`
- `docs/literature-review`

Branches describe the feature, not a permanent person. This allows the group to
choose and change its allocation later.

## Saving work

Inspect the changes before committing:

```bash
git status
git diff
```

Commit only relevant files:

```bash
git add path/to/file1 path/to/file2
git commit -m "Implement room waypoint loading"
git push -u origin feature/short-description
```

Do not use `git add .` until you have checked `git status`. It can accidentally
include API keys, generated logs, datasets, or build files.

## Pull requests

1. Open a pull request from the feature branch into `main`.
2. Explain what changed and how it was tested.
3. Ask at least one group member to review it.
4. Resolve conflicts together instead of overwriting another member's code.
5. Merge only when the relevant test passes.
6. Delete the remote feature branch after merging.

After a pull request is merged, every member updates their copy:

```bash
git switch main
git pull origin main
```

## Contribution evidence

The brief requires contributions to be clear in both the report and source files.
Keep `CONTRIBUTIONS.md` current and use Git commits and pull requests as evidence.
For important source files, add a short module docstring identifying the main
author and reviewers.

## Files that must never be committed

- API keys, tokens, passwords, or `.env` files
- `ros2_ws/build/`, `ros2_ws/install/`, or `ros2_ws/log/`
- Python virtual environments or cache files
- Large ROS bag recordings or raw videos
- Unlicensed third-party datasets or model weights

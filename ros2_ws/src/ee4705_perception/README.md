# ee4705_perception

This package is the Task 3 starter for scene description and visual question
answering. Its core code works with saved images before ROS2 camera integration.

## What each file does

- `vlm_client.py` reads the image and talks to a model provider.
- `scene_describer.py` contains the description and question prompts.
- `result_logger.py` writes each trial into the evaluation CSV format.
- `cli.py` provides the command you run in a terminal.
- `test/test_perception.py` verifies the offline pipeline without an API key.

## Run on Windows before Ubuntu is installed

Open PowerShell in the repository root:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install --upgrade pip
py -m pip install -e .\ros2_ws\src\ee4705_perception
```

Choose any JPEG image on your computer, then run the free offline check:

```powershell
vision_demo --image "C:\path\to\room.jpg" --provider mock
```

This confirms the program structure only. A mock response does not inspect the
objects in the image.

Run the tests:

```powershell
py -m unittest discover .\ros2_ws\src\ee4705_perception\test -v
```

## Run with a real OpenAI-compatible VLM

Install the optional API dependency:

```powershell
py -m pip install openai
```

Set the API key only in the current PowerShell session:

```powershell
$env:OPENAI_API_KEY="paste-your-key-here"
```

Run a scene description, replacing `YOUR_MODEL_NAME` with an available vision
model from the selected provider:

```powershell
vision_demo `
  --image "C:\path\to\room.jpg" `
  --provider openai-compatible `
  --model YOUR_MODEL_NAME `
  --log evaluation\vlm_scene_trials.csv `
  --trial-id prototype-001 `
  --scene-id prototype-room
```

Ask a follow-up question:

```powershell
vision_demo `
  --image "C:\path\to\room.jpg" `
  --question "What objects are on the floor?" `
  --provider openai-compatible `
  --model YOUR_MODEL_NAME
```

Never paste an API key into Python, GitHub, screenshots, or group messages.

## Later ROS2 integration

After native Ubuntu is ready, add a ROS2 camera subscriber that saves or converts
the latest `/camera/image_raw` frame and passes it to `SceneDescriber`. Keep the
provider and prompt code independent from ROS2 so it remains easy to test.

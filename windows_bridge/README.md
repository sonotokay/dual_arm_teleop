# windows_bridge

Windows-side MediaPipe hand-tracking + TCP server. WSL2 cannot pass through a
USB webcam, so this instead runs natively on Windows and streams newline-delimited 
JSON of detected hand state over TCP to the teleop node on the Linux side.

setup, run, and architecture are all in [workspace README](../README.md) .

## Files

- `camera_bridge.py` — is the main bridge. OpenCV preview with hand landmarks + slot
  labels, MediaPipe Hands detection, TCP server on `0.0.0.0:5555`. ESC to quit.
- `probe_camera.py` — minimal standalone preview with FPS, no networking. Used
  to confirm the webcam works before involving WSL at all.
- `requirements.txt` — just `mediapipe==0.10.14` and `opencv-python`.

## Quick start (Windows Side)

```powershell
py -3.10 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe camera_bridge.py
```

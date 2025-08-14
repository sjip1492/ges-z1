# ges-z1
# Z1 Gesture Elicitation Study UI – README

## Overview
This application provides a browser-based control interface for the Unitree Z1 robotic arm, integrated with ROS and MoveIt. It is designed to support a **gesture elicitation study** for ultrasonic parametric audio interactions, enabling quick triggering of both arm movements and audio playback controls.

The system consists of:
- **Flask API backend** (`app.py`) — connects the UI to ROS-based control scripts.
- **React-based frontend UI** — served by Flask, built with Vite, provides buttons and controls for arm movements, saved poses, and audio.
- **ROS helper modules** (`study_modules/joint_nudge.py` and `study_modules/state_manager.py`) — handle low-level MoveIt commands and joint state management.

---

## Key Features

### 1. **Nudge Controls (Arm Movement)**
- Move the Z1 arm in **small**, **medium**, or **large** increments.
- Directions: **left**, **right**, **up**, **down**.
- Uses ROS MoveIt to send joint space commands.
- Example backend mapping:
  - Left/Right → `joint1` (base yaw)
  - Up/Down → `joint3` (shoulder pitch)

### 2. **Saved State Management**
- **Save** current joint positions as a named state.
- **Load** saved states to move the arm to preset positions.
- **Edit** saved joint values directly via the UI.
- **Delete** unused states.
- States are stored in YAML (`z1_states/saved_states.yaml`).

### 3. **Referent Quick Actions**
Predefined study referents, accessible with a single click:
- Move up a little (nudge)
- Point at this object (load `point_at_object`)
- Point over there (load `point_over_there`)
- Point toward ceiling (load `point_toward_ceiling`)
- Point toward me (load `point_toward_me`)

### 4. **Audio Playback Controls**
- Upload audio tracks directly from the UI.
- Play/Pause audio.
- Skip to next track.
- Adjust volume up/down.
- Mute.
- Intended to coordinate audio playback with arm positioning during the study.

---

## ROS Integration
The backend uses:
- **`Z1Nudger`** (in `joint_nudge.py`) — wraps MoveIt to apply incremental joint movements.
- **`Z1StateManager`** (in `state_manager.py`) — reads/writes joint positions, saves them to YAML, and executes motions to reach them.
- **MoveIt Commander API** for motion planning and execution.
- Joint naming and direction mapping can be adjusted in `DIR_TO_JOINT` within `app.py`.

---

## Purpose in Gesture Elicitation Study
The UI is built to:
- Quickly execute **consistent, repeatable arm movements** and **pose changes** during participant sessions.
- Allow **experimenters** to trigger predefined referents without typing ROS commands.
- Synchronize **audio actions** with arm gestures to simulate ultrasonic directional audio use cases.
- Support both **social** and **spatial** interaction prompts in the study.

This reduces experimenter workload and ensures that gesture prompts are delivered with consistent robot behavior.

---

## How to Run

### 1. Build the UI
```bash
cd ui
npm install
npm run build
```

### 2. Start ROS Core and MoveIt
```bash
roscore
# In another terminal, launch your Z1 MoveIt configuration
# roslaunch z1_bringup real_arm.launch rviz:=true UnitreeGripperYN:=false 

roslaunch z1_bringup sim_arm.launch rviz:=true UnitreeGripperYN:=false # comment this and uncomment above for controlling the real arm (assuming ethernet connection and power is setup)
```

### 3. Run the Flask App
```bash
cd ..  # project root where app.py lives
python3 app.py
```

### 4. Open in Browser
Visit: [http://127.0.0.1:5001](http://127.0.0.1:5001)

---

## API Endpoints
- `GET /api/health` — health check.
- `GET /api/state/list` — list saved states.
- `POST /api/state/write` — save new state.
- `POST /api/state/load` — load saved state.
- `POST /api/state/delete` — delete state.
- `POST /api/nudge` — move a joint incrementally.

---

## Notes
- Ensure the correct MoveIt group and joint mapping for your Z1.
- If the UI does not load, verify `BUILD_DIR` in logs and ensure `ui/dist/index.html` exists.
- The YAML state file is persistent between runs and can be edited manually if needed.


---

## Example: Randomized Referent Run‑Through
This section illustrates how a **participant** completes a randomized sequence of referents while the **researcher** executes matching actions in the UI.

### Referent pool (study prompts)
- Move arm **small/medium/large** amount **left/right/up/down** *(nudge)*
- Increase volume *(audio)*
- Decrease volume *(audio)*
- Play/Pause sound *(audio)*
- Play next track *(audio)*
- Mute *(audio)*
- Point over there *(saved state: `point_over_there`)*
- Point at this object *(saved state: `point_at_object`)*
- Point toward ceiling *(saved state: `point_toward_ceiling`)*
- Point toward me *(saved state: `point_toward_me`)*

> Ensure those four pose names exist in `z1_states/saved_states.yaml`. Use **Saved States → Save Current** or `--write NAME current` to create them ahead of time.

### Randomize an order (before the participant arrives)
Use any randomizer (spreadsheet, Python, or your study orchestrator). Example Python one‑liner:
```python
import random
referents = [
  'nudge up small','nudge left medium','nudge right large','nudge down small',
  'audio volume +','audio volume -','audio play/pause','audio next','audio mute',
  'state point_over_there','state point_at_object','state point_toward_ceiling','state point_toward_me'
]
random.shuffle(referents)
print('
'.join(f"{i+1:02d}. {r}" for i, r in enumerate(referents)))
```
Print the list and keep it beside you during the session.

### Concrete example run (researcher actions ↔ participant referents)
Suppose the randomized list is:
1. **nudge up small**  
2. **state point_at_object**  
3. **audio play/pause**  
4. **nudge right medium**  
5. **state point_toward_me**  
6. **audio volume +**  
7. **audio next**  
8. **nudge left large**  
9. **state point_over_there**  
10. **audio mute**

**During the session:**
- Read the next prompt to the participant (or display it). Ask them to perform the gesture they believe should trigger that action.
- As soon as they gesture, **click the matching UI control** to provide immediate system feedback.

| Prompt (participant) | Researcher clicks in UI |
|---|---|
| *“Move the sound up a little.”* | **Referents → Move up a little** *(or Nudge → up small)* |
| *“Point at this object.”* | **Referents → Point at this object** *(loads `point_at_object`)* |
| *“Play/Pause the sound.”* | **Audio → Play** *(or Pause, depending on state)* |
| *“Move right a medium amount.”* | **Nudge → right medium** |
| *“Point toward me.”* | **Referents → Point toward me** *(loads `point_toward_me`)* |
| *“Increase the volume.”* | **Audio → Vol +** |
| *“Play the next track.”* | **Audio → Next** |
| *“Move left a large amount.”* | **Nudge → left large** |
| *“Point over there.”* | **Referents → Point over there** *(loads `point_over_there`)* |
| *“Mute the sound.”* | **Audio → Mute** |

### Logging & timing (recommended)
- Start a screen recorder or logging sheet that notes **prompt**, **gesture description (participant)**, **UI action clicked (researcher)**, **timestamp**, and **comments** (ambiguity, negotiation, confidence).
- Optional technical logs:
  - **ROS bag** of `/joint_states` and any controller topics during trials.
  - Append a simple text/CSV log when API endpoints are called (Flask can log each `/api/*` with `time.time()` and payload).
- If study needs randomized **blocks** (e.g., social vs. solo), generate separate shuffled lists per block and label them clearly.

### Pre‑flight checklist before each session
1. Z1 powered, MoveIt launched; **E‑stop** reachable.
2. Flask API running; UI reachable at `http://127.0.0.1:5001/`.
3. Verify **Saved States** exist and load correctly (ceiling / me / object / over there).
4. Test **Nudge** up/down/left/right small.
5. Load a few audio files; verify **Play/Pause/Next/Vol/Mute**.
6. Randomized list printed and visible to the researcher.

### Post‑trial housekeeping
- Save any newly useful poses via **Save Current** and name them descriptively.
- Export logs; note issues or ideas that emerged.
- Reset the arm to a neutral state (e.g., `ready`) and stop audio.

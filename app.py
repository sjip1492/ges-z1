#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
import subprocess

# Import your existing helpers
from study_modules.joint_nudge import Z1Nudger
from study_modules.state_manager import Z1StateManager

# Serve built UI from ./ui/dist
app = Flask(__name__, static_folder="ui/dist", static_url_path="")
CORS(app)  # keep CORS; harmless even on same origin


# Init once per process (runs in same ROS env)
nudger = Z1Nudger()
states = Z1StateManager()

# Map UI directions to a joint. Tweak to your liking:
# - Commonly: left/right = base yaw (joint1), up/down = shoulder pitch (joint3)
DIR_TO_JOINT = {
    "left":  ("joint1", "+"),  # base yaw negative
    "right": ("joint1", "-"),
    "up":    ("joint3", "-"),  # shoulder pitch positive
    "down":  ("joint3", "+"),
}
# --- Frontend routes (serve the built React app) ---

@app.route("/")
def frontend_index():
    return app.send_static_file("index.html")

@app.route("/<path:path>")
def frontend_static(path):
    # Don't shadow API routes
    if path.startswith("api/"):
        return jsonify(error="Not found"), 404
    try:
        return app.send_static_file(path)  # assets like /assets/*.js
    except Exception:
        # SPA fallback: let React Router handle it
        return app.send_static_file("index.html")

@app.route("/api")
def api_root():
    return jsonify(
        message="Z1 Flask API",
        health="/api/health",
        endpoints=[
            "/api/nudge (POST)",
            "/api/state/list (GET)",
            "/api/state/load (POST)",
            "/api/state/load-dynamic (POST)",
            "/api/state/write (POST)",
            "/api/state/delete (POST)",
        ],
    )

@app.route("/api/health", methods=["GET"])
def health():
    return jsonify(ok=True)

@app.route("/api/nudge", methods=["POST"])
def api_nudge():
    data = request.get_json(force=True)
    direction = str(data.get("direction", "")).lower()
    inc = str(data.get("inc", "small")).lower()
    if direction not in DIR_TO_JOINT:
        return jsonify(ok=False, error=f"Unknown direction '{direction}'"), 400
    joint, sign = DIR_TO_JOINT[direction]
    ok = nudger.nudge(joint, direction=sign, inc=inc)
    return (jsonify(ok=True) if ok else (jsonify(ok=False, error="execute failed"), 500))

@app.route("/api/state/list", methods=["GET"])
def api_state_list():
    # Return [{name, joints:{...deg}}]
    import yaml, os
    # states._read_all() is private; mirror its behavior
    try:
        with open(states.db_path, "r") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {"states": {}}
    out = [{"name": k, "joints": v} for k, v in (data.get("states") or {}).items()]
    out.sort(key=lambda x: x["name"])
    return jsonify(states=out)

@app.route("/api/state/load", methods=["POST"])
def api_state_load():
    data = request.get_json(force=True)
    name = data.get("name")
    if not name:
        return jsonify(ok=False, error="Missing 'name'"), 400
    ok = states.load_and_move(name)
    return (jsonify(ok=True) if ok else (jsonify(ok=False, error="execute failed"), 500))

# /state/load-dynamic

@app.route("/api/state/load-dynamic", methods=["POST"])
def api_state_load_dynamic():
    data = request.get_json(force=True)
    name = data.get("name")
    if not name:
        return jsonify(ok=False, error="Missing 'name'"), 400
    #ok = states.load_and_move(name)
    if name == "zigzag":
      ok = subprocess.run(
        ["rosrun", "z1_sdk", "zig_zag.py"],
        check=True,
        capture_output=False,
        text=True
      )
    elif name == "swing":
      ok = subprocess.run(
        ["rosrun", "z1_sdk", "up_down.py"],
        check=True,
        capture_output=False,
        text=True
      )
    return (jsonify(ok=True) if ok else (jsonify(ok=False, error="execute failed"), 500))

@app.route("/api/state/write", methods=["POST"])
def api_state_write():
    data = request.get_json(force=True)
    name = data.get("name")
    if not name:
        return jsonify(ok=False, error="Missing 'name'"), 400

    source = (data.get("source") or "").lower()
    joints = data.get("joints")

    if source in ["current", "current_position", "currentpos"]:
        states.write_state(name, states.current_state_deg())
        return jsonify(ok=True, source="current")

    if not isinstance(joints, dict):
        return jsonify(ok=False, error="Provide 'joints' dict or set source='current'"), 400

    # Expect degrees
    norm = {k: float(v) for k, v in joints.items()}
    states.write_state(name, norm)
    return jsonify(ok=True)

@app.route("/api/state/delete", methods=["POST"])
def api_state_delete():
    data = request.get_json(force=True)
    name = data.get("name")
    if not name:
        return jsonify(ok=False, error="Missing 'name'"), 400

    import yaml, os
    try:
        with open(states.db_path, "r") as f:
            db = yaml.safe_load(f) or {}
    except FileNotFoundError:
        db = {"states": {}}

    if name in db.get("states", {}):
        db["states"].pop(name)
        with open(states.db_path, "w") as f:
            yaml.safe_dump(db, f, sort_keys=True)
        return jsonify(ok=True)
    return jsonify(ok=False, error=f"State '{name}' not found"), 404

if __name__ == "__main__":
    # Make sure ROS is initialized before serving
    if not rospy.core.is_initialized():
        rospy.init_node("z1_ui_backend", anonymous=True, disable_signals=True)
    print("Z1 Flask API starting on http://0.0.0.0:5001")
    app.run(host="0.0.0.0", port=5001, debug=False)

import React, { useEffect, useMemo, useRef, useState } from "react";

// === CONFIG ===
// Change this if your Flask backend runs elsewhere
// const API_BASE = "http://localhost:5001/api";
const API_BASE = "/api";
// Direction -> (joint, sign) mapping is implemented on backend.
// Frontend only sends direction + increment.

export default function App() {
  const [states, setStates] = useState([]);
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState("");
  const [newStateName, setNewStateName] = useState("");
  const [editState, setEditState] = useState(null); // {name, joints}
  const audioRef = useRef(null);
  const [playlist, setPlaylist] = useState([]); // [{name, url, fileObj?}]
  const [currentTrackIdx, setCurrentTrackIdx] = useState(0);
  const [volume, setVolume] = useState(0.8);

  useEffect(() => {
    fetchStates();
  }, []);

  useEffect(() => {
    if (audioRef.current) audioRef.current.volume = volume;
  }, [volume]);

  const currentTrack = useMemo(
    () => playlist[currentTrackIdx] || null,
    [playlist, currentTrackIdx]
  );

  async function fetchStates() {
    setLoading(true);
    try {
      const res = await fetch(`${API_BASE}/state/list`);
      const data = await res.json();
      setStates(data.states || []);
    } catch (e) {
      console.error(e);
      setMessage("Failed to fetch states");
    } finally {
      setLoading(false);
    }
  }

  async function apiPost(path, body) {
    const res = await fetch(`${API_BASE}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    if (!res.ok) throw new Error(await res.text());
    return res.json().catch(() => ({}));
  }

  // --- ROBOT CONTROLS ---
  const nudge = async (direction, inc) => {
    setMessage("");
    try {
      await apiPost("/nudge", { direction, inc });
      setMessage(`Nudged ${direction} (${inc})`);
    } catch (e) {
      setMessage(`Nudge failed: ${e.message}`);
    }
  };

  const loadState = async (name) => {
    setMessage("");
    try {
      await apiPost("/state/load", { name });
      setMessage(`Loaded state '${name}'`);
    } catch (e) {
      setMessage(`Load failed: ${e.message}`);
    }
  };

  const loadStateDynamic = async (name) => {
    setMessage("");
    try {
      await apiPost("/state/load-dynamic", { name });
      setMessage(`Loaded state '${name}'`);
    } catch (e) {
      setMessage(`Load failed: ${e.message}`);
    }
  };

  const saveCurrentAs = async () => {
    if (!newStateName.trim()) {
      setMessage("Enter a name first");
      return;
    }
    try {
      await apiPost("/state/write", {
        name: newStateName.trim(),
        source: "current",
      });
      setMessage(`Saved current position as '${newStateName.trim()}'`);
      setNewStateName("");
      fetchStates();
    } catch (e) {
      setMessage(`Save failed: ${e.message}`);
    }
  };

  const deleteState = async (name) => {
    if (!confirm(`Delete state '${name}'?`)) return;
    try {
      await apiPost("/state/delete", { name });
      setMessage(`Deleted '${name}'`);
      fetchStates();
    } catch (e) {
      setMessage(`Delete failed: ${e.message}`);
    }
  };

  const updateState = async () => {
    if (!editState) return;
    try {
      await apiPost("/state/write", {
        name: editState.name,
        joints: editState.joints,
      });
      setMessage(`Updated '${editState.name}'`);
      setEditState(null);
      fetchStates();
    } catch (e) {
      setMessage(`Update failed: ${e.message}`);
    }
  };

  // --- AUDIO CONTROLS ---
  const onAddAudioFiles = (files) => {
    const items = Array.from(files).map((f) => ({
      name: f.name,
      url: URL.createObjectURL(f),
      fileObj: f,
    }));
    setPlaylist((prev) => prev.concat(items));
    if (!currentTrack) setCurrentTrackIdx(0);
  };

  const play = () => audioRef.current?.play();
  const pause = () => audioRef.current?.pause();
  const next = () => {
    if (playlist.length === 0) return;
    setCurrentTrackIdx((i) => (i + 1) % playlist.length);
  };

  // --- UI ---
  return (
    <div className="min-h-screen p-6 bg-neutral-950 text-neutral-100">
      <div className="max-w-6xl mx-auto space-y-6">
        <header className="flex items-center justify-between">
          <h1 className="text-2xl font-semibold">Z1 Control Panel</h1>
          <div className="text-sm opacity-70">Backend: {API_BASE}</div>
        </header>

        {message && (
          <div className="p-3 rounded-xl bg-neutral-800 shadow-sm">
            {message}
          </div>
        )}

        {/* Referents quick-actions */}
        <section className="grid md:grid-cols-3 gap-4">
          <Card title="Nudge (Move Arm)">
            <div className="grid grid-cols-3 gap-2">
              {[
                ["left", "small"],
                ["left", "medium"],
                ["left", "large"],
                ["right", "small"],
                ["right", "medium"],
                ["right", "large"],
                ["up", "small"],
                ["up", "medium"],
                ["up", "large"],
                ["down", "small"],
                ["down", "medium"],
                ["down", "large"],
              ].map(([dir, inc]) => (
                <Button key={`${dir}-${inc}`} onClick={() => nudge(dir, inc)}>
                  {dir} {inc}
                </Button>
              ))}
            </div>
          </Card>

          <Card title="Saved States">
            <div className="space-y-2">
              <div className="flex gap-2">
                <input
                  value={newStateName}
                  onChange={(e) => setNewStateName(e.target.value)}
                  placeholder="New state name"
                  className="flex-1 px-3 py-2 rounded-xl bg-neutral-900 border border-neutral-800"
                />
                <Button onClick={saveCurrentAs}>Save Current</Button>
                <Button onClick={fetchStates} variant="ghost">
                  Refresh
                </Button>
              </div>
              <div className="max-h-64 overflow-auto divide-y divide-neutral-800">
                {loading ? (
                  <div className="p-2 text-sm opacity-70">Loading…</div>
                ) : states.length === 0 ? (
                  <div className="p-2 text-sm opacity-70">
                    No states saved yet.
                  </div>
                ) : (
                  states.map((s) => (
                    <div key={s.name} className="py-2 flex items-center gap-2">
                      <div className="flex-1">
                        <div className="text-sm font-medium">{s.name}</div>
                        <div className="text-xs opacity-70">
                          {Object.entries(s.joints || {})
                            .map(([k, v]) => `${k}:${v}`)
                            .join("  ")}
                        </div>
                      </div>
                      <Button onClick={() => loadState(s.name)}>Load</Button>
                      <Button variant="ghost" onClick={() => setEditState(s)}>
                        Edit
                      </Button>
                      <Button
                        variant="ghost"
                        onClick={() => deleteState(s.name)}
                      >
                        Delete
                      </Button>
                    </div>
                  ))
                )}
              </div>
            </div>
          </Card>

          <Card title="Audio Controls">
            <div className="space-y-3">
              <div className="text-sm">Playlist: {playlist.length} tracks</div>
              <input
                type="file"
                accept="audio/*"
                multiple
                onChange={(e) => onAddAudioFiles(e.target.files)}
              />
              <div className="text-sm">
                Now Playing: {currentTrack ? currentTrack.name : "None"}
              </div>
              <audio ref={audioRef} src={currentTrack?.url} onEnded={next} />
              <div className="flex gap-2">
                <Button onClick={play}>Play</Button>
                <Button onClick={pause}>Pause</Button>
                <Button onClick={next}>Next</Button>
                <Button
                  variant="ghost"
                  onClick={() => setVolume(Math.max(0, volume - 0.05))}
                >
                  Vol -
                </Button>
                <Button
                  variant="ghost"
                  onClick={() => setVolume(Math.min(1, volume + 0.05))}
                >
                  Vol +
                </Button>
                <Button variant="ghost" onClick={() => setVolume(0)}>
                  Mute
                </Button>
              </div>
            </div>
          </Card>
        </section>

        {/* Referents Section */}
        <section>
          <Card title="Referents – Quick Access">
            <div className="grid md:grid-cols-2 gap-2">
              <Button onClick={() => nudge("up", "small")}>
                Move up a little
              </Button>
              <Button onClick={() => loadState("point_at_object")}>
                Point at this object
              </Button>
              <Button onClick={() => loadState("point_over_there")}>
                Point over there
              </Button>
              <Button onClick={() => loadState("point_toward_ceiling")}>
                Point toward ceiling
              </Button>
              <Button onClick={() => loadState("point_toward_me")}>
                Point toward me
              </Button>
              <Button onClick={() => loadStateDynamic("zigzag")}>
                Zig Zag
              </Button>
              <Button onClick={() => loadStateDynamic("swing")}>Swing</Button>
            </div>
          </Card>
        </section>

        {/* Edit modal */}
        {editState && (
          <div className="fixed inset-0 bg-black/60 flex items-center justify-center p-4">
            <div className="bg-neutral-900 rounded-2xl p-4 w-full max-w-xl space-y-3 border border-neutral-800">
              <div className="text-lg font-semibold">
                Edit state: {editState.name}
              </div>
              <div className="grid grid-cols-2 gap-2">
                {Object.entries(editState.joints || {}).map(([k, v]) => (
                  <label key={k} className="text-sm">
                    <div className="opacity-70 mb-1">{k}</div>
                    <input
                      className="w-full px-3 py-2 rounded-xl bg-neutral-950 border border-neutral-800"
                      type="number"
                      step="0.1"
                      value={v}
                      onChange={(e) =>
                        setEditState((prev) => ({
                          ...prev,
                          joints: {
                            ...prev.joints,
                            [k]: parseFloat(e.target.value),
                          },
                        }))
                      }
                    />
                  </label>
                ))}
              </div>
              <div className="flex gap-2 justify-end">
                <Button variant="ghost" onClick={() => setEditState(null)}>
                  Cancel
                </Button>
                <Button onClick={updateState}>Save</Button>
              </div>
            </div>
          </div>
        )}

        {/* Legend */}
        <section>
          <Card title="Supported Referents">
            <ul className="list-disc ml-6 space-y-1 text-sm opacity-90">
              <li>
                Move arm small/medium/large amount in the left/right/up/down
                direction (nudge)
              </li>
              <li>Increase/decrease volume</li>
              <li>Play/pause sound</li>
              <li>Play the next track</li>
              <li>Mute</li>
              <li>
                Point over there (saved state: <code>point_over_there</code>)
              </li>
              <li>
                Point at this object (saved state: <code>point_at_object</code>)
              </li>
              <li>
                Point toward ceiling (saved state:{" "}
                <code>point_toward_ceiling</code>)
              </li>
              <li>
                Point toward me (saved state: <code>point_toward_me</code>)
              </li>
            </ul>
          </Card>
        </section>
      </div>
    </div>
  );
}

function Card({ title, children }) {
  return (
    <div className="rounded-2xl bg-neutral-900 border border-neutral-800 p-4 shadow-sm">
      <div className="text-base font-medium mb-3">{title}</div>
      {children}
    </div>
  );
}

function Button({ children, onClick, variant }) {
  const base = "px-3 py-2 rounded-xl text-sm shadow-sm";
  const styles =
    variant === "ghost"
      ? "bg-transparent border border-neutral-800 hover:bg-neutral-800/40"
      : "bg-neutral-100 text-neutral-900 hover:bg-white";
  return (
    <button className={`${base} ${styles}`} onClick={onClick}>
      {children}
    </button>
  );
}

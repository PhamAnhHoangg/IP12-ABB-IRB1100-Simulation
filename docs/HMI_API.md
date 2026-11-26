# WP2 part of the IP12 web API

Our share of the WP3 UX/HMI API catalogue: camera, scans, scene, robot model and cell geometry. It follows the
catalogue's conventions: REST under `/api`, JSON, snake_case, lengths in mm, angles in degrees, timestamps ISO 8601 UTC,
errors as Error objects. The robot, the session, the package system, the plan and the tasks are not served here.

```bash
ros2 run irb1100_vision hmi_api.py --ros-args -p port:=8000     # Docker: port 8000 is published
```

## Endpoints

| Catalogue entry | Here | Content |
|---|---|---|
| `GET /health` | `GET /api/health` | `{"camera": "ok" \| "offline"}` (no picture for 3 s = offline); robot and package system come from the other parts |
| `POST /scans` (motion) | `POST /api/scans` | 202 `{scan_id}`. Takes a fresh picture and point cloud **where the camera is now** and builds the scene. Moving the robot to the scan pose happens before this call. `Idempotency-Key` supported. 503 `SUBSYSTEM_OFFLINE` without camera |
| `GET /scans/{id}` | `GET /api/scans/{id}` | `{id, state, origin, started_at, finished_at}`, state running · completed · failed |
| `GET /scene` | `GET /api/scene` | objects of the last completed scan (example below); 204 before the first scan |
| `GET /config/robot-model` | `GET /api/config/robot-model` | joints (origin, axis, limits) from the robot model, base pose, tool mount and TCP, model files |
| `GET /config/scenario` | `GET /api/config/scenario` | cell geometry: table, pallet, robot base. `slots` is empty: the slot layout is the planner's choice |
| model files | `GET /static/irb1100.glb`, `/static/gripper.glb` | glTF binary, mm, z up. Robot: nodes `base`, `link_1` ... `link_6`, nested, each at its joint. Gripper: in the flange frame `tool0` |
| WebSocket `/api/events` | `ws://host:8000/api/events` | `{type, seq, timestamp, data}`: `scan.status_changed`, `scene.updated`, `error.raised` |

## Scene (real output from the simulation)

```json
{ "scan_id": "scan_0a6c76eb", "timestamp": "2026-11-26T18:06:33.740Z",
  "objects": [
    { "id": "obj_1", "location": "source_area", "pose": { "x": -50.4, "y": -119.7, "z": 762.5, "yaw": 26.6 },
      "identification": { "state": "unidentified", "package_code": null, "confidence": 0.0, "unreadable_reason": "too_far" } } ],
  "pallet": { "id": "pallet_1", "slots": [] } }
```

- `pose`: box centre from the point cloud (`cloud_boxes.py`), cell frame. `yaw` is 0-90 deg: a square box looks the
  same every 90 deg.
- `location`: `gripper` (within 40 mm of the TCP), `pallet` (inside the pallet outline) or `source_area`.
- `identification`: a QR code read from the box top (OpenCV). `confidence` 0.99 when read, 0.3 when a code is seen but
  cannot be decoded, otherwise 0. `unreadable_reason`:

| Reason | When |
|---|---|
| `bad_light` | the box top is too dark, too bright or has almost no contrast |
| `too_far` | the box top covers fewer than 87 pixels (the smallest QR code needs about 29 modules × 3 px) |
| `damaged` | a code is found but cannot be decoded |
| `covered` | no code found on a box top that is big enough and well lit |
| `unknown` | the box is cut off at the picture edge |

## To agree with the web team

1. **Cell frame:** here it is the simulation's `world` frame; the robot base sits at (200, 0, 750) mm in it (`base_pose`).
2. **Code size:** the 25 mm test boxes are about 58 px wide from 30 cm, too small for any QR code (hence `too_far` above).
   Real packages with a code of at least 3-4 cm will work; the test boxes need to be bigger or seen from closer.
3. **Joint zero:** `zero_offset_deg` is 0 for all joints, assuming the ABB controller and the robot model count from the
   same zero. To be checked against the FlexPendant once.
4. **Table size** in `/config/scenario` is the simulation's (1.1 × 1.1 m); the real table is not measured yet.

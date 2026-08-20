# DOGZILLA web gateway

The web gateway is a small ROS 2 service for monitoring DOGZILLA and queuing
autonomous waypoint tasks. Its browser map renders the live occupancy grid,
shows the localized robot pose, accepts click-only pickup/drop-off selection,
and previews the real Nav2 path before a task is queued. It keeps task history
and reusable named locations in SQLite, then sends each validated delivery stop
to Nav2's `/navigate_to_pose` action.

## Start it

Build the image after pulling these files. After a saved map exists and mapping
has been stopped, the normal one-command startup is:

```bash
cd /home/pi/DOGZILLA
./deploy/dogzilla-map build
./deploy/dogzilla-map mission test1 --headless
./deploy/dogzilla-map mission password
```

`mission` starts Nav2 and the web gateway in order, waits for both health checks
and required ROS interfaces, and rolls both back if startup fails. `test1`
selects the matching PBStream/YAML/PGM map bundle. `--headless` skips RViz, and
`password` prints the dashboard login password. Startup never queues a goal.

The lower-level two-terminal equivalent remains available for diagnostics:

```bash
# Terminal 1
cd /home/pi/DOGZILLA
./deploy/dogzilla-map navigate test1 --headless

# Terminal 2
cd /home/pi/DOGZILLA
./deploy/dogzilla-web start test1
./deploy/dogzilla-web show-password
```

Open the URL printed by `start` on a device on the same trusted LAN, then use
the printed password to sign in. The default is `yahboom`; it lives in the
ignored `.env` file and is stored only in browser session storage.

Useful operator commands:

```bash
./deploy/dogzilla-map mission status
./deploy/dogzilla-map mission logs
./deploy/dogzilla-map mission stop
```

## What it monitors

- Robot mode and ROS node availability
- Battery percentage and freshness
- Localized map position, heading, and speed
- Occupancy map, selected waypoints, planned route, and joint telemetry
- Active mission, queue, progress, failures, and history

## Map editor

Select **Pickup** or **Drop-off**, then click a free map cell once. There is no
drag-and-drop interaction. Coordinates are read-only and generated from the
map transform, while heading uses a fixed eight-direction selector. This keeps
the submitted numbers predictable on desktop and touch devices.

The canvas distinguishes free, occupied, and unknown cells and overlays the
live `map -> base_link` robot pose. After both points are selected, the gateway
asks Nav2's `/compute_path_to_pose` action for a non-executing path and draws
that returned path. This is a preview only; it does not move the robot.

The browser performs an immediate cell check, but the browser is not trusted.
The gateway repeats validation against its own current occupancy snapshot when
previewing, saving a named location, and creating a task. It rejects goals that
are outside the map, in unknown space, occupied, or within the configured
clearance of those cells. The default clearance is 0.18 m.

The **Named locations** panel saves the currently selected pickup or drop-off.
Names are unique per map; saving the same name again updates it. Applying a
saved location changes only the editor. The delivery is not dispatched until
**Queue delivery** is pressed and all runtime safety gates pass.

The dashboard values come directly from these runtime sources:

| Displayed state | Authoritative source | Freshness / use |
| --- | --- | --- |
| Battery | `/battery_state` from `safe_base` | Shown with age; tasks require a reading no older than 12 seconds and at least 28% by default. |
| Pose | `map -> base_link` TF | True map-frame position and heading; dispatch requires a reading no older than 3 seconds. |
| Motion | `/odom` | Linear and angular speed are combined with the map-frame pose. |
| Joint positions | `/joint_states` from `safe_base` | Latest position count and stale marker are shown. |
| Map | transient-local `/map` | Map name, grid dimensions, and resolution; required before dispatch. |
| Robot/Nav2 mode | ROS node graph and `/navigate_to_pose` action | Refreshed every second; action availability is required before dispatch. |
| Mission state | SQLite task store and Nav2 goal results | Updated at every queue, waypoint, cancellation, and terminal transition. |

The JSON API is under `/api/v1`, with the `X-Dogzilla-Password` header required
for every API and event-stream request. The previous bearer token remains a
temporary compatibility fallback. `/healthz` and the static login page are the
only unauthenticated routes.

Core API routes:

```text
GET  /api/v1/state
GET  /api/v1/map
GET  /api/v1/tasks
GET  /api/v1/locations
POST /api/v1/locations
DELETE /api/v1/locations/{id}
GET  /api/v1/keepout-zones
POST /api/v1/keepout-zones
DELETE /api/v1/keepout-zones/{id}
POST /api/v1/routes/preview
POST /api/v1/tasks/delivery
POST /api/v1/tasks/route
POST /api/v1/tasks/{id}/cancel
POST /api/v1/map/switch/prepare
POST /api/v1/map/switch
POST /api/v1/autonomy/speed
POST /api/v1/drive
POST /api/v1/estop
POST /api/v1/estop/reset
GET  /api/v1/events
```

`/api/v1/autonomy/speed` accepts independent integer `speed_level` and
`turn_level` values from 1 through 9. It updates the safe-base limits applied
to Nav2 before an autonomous task starts. The manual `/api/v1/drive` code is
retained for future work but returns a conflict while the default
`DOGZILLA_WEB_MANUAL_DRIVE_ENABLED=false` setting is active, and the dashboard
does not show its direction pad.

A delivery request uses map-frame metres and radians:

```json
{
  "name": "Parts to lab",
  "map": "test1",
  "pickup": {"x": 0.4, "y": -0.2, "yaw": 0, "dwell_seconds": 5},
  "dropoff": {"x": 2.1, "y": 1.3, "yaw": 1.57, "dwell_seconds": 0}
}
```

Generic routes use the same waypoint fields in a `waypoints` array. Tasks move
through `queued`, `running`, and a terminal `completed`, `failed`, or
`cancelled` state. A process restart marks any interrupted active task failed;
it never silently resumes a partially completed delivery.

## Task and safety behavior

A delivery contains a pickup and drop-off coordinate in the active map frame.
The gateway rejects malformed, non-finite, out-of-range, wrong-map, occupied,
unknown, outside-map, or insufficient-clearance goals.
It does not dispatch queued tasks until Nav2, a map, fresh localization, and a
fresh battery reading at or above the configured threshold are available.

The browser emergency stop cancels the active Nav2 goal, publishes zero
velocity to both the priority teleop input and final velocity topic, and stays
latched until explicitly reset with a safe battery reading. It is a software
safety layer, not a certified physical emergency stop. Keep the robot supervised
and within reach during development.

The web container deliberately has no serial devices. The navigation container
remains the only owner of the controller and LiDAR ports. Both containers use
host networking and the same `ROS_DOMAIN_ID` so they can exchange ROS messages.

Do not forward port 8080 directly to the public internet. For remote access,
put it behind a VPN or a TLS-authenticated reverse proxy and change the default
password.

Optional `.env` tuning:

```text
DOGZILLA_WEB_OCCUPIED_THRESHOLD=50
DOGZILLA_WEB_GOAL_CLEARANCE=0.18
```

`DOGZILLA_WEB_OCCUPIED_THRESHOLD` is the first ROS occupancy value considered
blocked. `DOGZILLA_WEB_GOAL_CLEARANCE` is the minimum free radius around the
selected cell in metres. It is a goal-selection check, not a replacement for
the Nav2 footprint and costmap configuration.

# DOGZILLA web gateway

The web gateway is a small ROS 2 service for monitoring DOGZILLA and queuing
autonomous waypoint tasks. It serves a responsive browser dashboard, keeps
task history in SQLite, and sends each validated delivery stop to Nav2's
`/navigate_to_pose` action.

## Start it

Build the image after pulling these files, then start localization and Nav2:

```bash
cd /home/pi/DOGZILLA
./deploy/dogzilla-map build
./deploy/dogzilla-map navigate test1 --headless
```

In a second terminal, initialize and start the gateway for the same map:

```bash
cd /home/pi/DOGZILLA
./deploy/dogzilla-web init
./deploy/dogzilla-web start test1
./deploy/dogzilla-web show-token
```

Open the URL printed by `start` on a device on the same trusted LAN, then use
the printed token to sign in. The token lives in the ignored `.env` file and is
stored only in browser session storage.

Useful operator commands:

```bash
./deploy/dogzilla-web status
./deploy/dogzilla-web logs
./deploy/dogzilla-web stop
```

## What it monitors

- Robot mode and ROS node availability
- Battery percentage and freshness
- Localized map position, heading, and speed
- Map metadata and joint telemetry
- Active mission, queue, progress, failures, and history

The dashboard values come directly from these runtime sources:

| Displayed state | Authoritative source | Freshness / use |
| --- | --- | --- |
| Battery | `/battery_state` from `safe_base` | Shown with age; tasks require a reading no older than 12 seconds and at least 28% by default. |
| Pose and motion | `/odom` | Shown with age; dispatch requires localization no older than 3 seconds. |
| Joint positions | `/joint_states` from `safe_base` | Latest position count and stale marker are shown. |
| Map | transient-local `/map` | Map name, grid dimensions, and resolution; required before dispatch. |
| Robot/Nav2 mode | ROS node graph and `/navigate_to_pose` action | Refreshed every second; action availability is required before dispatch. |
| Mission state | SQLite task store and Nav2 goal results | Updated at every queue, waypoint, cancellation, and terminal transition. |

The JSON API is under `/api/v1`, with a bearer token required for every API
and event-stream request. `/healthz` and the static login page are the only
unauthenticated routes.

Core API routes:

```text
GET  /api/v1/state
GET  /api/v1/tasks
POST /api/v1/tasks/delivery
POST /api/v1/tasks/route
POST /api/v1/tasks/{id}/cancel
POST /api/v1/estop
POST /api/v1/estop/reset
GET  /api/v1/events
```

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
The gateway rejects malformed, non-finite, out-of-range, or wrong-map goals.
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
put it behind a VPN or a TLS-authenticated reverse proxy and rotate the token.

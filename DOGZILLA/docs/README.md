# DOGZILLA S2 documentation

This directory is the maintained technical handoff for the Yahboom DOGZILLA
S2 on this Raspberry Pi. It documents the software that is present and the
behavior that can be verified from source, public serial traffic and runtime
interfaces.

The complete editable Word export is
[DOGZILLA_S2_COMPLETE_DEVELOPER_DOCUMENTATION.docx](DOGZILLA_S2_COMPLETE_DEVELOPER_DOCUMENTATION.docx).
It contains every maintained Markdown chapter in this directory plus the
embedded four-layer pipeline diagram.

The embedded motor-controller firmware source is not available. These files do
not pretend to reconstruct its private gait, power-button, low-battery or servo
control internals. That boundary is documented explicitly so the next
developer does not mistake a Python API for the firmware itself.

## Start here

| Document | Audience and purpose |
| --- | --- |
| [DEVELOPER_HANDOFF.md](DEVELOPER_HANDOFF.md) | First read for a new maintainer: responsibilities, invariants, layout, runtime modes, testing and known limitations |
| [OPERATIONS_RUNBOOK.md](OPERATIONS_RUNBOOK.md) | Exact Pi-host commands for mapping, localization, missions, patrol, vision, calibration, diagnostics and safe shutdown |
| [FIRMWARE_AND_SERIAL.md](FIRMWARE_AND_SERIAL.md) | Controller boundary, installed Yahboom library, serial framing/registers, scaling, actions, IMU, battery, stand/rest safety and unknown firmware internals |
| [FRAMEWORK.md](FRAMEWORK.md) | Current hardware, container, ROS, web, vision and persistence architecture |
| [PIPELINES.md](PIPELINES.md) | End-to-end data/control flows for mapping, localization, Nav2, web tasks, patrol, keepouts, vision and deployment |
| [ROS_INTERFACES.md](ROS_INTERFACES.md) | Frames, topics, actions, services, nodes, Compose device ownership, web API and freshness rules |
| [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md) | Prioritized safety/reliability/calibration work with acceptance gates and explicit non-goals |

## Specialized documents

| Document | Scope |
| --- | --- |
| [COMPUTER_VISION.md](COMPUTER_VISION.md) | Camera modes, detector models, patrol hazards, alerts and guarded Yahboom lesson integration |
| [URDF_RTABMAP_MONO.md](URDF_RTABMAP_MONO.md) | Provisional URDF and isolated monocular RTAB-Map shadow experiment |
| [DOGZILLA_PIPELINE_SLIDE.svg](DOGZILLA_PIPELINE_SLIDE.svg) | Editable four-layer presentation diagram |
| [DOGZILLA_PIPELINE_SLIDE.png](DOGZILLA_PIPELINE_SLIDE.png) | Rendered presentation image |

Related source-area guides:

- [ROS package README](../ros2/dogzilla_slam/README.md)
- [deployment README](../deploy/README.md)
- [web mission README](../web/README.md)
- [calibration README](../calibration/README.md)
- [model README](../models/README.md)
- [navigation stability candidate](../plans/navigation-stability/README.md)

## Architecture at a glance

```text
Hardware layer
  MS200 LiDAR | DOGZILLA controller/IMU/battery/joints | mono camera
          |
          v
Perception layer
  LaserScan | corrected IMU | image processing and patrol detections
          |
          v
SLAM/localization layer
  Cartographer -> map, odom and robot pose
          |
          v
Navigation/application layer
  web tasks -> Nav2 -> command filters -> safe_base -> controller
```

Cartographer is the SLAM implementation used by this project; “SLAM” is the
general mapping/localization problem. Nav2 consumes the map and pose but does
not create them. The camera perception pipeline observes objects and people;
it is not the operational localization or collision authority.

## Source of truth order

When documentation and behavior disagree, investigate in this order:

1. physical safety and observed hardware state;
2. the currently running Docker image and complete session logs;
3. current source plus configuration and tests;
4. these maintained documents;
5. historical Yahboom examples and notebooks.

The original `app_dogzilla/` and `Samples/` files are useful protocol and
lesson references, but they bypass the ROS single-owner safety design and are
not the operational runtime.

## Documentation maintenance rules

Update documentation in the same change when any of these change:

- device ownership, serial protocol or controller limits;
- frames, topics, actions, services or QoS/freshness gates;
- Docker services, volumes, image dependencies or operator commands;
- map/calibration/database formats;
- task states, browser/API behavior or authentication;
- model classes, patrol readiness or alert retention;
- safety behavior, physical acceptance status or known limitations.

Label facts as one of: verified from source, observed on this robot, prepared
but inactive, experimental, or unknown. Never convert a guess into firmware
documentation because it produced one successful motion.

## Rebuild and Git are separate

Editing source does not modify an already running container. `dogzilla build`
creates a local Docker image, and restarting a mode deploys that image. Neither
operation commits or pushes Git. Conversely, a Git commit backs up source but
does not include Docker layers, task databases, calibration not committed by
policy, or all robot-specific runtime data.

## Rebuild the Word manual

After changing a chapter, rebuild the combined document from the repository:

```bash
python3 docs/build_word_manual.py
```

`python3` runs the installed interpreter and `docs/build_word_manual.py` reads
the maintained chapter order, renders the Markdown, embeds the pipeline image,
uses headless LibreOffice to create the DOCX, and validates the resulting Word
package. It does not start Docker, ROS or robot hardware.

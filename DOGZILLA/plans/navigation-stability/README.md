# Navigation stability candidate

Status: **prepared but inactive**.

Nothing in this directory is loaded by ROS, Docker, Cartographer, or Nav2.
`candidate-control` only changes source files when its `apply` or `revert`
command is explicitly run. Preparing this candidate did not rebuild the image
or restart the robot.

## Evidence and intent

The 2026-08-25 mission logs show the Cartographer pose-graph work queue growing
past 1,500 items while observed scan processing falls from about 10 Hz toward
7 Hz. The active behavior trees also request a new global plan at 1 Hz.

The candidate makes one conservative change set:

- reduce both Nav2 behavior-tree replanning rates from 1 Hz to 0.5 Hz;
- match `expected_planner_frequency` to 0.5 Hz so its warning remains useful;
- reduce Cartographer localization background threads from four to two;
- optimize every 30 nodes instead of every 10 nodes;
- reduce local/global constraint sampling from 0.20/0.05 to 0.10/0.02;
- retain two live localization submaps instead of three;
- publish pose TF at 20 Hz, trajectory state at 5 Hz, and submaps at 0.5 Hz;
- give Nav2 0.50 seconds of transform tolerance for normal 10 Hz scan age and
  short Pi scheduling jitter.

It deliberately does **not** change LiDAR sampling, controller frequency,
walking speed, costmap update rates, obstacle marking, collision detection, or
emergency-stop behavior. This keeps the first trial reversible and isolates
the CPU/timing variables.

## Prepared workflow

From `/home/pi/DOGZILLA`:

```bash
./plans/navigation-stability/candidate-control status
./plans/navigation-stability/candidate-control apply
```

After applying, run the package regression, rebuild Docker, and perform a
stationary timing check before a short open-floor route. Applying the patch
alone still does not alter a running container.

To undo the source changes before or after a trial:

```bash
./plans/navigation-stability/candidate-control revert
```

Do not combine this first trial with IMU, speed, footprint, or obstacle-layer
tuning. Compare Cartographer queue growth, LiDAR/TF age, controller deadline
misses, replans, and cross-track error against the existing recorder output.

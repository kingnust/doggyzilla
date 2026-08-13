# DOGZILLA `base_link` measuring reference

`base_link` is the geometric centre of DOGZILLA's rigid main torso in the
current URDF. It is not the floor, a foot, the LiDAR, or the Raspberry Pi.

Viewed from above, with DOGZILLA standing normally:

```text
                         +X forward
                             ^
                             |
             +Y left  <--- base_link
                             |
                        main torso
```

The positive axes follow ROS REP-103:

- `+X`: forward, toward DOGZILLA's front/camera
- `+Y`: DOGZILLA's left
- `+Z`: upward
- roll: rotation around `+X`
- pitch: rotation around `+Y`
- yaw: rotation around `+Z`

Measure from the centre of `base_link` to the camera lens centre. Enter the
translation in metres and the camera mounting angles in degrees:

```bash
dogzilla camera-extrinsics --measured X Y Z ROLL PITCH YAW
```

The current body dimensions are provisional, so the most reproducible physical
reference is the centre of the rigid torso box. The camera transform must be
measured on the actual robot before physical RTAB route acceptance.

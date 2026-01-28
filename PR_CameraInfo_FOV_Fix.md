# ROS2: Correct CameraInfo intrinsic computation (degrees to radians)

<!--
Checklist:

  - [ ] Branch rebased and up-to-date with target (ue4-dev or ue5-dev)
  - [ ] Extended README / docs if necessary
  - [ ] Code compiles correctly
  - [ ] All tests passing with `make check` (Linux)
  - [ ] CHANGELOG.md updated if required
-->

## Description
The ROS2 camera intrinsic matrix (sensor_msgs::CameraInfo K) previously computed focal length incorrectly by passing a degree FOV directly into std::tan(). This yielded unrealistically large focal values (e.g. ~289307 for width=1616, fov=60). The corrected formula converts FOV from degrees to radians before tan:

Before:
```
fx = width / (2.0 * tan(fov) * π / 360.0)
```
After:
```
fx = width / (2.0 * tan(fov * π / 360.0))
```
Resulting example (width=1616, height=1240, fov=60):
```
K = [1399.4970525, 0, 808,
     0, 1399.4970525, 620,
     0, 0, 1]
```
`fy` set equal to `fx` assuming square pixels and horizontal FOV source.

Fixes # (add issue number if available)

## Where has this been tested?
- Platforms: Linux x86_64
- Python versions: 3.8, 3.10
- Unreal Engine versions: (specify UE4.x or UE5.x depending on target branch)

## Possible Drawbacks
- Downstream consumers relying on old (incorrect) focal length must adjust; any precomputed projections or calibrations need regeneration.
- Recorded datasets using previous intrinsic may show scale differences.
- Assumes provided FOV is horizontal; pipelines interpreting it as vertical should verify.

## Checklist
- [ ] Rebased on correct target branch (ue4-dev / ue5-dev)
- [ ] Verified compilation succeeds
- [ ] Ran `make check` and tests pass
- [ ] No doc changes required (or updated if needed)
- [ ] CHANGELOG.md entry added under Fixed (if project practice requires)

## Changelog Entry (suggested)
Fixed: Corrected CameraInfo focal length computation (degree to radian conversion before tan).
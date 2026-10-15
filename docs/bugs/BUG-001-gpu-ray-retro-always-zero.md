# BUG-001 — `gpu_ray` ignores laser_retro; every voxel gets the default backscatter

- Status: Fixed
- Found: 2026-10-15, in plan 007 (code from plan 004, commit bcd1414)
- Fixed: 2026-10-15, in plan 007 commit
- Severity: major
- REQs: REQ-007

## Symptom

With `sensor_type:=gpu_ray`, the plate target in `sas_point_target.world` (`laser_retro` 1.0) barely
rose above the seabed clutter: matched-filter peak/median ≈ 2–4, and the peak landed −40 … +6
samples off on 7 of 16 elements. With CPU `ray` the same scene gave peak/median 12–34 and all
elements within ±1.3 samples. Debug summary: `raw retro max=0 | mu=[0.001995, 0.001995]` with
`gpu_ray`, `raw retro max=1 | mu=[0.002, 1] voxels mu>0.1: 56` with `ray`.

## Reproduction

```
ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world sensor_type:=gpu_ray \
  speed:=0.0 x:=0.0 debug:=true
python3 tests/validation/check_delay.py --target 0 -14.2606 4.8565 --radius 0.01
```

## Root cause

`ReadScan()` (`src/sas_gazebo/src/sas_sonar_plugin.cpp`) called `GpuRaySensor::Retro(i)`, which in
Gazebo 11 is a stub returning 0.0. All retro values were 0, so `BuildScatterers` used
`mu_default` everywhere. The unit tests feed `Scan.retro` directly and could not see this.

## Fix

Read retro from the raw GPU laser frame via `ConnectNewLaserFrame`, with a per-ping alignment check
against `Ranges()` (ADR-0011). Worlds also set `<laser_retro>` on visuals, which `gpu_ray` reads.

## Regression test

`tests/validation/check_delay.py` on `sas_point_target.world` with `gpu_ray` (plan 007 exit check):
before the fix FAIL (peak/median ≈ 3); after the fix PASS, all 16 elements −0.81 … +0.37 samples,
peak/median 23–38. Plan 008 V1 runs it for both sensor types. The debug summary line
(`debug:=true`) shows the retro range per ping.

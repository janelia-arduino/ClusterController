# ClusterController Bring-Up Summary

## Current Status

- The board on the bench is flashed with the current known-good rewrite firmware plus firmware-side travel clamps.
- Cluster `10` is confirmed reachable over Ethernet at `192.168.10.10`.
- The single-cluster `1x7` setup is working through `hex_maze_interface` over Ethernet for the tested workflows.

## Current Firmware Behavior

- Homing uses the rewrite's custom home flow, not the `TMC51X0::beginHomeToStall()` / `endHome()` flow.
- Hardware stall-stop is enabled during homing in the rewrite path.
- After successful home, the firmware restores the normal runtime driver/controller settings.
- Normal move targets are clamped to `0..550 mm`.
- Home travel limit is clamped to `0..650 mm`, then applied as the negative home target.

Relevant file:
- [rewrite_prism.cpp](/home/peter/Repositories/arduino/ClusterController/firmware/rewrite_prism.cpp)

## Latest Verified Single-Cluster Results

Using cluster `10` through `hex_maze_interface` over Ethernet:

- Home with `travel_limit=650`, `max_velocity=20`, `run_current=50`, `stall_threshold=10`
  - `homed`: `[1, 1, 1, 1, 1, 1, 1]`
  - `outcomes`: `["STALL", "STALL", "STALL", "STALL", "STALL", "STALL", "STALL"]`
  - `positions_mm`: `[0, 0, 0, 0, 0, 0, 0]`
- Move to `40 mm`
  - final positions: `[35, 35, 35, 35, 35, 35, 35]`
- Move to `600 mm`
  - final positions: `[545, 545, 545, 545, 545, 545, 545]`
  - this confirms the `550 mm` move clamp is active on hardware

## What Was Tried And Rejected

- A refactor to use the `TMC51X0` library home methods directly (`beginHomeToStall()` / `homed()` / `endHome()`) was tested and is **not safe to use on the rig in its current integration**.
- During that experiment, one prism ran far in the positive direction to a hard stop.
- That experimental change was reverted.
- The board was reflashed back to the last known-good custom homing path before continuing.

Conclusion:
- Do **not** re-enable the library-managed home path on live hardware without further offline debugging.

## Python Side Status

Repo:
- [hex_maze_interface_python](/home/peter/Repositories/python/hex_maze_interface_python)

Current useful changes there:
- `power_on_cluster()` now waits for the firmware's 2 second prism setup window before returning.
- A live hardware smoke runner exists.
- Python tests were passing earlier in the session.

Useful files:
- [hex_maze_interface.py](/home/peter/Repositories/python/hex_maze_interface_python/hex_maze_interface/hex_maze_interface.py)
- [hardware_smoke_test.py](/home/peter/Repositories/python/hex_maze_interface_python/hardware_smoke_test.py)

## What Still Needs To Be Done

### Before Full `7x7`

- Run more single-cluster experiment-style Ethernet tests using the exact command pattern used by the real experiment code.
- Specifically test:
  - repeated home cycles
  - repeated single-target moves
  - repeated double-target moves
  - pause/resume during motion
  - pause/resume with queued double-target motion
  - longer soak cycles to look for the legacy "stops moving" failure

### For `7x7` Bring-Up

- Flash all seven cluster boards with the same known-good clamped firmware.
- Confirm each board's IP from the TCA6408 switch setting.
- Bring up one cluster at a time over Ethernet first.
- Then run a staged multi-cluster test from `hex_maze_interface`.
- Only after that, run the actual experiment command sequence across all seven clusters.

## Recommended Next Session Plan

1. Confirm the bench board still responds at `192.168.10.10`.
2. Run a short single-cluster validation:
   - power cycle
   - pre-position
   - home with `travel_limit=650`
   - move to an interior target
   - move to `600 mm` and verify it clamps near `550 mm`
3. Run the exact experiment-style Python command sequence on the single cluster.
4. If that remains solid, flash the other six cluster boards.
5. Start staged `7x7` Ethernet bring-up.

## Important Safety Notes

- Stay on the current custom homing path for live hardware.
- Keep the firmware clamp in place:
  - moves: `0..550 mm`
  - home travel: `0..650 mm`
- Do not retry the `TMC51X0` library-managed home integration on the full rig without a safer debug plan.

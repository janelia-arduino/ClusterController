# ClusterController Finish Checklist

This checklist is optimized for getting the rewrite usable on real hardware as
fast as possible while keeping the compatibility target on the Python API.

## Fast Path Goal

Target for the next 1 to 2 days of constant development work:

- one real cluster responds over Ethernet
- `HexMazeInterface` can verify, power cycle, home, and move the attached prism
- the rewrite remains the active path
- temporary debug instrumentation is allowed
- release-quality cleanup is deferred

This is narrower than "fully finished". The point is to get a reliable working
path first, then clean it up.

## Day 1: Make Homing Work Reliably

- [x] Compare rewrite one-prism homing flow against `TMC51X0/examples/SPI/ClusterHomeBench/ClusterHomeBench.ino`
- [x] Make rewrite `begin_home()` and prism service flow structurally match the bench path for one prism
- [ ] Add temporary serial logging in the rewrite prism path for:
  - homing start
  - homing state changes
  - `homed()`
  - `homeFailed()`
  - recovery attempts
  - communication loss
- [x] Confirm the prism actually moves during `home_cluster()`
- [x] Confirm `homed_cluster()` changes from all-zero to the expected homed state
- [x] Confirm `read_positions_cluster()` changes after homing
- [x] Replace strict stall-only success with the current legacy-compatible policy:
  - stall detection still counts as success
  - reaching the commanded negative target without stall also counts as success
- [x] Wire in `recoverIfUnhealthy()` or equivalent conservative recovery in the prism maintenance loop
- [x] Re-test repeated sequence 5 to 10 times on the real experimental travel:
  - power on
  - home
  - poll homed
  - read position

## Day 2: Make Motion Usable From Host Code

- [x] Validate `pause_cluster()` and `resume_cluster()` after successful homing
- [x] Validate `write_target_prism()` on the attached prism
- [x] Validate `write_targets_cluster()` for the active hardware layout
- [x] Validate `write_double_target_prism()` on the attached prism
- [x] Validate `write_double_targets_cluster()` for the active hardware layout used now
- [x] Confirm `read_positions_cluster()` tracks commanded motion well enough for host use
- [x] Re-test repeated sequence 5 to 10 times:
  - power on
  - home
  - move
  - pause
  - resume
  - read positions
  - power off

## Host Software Checklist

- [ ] Keep existing `hex_maze_interface` high-level methods working:
  - `communicating_cluster()`
  - `power_on_cluster()`
  - `home_cluster()`
  - `write_targets_cluster()`
  - `pause_cluster()`
  - `resume_cluster()`
  - `read_positions_cluster()`
  - `read_run_current_cluster()`
  - `read_controller_parameters_cluster()`
- [x] Verify `HexMazeInterface().verify_cluster(10)` still succeeds against the rewrite
- [ ] Update host transport/parsing only if the rewrite response behavior now differs materially
- [ ] Add or refresh a minimal hardware integration script that exercises:
  - verify
  - power cycle
  - home
  - move
  - read positions
- [ ] Defer broad host refactors until the firmware path is stable

## Dependency Checklist

### `TMC51X0`

- [ ] Use explicit v4 recovery helpers in firmware where prism state may drift:
  - `recoverFromDeviceReset()`
  - `recoverIfNeeded()`
  - `recoverIfUnhealthy()`
- [ ] Respect v4 stall-home semantics:
  - `homed()` means confirmed home
  - `homeFailed()` means stop treating the cycle as success
- [ ] Re-run the standalone homing bench sketch if rewrite behavior diverges again

### `TCA6408`

- [ ] Reconfirm cluster address read on the target hardware if address behavior looks suspect
- [ ] Otherwise treat `TCA6408` as low-risk and do not spend time here unless an address or interrupt symptom appears

## Minimum "Up And Running" Definition

Call the system up and running when all of the following are true on the real
hardware you care about now:

- [x] firmware boots reliably
- [x] Ethernet connects reliably
- [x] `verify_cluster()` succeeds
- [x] `power_on_cluster()` succeeds repeatedly
- [x] `home_cluster()` produces real motion and a real homed result
- [x] `write_target_prism()` or `write_targets_cluster()` moves the prism
- [x] `read_positions_cluster()` reports changing positions
- [x] 5 to 10 repeated cycles complete without the prism silently dropping out

## Deferred Until After It Works

These items matter, but they should not block the fast-path push:

- [ ] remove or hide temporary debug commands `0x19`, `0x1A`, `0x1B`
- [ ] remove temporary serial diagnostics
- [ ] clean up firmware module boundaries
- [ ] decide whether protocol `0x04` stays or changes
- [ ] broaden tests to full multi-prism coverage
- [ ] add broader `TMC51X0` validation coverage:
  - UART hardware validation
  - switch / homing bench validation record
  - `TMC5160A` coverage
  - second-MCU coverage
- [ ] refresh release documentation

## Execution Notes

- Do not add more public protocol before homing and basic motion work on the real setup.
- Favor temporary diagnostics over blind debugging.
- Favor one attached prism that works repeatedly over partial support for all seven.
- Commit at working checkpoints, not only at the end.

## Current Hardware Status

- Verified live on cluster `10` at `192.168.10.10`.
- Active prism on the current bench appears to be prism index `1`.
- Short-travel homing (`travel_limit=40`) is stable across repeated power cycles.
- Longer-travel cycling (`travel_limit=120`) passed 5 repeated power-cycle,
  home, cluster-target, and return-to-zero runs.
- The legacy-compatible fallback is active: reaching the commanded negative target
  without a stall event still marks the prism homed and zeros position.
- `pause_prism()`, `pause_cluster()`, `resume_prism()`, and `resume_cluster()`
  hold position during the pause window and resume motion afterwards.
- `write_double_target_prism()` now queues correctly:
  raw target stays on the first target, then switches to the second target only
  after the first is reached.
- `reset_cluster()` now leaves the cluster in a clean unhomed powered-off state.

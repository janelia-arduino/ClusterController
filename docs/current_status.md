# Current Rewrite Status

## Recommended Commit Boundary

This is a reasonable checkpoint to commit and push, but it should be treated as:

- a hardware-validated single-cluster rewrite checkpoint
- a stable `pico-rewrite` flash and host-verification checkpoint
- ready for broader staged validation on larger attached setups

The rewrite firmware is validated for transport, power sequencing, homing,
pause/resume, reset recovery, and cluster-wide prism motion on the current
single-cluster bench. The next work is larger attached setup validation, not
basic rewrite bring-up.

## What Is Working

The rewrite firmware in `firmware/` is now the active development path.

Hardware-validated on the connected `pico-w5500-evb` and single `TMC5130`
prism board:

- RP2040 boot and loop-driven firmware structure
- `W5500lwIP` Ethernet bring-up on the real board
- static IP and TCP server on port `7777`
- `read_cluster_address`
- `communicating_cluster`
- `beep_cluster`
- `led_on_cluster`
- `led_off_cluster`
- `power_on_cluster`
- `power_off_cluster`
- `pause_cluster`
- `resume_cluster`
- `read_positions_cluster`
- `read_run_current_cluster`
- `write_run_current_cluster`
- `read_controller_parameters_cluster`
- `write_controller_parameters_cluster`
- `home_cluster`
- `homed_cluster`
- `write_target_prism`
- `write_targets_cluster`
- `write_double_target_prism`
- `write_double_targets_cluster`
- prism communication on the attached board at CS pin `8`
- host-side `HexMazeInterface().verify_cluster(10)` succeeds
- repo `hardware_smoke_test.py --clusters 10` succeeds
- repo `hardware_preinstall_acceptance_test.py --cluster 10 --prism 1`
  succeeds
- prism power-cycle recovery through `power_off_cluster()` and
  `power_on_cluster()`
- reset recovery through `reset_cluster()` followed by `power_on_cluster()`
- cluster-wide target writes remain reliable after cluster homing
- pause and resume now hold position on the bench during the pause window
- rewrite firmware build and USB flash through PlatformIO `pico-rewrite`
- `ClusterController` build tasks now use repo-local `PLATFORMIO_CORE_DIR`
- `ClusterController` now pins `TMC51X0` to `4.0.3`

Current rewrite defaults are intentionally close to the original firmware for
driver/controller setup.

## What Is Not Working Yet

The main unfinished area is broader staged hardware validation:

- repeated soak validation on the current single-cluster bench
- one-cluster repeated seven-prism validation after any further firmware change
- full-rig seven-cluster validation
- cleanup of temporary debug protocol once the rewrite is called complete

## Important Findings From Today

### 1. Transport is no longer the main risk

The rewrite transport path is functioning on real hardware.

### 2. Standalone `TMC51X0` motion works on this hardware

Standalone testing in the `TMC51X0` repo confirmed:

- normal motion works on the same prism hardware
- recovery and power cycling work

This means the hardware and the underlying library are not fundamentally stuck.

### 3. Remaining motion bugs were rewrite integration issues

A standalone bench sketch was added in the `TMC51X0` repo:

- `/home/peter/Repositories/arduino/TMC51X0/examples/SPI/ClusterHomeBench/ClusterHomeBench.ino`

That sketch uses:

- `SPI1` on pins `10/11/12`
- prism CS pin `8`
- prism power on `GP15`
- ClusterController-style converter, driver, controller, and homing parameters

It did not reproduce the rewrite's earlier long-running active homing behavior.

That confirmed the remaining bugs were in the rewrite integration path, not just
in the raw driver configuration. The final single-cluster fixes on this bench
were:

- remove the periodic status LED blink from the active rewrite path
- force `HoldMode` during pause and restore `PositionMode` on resume
- avoid silently dropping a post-home cluster target when a prism hits a
  transient communication miss during `write_target()`

### 4. Useful reference extracted from `LED-Display_G4.1_ArenaController_Slim`

The LED-display rewrite is still a useful reference for structure, even if it is
not needed as an active repo dependency for the next session.

Useful points copied from that project:

- the main value is architectural, not hardware-specific
- it successfully replaced the old QP/state-machine design with a small
  loop-driven design and explicit state tracking
- it keeps the top-level loop ordered and simple:
  1. service network input
  2. process at most one ready command
  3. service the main device state/work
  4. flush network responses
- it separates responsibilities cleanly:
  - `main.cpp` for setup and loop ordering
  - `NetworkManager` for TCP accept/read/parse/write
  - `CommandProcessor` for command handling and device state transitions
  - hardware/storage managers kept out of protocol parsing
- its network layer uses explicit RX buffering, command-ready flags, and delayed
  response flushing rather than doing all work inline with the socket read

Implication for `ClusterController`:

- keep the rewrite moving toward the same shape:
  - network service
  - command dispatch
  - prism/cluster service tick
  - response flush
- avoid drifting back toward hidden state-machine behavior spread across command
  handlers
- if homing debugging continues to be difficult over Ethernet alone, add simple
  serial diagnostics in the rewrite the same way the standalone bench sketches do
  rather than adding more protocol complexity

## Current Rewrite State

Files of interest:

- `firmware/main.cpp`
- `firmware/rewrite_app.cpp`
- `firmware/rewrite_bsp.cpp`
- `firmware/rewrite_cluster_address.cpp`
- `firmware/rewrite_network.cpp`
- `firmware/rewrite_prism.cpp`

Important implementation notes:

- the active rewrite no longer uses a periodic PCB status blink
- `power_on_cluster()` reinitializes prism setup asynchronously so the host does
  not time out waiting for a response
- `reset_cluster()` intentionally leaves the cluster communicating but prism
  power off; host recovery should call `power_on_cluster()` afterwards
- pause now forces the controller into `HoldMode`, then returns to
  `PositionMode` on resume
- post-home target writes now queue instead of being silently lost on a
  transient prism communication miss
- controller-parameter writes are still intentionally conservative and should
  not be assumed to be motion-safe in the final sense
- active extra debug command still present during bring-up:
  - `0x19` for `read_home_outcomes_cluster`

Any additional temporary debug commands mentioned in older notes should be
treated as stale unless reintroduced in code.

## Best Next Step Tomorrow

Resume with staged hardware validation rather than more firmware changes first:

1. Re-run smoke and acceptance checks after any future firmware change.
2. On the attached `1` cluster / `7` prism setup, run a longer soak with repeated
   power, home, move, pause, resume, and reset-recovery cycles.
3. On the full `7 x 7` rig, validate discovery, per-cluster verify, and
   cluster-to-cluster behavior under network load.

## Suggested Commit Message

Suggested message for this checkpoint:

`rewrite: stabilize single-cluster pause and post-home target writes`

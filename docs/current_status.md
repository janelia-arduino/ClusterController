# Current Rewrite Status

## Recommended Commit Boundary

This is a reasonable checkpoint to commit and push, but it should be treated as:

- a solid rewrite transport, motion, and single-cluster rewrite checkpoint
- a dependency-sync checkpoint with `TMC51X0` bumped to `4.0.3`
- ready for staged hardware validation tomorrow, starting with single-cluster
  Python readback checks

The rewrite firmware is useful and validated for transport, cluster-level
commands, power sequencing, homing, and basic prism motion on the current bench.
The next work is staged hardware validation across larger attached setups, not
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
- prism power-cycle recovery through `power_off_cluster()` and
  `power_on_cluster()`
- rewrite firmware build and USB flash through PlatformIO `pico-rewrite`
- `ClusterController` build tasks now use repo-local `PLATFORMIO_CORE_DIR`
- `ClusterController` now pins `TMC51X0` to `4.0.3`

Current rewrite defaults are intentionally close to the original firmware for
driver/controller setup.

## What Is Not Working Yet

The main unfinished area is broader staged hardware validation:

- single-cluster Python communication and non-destructive readback checks after
  today's firmware flash
- one-cluster seven-prism validation
- full-rig seven-cluster validation
- cleanup of temporary debug commands before calling the rewrite complete

## Important Findings From Today

### 1. Transport is no longer the main risk

The rewrite transport path is functioning on real hardware.

### 2. Standalone `TMC51X0` motion works on this hardware

Standalone testing in the `TMC51X0` repo confirmed:

- normal motion works on the same prism hardware
- recovery and power cycling work

This means the hardware and the underlying library are not fundamentally stuck.

### 3. Standalone homing behavior does not match rewrite behavior

A standalone bench sketch was added in the `TMC51X0` repo:

- `/home/peter/Repositories/arduino/TMC51X0/examples/SPI/ClusterHomeBench/ClusterHomeBench.ino`

That sketch uses:

- `SPI1` on pins `10/11/12`
- prism CS pin `8`
- prism power on `GP15`
- ClusterController-style converter, driver, controller, and homing parameters

It does not reproduce the rewrite's long-running active homing behavior.

That means the remaining bug is in the rewrite integration path, not just in the
raw driver configuration.

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

- heartbeat beeping was removed for office use
- `power_on_cluster()` reinitializes prism setup asynchronously so the host does
  not time out waiting for a response
- controller-parameter writes are still intentionally conservative and should
  not be assumed to be motion-safe in the final sense
- temporary debug commands were added during homing investigation:
  - `0x19`
  - `0x1A`
  - `0x1B`

Those debug commands are useful for continued bring-up but should eventually be
removed or hidden before calling the rewrite complete.

## Best Next Step Tomorrow

Resume with staged hardware validation rather than more firmware changes first:

1. On the current `1` cluster / `1` prism setup, run Python communication and
   non-destructive readback checks.
2. On the `1` cluster / `7` prism setup, validate cluster-wide homing and
   target-write behavior.
3. On the full `7 x 7` rig, validate discovery, per-cluster verify, and
   cluster-to-cluster behavior under network load.

## Suggested Commit Message

Suggested message for this checkpoint:

`rewrite: pin TMC51X0 4.0.3 and validate pico-rewrite flash path`

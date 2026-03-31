# Current Rewrite Status

## Recommended Commit Boundary

This is a reasonable checkpoint to commit and push, but it should be treated as:

- a solid rewrite transport and non-motion compatibility checkpoint
- an active homing investigation checkpoint
- not yet a "homing works" checkpoint

The rewrite firmware is useful and validated for transport, cluster-level commands,
power sequencing, and basic prism communication. The remaining blocker is
single-prism stall homing behavior in the integrated rewrite.

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
- prism communication on the attached board at CS pin `8`
- host-side `HexMazeInterface().verify_cluster(10)` succeeds
- prism power-cycle recovery through `power_off_cluster()` and
  `power_on_cluster()`

Current rewrite defaults are intentionally close to the original firmware for
driver/controller setup.

## What Is Not Working Yet

`home_cluster()` is implemented but not validated.

Current behavior on the rewrite firmware:

- `home_cluster(10, HomeParameters())` returns success to the host
- `homed_cluster(10)` stays all zero over repeated polls
- `read_positions_cluster(10)` stays at `0` for the attached prism
- this should be treated as incomplete behavior, not successful homing

So the main remaining blocker is:

- integrated stall homing behavior for the single connected prism

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

Do not add more public protocol commands first.

Resume with the single-prism homing investigation:

1. Compare the rewrite `begin_home()` and service loop against the standalone
   `ClusterHomeBench` path until they are structurally identical for one prism.
2. Add temporary serial logging to the rewrite prism path if needed, instead of
   continuing blind Ethernet-only debugging.
3. Once one-prism homing is reliable, revalidate:
   - `home_cluster()`
   - `homed_cluster()`
   - `read_positions_cluster()`
   - `pause_cluster()`
   - `resume_cluster()`
4. After homing is stable, move on to target writes and the remaining motion
   commands.

## Suggested Commit Message

Suggested message for this checkpoint:

`rewrite: checkpoint transport and prism bring-up; homing investigation in progress`

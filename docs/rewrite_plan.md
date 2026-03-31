# ClusterController Rewrite Plan

## Goal

Rewrite `ClusterController` to improve reliability and simplify maintenance while preserving the public Python API exposed by `hex_maze_interface` as much as possible.

The firmware rewrite should:

- remove the QP framework
- replace the current event-driven architecture with a simple loop-driven design
- replace or greatly simplify the current Ethernet integration
- avoid copying the current homing state machine blindly
- use the hardened `TMC51X0` library recovery capabilities intentionally

## Compatibility Boundary

The primary compatibility target is the Python API, not the current wire protocol.

Researchers should ideally keep using the same high-level Python methods:

- `communicating_cluster()`
- `power_on_cluster()`
- `home_cluster()`
- `write_targets_cluster()`
- `pause_cluster()`
- `resume_cluster()`
- `read_positions_cluster()`
- `read_run_current_cluster()`
- `read_controller_parameters_cluster()`

The wire protocol may change if both firmware and `hex_maze_interface` are updated together.

## Current Risks

### 1. Ethernet stack uncertainty

`QNEthernet` is not a fit for the current RP2040 target.

Decision:

- use the Arduino-Pico built-in `W5500lwIP` stack
- build the TCP server using `WiFiServer` and `WiFiClient`
- keep `Mongoose` only as a fallback if the built-in stack proves problematic on the actual hardware

Reasoning:

- the RP2040 Arduino core in the local toolchain already includes W5500 support
- it exposes a small TCP server/client API without requiring Mongoose
- it supports static IP configuration, link status, SPI tuning, and optional interrupt-driven packet handling
- it is a better fit for the required use case than a general-purpose networking framework

Planned transport model:

- `#include <W5500lwIP.h>`
- instantiate `Wiznet5500lwIP eth(cs_pin, SPI, int_pin);`
- configure static IP from the cluster address before `begin()`
- use `WiFiServer server(7777);`
- accept one client at a time with `accept()`
- use explicit RX/TX buffers and deterministic request/response handling

### 2. Homing reliability uncertainty

We should assume the current homing logic is not trustworthy enough to copy directly.

Known concerns:

- the current QP homing state machine appears to have an unsafe retry path
- disconnected prisms do not appear to have a robust recovery path
- the old firmware does not appear to use the newer `TMC51X0` recovery hooks as part of runtime fault handling
- the original failure could still involve electrical faults or driver shutdowns

### 3. Protocol redesign uncertainty

We may want a cleaner request/response protocol with:

- explicit framing
- explicit status codes
- request identifiers
- clearer error reporting
- optional aggregated status reads

But we should validate the transport choice first before finalizing protocol changes.

## Target Architecture

The rewrite should move toward a small set of plain modules:

- `main.cpp`
  - initialization
  - top-level service loop
- `NetworkManager`
  - static IP setup from cluster address
  - TCP server
  - RX/TX buffering
  - packet framing
- `CommandProcessor`
  - decode requests
  - validate parameters
  - dispatch to cluster/prism operations
  - build responses
- `ClusterManager`
  - cluster-wide settings
  - power state
  - service tick
  - dispatch to prisms
- `PrismController`
  - per-prism state
  - communication status
  - homing state
  - queued targets
  - recovery and fault handling
- `Bsp` or hardware adapters
  - TCA6408 address read
  - W5500 setup
  - TMC51X0 access
  - LED, tone, watchdog, power control

## Homing Design Principles

The new homing implementation should be explicit and conservative.

Each prism should track:

- communication healthy or not
- configured or not
- homing state
- last successful contact time
- last fault reason
- retry count

Homing should follow a clear sequence:

1. verify communication
2. recover or reinitialize if needed
3. configure homing mode
4. start homing exactly once
5. poll progress with timeout
6. classify result as success, timeout, communication fault, or device fault
7. perform explicit cleanup
8. allow bounded recovery attempts
9. leave the prism in a recoverable fault state rather than a dead-end state

## First Uncertainty-Reduction Steps

These should happen before a full detailed implementation plan.

### Step 1: transport spike

Build a minimal loop-based firmware spike that:

- boots on RP2040
- reads the cluster address
- brings up Ethernet using `W5500lwIP`
- uses the Wiznet interrupt pin if available
- accepts a single TCP client
- handles a single simple command such as:
  - `read-cluster-address`, or
  - `communicating-cluster`

Validate during this spike:

- static IP assignment based on cluster address
- link detection and reconnect behavior
- repeated connect/disconnect cycles
- partial and malformed packet handling
- one-client request/response operation with `WiFiServer` and `WiFiClient`

Fallback only if this spike fails:

- return to a simplified `Mongoose` design

### Step 2: homing/recovery spike

Build a standalone prism service experiment that:

- initializes one prism
- starts a home-to-stall operation
- polls status without QP
- uses `TMC51X0` recovery methods when communication or mirror state indicates a problem
- logs enough detail to distinguish:
  - normal success
  - ordinary home failure
  - communication loss
  - possible electrical/device reset behavior

Decision to make after this spike:

- final homing state model
- retry policy
- runtime recovery policy

### Step 3: protocol decision

After the transport spike succeeds, decide whether to:

- keep protocol version `0x04` with internal cleanup only, or
- define a new protocol version with clearer framing and errors

## Milestones

### Milestone 1

Create loop-driven firmware skeleton and minimal network command path.

### Milestone 2

Implement cluster address, communication check, LED, beep, and power commands.

### Milestone 3

Implement per-prism communication/configuration management.

### Milestone 4

Implement new homing and recovery logic for one prism, then expand to all prisms.

### Milestone 5

Implement target writes, pause/resume, and position reads.

### Milestone 6

Implement controller parameter read/write commands.

### Milestone 7

Update `hex_maze_interface` transport layer and add compatibility tests.

## Success Criteria

The rewrite is successful when:

- the Python API remains stable or nearly stable for researchers
- the firmware is smaller and easier to understand than the current QP version
- transport behavior is deterministic and easy to debug
- prism communication failures can be detected and recovered intentionally
- homing failures are classified and handled explicitly
- the system can run repeated homing and motion cycles without prisms silently dropping out

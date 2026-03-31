# ClusterController Firmware Layout

The repository now keeps the legacy firmware and the rewrite side by side so
development can continue without mixing the two designs.

## Build Targets

- `pio run -e pico-legacy`
  - builds the existing QP-based firmware from the legacy library path
- `pio run -e pico-rewrite`
  - builds the new loop-driven rewrite library path
- `pio run -e feather-rewrite`
  - builds the same rewrite entrypoint for the Feather RP2040 target

## Source Split

- `firmware/main.cpp`
  - active Arduino entrypoint selected by build flag
- `firmware/bsp.cpp`, `firmware/mongoose.c`, `firmware/mongoose.h`, `firmware/mongoose_config.h`
  - active legacy-only support sources kept in the shared firmware source root so the legacy build remains self-contained
- `examples/ClusterControllerLegacy/`
  - archived legacy sketch files and legacy Mongoose files kept for reference
- `examples/ClusterControllerRewrite/`
  - archived rewrite sketch entrypoint kept for reference
- `src/ClusterController/`
  - legacy implementation sources
- `src/ClusterControllerRewrite/`
  - rewrite implementation sources

The intent is to grow the rewrite in isolation, validate it on hardware in
small steps, and remove the legacy path only after the new firmware is proven.

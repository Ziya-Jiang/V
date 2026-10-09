# motor_data

Small C helper library for RMD-style motor state storage and decoding.

This project is not a CAN driver and not a USB-CAN passthrough layer. It only provides a motor data structure plus helper functions used by the current host-side demo code.

## Current capability

- Defines `motor_data`, containing:
  - motor id
  - motor type
  - gear ratio
  - encoder resolution
  - position, velocity, torque, current
  - raw/last position and multi-turn circle tracking state
- Defines motor type enum values:
  - `RMD_V1`
  - `RMD_V3`
  - `RMD_V4`
  - `RMD_V4_1`
  - `RMD_V999`
- Provides:
  - `motor_init()` to initialize a `motor_data` instance.
  - `motor_decoding()` to scale decoded fields and maintain multi-turn position tracking.
- Builds as an object/interface target via CMake for use by `usb_can_upper`.

## Current behavior

`motor_decoding()` assumes that raw `position`, `velocity`, and `torque` fields have already been filled by upper-layer protocol decoding before it is called.

For `RMD_V1`:

- torque is scaled by `33.0 / 2048 / 1.6`.
- velocity is divided by `gear_ratio`.
- position is converted using `360.0 / gear_ratio / resolution`.

For `RMD_V3`:

- torque is scaled by `0.01`.

The function also tracks encoder wraparound by updating `circle`, `last_position`, and `has_first_data`.

## Important current limitations

- No CAN frame parsing is implemented here.
- No USB, SPI, libusb, SocketCAN, or firmware interaction is implemented here.
- RMD command packing/response decoding is outside this library, currently in `usb_can_upper/src/usb_can_rmd_motor.*` and demo code.
- The helper is motor-specific and should remain separate from any future generic CAN passthrough API.
- `motor_decoding()` returns `int` but currently has no explicit success return at the end.

## Build

```bash
cmake -S . -B build
cmake --build build
```

When used from `usb_can_upper`, it is added as a sibling subdirectory and linked through `MOTOR_DATA_INTERFACE`.

## Recommended wording for current status

This is a motor data helper library used by the existing demo application. It should be treated as an optional application-layer example, not as part of the generic USB-CAN transport.

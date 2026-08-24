# Contributing to ScanBot3000 Firmware

Contributions that improve correctness, portability, documentation, diagnostics, or safety are welcome.

## Development workflow

1. Start from the current `main` branch and make a focused change.
2. Build every affected PlatformIO environment.
3. For motion, sensor, protocol, or pin-map changes, document what hardware was actually tested.
4. Keep protocol changes synchronized with `ScanBot3000-control` and `ScanBot3000-kinematics` when applicable.
5. Update README/docs when commands, wiring, dependencies, or behavior change.

## Public-data rules

Never commit credentials, API keys, tokens, Wi-Fi configuration, private network addresses, hostnames, personal email addresses, identifying filesystem paths, serial numbers, device IDs, machine-specific ports, logs containing private data, or local build/editor state. Use explicit placeholders such as `<host>`, `<serial-device>`, and `<path>`.

Firmware target codenames that are part of the source/API naming are acceptable; deployment-specific machine names are not.

## Pull-request checklist

- [ ] Affected PlatformIO environments build.
- [ ] Pin mappings and hardware assumptions remain accurate.
- [ ] Motion/safety behavior is described without unsupported guarantees.
- [ ] No generated `.pio`, editor, log, or secret material is included.
- [ ] Cross-repository interfaces remain documented.
- [ ] Licensing and third-party notices remain intact.

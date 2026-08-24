# AI / Agent Prompting

Use prompts that force changes to stay grounded in repository evidence and real hardware constraints.

## Firmware change review

```text
Review this ScanBot3000 firmware change for correctness. Use only the code, platformio.ini, and documented hardware as evidence. Identify affected pins, serial protocols, timing, motion limits, sensor assumptions, and cross-repository API consequences. Do not invent unverified hardware behavior.
```

## Public-data review

```text
Audit the proposed firmware diff for public release. Flag credentials, tokens, private addresses, hostnames, personal data, identifying paths, generated PlatformIO/editor files, machine-local dependencies, or logs. Distinguish real values from obvious placeholders.
```

## Documentation update

```text
Update the firmware documentation to match this code change. Separate verified project hardware from unknown minimum/recommended requirements. Preserve exact commands and units from source; mark anything not demonstrated by repository evidence as unknown.
```

Never paste real credentials, private infrastructure, serial numbers, or private logs into an AI prompt. Redact environment-specific values first.

# Security Policy

## Security model

ScanBot3000 firmware is embedded control software for physical machinery. Its USB/UART command interfaces do not implement authentication or authorization. Treat physical serial access and any network service bridged to that serial interface as privileged control access.

Do not expose the ScanBot3000 control server directly to the public internet. Place network controls, authentication, segmentation, and remote-access policy outside this firmware layer.

## Physical safety

Commands can energize motors and cause motion. Validate wiring, end stops, motor direction, travel limits, and emergency-stop provisions before operating unattended or at full speed. Software stop behavior is not represented as a certified safety function.

## Reporting a vulnerability

Use GitHub's private vulnerability-reporting or Security Advisory mechanism for this repository when available. If private reporting is unavailable, contact the repository owner through GitHub without publishing exploit details, credentials, or private infrastructure information in a public issue.

## Sensitive information

Do not include tokens, secrets, private addresses, hostnames, personal data, serial numbers, identifying paths, or deployment configuration in reports or test fixtures. Redact those values before attaching logs.

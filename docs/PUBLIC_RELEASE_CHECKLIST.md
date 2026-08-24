# Public Release Checklist

## Repository state

- [x] Canonical repository name uses `ScanBot3000-firmware`.
- [x] README uses the ScanBot3000 project identity and links the project home, control server, and kinematics repositories.
- [x] Apache License 2.0 + Commons Clause 1.0 terms, NOTICE, CONTRIBUTING, and SECURITY files are present.
- [x] Setup, requirements, architecture, troubleshooting, and AI/agent guidance are present.

## Cleanup and privacy

- [x] Tracked `.vscode` editor state removed.
- [x] Internal agent instruction files removed from the public tree.
- [x] Redundant vendored `AccelStepper` copy removed; PlatformIO already declares the dependency.
- [x] Missing machine-local VL6180X dependency path replaced with the official PlatformIO registry package name.
- [x] Current-tree review found no committed credentials, API keys, passwords, Wi-Fi SSIDs, private IPv4 addresses, personal filesystem paths, or personal email addresses in project files.
- [x] Environment/build artifacts are covered by `.gitignore`.

## History

- [x] Existing development history was reviewed before release.
- [x] Historical commit metadata included personal/example email identities and development-only history not suitable for the requested single-commit public baseline.
- [x] `main` is rewritten to one parentless `Initial public release` commit containing only the sanitized current tree.
- [x] No additional repository branches were present during the release audit.
- [x] No tag namespace was returned by the available Git-ref check during the release audit.

## Accuracy

- [x] Hardware claims are limited to the Teensy 4.1 / ESP32-S3 / sensor configuration evidenced by source and existing docs.
- [x] General minimum CPU/RAM/power/motor requirements are explicitly left unknown where not evidenced.
- [x] UART rates, PlatformIO environments, and command examples match repository configuration.
- [ ] Revalidate the sanitized VL6180X registry dependency on physical R-axis hardware before treating this release as a production baseline.

## Final operator review

Before changing visibility, review the rendered README, LICENSE/NOTICE, hardware requirements, and the open revalidation item above.

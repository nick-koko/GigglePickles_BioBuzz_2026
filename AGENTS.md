# Repository instructions

## Required hardware and configuration

- Resolve every device required by the selected opmode or subsystem mode during Init. Use `hardwareMap.get(...)` for required devices and let its exception surface immediately so the Driver Station reports the missing or misnamed device before Play.
- Avoid defensive null checks, `tryGet(...)`, catch blocks that swallow lookup errors, and dummy devices or default readings for required dependencies. If a required setting is invalid, raise a clear error at Init.
- Treat hardware as optional only when an explicitly selected mode is designed to run without it. Keep runtime recovery for devices that were found at Init and later give bad readings.

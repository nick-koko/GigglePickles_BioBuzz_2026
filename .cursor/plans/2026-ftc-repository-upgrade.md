---
name: 2026 FTC Repository Upgrade
overview: Upgrade the clean 2025 production baseline one dependency layer at a time. Keep the newly released NextFTC v2 migration isolated until the FTC kickoff SDK is published and the conventional dependency upgrades are stable.
todos:
  - id: create-repo
    content: Create nick-koko/GigglePickles_BioBuzz_2026 from exact 2025 commit 91a7175, push it as main, and tag the baseline
    status: completed
  - id: prune-legacy
    content: Remove unneeded 2025 OpModes/files, remove Fateweaver usage, and establish the smaller upgrade baseline
    status: completed
  - id: upgrade-ftc
    content: Apply the complete FTC 11.2.1 tooling/workspace upgrade and separate official from third-party dependencies
    status: completed
  - id: upgrade-vendors
    content: Upgrade Pedro to 2.1.2, verify Panels 1.0.12 and NextFTC v1, and validate each change independently
    status: pending
  - id: add-build-guardrails
    content: Ignore generated script artifacts and add a JDK 17 compile-only Android workflow
    status: completed
  - id: upgrade-kickoff-sdk
    content: After release, merge the full official kickoff SDK update on its own branch and validate it
    status: pending
  - id: migrate-nextftc-v2
    content: On a separate branch, remove v1, establish a NextFTC v2 vertical slice, and port only active 2026 code
    status: pending
isProject: true
---

# 2026 FTC Repository and Dependency Upgrade

## Completed baseline
- The public repository is `nick-koko/GigglePickles_BioBuzz_2026`, with `GigglePickles_BioBuzz_2026` as the local folder.
- The 2025 GitHub repository's default branch was `master`, not `main`. The new repository's `main` and the `2025-baseline` tag both resolve to `91a717517408b4d933c7fb2530680ced4f84c449`.
- The new checkout was verified clean before this project plan was added.

## Stage 1: Prune the carried-forward code
- Remove unneeded 2025 OpModes, paths, examples, tuning programs, and season-specific files before dependency work. Commit that cleanup separately so deleted code remains recoverable from `2025-baseline`.
- Remove every `FateComponent` import/registration that remains after pruning, then remove `dev.nextftc.extensions:fateweaver:1.0.0`. Current usage is limited to component registration in autonomous classes; the project does not call another Fateweaver API.
- Run the existing Gradle build after cleanup. Treat the resulting smaller tree as the code surface that subsequent FTC, Pedro, and NextFTC migrations must preserve.

## Stage 2: Make the official FTC layer easy to update
- Create `upgrade/ftc-11.2.1` from the cleaned baseline. Compare the complete official [FTC SDK v11.2.1](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v11.2.1) workspace, not only Maven coordinates.
- Update FTC artifacts in [build.dependencies.gradle](../../build.dependencies.gradle) from `11.0.0` to `11.2.1`, Gradle wrapper in [gradle/wrapper/gradle-wrapper.properties](../../gradle/wrapper/gradle-wrapper.properties) from 8.9 to 9.1, and Android Gradle Plugin structure/version in [build.gradle](../../build.gradle) from 8.7.0 to 8.13.2. Reconcile official module/build-script changes in [build.common.gradle](../../build.common.gradle), [FtcRobotController/build.gradle](../../FtcRobotController/build.gradle), and manifests while retaining `compileSdk 34` in both Android modules for Pedro and Panels compatibility.
- Require JDK 17 and Android Studio Narwhal 3 Feature Drop or newer. Run Gradle with `--warning-mode all`; resolve project-owned Gradle 9 errors while recording harmless third-party warnings rather than masking them.
- Restore [build.dependencies.gradle](../../build.dependencies.gradle) to FTC-owned dependencies only. Move Pedro, Panels, and NextFTC declarations/repositories into [TeamCode/build.gradle](../../TeamCode/build.gradle), or a TeamCode-only applied Gradle file, so the kickoff SDK can be merged without repeatedly disentangling vendor libraries.

## Stage 3: Upgrade low-risk libraries independently
- On separate commits or branches, update Pedro Pathing `2.1.1 -> 2.1.2`, retaining telemetry `1.0.0`. This is a patch release containing follower and heading fixes and is the current [official Pedro release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v2.1.2).
- Keep Panels at `com.bylazar:fullpanels:1.0.12`; it is already the current version shown by the [Panels site](https://panels.bylazar.com/). Verify dependency resolution and dashboard startup instead of making a no-op version change.
- Keep NextFTC v1 at `1.1.0` during this baseline stage. Confirm `bindings:1.0.1` and Pedro extension `1.0.0` still resolve and compile against FTC 11.2.1; Fateweaver will already be gone.
- Use Gradle dependency insight to determine whether the directly declared NextFTC `hardware` and `bindings` artifacts can be removed; neither is imported by current TeamCode. Do not add or upgrade Road Runner because this project does not use it.
- After each dependency commit, run `gradlew.bat :TeamCode:assembleDebug --warning-mode all`. Then install the APK on a Control Hub or phone and smoke-test OpMode discovery, Panels connection, bulk caching, Pedro localization/follower initialization, and one harmless subsystem command.

## Stage 4: Add compile-only build protection
- Extend [.gitignore](../../.gitignore) for Python `__pycache__`, `*.pyc`, and script temp directories so generated artifacts do not recur.
- Add a small GitHub Actions build using JDK 17 and `:TeamCode:assembleDebug --warning-mode all`. Its purpose is limited to dependency resolution, duplicate-class detection, Gradle compatibility, and Java compilation; do not add an Android emulator or claim it validates robot execution.
- Keep Control Hub installation and hardware smoke tests as the release gate. Add desktop unit tests later only for logic that is genuinely hardware-independent, such as extracted coordinate or math calculations; do not mock the entire FTC runtime.
- Merge the verified FTC 11.2.1/Pedro/Panels baseline into `main` and tag it `2026-pre-kickoff`.

## Stage 5: Handle kickoff SDK separately
- After kickoff, re-check FIRST's releases rather than assuming the version will be `12.0`. Create `upgrade/ftc-kickoff-sdk`, diff the full official release against the pre-kickoff tag, and update wrapper, AGP, build scripts, manifests, artifacts, compile SDK, and any API changes as one isolated change.
- Re-run the same compile and hardware smoke tests before merging. This avoids mixing an unreleased FTC SDK transition with the NextFTC rewrite.

## Stage 6: Migrate active code to NextFTC v2 in isolation
- Create `migration/nextftc-v2` only after the kickoff SDK branch is stable. Re-check the latest release; as of September 4, 2026, the new suite is [v0.2.0](https://github.com/NextFTC/NextFTCSuite/releases/tag/v0.2.0).
- Remove all v1 artifacts before adding `dev.nextftc.v2:control`, `hardware`, and `robot`; v1 and v2 cannot coexist because they expose duplicate classes. Add the Dairy repository required by the [v2 installation guide](https://nextftc.dev/introduction/installation/).
- Treat this as an API redesign, not a version bump: the official v1-to-v2 migration guide is still “coming soon.” Use the v2 examples and source as the authority and keep every conversion in the isolated branch.
- Prove the new architecture with one minimal vertical slice first: robot/OpMode registration, one `Mechanism`, one command, gamepad trigger, hardware lookup, and Pedro/Ivy scheduling. Then port only code selected for the 2026 robot.
- Replace v1 concepts (`NextFTCOpMode`, `Subsystem`, `SubsystemComponent`, `ActiveOpMode`, command groups, bindings) with v2 `NextRobot`/`NextOpMode`, `Mechanism`, Ivy scheduler, and v2 hardware access. Replace the old Pedro extension usages explicitly; do not expect its v1 adapter to work with v2.
- Start active-code validation with [Constants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java) and a small OpMode before touching [Pickles2025Teleop.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Pickles2025Teleop.java). Its Limelight/PoseTracker fusion and direct Pedro lifecycle access are the highest-risk integration surface.
- Leave any additional unported code out of `TeamCode/src/main`; its complete executable version remains available at the `2025-baseline` tag. Merge v2 only when the selected active OpModes compile and pass on-robot lifecycle, emergency-stop, gamepad, subsystem, and autonomous-path smoke tests.

## Acceptance checks
- New public repository has default branch `main`, baseline tag/history, and no generated artifacts.
- Unneeded 2025 code and Fateweaver are removed before any dependency migration, with cleanup preserved as its own commit.
- Pre-kickoff `main` builds with FTC 11.2.1, Pedro 2.1.2, Panels 1.0.12, and NextFTC v1.1.0.
- Kickoff SDK and NextFTC v2 are separate reviewable changes with a green compile-only build and Control Hub validation at each boundary.
- Dependency declarations are separated so future FTC SDK updates can be applied from the official workspace with minimal conflicts.
- Pedro path syntax changes are checked against [scripts/export_pedro_paths.py](../../scripts/export_pedro_paths.py), and active teleop validation covers Limelight-to-Pedro pose fusion rather than only APK compilation.

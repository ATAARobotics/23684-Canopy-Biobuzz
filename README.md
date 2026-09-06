# Team 23684 Tech Titans

[![Build](https://github.com/ATAARobotics/23684-Biobuzz/actions/workflows/ci.yml/badge.svg)](https://github.com/ATAARobotics/23684-Biobuzz/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![FIRST Tech Challenge](https://img.shields.io/badge/FTC-2026--2027-blue)](https://www.firstinspires.org/robotics/ftc)
[![Team: 23684](https://img.shields.io/badge/Team-23684-blueviolet)](https://www.firstinspires.org/)

Robot code and tooling for **FIRST Tech Challenge Team 23684 (Tech Titans)**, competing in the **2026-2027 Biobuzz season**.

This repository is a hard fork of the official
[FTC Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FTCRobotController),
maintained as our team's working codebase for the season.

## Quick Start

```sh
# Clone
git clone https://github.com/ATAARobotics/23684-Biobuzz.git
cd 23684-Biobuzz

# Format
dprint fmt

# Build (also runs Android Lint)
./gradlew assembleDebug

# Lint only
./gradlew lintDebug
```

Requires **Android Studio Ladybug (2024.2)** or later and **JDK 17**.

### Installing dprint

```sh
curl -fsSL https://dprint.dev/install.sh | sh
```

Or via package manager — see [dprint.dev/install](https://dprint.dev/install/).

## Repository Layout

| Path | Purpose |
|---|---|
| `TeamCode/` | Our OpModes, subsystems, and team-specific code |
| `FtcRobotController/` | The unmodified FTC SDK app shell |
| `scripts/` | Git hooks installed by Gradle |
| `dprint.jsonc` | Java formatter config |
| `.github/workflows/` | CI (advisory checks only) |
| `AGENTS.md` | Conventions for AI-assisted work |
| `LICENSE`, `SECURITY.md` | License and security policy |

## Workflow

- Branch off `main` for each change.
- All work goes through a pull request — direct pushes to `main` are blocked.
- CI runs build, format, and lint on every PR — all **advisory**, none block merge.
- Force pushes are allowed on `main` for emergency fixes at competitions.
- Before opening a PR, test on the robot and check off the
  [PR template checklist](.github/pull_request_template.md).

## Syncing with upstream SDK

```sh
git fetch upstream
git merge upstream/main
```

Resolve conflicts in `FtcRobotController/` (rare) and `build.*.gradle` (rare).

## Contributing

This is a private-team repo. Internal members commit directly; outside
contributors should open an issue first to discuss changes.

## License

Our team's contributions are released under the [MIT License](LICENSE) — Copyright (c) 2026-2027 Team 23684 | Tech Titans.

The inherited FTC SDK code (under `FtcRobotController/` and the original
Gradle/SDK plumbing) remains BSD-3-Clause licensed by FIRST; see
[LICENSE.SDK](LICENSE.SDK).
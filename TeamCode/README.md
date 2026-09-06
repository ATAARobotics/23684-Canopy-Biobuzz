# TeamCode

Subsystem map and conventions for team OpModes.

## Layout

```
TeamCode/
└── src/main/java/org/firstinspires/ftc/teamcode/
    ├── subsystems/        # one file per mechanism (drive, intake, shooter, ...)
    ├── opmode/
    │   ├── teleop/        # driver-controlled
    │   ├── auto/          # autonomous routines
    │   └── test/          # bench/diagnostic OpModes
    └── util/              # constants, math helpers, hardware mappings
```

## Conventions

- One subsystem per file, matching the class name.
- Hardware map (motor/servo/port names) lives in a single `Constants` class.
- Autonomous routines live in `opmode/auto/`; one file per match strategy.
- Match logs and tuning numbers live with the OpMode that produced them.

## Adding a new OpMode

1. Pick the right folder (`teleop/`, `auto/`, or `test/`).
2. Extend `LinearOpMode` or `OpMode` (be consistent with what's already there).
3. Add `@TeleOp` or `@Autonomous` with a descriptive `name` and a sensible
   `group` so it shows up organized on the Driver Station.
4. Test on the robot before opening a PR.

## Reference samples

The official FTC sample OpModes are in
`FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`.
Use them as a starting point, then move your customized version here.
# Team 2611's 2027 robot

The Jacktown Vectors' robot program for the 2027 season, the first on Systemcore. It's made from
the [FRC robot template](https://github.com/mikegrundvig/frc-robot-template): WPILib 2027,
Commands v3, AdvantageKit for logging and replay, and Phoenix 6. The robot program every robot
shares, the [runtime](https://github.com/mikegrundvig/frc-robot-runtime), comes in as a library at
one version, so this repository holds only what's the team's: which robots it has, and what they
do.

> **TODO(team): the driving robot's values are placeholders.** `DrivingRobot.java`'s block of
> values to check (CAN IDs, the CANivore's name, the four module zeros, the drive inversions, the
> gear ratio, the steering motor) is still the template's, copied from a 2026 robot. The team has
> the real ones from its bench session and brings them over. Until then, the program won't drive
> the real robot correctly.

## The robots

| Robot | What it is | Run it |
|---|---|---|
| `driving` | The swerve drive alone: four modules, a Pigeon 2, a CANivore. What a deploy runs. | `./gradlew run` (simulator), `./gradlew run -PhwSim` (the real robot, from a laptop) |
| `vision-bench` | The same robot with the two vision coprocessors and their ThriftyCams on the bench, for proving the vision side before the cameras are mounted. Deleted once they are. | `./gradlew run -PhwSim -Probot=vision-bench` |

They're in `src/main/java/frc/robot/`: `Robots.java` lists them and says which one runs,
`DrivingRobot.java` and `VisionBenchRobot.java` describe them, and `Controls.java` is what the
robot does, its commands, buttons and autos. It's empty so far: the driver's sticks drive, and the
back button resets the heading.

## Quick start

You need git and any Java installation, to start the Gradle wrapper. Gradle downloads the exact
Java the robot code needs, so no WPILib installer is needed to build, test or simulate.

```bash
./gradlew ci                        # every check CI runs; green before you push
./gradlew test -Pquick --continuous # rerun the tests on every save while you work
./gradlew run                       # the simulator, driven from Pitwall (http://localhost:5820/#sim)
./gradlew run -PhwSim               # the real robot's CTRE devices, driven from this laptop
./gradlew deploy                    # to Systemcore, after every check passes
```

The team number, 2611, is already stored (`.wpilib/wpilib_preferences.json`).

## Driving the real robot from a laptop

Before there's a Systemcore, a laptop runs the robot program itself: the CANivore's USB cable goes
into the laptop, which drives the motors over CAN, using CTRE's hardware-attached mode. Use a
Windows laptop: on Linux it needs CTRE's kernel module. The runtime's
[bench guide](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/bench.md) has the
steps and the safety checklist. Read the checklist before anything moves.

The vision coprocessors join over a network switch, with the laptop at `10.26.11.2`, Systemcore's
own address: they treat it as the robot. Their images are in
[robot-images](https://github.com/2611-Vectors/robot-images), `vision-1` at `10.26.11.11` and
`vision-2` at `10.26.11.12`.

## Where to read more

- The runtime's guides: [mechanisms](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/mechanisms.md),
  [driving](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/driving.md),
  [autos](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/autos.md),
  [values](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/values.md) (every
  configuration value, and how to measure yours),
  [calibration](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/calibration.md),
  [testing](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/testing.md), and
  [Visor](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/visor.md).
- The template's [first power-on](https://github.com/mikegrundvig/frc-robot-template/blob/reference/docs/first-power-on.md)
  (a Systemcore out of the box), [editors](https://github.com/mikegrundvig/frc-robot-template/blob/reference/docs/editors.md),
  [upgrading](https://github.com/mikegrundvig/frc-robot-template/blob/reference/docs/upgrading.md)
  and [kickoff](https://github.com/mikegrundvig/frc-robot-template/blob/reference/docs/kickoff.md)
  (a new season's field).
- [AGENTS.md](AGENTS.md): what's different from older FRC code, for anyone (or any coding
  assistant) used to WPILib 2025 and Commands v2.

## Licenses

BSD 3-Clause, the template's ([LICENSE](LICENSE)). The libraries' own licenses are in
[licenses/](licenses).

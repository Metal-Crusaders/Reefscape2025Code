# FRC Robot 2025 — Reefscape

A command-based Java robot project for the 2025 FRC game *Reefscape*, built on WPILib and CTRE Phoenix 6. The robot is a swerve drive bot capable of scoring coral at L1/L2/L3 reef levels, collecting and processing algae, and executing multi-step autonomous routines.

---

## Hardware

| Component | Details |
|-----------|---------|
| Drivetrain | CTRE Swerve (4x Kraken X60 drive, 4x TalonFX steer, CANcoder encoders, Pigeon 2 IMU) |
| CAN Bus | Canivore named `canivore` |
| Elevator | 2x REV SparkMax (IDs 17/18), brushless, position PID + trapezoid profile |
| Coral Shooter | 2x REV SparkMax (IDs 13/14), beam-break sensor on DIO 3/4 |
| Algae Claw | REV SparkMax (ID 15), REV Color Sensor V3 for proximity detection |
| Algae Pivot | REV SparkMax (ID 16), position PID + trapezoid profile |
| Cameras | 2x PhotonVision cameras for AprilTag pose estimation |
| Operator Boards | 2x custom APac joystick boards (ports 4 and 5) |
| Driver Controller | Xbox controller (port 0) |
| Operator Controller | Xbox controller (port 1) |

---

## Software Dependencies

- WPILib 2025
- CTRE Phoenix 6
- REV SparkMax / REVLib
- PathPlanner (path following + pathfinding)
- PhotonVision
- OpenCV (reef display via CameraServer)

---

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                     # Entry point
├── Robot.java                    # TimedRobot lifecycle
├── RobotContainer.java           # Subsystems, commands, bindings, auto selector
│
├── commands/
│   ├── auto/
│   │   ├── center/               # Center-start autonomous routines
│   │   ├── openSide/             # Open-side autonomous routines
│   │   └── processorSide/        # Processor-side autonomous routines
│   ├── coroutines/               # Multi-subsystem coordinated command groups
│   │   ├── extradriver/          # Variants that give driver joystick control mid-sequence
│   │   └── nodriver/             # Fully automated variants
│   ├── elevator/                 # Elevator preset and teleop commands
│   ├── scoring/
│   │   ├── algae/                # Algae claw/pivot commands
│   │   └── coral/                # Coral intake/score commands
│   ├── swerve/                   # Drivetrain commands (teleop, auto drive, pathfinding)
│   └── utils/                    # Helper commands (alliance conditional, display, etc.)
│
├── constants/
│   ├── Constants.java            # All tuning constants, poses, PID gains
│   ├── MathUtils.java            # Geometry helpers (closest pose, path edge projection)
│   └── TunerConstants.java       # CTRE-generated swerve hardware constants
│
├── oi/
│   ├── CustomAPacOI.java         # Operator board button mapping
│   └── MyButton.java             # Extended JoystickButton
│
└── subsystems/
    ├── camera/AprilTagCamera.java
    ├── climber/Climber.java       # Stub (not yet implemented)
    ├── elevator/Elevator.java
    ├── scoring/
    │   ├── AlgaeClaw.java
    │   ├── AlgaePivot.java
    │   └── CoralShooter.java
    ├── swerve/
    │   ├── CommandSwerveDrivetrain.java
    │   └── SwerveTelemetry.java
    └── util/
        ├── KarthikADStarPathing.java  # Custom AD* pathfinder (runs in background thread)
        ├── ReefAlignmentPID.java
        ├── ReefDisplay.java           # Streams pose images over CameraServer
        └── ReefCommandManager.java

src/main/deploy/pathplanner/
├── paths/                         # All PathPlanner .path files
├── autos/                         # Auto .auto files
├── navgrid.json                   # Obstacle grid for pathfinding
└── settings.json                  # Robot dimensions and default constraints
```

---

## Elevator Positions

| Position | Encoder Ticks |
|----------|--------------|
| Rest (L1) | 0 |
| Processor/Algae hold | 10 |
| L2 Coral | 14.8 |
| L3 Coral | 43.8 |
| Low Algae | 40.8 |
| High Algae | 68.8 |
| Max | 69 |

---

## Driver Controls (Xbox — Port 0)

| Input | Action |
|-------|--------|
| Left stick | Translate (field-centric) |
| Right stick X | Rotate |
| Y button (hold) | Snap heading to nearest reef face |
| X button (hold) | Snap heading to nearest coral station |
| B button (hold) | Snap heading to processor (-90° blue / +90° red) |
| A button (hold) | Point toward reef center |
| Start | Reset gyro heading to 0° |
| Back | Auto-drive to processor |

---

## Operator Board Controls (APac — Ports 4/5)

| Button | Action |
|--------|--------|
| L3 Left / L3 Right | Score coral at L3 (left/right branch) |
| L2 Left / L2 Right | Score coral at L2 (left/right branch) |
| L1 | Score coral at L1 |
| Intake | Intake coral from station |
| Algae High | High algae grab (coral-first sequence) |
| Algae Low | Low algae grab sequence |
| Algae Process | Process algae at processor |
| Rest Mode | Return all mechanisms to rest |
| Coral Shake | Nudge elevator to dislodge stuck coral |
| Close Center / Left / Right | Drive to close reef face (center/left/right) |
| Far Center / Left / Right | Drive to far reef face |
| Reef Drive Rest | Cancel active reef auto-drive |

All operator board actions include a 50ms debounce. Scoring actions also require coral to be present (beam break check) before executing.

---

## Autonomous Routines

| Name | Starting Position | Description |
|------|------------------|-------------|
| No Auto | Any | Resets odometry only |
| L3 + Double Process Algae | Center | Scores L3, collects and processes low + high algae twice |
| Two L3 Process Algae | Processor side | Two L3 coral scores, collects algae, processes at processor |
| Two L3 + L2 | Processor side | Two L3 scores, one L2 score, algae process |
| Two L3 + Process Algae + Buddy Taxi | Processor side | Two L3 scores, algae process, buddy bump |
| Two L3 + L2 | Open side | Same as processor-side variant on open side |
| Two L3 + Buddy Taxi | Open side | Two L3 scores, algae process, buddy bump on open side |

### Alliance Flipping
All paths are authored for Blue alliance. PathPlanner automatically flips them for Red alliance based on `DriverStation.getAlliance()`. Odometry is seeded from the first path's starting holonomic pose at auto init.

---

## Pathfinding

The robot uses a custom **AD\* pathfinder** (`KarthikADStarPathing`) that runs on a background thread and reads obstacle data from `navgrid.json`. This is used for long-range `driveToPose` calls during teleop (processor auto-drive) and some auto segments.

PathPlanner's `FollowPathCommand` with a `PPHolonomicDriveController` is used for pre-planned path segments. For close-range alignment (reef approach, final scoring position), a custom PID-based `CloseDriveToPose` command is used.

---

## Vision

Two PhotonVision cameras (`testCam` and `testCam2`) provide multi-tag AprilTag pose estimates. Measurements are added to the drivetrain via `addVisionMeasurement`. Each camera has a toggle in SmartDashboard. Only reef-side AprilTag IDs (6–11 for Red, 17–22 for Blue) are accepted.

---

## Key Constants (tuning reference)

| Constant | Value |
|----------|-------|
| Max drive speed | 2.5 m/s |
| PathPlanner translation PID | kP=4.5 |
| PathPlanner rotation PID | kP=2.5 |
| Close drive translation PID | kP=1.5 |
| Close drive rotation PID | kP=2.0 |
| Algae proximity threshold | 150 (color sensor units) |
| Elevator deadband | ±5 ticks |
| Algae pivot deadband | ±50 ticks |

---

## SmartDashboard / Telemetry

- **Auto Selector** — dropdown to choose autonomous routine
- **Command Scheduler Visualized** — live command scheduler state
- **BEAM EXISTS CORAL SHOOTER** — coral beam-break state
- **DriveState/** (NetworkTables) — full swerve state (pose, speeds, module states)
- **Pose/robotPose** — Field2d robot pose
- **Module 0–3** — Mechanism2d swerve module visualizations
- **AprilTag Detected** / **Vision Pose** — vision pipeline output
- Per-camera enabled toggles

---

## Building and Deploying

```bash
# Build
./gradlew build

# Deploy to robot (requires network connection to robot)
./gradlew deploy

# Simulate
./gradlew simulateJava
```

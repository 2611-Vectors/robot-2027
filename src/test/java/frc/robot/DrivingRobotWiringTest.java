package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.PhoenixDrivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation3d;

/**
 * The driving robot's drivetrain built as the real robot builds it, through the runtime's CTRE
 * drivetrain ({@link PhoenixDrivetrain}), on Phoenix's own simulated devices at the robot's CAN
 * IDs: each corner's module reads its own devices, and each device holds the zero and the direction
 * the block at the top of {@link DrivingRobot} gives that corner. So a module listed in the wrong
 * order, an ID typed into the wrong module, or a left side's inversion given to a right module
 * fails here, not on the bench.
 *
 * <p>What it can't say is whether the block's values are the chassis's own: that's the Tuner X
 * record's ({@link DrivingRobotRecordTest}).
 */
@ExtendWith(RobotSim.class)
class DrivingRobotWiringTest {
  /**
   * How near the wheel's reading must be, in radians. Phoenix's simulated position reads a little
   * short (about 0.002 rad in two turns); a wrong direction, gearing or module is turns out.
   */
  private static final double WHEEL_RADIANS = 0.01;

  /** How near a module's angle must be, in rotations: another module's is 0.02 away. */
  private static final double ANGLE_ROTATIONS = 1e-3;

  /** How long a read of a device's settings waits for them, in seconds. */
  private static final double SETTINGS_WAIT_SECONDS = 1;

  private static final CANBus BUS = new CANBus(DrivingRobot.CANIVORE_NAME);

  /** One corner, as the block names its values: its devices' IDs, its zero, its drive's side. */
  private record Corner(
      String name,
      int drive,
      int steer,
      int cancoder,
      double zeroRotations,
      boolean driveInverted) {}

  /** The four corners, in the order the drivetrain lists its modules. */
  private static final List<Corner> CORNERS =
      List.of(
          new Corner(
              "front left",
              DrivingRobot.FRONT_LEFT_DRIVE_ID,
              DrivingRobot.FRONT_LEFT_STEER_ID,
              DrivingRobot.FRONT_LEFT_CANCODER_ID,
              DrivingRobot.FRONT_LEFT_ZERO.in(Rotations),
              DrivingRobot.LEFT_DRIVES_INVERTED),
          new Corner(
              "front right",
              DrivingRobot.FRONT_RIGHT_DRIVE_ID,
              DrivingRobot.FRONT_RIGHT_STEER_ID,
              DrivingRobot.FRONT_RIGHT_CANCODER_ID,
              DrivingRobot.FRONT_RIGHT_ZERO.in(Rotations),
              DrivingRobot.RIGHT_DRIVES_INVERTED),
          new Corner(
              "back left",
              DrivingRobot.BACK_LEFT_DRIVE_ID,
              DrivingRobot.BACK_LEFT_STEER_ID,
              DrivingRobot.BACK_LEFT_CANCODER_ID,
              DrivingRobot.BACK_LEFT_ZERO.in(Rotations),
              DrivingRobot.LEFT_DRIVES_INVERTED),
          new Corner(
              "back right",
              DrivingRobot.BACK_RIGHT_DRIVE_ID,
              DrivingRobot.BACK_RIGHT_STEER_ID,
              DrivingRobot.BACK_RIGHT_CANCODER_ID,
              DrivingRobot.BACK_RIGHT_ZERO.in(Rotations),
              DrivingRobot.RIGHT_DRIVES_INVERTED));

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /** Leaves every motor neutral: Phoenix's simulated devices outlive a test. */
  @AfterEach
  void leaveTheMotorsNeutral() {
    for (Corner corner : CORNERS) {
      new TalonFX(corner.drive(), BUS).setControl(new NeutralOut());
      new TalonFX(corner.steer(), BUS).setControl(new NeutralOut());
    }
  }

  /**
   * Each corner's devices hold that corner's settings: the CANcoder its zero and direction, the
   * drive its side's direction, the steering its direction and that corner's CANcoder. Then each
   * corner's drive is turned a different number of wheel turns and each CANcoder a different angle,
   * so a module reading another corner's devices reads another corner's numbers.
   */
  @Test
  void eachCornersModuleReadsItsOwnDevicesWithItsOwnZeroAndDirections()
      throws InterruptedException {
    double driveGearing = DrivingRobot.MODULE.driveGearing().ratio();
    double coupling = DrivingRobot.MODULE.couplingRatio();
    for (Corner corner : CORNERS) {
      new TalonFX(corner.drive(), BUS).getSimState().setRawRotorPosition(0);
      CANcoder cancoder = new CANcoder(corner.cancoder(), BUS);
      cancoder.getSimState().setRawPosition(0);
      cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
    }
    new Pigeon2(DrivingRobot.PIGEON_ID, BUS).getSimState().setRawYaw(0);

    SwerveWiring wiring = DrivingRobot.DRIVETRAIN.wiring().orElseThrow();
    DrivetrainControl control = DrivingRobot.CONFIG.drivetrain();
    try (PhoenixDrivetrain drivetrain =
        new PhoenixDrivetrain(wiring, DrivingRobot.DRIVETRAIN, Rotation3d.ZERO)) {
      ModuleIO[] modules = drivetrain.modules();
      assertThat(modules).hasSameSizeAs(CORNERS);
      // The drivetrain sends each module its gains and limits, and with them its configuration.
      for (ModuleIO module : modules) {
        module.configure(control.gains(), control.limits());
      }

      for (int i = 0; i < CORNERS.size(); i++) {
        Corner corner = CORNERS.get(i);
        holdsItsSettings(corner);

        // A different number of wheel turns, and a different angle, at each corner.
        double wheelTurns = i + 1;
        double magnetRotations = 0.02 * (i + 1);
        new TalonFX(corner.drive(), BUS)
            .getSimState()
            .setRawRotorPosition(wheelTurns * driveGearing);
        new CANcoder(corner.cancoder(), BUS).getSimState().setRawPosition(magnetRotations);

        double angle =
            (DrivingRobot.CANCODERS_INVERTED ? -magnetRotations : magnetRotations)
                + corner.zeroRotations();
        double driveReading = corner.driveInverted() ? -wheelTurns : wheelTurns;
        // The coupling takes the module's turn out of the wheel's reading.
        double wheel = 2 * Math.PI * (driveReading - coupling * angle / driveGearing);
        ModuleIO module = modules[i];
        ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
        boolean read =
            eventually(
                () -> readUnderTheLock(drivetrain, () -> module.updateInputs(inputs)),
                () ->
                    Math.abs(inputs.steerAngle.getRotations() - angle) < ANGLE_ROTATIONS
                        && Math.abs(inputs.drivePositionRad - wheel) < WHEEL_RADIANS);
        assertThat(read)
            .as(
                "module %d, the %s, reads %s of a turn and %s rad; its own devices say %s and %s",
                i,
                corner.name(),
                inputs.steerAngle.getRotations(),
                inputs.drivePositionRad,
                angle,
                wheel)
            .isTrue();
        assertThat(inputs.unconfigured)
            .as("the %s's devices took their settings", corner.name())
            .isEmpty();
      }

      // The Pigeon 2 at its own ID, read at its true size until its scale is checked.
      new Pigeon2(DrivingRobot.PIGEON_ID, BUS).getSimState().setRawYaw(30);
      GyroIO.GyroIOInputs gyro = new GyroIO.GyroIOInputs();
      boolean turned =
          eventually(
              () -> readUnderTheLock(drivetrain, () -> drivetrain.gyro().updateInputs(gyro)),
              () -> Math.abs(gyro.yaw.getDegrees() - 30) < 0.05);
      assertThat(turned).as("the Pigeon 2 turned 30°, read %s", gyro.yaw.getDegrees()).isTrue();
    }
  }

  /** What one corner's three devices hold, read back from each. */
  private static void holdsItsSettings(Corner corner) throws InterruptedException {
    MagnetSensorConfigs magnet = new MagnetSensorConfigs();
    CANcoder cancoder = new CANcoder(corner.cancoder(), BUS);
    readBack(() -> cancoder.getConfigurator().refresh(magnet, SETTINGS_WAIT_SECONDS));
    assertThat(magnet.MagnetOffset)
        .as("the %s's zero", corner.name())
        .isCloseTo(corner.zeroRotations(), within(1e-3));
    assertThat(magnet.SensorDirection)
        .as("the %s's CANcoder's direction", corner.name())
        .isEqualTo(
            DrivingRobot.CANCODERS_INVERTED
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive);

    MotorOutputConfigs driveOutput = new MotorOutputConfigs();
    TalonFX drive = new TalonFX(corner.drive(), BUS);
    readBack(() -> drive.getConfigurator().refresh(driveOutput, SETTINGS_WAIT_SECONDS));
    assertThat(driveOutput.Inverted)
        .as("the %s's drive's direction", corner.name())
        .isEqualTo(direction(corner.driveInverted()));

    MotorOutputConfigs steerOutput = new MotorOutputConfigs();
    FeedbackConfigs steerFeedback = new FeedbackConfigs();
    TalonFX steer = new TalonFX(corner.steer(), BUS);
    readBack(() -> steer.getConfigurator().refresh(steerOutput, SETTINGS_WAIT_SECONDS));
    readBack(() -> steer.getConfigurator().refresh(steerFeedback, SETTINGS_WAIT_SECONDS));
    assertThat(steerOutput.Inverted)
        .as("the %s's steering's direction", corner.name())
        .isEqualTo(direction(DrivingRobot.STEERING_INVERTED));
    assertThat(steerFeedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RemoteCANcoder);
    assertThat(steerFeedback.FeedbackRemoteSensorID)
        .as("the %s's steering reads its own CANcoder", corner.name())
        .isEqualTo(corner.cancoder());
  }

  private static InvertedValue direction(boolean inverted) {
    return inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  }

  /** Reads some of a device's settings back from it, asking until it answers. */
  private static void readBack(Supplier<StatusCode> read) throws InterruptedException {
    assertThat(eventually(() -> {}, () -> read.get().isOK())).as("read back").isTrue();
  }

  /** Waits up to two seconds, reading again every few milliseconds, for something to be true. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  /** Reads under the drivetrain's lock, as the robot's loop does. */
  private static void readUnderTheLock(PhoenixDrivetrain drivetrain, Runnable read) {
    drivetrain.odometryLock().lock();
    try {
      read.run();
    } finally {
      drivetrain.odometryLock().unlock();
    }
  }
}

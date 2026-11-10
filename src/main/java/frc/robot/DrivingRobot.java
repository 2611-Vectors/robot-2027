package frc.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SwerveWiring.PIGEON_2;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;

/**
 * The driving robot: the team's swerve drive, a square frame on four swerve modules with nothing
 * else on it. Each module has two Kraken motors, run by the Talon FX built into each one: one
 * drives the wheel and one steers it. A CANcoder on top of each module reads which way its wheel
 * points. A Pigeon 2 tells the robot which way it faces. All of them are on one CANivore.
 *
 * <p>TODO(team): THIS IS A PLACEHOLDER. Every value in the block below is the template's, copied
 * from a 2026 robot's code, not this robot's. The team has the real ones from its bench session
 * (CAN IDs, the CANivore's name, the four zeros, the motors' and CANcoders' directions, the gear
 * ratio, the steering motor) and brings them over here. Until then this program won't drive the
 * real robot correctly: its wheels will point the wrong way. Remove this TODO once every value in
 * the block says "Checked".
 *
 * <p>Run it in the simulator: {@code ./gradlew run}. Drive the real robot from a laptop, with its
 * CANivore plugged into the laptop: {@code ./gradlew run -PhwSim} (runtime/docs/bench.md says how,
 * and what to check first).
 *
 * <p>Read it from the top. First come the values to check on the real robot, all in one block. Then
 * what the robot is: its frame and its modules. Last, how the program drives it, with current
 * limits kept low for the bench.
 */
public final class DrivingRobot {
  private DrivingRobot() {}

  // ==== CHECK THESE ON THE CHASSIS ===============================================================
  //
  // TODO(team): PLACEHOLDER VALUES. Replace every value marked "Unconfirmed" with the driving
  // robot's own, from the team's bench session, before driving it.
  //
  // Every value to check is copied from the 2026 robot's code, not read off this robot, so each one
  // is marked "Unconfirmed". This block holds the CANivore, the gear ratio and the steering motor;
  // the CAN IDs, zeros and directions are marked where the drivetrain says them, on the line above
  // each corner. Check each in Phoenix Tuner X: its device list shows every device's CAN ID and the
  // CANivore it's on. If a value differs, change it. Once a value is checked, change its
  // "Unconfirmed" to "Checked" and the date. A deploy refuses while any value is still
  // "Unconfirmed:" (./gradlew unconfirmedValues lists them); the simulator and ci don't mind.
  //
  // Write what the session read into src/test/resources/driving-tuner-x.json as well, the record
  // DrivingRobotRecordTest compares every ID, zero and direction against (that test says how the
  // record looks). DrivingRobotWiringTest checks each module is built from its own corner's
  // values.

  /** Unconfirmed: the CANivore's name, as Tuner X shows it. */
  public static final CanBus CANIVORE = CanBus.canivore("canivore");

  // Unconfirmed: the drive gear ratio, motor turns for one turn of the wheel. An MK5n's ratio is
  // set by the pinion gear on its drive motor. This is the 2026 robot's, 5.27 to 1; the other 2026
  // robot's code has 6.03 to 1, a smaller pinion. Look at the pinion, or turn a wheel once by hand
  // and count the drive motor's turns in Tuner X (its rotor position).
  static final double DRIVE_GEAR_RATIO = 5.2734375;

  // Unconfirmed: the steering motor. SDS sells the MK5n for a Kraken X44 on its steering, as the
  // other 2026 robot's Tuner X project has it. An X44 is narrower than the X60 that drives the
  // wheel (44 mm across, against 60 mm), and its label says which it is.
  static final Motor STEER_MOTOR = Motor.KRAKEN_X44;

  // ==== END OF THE VALUES TO CHECK ===============================================================

  /**
   * One SDS MK5n module, all four the same. A Kraken X60 drives the wheel, without FOC: FOC needs a
   * Phoenix Pro license on the motor. The steering turns 287/11 (about 26.09) times for one turn of
   * the module, as SDS says. The wheels are 4 inches across, so 2 inches from the middle to the
   * tread.
   *
   * <p>Turning a module also turns its wheel a little, through the gears inside it: 3.375 turns of
   * the drive motor for each turn of the module, the 2026 robot's number for its 5.27 to 1 modules.
   * The program takes that out of the wheel's distance. Change it with the drive gear ratio: it's
   * the drive gear ratio divided by 1.5625 (25/16), so 3.857 for a 6.03 to 1 module.
   *
   * <p>How hard the wheel and the steering are to spin up, 0.01 kg m² each, is Tuner X's guess,
   * which only the simulator uses.
   */
  public static final DrivetrainGeometry.ModuleType MODULE =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(STEER_MOTOR)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.of(DRIVE_GEAR_RATIO))
          .steerGearing(Gearing.of(287.0 / 11.0))
          .couplingRatio(DRIVE_GEAR_RATIO / (25.0 / 16.0))
          .driveInertia(KilogramSquareMeters.of(0.01))
          .steerInertia(KilogramSquareMeters.of(0.01))
          .build();

  /**
   * How the chassis is driven on the bench, from the template's defaults, its limits low on purpose
   * while the chassis is on blocks. A wheel spinning in the air needs only a few amps, so 20 A
   * leaves plenty, and a jammed wheel or a wrong setting can't draw much. Raise them on purpose
   * before driving on the floor (the template's defaults are 30 A on the drive's stator and 40 A on
   * the steering's: {@code DrivetrainControl.defaults()}). The top speeds are low too: 1.5 meters a
   * second, and half a turn a second.
   *
   * <p>How hard the motors work to follow what they're asked. The drive's kV is left to the
   * modules: what the X60 and the gearing should need, worked out from their numbers. Its kS and kP
   * are the 2026 robot's Tuner X gains, changed into this program's units (per meter a second). The
   * steering's are the 2026 robot's too, Tuner X's stock, which are the defaults. Tune them on the
   * chassis, live from Pitwall's Tune view, once tuning is switched on there.
   */
  public static final DrivetrainControl BENCH =
      DrivetrainControl.defaults()
          .withDriveStatorLimit(Amps.of(20))
          .withDriveSupplyLimit(Amps.of(20))
          .withDriveSupplyLowerLimit(Amps.of(10))
          .withSteerStatorLimit(Amps.of(20))
          .withMaxSpeed(MetersPerSecond.of(1.5))
          .withMaxSpeed(RotationsPerSecond.of(0.5))
          .withDriveKs(0.14)
          .withDriveKp(0.3);

  /**
   * The four modules, 10.375 inches forward or back of the middle and 10.375 inches left or right,
   * and how they're wired, all on the CANivore: the Pigeon 2, then each corner's drive motor's
   * Talon FX, its steering motor's, and its CANcoder with its zero. The Pigeon reads turns at their
   * true size until the ten-spin check measures it (runtime/docs/calibration.md): {@code
   * PIGEON_2.can(CANIVORE, 1).yawScale(1.0012)}.
   *
   * <p>Check every ID in Tuner X. Measure each zero by "Zeroing a swerve module"
   * (runtime/docs/tuner-x.md). Directions, chassis on blocks: driving forward slowly, every wheel
   * rolls forward; asked for a small turn, every module settles; turned counterclockwise by hand,
   * seen from above, SteerAngleRotations grows. A device that fails its check gets {@code
   * .clockwise()}, or loses it.
   *
   * <p>It's driven as {@link #BENCH} says, gently, for the bench.
   */
  public static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.uniform(MODULE, Inches.of(10.375), Inches.of(10.375))
          // Unconfirmed: the Pigeon 2's CAN ID.
          .gyro(PIGEON_2.can(CANIVORE, 1))
          // Unconfirmed: the steering motors' and the CANcoders' directions (none clockwise).
          // Unconfirmed: front left's three IDs, its zero, and its drive's direction.
          .frontLeft(
              TALON_FX.can(CANIVORE, 23),
              TALON_FX.can(CANIVORE, 24),
              CANCODER.can(CANIVORE, 12).offset(Rotations.of(0.10693359375)))
          // Unconfirmed: front right's three IDs, its zero, and its drive's direction.
          .frontRight(
              TALON_FX.can(CANIVORE, 21).clockwise(),
              TALON_FX.can(CANIVORE, 22),
              CANCODER.can(CANIVORE, 11).offset(Rotations.of(-0.031494140625)))
          // Unconfirmed: back left's three IDs, its zero, and its drive's direction.
          .backLeft(
              TALON_FX.can(CANIVORE, 25),
              TALON_FX.can(CANIVORE, 26),
              CANCODER.can(CANIVORE, 13).offset(Rotations.of(-0.12939453125)))
          // Unconfirmed: back right's three IDs, its zero, and its drive's direction.
          .backRight(
              TALON_FX.can(CANIVORE, 27).clockwise(),
              TALON_FX.can(CANIVORE, 28),
              CANCODER.can(CANIVORE, 14).offset(Rotations.of(-0.38427734375)))
          .control(BENCH);

  /**
   * The chassis as the program runs it: {@code ./gradlew run -Probot=driving} chooses it by name
   * ({@link Robots#current}).
   *
   * <p>Its frame is about 26 inches square, from where the 2026 robot's code puts its modules,
   * 10.375 inches from the middle each way, with a module's wheel 2.625 inches in from the frame's
   * edge. Measure it and change it if it's different. It has no bumpers on the bench, so the
   * program takes the usual 3 inch ones, which only place it on the field and in the simulator, and
   * says so at startup.
   *
   * <p>It hasn't been weighed yet, so the weight is left out and the program takes it as heavy as
   * the season allows, and says so at startup. Weigh it with its battery on, and add what the scale
   * says: {@code .weight(Pounds.of(42.5))}. How hard it is to spin isn't measured either, so the
   * model works it out as a solid box's of that weight and size.
   *
   * <p>Its Pigeon 2 is the drivetrain's, taken as sitting in the middle, level, until its place is
   * measured: {@code PIGEON_2.can(CANIVORE, 1).at(mount)}. No backup IMU: that's Systemcore's own,
   * and the chassis has no Systemcore yet. Its cameras' and pose estimator's settings are the
   * template's own.
   */
  public static final RobotConfig CONFIG =
      RobotConfig.of(
          RobotDefinition.builder()
              .name("driving")
              .body(
                  Body.builder()
                      .frameLength(Inches.of(26))
                      .frameWidth(Inches.of(26))
                      .overallHeight(Inches.of(8))
                      .build())
              .drivetrain(DRIVETRAIN)
              .build());
}

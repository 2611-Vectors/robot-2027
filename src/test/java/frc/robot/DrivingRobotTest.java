package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The swerve test chassis ({@link DrivingRobot}): it passes the robot model's checks (building it
 * is the check, and {@link RobotsTest} builds every robot), and it's fit for the bench: a
 * drivetrain alone, with no Phoenix Pro feature, low limits, and every value still to be checked on
 * the real chassis marked where it's written. Its devices and their settings, corner by corner, are
 * {@link DrivingRobotWiringTest}'s; whether its values are the chassis's own is {@link
 * DrivingRobotRecordTest}'s.
 */
class DrivingRobotTest {
  /** The file a student opens: the test reads it as they do. */
  private static final Path SOURCE =
      Path.of(
          System.getProperty("frc.projectDir", "."), "src/main/java/frc/robot/DrivingRobot.java");

  @Test
  void itFitsTheSeasonAndHasOnlyADrivetrain() {
    assertThat(Rebuilt.LIMITS.problems(DrivingRobot.CONFIG.definition())).isEmpty();
    assertThat(DrivingRobot.CONFIG.definition().cameras()).isEmpty();
    assertThat(DrivingRobot.CONFIG.definition().joints()).isEmpty();
    assertThat(DrivingRobot.CONFIG.definition().pdh()).isEmpty();
    assertThat(DrivingRobot.CONFIG.definition().backupImu())
        .as("no Systemcore, so no backup IMU")
        .isEmpty();
    assertThat(Robots.asked(java.util.Optional.of("driving"))).isSameAs(DrivingRobot.CONFIG);
  }

  /**
   * Nothing that needs a Phoenix Pro license, which the chassis's motors don't have: no FOC. The
   * steering reads its CANcoder remotely, which the runtime always does (ModuleIOTalonFX).
   */
  @Test
  void itNeedsNoPhoenixPro() {
    assertThat(List.of(DrivingRobot.MODULE.driveMotor(), DrivingRobot.MODULE.steerMotor()))
        .containsExactly(Motor.KRAKEN_X60, DrivingRobot.STEER_MOTOR);
    Drivetrain drivetrain = DrivingRobot.CONFIG.definition().drivetrain();
    assertThat(List.of(drivetrain.driveCommutation(), drivetrain.steerCommutation()))
        .containsOnly(MechanismWiring.Commutation.TRAPEZOIDAL);
  }

  /**
   * Low limits for the bench, named so they're raised on purpose: no more than 20 A through any
   * motor, and slow.
   */
  @Test
  void itsLimitsAreTheBenchs() {
    DrivetrainControl.Limits limits = DrivingRobot.CONFIG.drivetrain().limits();
    assertThat(limits).isEqualTo(DrivingRobot.BENCH.limits());
    assertThat(limits.driveStatorCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.driveSupplyCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.steerStatorCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.maxLinearSpeed().orElseThrow().in(MetersPerSecond)).isLessThanOrEqualTo(1.5);
  }

  /**
   * A student finds every value to check marked where it's written: the CANivore, the gear ratio
   * and the steering motor in the one block, which holds nothing else but its values and their
   * comments, and the gyro's and each corner's IDs, zeros and directions on the lines above them in
   * the drivetrain.
   */
  @Test
  void everyUnconfirmedValueIsInTheBlockOrTheDrivetrain() throws IOException {
    List<String> lines = Files.readAllLines(SOURCE);
    int start = indexOf(lines, "// ==== CHECK THESE ON THE CHASSIS");
    int end = indexOf(lines, "// ==== END OF THE VALUES TO CHECK");
    int drivetrain = indexOf(lines, "public static final Drivetrain DRIVETRAIN =");
    int drivetrainEnd = endOfStatement(lines, drivetrain);

    assertThat(start).isPositive().isLessThan(end).isLessThan(drivetrain);
    int inTheDrivetrain = 0;
    for (int i = 0; i < lines.size(); i++) {
      if (!lines.get(i).contains("Unconfirmed")) {
        continue;
      }
      boolean inTheBlock = i > start && i < end;
      boolean onTheDrivetrain = i > drivetrain && i < drivetrainEnd;
      assertThat(inTheBlock || onTheDrivetrain)
          .as("line %d: %s", i + 1, lines.get(i).strip())
          .isTrue();
      if (onTheDrivetrain) {
        inTheDrivetrain++;
      }
    }
    assertThat(inTheDrivetrain).as("the gyro, the directions, and each corner").isEqualTo(6);
    for (String line : lines.subList(start + 1, end)) {
      String code = line.strip();
      assertThat(
              code.isEmpty()
                  || code.startsWith("//")
                  || code.startsWith("/**")
                  || code.startsWith("static final")
                  || code.startsWith("public static final"))
          .as(code)
          .isTrue();
    }
  }

  /** The line a statement that starts at a line ends on: the first after it ending with ";". */
  private static int endOfStatement(List<String> lines, int start) {
    for (int i = start; i < lines.size(); i++) {
      if (lines.get(i).strip().endsWith(";")) {
        return i;
      }
    }
    return lines.size();
  }

  private static int indexOf(List<String> lines, String marker) {
    for (int i = 0; i < lines.size(); i++) {
      if (lines.get(i).strip().startsWith(marker)) {
        return i;
      }
    }
    return -1;
  }
}

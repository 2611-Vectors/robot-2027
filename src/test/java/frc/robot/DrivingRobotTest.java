package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
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
 * the real chassis in its one block. Its devices and their settings, corner by corner, are {@link
 * DrivingRobotWiringTest}'s; whether the block's values are the chassis's own is {@link
 * DrivingRobotRecordTest}'s.
 */
class DrivingRobotTest {
  /** The file a student opens: the test reads it as they do. */
  private static final Path SOURCE =
      Path.of(
          System.getProperty("frc.projectDir", "."), "src/main/java/frc/robot/DrivingRobot.java");

  @Test
  void itFitsTheSeasonAndHasOnlyADrivetrain() {
    assertThat(Rebuilt.LIMITS.problems(DrivingRobot.ROBOT)).isEmpty();
    assertThat(DrivingRobot.ROBOT.cameras()).isEmpty();
    assertThat(DrivingRobot.ROBOT.joints()).isEmpty();
    assertThat(DrivingRobot.ROBOT.pdh()).isEmpty();
    assertThat(DrivingRobot.ROBOT.imus().backup()).as("no Systemcore, so no backup IMU").isEmpty();
    assertThat(Robots.asked(java.util.Optional.of("driving"))).isSameAs(DrivingRobot.CONFIG);
  }

  /**
   * Nothing that needs a Phoenix Pro license, which the chassis's motors don't have: no FOC. The
   * steering reads its CANcoder remotely, which the runtime always does (ModuleIOTalonFX).
   */
  @Test
  void itNeedsNoPhoenixPro() {
    assertThat(List.of(DrivingRobot.MODULE.driveMotor(), DrivingRobot.MODULE.steerMotor()))
        .containsExactly(Motor.KRAKEN_X60, DrivingRobot.STEER_MOTOR)
        .noneMatch(motor -> motor.name().contains("FOC"));
  }

  /**
   * Low limits for the bench, named so they're raised on purpose: no more than 20 A through any
   * motor, and slow.
   */
  @Test
  void itsLimitsAreTheBenchs() {
    DrivetrainControl.Limits limits = DrivingRobot.CONFIG.drivetrain().limits();
    assertThat(limits).isSameAs(DrivingRobot.BENCH_LIMITS);
    assertThat(limits.driveStatorCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.driveSupplyCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.steerStatorCurrent().in(Amps)).isLessThanOrEqualTo(20);
    assertThat(limits.maxLinearSpeed().in(MetersPerSecond)).isLessThanOrEqualTo(1.5);
  }

  /**
   * A student finds every value to check in one place: each "Unconfirmed" is inside the block, and
   * the block holds nothing else but its values and their comments.
   */
  @Test
  void everyUnconfirmedValueIsInTheOneBlock() throws IOException {
    List<String> lines = Files.readAllLines(SOURCE);
    int start = indexOf(lines, "// ==== CHECK THESE ON THE CHASSIS");
    int end = indexOf(lines, "// ==== END OF THE VALUES TO CHECK");

    assertThat(start).isPositive().isLessThan(end);
    for (int i = 0; i < lines.size(); i++) {
      if (lines.get(i).contains("Unconfirmed")) {
        assertThat(i).as("line %d: %s", i + 1, lines.get(i).strip()).isBetween(start, end);
      }
    }
    for (String line : lines.subList(start + 1, end)) {
      String code = line.strip();
      assertThat(
              code.isEmpty()
                  || code.startsWith("//")
                  || code.startsWith("/**")
                  || code.startsWith("static final"))
          .as(code)
          .isTrue();
    }
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

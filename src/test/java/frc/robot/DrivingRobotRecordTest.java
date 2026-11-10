package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;

/**
 * The driving robot's values against the record of the team's Tuner X session on the chassis:
 * {@code src/test/resources/driving-tuner-x.json}, written by hand from what Tuner X and the
 * zeroing procedure showed, never from {@link DrivingRobot}. Once it's there, a value typed wrong
 * in the block, or into the wrong module, fails here.
 *
 * <p>Until the session happens, the record is absent: {@code theTunerXRecordIsAbsent} passes and
 * says so by its name, and the comparison is skipped. Once it's written, that test is skipped and
 * the comparison runs. The record looks like this, each module in the drivetrain's order (front
 * left, front right, back left, back right), each zero in rotations, as the procedure "Zeroing a
 * swerve module" (runtime/docs/tuner-x.md) gives it:
 *
 * <pre>{@code
 * {
 *   "taken": "the session's date",
 *   "canivore": "the CANivore's name, as Tuner X shows it",
 *   "pigeon": 0,
 *   "modules": [
 *     {"corner": "front left", "drive": 0, "steer": 0, "cancoder": 0, "zeroRotations": 0.0,
 *      "driveInverted": false, "steerInverted": false, "cancoderInverted": false},
 *     ... the front right, back left and back right the same way
 *   ]
 * }
 * }</pre>
 *
 * <p>With the record in and this test green, change each value's "Unconfirmed:" in the block to
 * "Checked" and the date: a deploy refuses while any value is still unconfirmed.
 */
class DrivingRobotRecordTest {
  /** The record of the Tuner X session. */
  private static final Path RECORD =
      Path.of(System.getProperty("frc.projectDir", "."), "src/test/resources/driving-tuner-x.json");

  /** The corners in the drivetrain's order, as the record names them. */
  private static final List<String> CORNERS =
      List.of("front left", "front right", "back left", "back right");

  /**
   * No Tuner X session on the chassis yet, so the block's values are the 2026 robot's, copied, and
   * nothing here can say they're this chassis's. Skipped once the record is written.
   */
  @Test
  void theTunerXRecordIsAbsent() {
    assumeFalse(Files.exists(RECORD), "the Tuner X record is written: the comparison runs instead");
    assertThat(RECORD).doesNotExist();
  }

  /** Every ID, zero and direction in the drivetrain is the record's, module by module. */
  @Test
  @SuppressWarnings("unchecked")
  void itsValuesAreTheTunerXRecords() throws IOException {
    assumeTrue(
        Files.exists(RECORD),
        "no Tuner X record yet (src/test/resources/driving-tuner-x.json): the driving robot's"
            + " IDs, zeros and directions are the 2026 robot's");
    Map<String, Object> record =
        (Map<String, Object>)
            Jsonb.instance().type(Object.class).fromJson(Files.readString(RECORD));
    SwerveWiring wiring = DrivingRobot.DRIVETRAIN.wiring().orElseThrow();

    assertThat((String) record.get("taken")).as("the session's date").isNotBlank();
    assertThat(wiring.canBus()).isEqualTo(CanBus.canivore((String) field(record, "canivore")));
    assertThat(number(record, "pigeon").intValue()).as("the Pigeon 2").isEqualTo(wiring.gyroId());
    List<Object> modules = (List<Object>) field(record, "modules");
    assertThat(modules).as("the record's modules").hasSameSizeAs(CORNERS);
    for (int i = 0; i < CORNERS.size(); i++) {
      Map<String, Object> measured = (Map<String, Object>) modules.get(i);
      SwerveWiring.ModuleWiring module = wiring.modules().get(i);
      String corner = CORNERS.get(i);

      assertThat(measured.get("corner")).as("module %d", i).isEqualTo(corner);
      assertThat(module.driveId()).as("the %s's drive", corner).isEqualTo(id(measured, "drive"));
      assertThat(module.steerId()).as("the %s's steering", corner).isEqualTo(id(measured, "steer"));
      assertThat(module.encoderId())
          .as("the %s's CANcoder", corner)
          .contains(id(measured, "cancoder"));
      assertThat(module.encoderOffset().in(Rotations))
          .as("the %s's zero", corner)
          .isCloseTo(number(measured, "zeroRotations").doubleValue(), within(1e-9));
      assertThat(module.driveInverted())
          .as("the %s's drive's direction", corner)
          .isEqualTo(measured.get("driveInverted"));
      assertThat(module.steerInverted())
          .as("the %s's steering's direction", corner)
          .isEqualTo(measured.get("steerInverted"));
      assertThat(module.encoderInverted())
          .as("the %s's CANcoder's direction", corner)
          .isEqualTo(measured.get("cancoderInverted"));
    }
  }

  private static int id(Map<String, Object> module, String name) {
    return number(module, name).intValue();
  }

  private static Number number(Map<String, Object> values, String name) {
    return (Number) field(values, name);
  }

  /** One of the record's fields, which the record must have. */
  private static Object field(Map<String, Object> values, String name) {
    return Objects.requireNonNull(values.get(name), () -> "the record has no " + name);
  }
}

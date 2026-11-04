package frc.robot;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import java.util.List;
import java.util.Optional;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>Today that's the driving robot ({@link DrivingRobot}), the swerve drive alone, and the same
 * robot with the vision coprocessors on the bench ({@link VisionBenchRobot}). The 2027 robot is
 * added here once it's designed: runtime/docs/values.md says how to measure each value, and
 * runtime/docs/mechanisms.md how to describe a mechanism.
 */
public final class Robots {
  private Robots() {}

  /** Every robot the team has. Add each real robot as it is built and measured. */
  public static List<RobotConfig> all() {
    return List.of(DrivingRobot.CONFIG, VisionBenchRobot.CONFIG);
  }

  /**
   * The system property that names the robot to run. The build sets it from {@code -Probot=NAME}
   * ({@code ./gradlew run -PhwSim -Probot=vision-bench}), so a laptop can run any of the team's
   * robots without an edit here. Nothing sets it on a real robot.
   */
  static final String ROBOT_PROPERTY = "frc.robot";

  /**
   * The robot this program is running on: the one place that decides. It's the driving robot,
   * unless a laptop asked for another by name. Once there are several real robots, this chooses by
   * something the robot itself knows, such as its controller's serial number, never by editing this
   * before each deploy.
   */
  public static RobotConfig current() {
    return asked(Optional.ofNullable(System.getProperty(ROBOT_PROPERTY)));
  }

  /**
   * The robot that was asked for by name, or the driving robot when none was.
   *
   * @throws IllegalArgumentException for a name that is none of the team's robots, saying which
   *     names there are
   */
  static RobotConfig asked(Optional<String> name) {
    if (name.isEmpty()) {
      return DrivingRobot.CONFIG;
    }
    for (RobotConfig robot : all()) {
      if (robot.name().equals(name.get())) {
        return robot;
      }
    }
    List<String> names = all().stream().map(RobotConfig::name).toList();
    throw new IllegalArgumentException(
        "No robot is named \"" + name.get() + "\". The robots are: " + String.join(", ", names));
  }
}

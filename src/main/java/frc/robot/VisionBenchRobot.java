package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.CameraStation;
import org.wpilib.units.measure.Distance;

/**
 * The driving robot with the two vision coprocessors and two ThriftyCams, for proving the vision
 * side on the bench before the cameras are mounted: {@code ./gradlew run -PhwSim
 * -Probot=vision-bench}. Only a laptop can run it, since a deployed program has no {@code -Probot}.
 *
 * <p>The cameras sit on the bench, not on the robot, so the two stations below are guesses and the
 * poses they give are meaningless; what it proves is the chain: the robot installing and
 * configuring Visor on each board, the boards' clocks, frames arriving. Once the cameras are
 * mounted, stations at their measured places move into {@link DrivingRobot}, and this robot is
 * deleted.
 *
 * <p>The boards are numbered, not named for a place on the robot: {@code vision-N} is {@code
 * 10.26.11.1N}, from the team's robot-images repository. Which board a camera is on is found from
 * what each board reports, by the camera's serial.
 */
public final class VisionBenchRobot {
  private VisionBenchRobot() {}

  /**
   * {@code vision-1}, by its Spotter agent's address. The robot pushes it Visor's pack itself,
   * since cameras on it are on Visor.
   */
  static final String VISION_1 = "10.26.11.11";

  /** {@code vision-2}. */
  static final String VISION_2 = "10.26.11.12";

  /** The bench's left spot, as if at the robot's front left, facing forward. A guess: see above. */
  static final CameraStation BENCH_LEFT = station("bench-left", Inches.of(4));

  /** The bench's right spot, as if at the front right. */
  static final CameraStation BENCH_RIGHT = station("bench-right", Inches.of(-4));

  /**
   * The driving robot with cameras 1 and 2 on the bench, and the boards they're plugged into. Each
   * camera brings its own device and lens from {@link ThriftyCams}, where they're written together
   * because a camera given another's lens points its poses about 4 degrees off. Swap a camera by
   * naming another one here.
   */
  public static final RobotConfig CONFIG =
      DrivingRobot.CONFIG
          .named("vision-bench")
          .withCameras(
              BENCH_LEFT.holds(ThriftyCams.CAMERA_1), BENCH_RIGHT.holds(ThriftyCams.CAMERA_2))
          .withCoprocessors(VISION_1, VISION_2);

  /** A spot on the bench, 12 in ahead and 7 in up, level and facing forward. */
  private static CameraStation station(String name, Distance left) {
    return CameraStation.named(name)
        .at(Inches.of(12), left, Inches.of(7))
        .facing(Degrees.of(0), Degrees.of(0));
  }
}

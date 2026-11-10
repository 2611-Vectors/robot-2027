package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;

/**
 * The driving robot with the two vision coprocessors and their ThriftyCams, for proving the vision
 * side on the bench before the cameras are mounted: {@code ./gradlew run -PhwSim
 * -Probot=vision-bench}. Only a laptop can run it, since a deployed program has no {@code -Probot}.
 *
 * <p>The cameras sit on the bench, not on the robot, so the places below are guesses and the poses
 * they give are meaningless; what it proves is the chain: the robot installing and configuring
 * Visor on each board, the boards' clocks, frames arriving. Once the cameras are mounted, they move
 * into {@link DrivingRobot} at their measured places, and this robot is deleted.
 *
 * <p>The boards are numbered, not named for a place on the robot: {@code vision-N} is {@code
 * 10.26.11.1N}, from the team's robot-images repository. Which cameras a board carries is found
 * from what each board reports, by each camera's device.
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

  // TODO(team): each camera's device, from `visor cameras` on the board: its /dev/v4l/by-id/ name,
  // or its by-path/ one. These placeholders match no camera, so until they're filled in the robot
  // configures nothing and says each camera is plugged into no coprocessor.
  static final String CAMERA_1_DEVICE = "/dev/v4l/by-id/usb-from-visor-cameras-1-video-index0";
  static final String CAMERA_2_DEVICE = "/dev/v4l/by-id/usb-from-visor-cameras-2-video-index0";

  /**
   * A ThriftyCam at the size and rate it runs at, from {@code visor cameras}: 1600x1304 at 60. Its
   * field of view comes from each camera's lens ({@link ThriftyCamLenses}): sold as 80°, the four
   * calibrations of 2026-11-04 put it at 73° to 74° across the diagonal. Its latency is Visor's
   * typical one until the Vision log measures it, and the robot says so at startup.
   */
  static final CameraConfig.Sensor THRIFTYCAM = CameraConfig.Sensor.of(1600, 1304, Hertz.of(60));

  // The two cameras are the ones labelled 1 and 2, plugged into the boards on 2026-11-04, and each
  // takes its own lens from ThriftyCamLenses. The four lenses' centers differ by up to about 100
  // pixels, about 4° of pointing, so a camera given another's lens points its poses that far off:
  // the number on the camera's label has to be the lens it's given here. Swap a camera, and change
  // its lens with it.

  /** Camera 1, placed as if at the front left, facing forward. A guess: see above. */
  static final CameraConfig CAMERA_1 =
      camera("one", Inches.of(4), CAMERA_1_DEVICE, ThriftyCamLenses.CAM_1);

  /** Camera 2, as if at the front right. */
  static final CameraConfig CAMERA_2 =
      camera("two", Inches.of(-4), CAMERA_2_DEVICE, ThriftyCamLenses.CAM_2);

  /** The driving robot with the two cameras and the boards they're plugged into. */
  public static final RobotConfig CONFIG =
      DrivingRobot.CONFIG
          .named("vision-bench")
          .withCameras(CAMERA_1, CAMERA_2)
          .withCoprocessors(VISION_1, VISION_2);

  private static CameraConfig camera(
      String name, org.wpilib.units.measure.Distance left, String device, Software.Lens lens) {
    return CameraConfig.builder()
        .name(name)
        .forward(Inches.of(12))
        .left(left)
        .up(Inches.of(7))
        .tiltUp(Degrees.of(0))
        .yaw(Degrees.of(0))
        .sensor(THRIFTYCAM)
        .software(Software.visor().device(device).lens(lens).build())
        .build();
  }
}

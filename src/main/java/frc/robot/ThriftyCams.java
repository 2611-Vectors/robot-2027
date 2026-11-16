package frc.robot;

import static org.wpilib.units.Units.Hertz;

import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.ImageSensor;
import java.util.List;

/**
 * The team's four ThriftyCams, each by the number on its label. Each camera's number, its device
 * and its lens calibration are written here together, once, because they belong to one physical
 * camera: the four lenses' centers differ by up to about 100 pixels, about 4 degrees of pointing,
 * so a camera given another camera's lens points its poses that far off. A robot puts a camera in a
 * station by naming it here ({@code FRONT_LEFT.holds(ThriftyCams.CAMERA_1)}), and swapping a camera
 * is changing that one name.
 *
 * <p>Every camera has its own USB serial since its firmware update of 2026-11-15, so each device
 * below names that one camera, whichever USB port or coprocessor it's plugged into. The robot finds
 * which coprocessor each camera is on by its serial.
 */
public final class ThriftyCams {
  private ThriftyCams() {}

  /**
   * A ThriftyCam, at the size and rate the team runs it: 1600 x 1304 at 60 frames a second. Its
   * exposure is the library's default, a fixed 1 ms, which keeps tags sharp while the robot turns.
   *
   * <p>Its gain starts at 160. The cameras' firmware 5.10 takes a gain from 16 to 255. One sweep on
   * 2026-11-15, in a dim room at 1 ms, read every tag from a gain of 118 up. 160 was chosen as a
   * starting point that still reads tags in a room that dim and sits in the middle of the range in
   * light three to four times brighter. Each venue adjusts it from there, in the pit, and keeps its
   * own value.
   */
  public static final CameraType THRIFTYCAM =
      CameraType.of("ThriftyCam", ImageSensor.of(1600, 1304, Hertz.of(60))).gain(160);

  /** Camera 1. */
  public static final Camera CAMERA_1 =
      Camera.visor(1, THRIFTYCAM)
          .device("/dev/v4l/by-id/usb-Thrifty_Bot_Thrifty_Cam_TCF2CF2267CBF621-video-index0")
          .lens(ThriftyCamLenses.CAM_1);

  /** Camera 2. */
  public static final Camera CAMERA_2 =
      Camera.visor(2, THRIFTYCAM)
          .device("/dev/v4l/by-id/usb-Thrifty_Bot_Thrifty_Cam_TC15546402E4E7DA-video-index0")
          .lens(ThriftyCamLenses.CAM_2);

  /** Camera 3. */
  public static final Camera CAMERA_3 =
      Camera.visor(3, THRIFTYCAM)
          .device("/dev/v4l/by-id/usb-Thrifty_Bot_Thrifty_Cam_TC4D7BA86A565435-video-index0")
          .lens(ThriftyCamLenses.CAM_3);

  /**
   * Camera 4. Its lens was refocused on 2026-11-15, so the lens numbers it carries are from its old
   * focus: calibrate it again before trusting its poses.
   */
  public static final Camera CAMERA_4 =
      Camera.visor(4, THRIFTYCAM)
          .device("/dev/v4l/by-id/usb-Thrifty_Bot_Thrifty_Cam_TCCA73715A72342A-video-index0")
          .lens(ThriftyCamLenses.CAM_4);

  /** Every camera, camera 1 first. */
  public static List<Camera> all() {
    return List.of(CAMERA_1, CAMERA_2, CAMERA_3, CAMERA_4);
  }
}

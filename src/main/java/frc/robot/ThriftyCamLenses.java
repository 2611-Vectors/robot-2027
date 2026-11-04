package frc.robot;

import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import java.util.List;

/**
 * Each of the team's ThriftyCams' lens, by the number on its label. A lens belongs to one camera:
 * the four differ by up to about 100 pixels in where their image's center falls, so a camera moved
 * to another robot or board takes its own lens with it, and a camera that's refocused, dropped hard
 * or has its lens turned is calibrated again.
 *
 * <p>Each was calibrated on 2026-11-04, on a laptop with the camera plugged in, at 1600x1304 (the
 * size the cameras run at), its focus set for 2 to 4 m and locked first. The board was the team's
 * ChArUco board: 15x15 squares of 1.125 in, 0.875 in markers, OpenCV's DICT_4X4_1000. Frames were
 * taken with Visor's {@code tools/capture.py} and solved with its {@code tools/calibrate.py}. Each
 * constant notes the session's error (RMS reprojection, pixels), how many views counted, and the
 * focal length's uncertainty.
 */
public final class ThriftyCamLenses {
  private ThriftyCamLenses() {}

  /** Camera 1: 50 views, RMS 0.548 px, fx ± 4.5 px, cx ± 1.8 px. */
  public static final Lens CAM_1 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1385.626837)
          .fy(1387.880881)
          .cx(841.681532)
          .cy(688.006579)
          .distortion(
              List.of(-0.0420413004, 0.0291864455, 0.000247843132, -0.0024258741, -0.0766996463))
          .build();

  /** Camera 2: 61 views, RMS 0.522 px, fx ± 4.2 px, cx ± 3.1 px. */
  public static final Lens CAM_2 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1388.200505)
          .fy(1390.633261)
          .cx(792.484826)
          .cy(641.577275)
          .distortion(
              List.of(-0.0107584474, -0.0701060132, -0.00152207541, -0.00202290946, 0.113977379))
          .build();

  /** Camera 3: 50 views, RMS 0.666 px, fx ± 3.4 px, cx ± 1.4 px. */
  public static final Lens CAM_3 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1363.663485)
          .fy(1365.215837)
          .cx(774.076122)
          .cy(651.502170)
          .distortion(
              List.of(-0.043596118, 0.0157384627, -0.000267201867, 4.96929739e-05, 0.00678722726))
          .build();

  /** Camera 4: 50 views, RMS 0.626 px, fx ± 2.0 px, cx ± 1.8 px. */
  public static final Lens CAM_4 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1381.374761)
          .fy(1382.239361)
          .cx(744.486343)
          .cy(564.735267)
          .distortion(
              List.of(-0.00860284174, -0.109650282, -0.00182367803, -0.00160133724, 0.17954197))
          .build();

  /** Every camera's lens, camera 1 first. */
  public static List<Lens> all() {
    return List.of(CAM_1, CAM_2, CAM_3, CAM_4);
  }
}

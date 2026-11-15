package frc.robot;

import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import java.util.List;

/**
 * Each of the team's ThriftyCams' lens, by the number on its label. A lens belongs to one camera:
 * the four differ by up to about 100 pixels in where their image's center falls (about 4° of
 * pointing), and that's real, each lens sitting a little off its sensor's middle, not an error in
 * the calibration. So a camera moved to another robot or board takes its own lens with it, and a
 * camera that's refocused, dropped hard or has its lens turned is calibrated again.
 *
 * <p>Each camera's frames were taken on 2026-11-04, on a laptop with the camera plugged in, at
 * 1600x1304 (the size the cameras run at), its focus set for 2 to 4 m and locked first. The board
 * was the team's ChArUco board: 15x15 squares of 1.125 in, 0.875 in markers, OpenCV's
 * DICT_4X4_1000. Frames were taken with Visor's {@code tools/capture.py}, and solved again on
 * 2026-11-09 with its fixed {@code tools/calibrate.py}, which holds the last distortion term (k3)
 * at 0 when the views can't decide it. Each constant notes the session's error (RMS reprojection,
 * pixels), how many views counted, and whether the session was accepted.
 *
 * <p>A provisional lens is good in the center and the middle of the frame, but the board never
 * reached the frame's edges, so tags near an edge are measured less well. It needs a new session,
 * with the board's corners reaching every corner of the frame.
 *
 * <p>{@code src/test/resources/thriftycam-lenses.json} records the calibrations' numbers, and the
 * test checks these against it: a camera calibrated again has its numbers changed in both.
 */
public final class ThriftyCamLenses {
  private ThriftyCamLenses() {}

  /**
   * Camera 1: 50 views, RMS 0.548 px. Provisional: the center and the middle of the frame are good,
   * the edges weren't covered. Calibrate it again, the board reaching every corner of the frame.
   */
  public static final Lens CAM_1 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1385.453346)
          .fy(1387.693951)
          .cx(841.748551)
          .cy(688.089816)
          .distortion(List.of(-0.0379933425, -0.00505548711, 0.000259603563, -0.00242699815, 0.0))
          .build();

  /**
   * Camera 2: 60 views, RMS 0.498 px. Provisional: the center and the middle of the frame are good,
   * the edges weren't covered. Calibrate it again, the board reaching every corner of the frame.
   */
  public static final Lens CAM_2 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1388.745831)
          .fy(1391.114571)
          .cx(794.961953)
          .cy(645.687058)
          .distortion(List.of(-0.0213614477, -0.00264023068, -0.000586039413, -0.00138722088, 0.0))
          .build();

  /** Camera 3: 50 views, RMS 0.666 px. Accepted: its session passed the tool's checks. */
  public static final Lens CAM_3 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1363.716868)
          .fy(1365.258832)
          .cx(774.129216)
          .cy(651.470545)
          .distortion(List.of(-0.044568638, 0.0204994724, -0.000271843966, 5.40701723e-05, 0.0))
          .build();

  /**
   * Camera 4: 50 views, RMS 0.627 px. Provisional: the center and the middle of the frame are good,
   * the edges weren't covered. Calibrate it again, the board reaching every corner of the frame.
   */
  public static final Lens CAM_4 =
      Lens.builder()
          .width(1600)
          .height(1304)
          .fx(1380.886019)
          .fy(1381.847188)
          .cx(744.466772)
          .cy(564.815704)
          .distortion(List.of(-0.0231583075, -0.0103007031, -0.00173185198, -0.00157842134, 0.0))
          .build();

  /** Every camera's lens, camera 1 first. */
  public static List<Lens> all() {
    return List.of(CAM_1, CAM_2, CAM_3, CAM_4);
  }
}

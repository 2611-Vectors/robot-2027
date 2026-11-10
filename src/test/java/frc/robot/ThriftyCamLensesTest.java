package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The ThriftyCams' lenses: each exactly the numbers its calibration gave, from the record of the
 * calibration results ({@code src/test/resources/thriftycam-lenses.json}), so a number mistyped or
 * pasted into the wrong camera fails; each at the size the cameras run at, so the build takes it
 * for a camera of that size; and each one plausible for the ThriftyCam's lens, so a record that's
 * wrong itself still fails. Then the bench robot's two cameras, each with its own lens.
 */
class ThriftyCamLensesTest {
  /** The record of the calibration results, as Visor's calibration tool gave them. */
  private static final Path RECORD =
      Path.of(
          System.getProperty("frc.projectDir", "."), "src/test/resources/thriftycam-lenses.json");

  /** One camera's calibration, as the record has it. */
  private record Calibrated(int camera, String verdict, Lens lens) {}

  @Test
  void everyLensIsItsCalibrationsNumbers() throws IOException {
    List<Calibrated> record = calibrations();
    List<Lens> lenses = ThriftyCamLenses.all();

    assertThat(record).extracting(Calibrated::camera).containsExactly(1, 2, 3, 4);
    for (int i = 0; i < lenses.size(); i++) {
      Calibrated calibrated = record.get(i);
      // A Lens is a record: equal only when every number is the same, to the last digit.
      assertThat(lenses.get(i)).as("camera %d", calibrated.camera()).isEqualTo(calibrated.lens());
      assertThat(calibrated.verdict())
          .as("camera %d's session", calibrated.camera())
          .isIn("accepted", "provisional");
    }
  }

  @Test
  void everyLensIsAtTheCamerasSizeAndPlausible() {
    for (Lens lens : ThriftyCamLenses.all()) {
      assertThat(lens.width()).isEqualTo(VisionBenchRobot.THRIFTYCAM.widthPixels());
      assertThat(lens.height()).isEqualTo(VisionBenchRobot.THRIFTYCAM.heightPixels());
      // About a 73° diagonal field of view: a focal length near 1380 px at this size.
      assertThat(lens.fx()).isBetween(1300.0, 1450.0);
      assertThat(lens.fy() / lens.fx()).isBetween(0.99, 1.01);
      // The center near the middle of the frame, within the lens mount's play.
      assertThat(lens.cx()).isBetween(600.0, 1000.0);
      assertThat(lens.cy()).isBetween(450.0, 850.0);
      // OpenCV's five terms, the last (k3) held at 0: the sessions' views can't decide it.
      assertThat(lens.distortion()).hasSize(5);
      assertThat(lens.distortion().get(4)).isZero();
    }
  }

  /** Camera "one" is the camera labelled 1, and "two" the camera labelled 2. */
  @Test
  void theBenchsCamerasTakeTheirOwnLenses() {
    assertThat(VisionBenchRobot.CAMERA_1.name()).isEqualTo("one");
    assertThat(lensOf(VisionBenchRobot.CAMERA_1)).contains(ThriftyCamLenses.CAM_1);
    assertThat(VisionBenchRobot.CAMERA_2.name()).isEqualTo("two");
    assertThat(lensOf(VisionBenchRobot.CAMERA_2)).contains(ThriftyCamLenses.CAM_2);
  }

  private static Optional<Lens> lensOf(CameraConfig camera) {
    if (camera.software() instanceof CameraConfig.Software.Visor visor) {
      return visor.lens();
    }
    return Optional.empty();
  }

  @SuppressWarnings("unchecked")
  private static List<Calibrated> calibrations() throws IOException {
    Map<String, Object> record =
        (Map<String, Object>)
            Jsonb.instance().type(Object.class).fromJson(Files.readString(RECORD));
    List<Calibrated> calibrations = new ArrayList<>();
    for (Object entry : (List<Object>) field(record, "cameras")) {
      Map<String, Object> camera = (Map<String, Object>) entry;
      List<Double> distortion = new ArrayList<>();
      for (Object term : (List<Object>) field(camera, "distortion")) {
        distortion.add(((Number) term).doubleValue());
      }
      Lens lens =
          Lens.builder()
              .width(number(camera, "width").intValue())
              .height(number(camera, "height").intValue())
              .fx(number(camera, "fx").doubleValue())
              .fy(number(camera, "fy").doubleValue())
              .cx(number(camera, "cx").doubleValue())
              .cy(number(camera, "cy").doubleValue())
              .distortion(distortion)
              .build();
      calibrations.add(
          new Calibrated(
              number(camera, "camera").intValue(), (String) field(camera, "verdict"), lens));
    }
    return calibrations;
  }

  private static Number number(Map<String, Object> camera, String name) {
    return (Number) field(camera, name);
  }

  /** One of the record's fields, which every entry has. */
  private static Object field(Map<String, Object> values, String name) {
    return Objects.requireNonNull(values.get(name), () -> "the record has no " + name);
  }
}

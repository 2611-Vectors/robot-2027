package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.CameraConfig.Software.Lens;
import org.junit.jupiter.api.Test;

/**
 * The ThriftyCams' lenses: each at the size the cameras run at, so the build takes it for a camera
 * of that size, and each one plausible for the ThriftyCam's lens.
 */
class ThriftyCamLensesTest {
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
      assertThat(lens.distortion()).hasSize(5);
    }
  }
}

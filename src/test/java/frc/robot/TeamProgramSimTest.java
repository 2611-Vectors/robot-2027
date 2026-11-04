package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * The team's robot program, put together as its {@code Main} hands it to the runtime: each of its
 * robots, this season's field, and its controls. It starts, and drives its loop.
 */
@SimulationTest
class TeamProgramSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /**
   * Every robot runs with the team's own {@code Controls}, which binds nothing and declares no auto
   * until the team writes them: the runtime's own binding and "Do nothing" are all there is.
   */
  @ParameterizedTest
  @ValueSource(strings = {"driving", "vision-bench"})
  void eachRobotRunsWithTheTeamsControls(String name) {
    sim.build(
        Robots.asked(java.util.Optional.of(name)),
        Rebuilt.GAME,
        Controls::new,
        SensorErrors.TYPICAL);
    sim.run(1);

    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .containsExactly("driver back: on press: Reset heading");
    assertThat(sim.autos().names()).containsExactly(Autos.DO_NOTHING);
  }
}

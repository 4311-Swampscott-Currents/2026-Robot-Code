package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * HopperSubsystem
 *
 * <p>Conveyor motor that moves balls from the intake area toward the kicker. Runs open-loop.
 *
 * <p>CURRENT LIMITS: Stator: 40A — conveyor is a light load, but may spike during jams Supply: 30A
 * — consistent with the 30A breaker on secondary mechanisms
 */
public class Hopper extends SubsystemBase {

  private final TalonFX conveyorMotor = new TalonFX(Constants.HopperConstants.M_HOPPER_ID);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public Hopper() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Current Limits ---
    config.CurrentLimits.StatorCurrentLimit = Constants.HopperConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.HopperConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Brake: balls don't roll back when conveyor stops
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    conveyorMotor.getConfigurator().apply(config);
  }

  public void runForward() {
    conveyorMotor.setControl(
        dutyCycleRequest.withOutput(Constants.HopperConstants.FORWARD_PERCENT));
  }

  public void runReverse() {
    conveyorMotor.setControl(
        dutyCycleRequest.withOutput(Constants.HopperConstants.REVERSE_PERCENT));
  }

  public void runPercent(double percent) {
    conveyorMotor.setControl(dutyCycleRequest.withOutput(percent));
  }

  public void stop() {
    conveyorMotor.setControl(neutralRequest);
  }

  /**
   * Placeholder for beam-break sensor. Replace with: private final DigitalInput sensor = new
   * DigitalInput(port); return !sensor.get();
   */
  public boolean hasBall() {
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Hopper/OutputPercent", conveyorMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Hopper/StatorAmps", conveyorMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Hopper/HasBall", hasBall());
  }
}

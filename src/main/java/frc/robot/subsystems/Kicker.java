package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * KickerSubsystem
 *
 * <p>Single motor that feeds balls from the hopper into the shooter flywheels. Runs open-loop (duty
 * cycle). Only activate after ShooterSubsystem.atSetpoint() returns true.
 *
 * <p>CURRENT LIMITS: Stator: 40A — kicker runs in short bursts, doesn't need high sustained torque
 * Supply: 30A — protects the 30A breaker typically used for smaller mechanisms
 */
public class Kicker extends SubsystemBase {

  private final TalonFX kickerLeftMotor =
      new TalonFX(Constants.KickerConstants.M_KICKER_LEFT_ID, TunerConstants.kCANBus);
  private final TalonFX kickerRightMotor =
      new TalonFX(Constants.KickerConstants.M_KICKER_RIGHT_ID, TunerConstants.kCANBus);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final Follower followRequest =
      new Follower(Constants.KickerConstants.M_KICKER_LEFT_ID, MotorAlignmentValue.Opposed);
  private final NeutralOut neutralRequest = new NeutralOut();

  public Kicker() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Current Limits ---
    config.CurrentLimits.StatorCurrentLimit = Constants.KickerConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.KickerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Brake: prevents coasting from accidentally feeding a ball
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerLeftMotor.getConfigurator().apply(config);

    kickerRightMotor.getConfigurator().apply(config);

    kickerRightMotor.setControl(followRequest);
  }

  /** Run kicker at full speed to feed a ball. */
  public void kick() {
    kickerLeftMotor.setControl(dutyCycleRequest.withOutput(Constants.KickerConstants.KICK_PERCENT));
  }

  /** Run at a custom duty cycle. Negative = reverse (unjam). */
  public void runPercent(double percent) {
    kickerLeftMotor.setControl(dutyCycleRequest.withOutput(percent));
  }

  public void stop() {
    kickerLeftMotor.setControl(neutralRequest);
    kickerRightMotor.setControl(neutralRequest);
    kickerRightMotor.setControl(followRequest);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Kicker/LeftOutputPercent", kickerLeftMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/LeftStatorAmps", kickerLeftMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/RightOutputPercent", kickerRightMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/RightStatorAmps", kickerRightMotor.getStatorCurrent().getValueAsDouble());
  }
}

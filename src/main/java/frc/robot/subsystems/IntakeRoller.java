package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * IntakeRollerSubsystem
 *
 * <p>Roller wheel at the end of the intake arm. Runs open-loop. Commanded in parallel with
 * IntakeArmSubsystem during intake.
 *
 * <p>CURRENT LIMITS: Stator: 40A — roller is lightly loaded most of the time, but can spike hard
 * when first contacting a ball on the floor Supply: 30A — keeps total draw reasonable during
 * simultaneous intake + drive
 */
public class IntakeRoller extends SubsystemBase {

  private final TalonFX rollerMotor =
      new TalonFX(Constants.IntakeRollerConstants.M_Intake_Roller_ID, TunerConstants.kCANBus);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public IntakeRoller() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Current Limits ---
    config.CurrentLimits.StatorCurrentLimit = Constants.IntakeRollerConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.IntakeRollerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Coast: roller doesn't need to hold position - claude is wrong
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerMotor.getConfigurator().apply(config);
  }

  public void intake() {
    rollerMotor.setControl(
        dutyCycleRequest.withOutput(Constants.IntakeRollerConstants.INTAKE_PERCENT));
  }

  public void eject() {
    rollerMotor.setControl(
        dutyCycleRequest.withOutput(Constants.IntakeRollerConstants.EJECT_PERCENT));
  }

  public void runPercent(double percent) {
    rollerMotor.setControl(dutyCycleRequest.withOutput(percent));
  }

  public void stop() {
    rollerMotor.setControl(neutralRequest);
  }

  public TalonFX getIntakeRoller() {
    return rollerMotor;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "IntakeRoller/OutputPercent", rollerMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "IntakeRoller/StatorAmps", rollerMotor.getStatorCurrent().getValueAsDouble());
  }
}

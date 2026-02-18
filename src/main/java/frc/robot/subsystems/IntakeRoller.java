package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  private final TalonFX rollerMotor = new TalonFX(Constants.IntakeRoller.M_Intake_Roller_ID);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public IntakeRoller() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Current Limits ---
    config.CurrentLimits.StatorCurrentLimit = Constants.IntakeRoller.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.IntakeRoller.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Coast: roller doesn't need to hold position
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerMotor.getConfigurator().apply(config);
  }

  public void intake() {
    rollerMotor.setControl(dutyCycleRequest.withOutput(Constants.IntakeRoller.INTAKE_PERCENT));
  }

  public void eject() {
    rollerMotor.setControl(dutyCycleRequest.withOutput(Constants.IntakeRoller.EJECT_PERCENT));
  }

  public void runPercent(double percent) {
    rollerMotor.setControl(dutyCycleRequest.withOutput(percent));
  }

  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "IntakeRoller/OutputPercent", rollerMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "IntakeRoller/StatorAmps", rollerMotor.getStatorCurrent().getValueAsDouble());
  }
}

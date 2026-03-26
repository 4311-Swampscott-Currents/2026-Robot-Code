package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

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
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private double targetRPS = 0.0;

  public Kicker() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Current Limits ---
    config.CurrentLimits.StatorCurrentLimit = Constants.KickerConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.KickerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // --- Velocity PID (Slot 0) ---
    config.Slot0.kP = Constants.KickerConstants.kP;
    config.Slot0.kI = Constants.KickerConstants.kI;
    config.Slot0.kD = Constants.KickerConstants.kD;
    config.Slot0.kS = Constants.KickerConstants.kS;
    config.Slot0.kV = Constants.KickerConstants.kV;

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

  /**
   * Spin the kickers to the target speed in rotations per second. Only the left motor is commanded
   * — the right follows automatically.
   *
   * @param rps Target speed (positive = shoot direction)
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    kickerLeftMotor.setControl(velocityRequest.withVelocity(rps));
    // m_leftShooter follows automatically — do NOT call setControl on it here
  }

  public void setVoltage(double volts) {
    kickerLeftMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void stop() {
    kickerLeftMotor.setControl(neutralRequest);
    kickerRightMotor.setControl(neutralRequest);
    kickerRightMotor.setControl(followRequest);
  }

  /**
   * Returns true when the left motor (and by extension the follower) is within RPS tolerance of the
   * target. Gate the indexer behind this?
   */
  public boolean atSetpoint() {
    double error = Math.abs(kickerLeftMotor.getClosedLoopError().getValueAsDouble());
    return targetRPS > 0 && error < Constants.ShooterConstants.TOLERANCE_RPS;
  }

  public boolean isRunning() {
    return targetRPS > 0.0;
  }

  public double getRightVelocityRPS() {
    return kickerRightMotor.getVelocity().getValueAsDouble();
  }

  public double getLeftVelocityRPS() {
    return kickerLeftMotor.getVelocity().getValueAsDouble();
  }

  public TalonFX getRightKicker() {
    return kickerRightMotor;
  }

  public TalonFX getLeftKicker() {
    return kickerLeftMotor;
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with kicker status
    SmartDashboard.putNumber(
        "Kicker/LeftOutputPercent", kickerLeftMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/LeftStatorAmps", kickerLeftMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/RightOutputPercent", kickerRightMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Kicker/RightStatorAmps", kickerRightMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Kicker/TargetRPS", targetRPS);

    // Log the same data to the data logger for offline analysis
    Logger.recordOutput(
        "Kicker/LeftOutputPercent", kickerLeftMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(
        "Kicker/LeftStatorAmps", kickerLeftMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Kicker/RightOutputPercent", kickerRightMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(
        "Kicker/RightStatorAmps", kickerRightMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Kicker/TargetRPS", targetRPS);
  }
}

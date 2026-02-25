package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * ShooterSubsystem
 *
 * <p>Two Kraken motors mounted opposite each other. The bottom motor is configured as a follower
 * with opposeMasterDirection = true, meaning it mirrors the top motor's output but inverted — so
 * both wheels spin toward each other and propel the ball forward.
 *
 * <p>You only ever command the top motor. The bottom follows automatically at 1000Hz on its own
 * control loop, which is tighter than sending two separate setControl() calls.
 *
 * <p>CURRENT LIMITS: Stator: 80A — primary protection against motor heat during sustained spin-up
 * Supply: 60A — protects wiring and PDH breaker
 *
 * <p>TUNING ORDER: 1. Characterize kV: ramp voltage slowly, record rot/sec at steady state. kV =
 * voltage / velocity. Typical Kraken: 0.10–0.14. 2. Set kS: voltage that barely overcomes static
 * friction. 3. Raise kP from 0 until setpoint is reached quickly without oscillation.
 */
public class Shooter extends SubsystemBase {

  // -----------------------------------------------------------------------
  // Hardware
  // -----------------------------------------------------------------------
  private final TalonFX m_rightShooter = new TalonFX(Constants.ShooterConstants.M_RIGHT_SHOOTER_ID);
  private final TalonFX m_leftShooter = new TalonFX(Constants.ShooterConstants.M_LEFT_SHOOTER_ID);

  // Command only the top motor — left follows automatically
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withSlot(0) /*.withEnableFOC(true)*/;

  // Follower request: copies top motor output, but opposes direction
  // because the motors are physically mounted facing each other
  private final Follower followerRequest =
      new Follower(Constants.ShooterConstants.M_RIGHT_SHOOTER_ID, MotorAlignmentValue.Opposed);

  private double targetRPS = 0.0;

  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------
  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- Velocity PID (Slot 0) ---
    config.Slot0.kP = Constants.ShooterConstants.kP;
    config.Slot0.kI = Constants.ShooterConstants.kI;
    config.Slot0.kD = Constants.ShooterConstants.kD;
    config.Slot0.kS = Constants.ShooterConstants.kS;
    config.Slot0.kV = Constants.ShooterConstants.kV;

    // --- Current Limits ---
    // Stator: limits motor winding current → prevents overheating
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // Supply: limits current drawn from battery → protects wiring/breaker
    config.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Coast so flywheels spin down naturally — never brake a spinning flywheel
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Top motor: positive command = shoots forward
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_rightShooter.getConfigurator().apply(config);

    // Bottom motor: same current limits, coast neutral
    // Direction is handled by the Follower request (opposeMasterDirection)
    // so inversion here doesn't matter — but set it consistently anyway
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_leftShooter.getConfigurator().apply(config);

    // Lock the bottom motor into follower mode.
    // This persists until you call stopMotor() or send a different request.
    m_leftShooter.setControl(followerRequest);
  }

  // -----------------------------------------------------------------------
  // Public API
  // -----------------------------------------------------------------------

  /**
   * Spin the flywheels to the target speed in rotations per second. Only the top motor is commanded
   * — the bottom follows automatically.
   *
   * @param rps Target speed (positive = shoot direction)
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    m_rightShooter.setControl(velocityRequest.withVelocity(rps));
    // m_leftShooter follows automatically — do NOT call setControl on it here
  }

  public void setVoltage(double volts) {
    m_rightShooter.setControl(new VoltageOut(volts));
  }

  /**
   * Stop both motors. Clears follower mode on the bottom motor temporarily; re-enable by calling
   * setVelocity() again or re-applying the follower request.
   */
  public void stop() {
    targetRPS = 0.0;
    m_rightShooter.stopMotor();
    m_leftShooter.stopMotor();
    // Re-apply follower so bottom is ready for next setVelocity() call
    m_leftShooter.setControl(followerRequest);
  }

  /**
   * Returns true when the top motor (and by extension the follower) is within RPM tolerance of the
   * target. Gate the kicker behind this.
   */
  public boolean atSetpoint() {
    double error = Math.abs(m_rightShooter.getClosedLoopError().getValueAsDouble());
    return targetRPS > 0 && error < Constants.ShooterConstants.TOLERANCE_RPS;
  }

  public boolean isRunning() {
    return targetRPS > 0.0;
  }

  public double getRightVelocityRPS() {
    return m_rightShooter.getVelocity().getValueAsDouble();
  }

  public double getLeftVelocityRPS() {
    return m_leftShooter.getVelocity().getValueAsDouble();
  }

  // -----------------------------------------------------------------------
  // Periodic
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/TargetRPS", targetRPS);
    SmartDashboard.putNumber("Shooter/RightRPS", getRightVelocityRPS());
    SmartDashboard.putNumber("Shooter/LeftRPS", getLeftVelocityRPS());
    SmartDashboard.putBoolean("Shooter/AtSetpoint", atSetpoint());
    SmartDashboard.putNumber(
        "Shooter/TopStatorAmps", m_rightShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/BottomStatorAmps", m_leftShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/TestVoltage", 0.0);
  }
}

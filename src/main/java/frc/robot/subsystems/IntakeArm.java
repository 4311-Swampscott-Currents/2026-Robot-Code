package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * IntakeArmSubsystem
 *
 * <p>Simple hard-stop control for the intake arm pivot. No Motion Magic, no PID, no feedforward
 * tuning required.
 *
 * <p>The arm has two positions — DEPLOYED and RETRACTED — each with a physical hard stop. The motor
 * runs at a fixed duty cycle toward the target stop, then cuts out when the stop is detected.
 *
 * <p>── Stop Detection ─────────────────────────────────────────────────────────── Three
 * independent conditions checked every loop. Arm is considered at its stop when ANY of the
 * following triggers:
 *
 * <p>1. POSITION — Through Bore encoder reaches the known stop position. Most reliable in normal
 * conditions. Set DEPLOYED_POSITION and RETRACTED_POSITION slightly inside the physical stop so
 * this triggers just before full contact.
 *
 * <p>2. VELOCITY + CURRENT (combined) — velocity drops near zero AND current spikes simultaneously.
 * Both must be true together to avoid false positives at startup (velocity is zero but current is
 * also zero before the motor has energized). This catches the stop even if the encoder drifts.
 *
 * <p>── Current Limits ─────────────────────────────────────────────────────────── Stator limit set
 * low (20A) because: - Arm intentionally stalls against a hard stop - 20A moves the arm fine but
 * won't overheat during stall - No need for the 60A Motion Magic required for profiled moves
 *
 * <p>── Tuning Order ───────────────────────────────────────────────────────────── 1. Watch
 * Arm/ThroughBoreRaw — move arm, verify it changes smoothly If backwards, set ENCODER_INVERTED =
 * true in Constants 2. Retract arm fully → read Arm/ThroughBoreRaw → paste as ENCODER_OFFSET 3.
 * Move to each stop → read Arm/ThroughBoreAdjusted → paste as DEPLOYED_POSITION /
 * RETRACTED_POSITION (set ~0.01 inside physical stop) 4. Adjust DEPLOY/RETRACT_DUTY_CYCLE until arm
 * moves at a comfortable speed 5. Watch Arm/StatorAmps while arm hits stop → set STALL_CURRENT_AMPS
 * above normal moving current but below peak stall current you observe
 */
public class IntakeArm extends SubsystemBase {

  // -----------------------------------------------------------------------
  // Hardware
  // -----------------------------------------------------------------------
  private final TalonFX pivotMotor = new TalonFX(Constants.IntakeArmConstants.PIVOT_MOTOR_ID);
  // private final DutyCycleEncoder throughBoreEncoder =
  //     new DutyCycleEncoder(Constants.IntakeArmConstants.THROUGH_BORE_DIO_PORT);

  // -----------------------------------------------------------------------
  // Control requests
  // -----------------------------------------------------------------------
  private final DutyCycleOut deployRequest =
      new DutyCycleOut(Constants.IntakeArmConstants.DEPLOY_DUTY_CYCLE).withEnableFOC(false);
  private final DutyCycleOut retractRequest =
      new DutyCycleOut(Constants.IntakeArmConstants.RETRACT_DUTY_CYCLE).withEnableFOC(false);
  private final DutyCycleOut manualRequest =
      new DutyCycleOut(0).withEnableFOC(false); // value set at call time via withOutput()
  private final NeutralOut neutralRequest = new NeutralOut();

  // -----------------------------------------------------------------------
  // State tracking
  // -----------------------------------------------------------------------
  /**
   * Tracks the current state of the intake arm for use in RobotContainer button bindings and
   * dashboard display.
   */
  public enum ArmState {
    DEPLOYED,
    RETRACTED,
    DEPLOYING,
    RETRACTING
  }

  private ArmState currentState = ArmState.RETRACTED;

  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------
  public IntakeArm() {
    configurePivotMotor();
    seedMotorFromEncoder();

    // Pre-populate dashboard entries so they appear before first command runs
    SmartDashboard.putBoolean("Arm/IsDeployed", false);
    SmartDashboard.putBoolean("Arm/IsRetracted", true);
    SmartDashboard.putBoolean("Arm/Stalled", false);
    SmartDashboard.putString("Arm/State", currentState.name());
    SmartDashboard.putNumber("Arm/PositionRot", 0.0);
    SmartDashboard.putNumber("Arm/VelocityRPS", 0.0);
    SmartDashboard.putNumber("Arm/StatorAmps", 0.0);
    SmartDashboard.putNumber("Arm/ThroughBoreRaw", 0.0);
    SmartDashboard.putNumber("Arm/ThroughBoreAdjusted", 0.0);
    SmartDashboard.putBoolean("Arm/EncoderConnected", false);
  }

  // -----------------------------------------------------------------------
  // Configuration
  // -----------------------------------------------------------------------
  private void configurePivotMotor() {
    TalonFXConfiguration intakeArmTalonFXConfiguration = new TalonFXConfiguration();

    // Low current limits — arm intentionally stalls at hard stops.
    // 20A is enough torque to move the arm but safe for brief stalls.
    intakeArmTalonFXConfiguration.CurrentLimits.StatorCurrentLimit =
        Constants.IntakeArmConstants.STATOR_CURRENT_LIMIT;
    intakeArmTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeArmTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakeArmConstants.SUPPLY_CURRENT_LIMIT;
    intakeArmTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio so velocity/position readings are in arm shaft units,
    // not motor shaft units — consistent with Through Bore encoder values
    intakeArmTalonFXConfiguration.Feedback.SensorToMechanismRatio =
        Constants.IntakeArmConstants.GEAR_RATIO;

    // Soft limits — hardware enforced position boundaries.
    // Set slightly OUTSIDE your stop positions so soft limits only trigger
    // if the primary stop detection fails entirely.
    intakeArmTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    intakeArmTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.IntakeArmConstants.SOFT_LIMIT_MAX;
    intakeArmTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    intakeArmTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.IntakeArmConstants.SOFT_LIMIT_MIN;

    // Brake mode — holds arm in place when motor is stopped.
    // Prevents back-driving under gravity when neutral.
    intakeArmTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotMotor.getConfigurator().apply(intakeArmTalonFXConfiguration);
  }

  /**
   * Seeds the TalonFX internal encoder from the Through Bore absolute position. Called once in
   * constructor. Can be re-called via a button if encoder reading drifts after a hard impact.
   */
  public void seedMotorFromEncoder() {
    // pivotMotor.setPosition(getAbsolutePositionRotations());
  }

  /**
   * Returns true when the arm has reached the deployed hard stop.
   *
   * <p>Triggers when EITHER: - Position: encoder >= DEPLOYED_POSITION - Stall: velocity near zero
   * AND current high (both required together)
   *
   * <p>Position is the primary condition. Stall is the backup if encoder drifts.
   */
  public boolean isAtDeployedStop() {
    boolean atPosition = getPositionRotations() >= Constants.IntakeArmConstants.DEPLOYED_POSITION;
    return atPosition || isStalled();
  }

  /**
   * Returns true when the arm has reached the retracted hard stop.
   *
   * <p>Same logic as isAtDeployedStop() but in the retract direction.
   */
  public boolean isAtRetractedStop() {
    boolean atPosition = getPositionRotations() <= Constants.IntakeArmConstants.RETRACTED_POSITION;
    return atPosition || isStalled();
  }

  /**
   * Returns true when the motor appears to be stalling against a hard stop.
   *
   * <p>Requires BOTH: - Velocity < STALL_VELOCITY_RPS (arm has stopped moving) - Current >
   * STALL_CURRENT_AMPS (motor is working hard against resistance)
   *
   * <p>Requiring both distinguishes a genuine stall from startup state (velocity zero but current
   * also near zero before motor energizes).
   */
  public boolean isStalled() {
    boolean velocityNearZero =
        Math.abs(getVelocityRPS()) < Constants.IntakeArmConstants.STALL_VELOCITY_RPS;
    boolean highCurrent = getStatorAmps() > Constants.IntakeArmConstants.STALL_CURRENT_AMPS;
    return velocityNearZero && highCurrent;
  }

  private void runDeploy() {
    currentState = ArmState.DEPLOYING;
    pivotMotor.setControl(deployRequest);
  }

  private void runRetract() {
    currentState = ArmState.RETRACTING;
    pivotMotor.setControl(retractRequest);
  }

  public void stop() {
    pivotMotor.setControl(neutralRequest);
  }

  /**
   * Moves the arm to the deployed position and stops when the hard stop is reached.
   *
   * <p>Self-terminating — bind with onTrue() in RobotContainer, not whileTrue(). The command
   * finishes on its own; no need to specify a stop condition externally.
   *
   * <p>Stop detection (isAtDeployedStop) triggers on EITHER: - Position: encoder >=
   * DEPLOYED_POSITION - Stall: velocity near zero AND current high simultaneously
   *
   * <p>The AND condition in isStalled() naturally prevents false triggering at startup — current is
   * near zero before the motor energizes even though velocity is also zero, so isStalled() returns
   * false until genuine stall.
   */
  public Command deployCommand() {
    return Commands.run(this::runDeploy, this)
        .until(this::isAtDeployedStop)
        .finallyDo(
            () -> {
              stop();
              currentState = ArmState.DEPLOYED;
            });
  }

  /**
   * Moves the arm to the retracted position and stops when the hard stop is reached.
   *
   * <p>Self-terminating — bind with onTrue() in RobotContainer, not whileTrue().
   */
  public Command retractCommand() {
    return Commands.run(this::runRetract, this)
        .until(this::isAtRetractedStop)
        .finallyDo(
            () -> {
              stop();
              currentState = ArmState.RETRACTED;
            });
  }

  /**
   * Moves the arm at a manually specified duty cycle. Useful for tuning DEPLOY_DUTY_CYCLE /
   * RETRACT_DUTY_CYCLE constants, or as an operator override if auto stop detection is unreliable.
   *
   * <p>Positive dutyCycle → deploy direction Negative dutyCycle → retract direction
   *
   * <p>Use with whileTrue() since this command never self-terminates: operatorController.povUp()
   * .whileTrue(intakeArm.manualCommand(0.15));
   *
   * @param dutyCycle duty cycle from -1.0 to 1.0
   */
  public Command manualCommand(double dutyCycle) {
    return Commands.run(() -> pivotMotor.setControl(manualRequest.withOutput(dutyCycle)), this)
        .finallyDo(() -> stop());
  }

  /**
   * Performs acts of witchcraft and heresy against the Omnissiah Wont work in practice, use
   * Commands.either in RobotContainer instead
   */
  public Command toggleDeploy() {
    if (currentState == ArmState.DEPLOYED || currentState == ArmState.DEPLOYING) {
      return retractCommand();
    }
    return deployCommand();
  }

  /** Motor velocity in arm shaft RPS (gear ratio applied). */
  public double getVelocityRPS() {
    return pivotMotor.getVelocity().getValueAsDouble();
  }

  /** Motor position in arm shaft rotations (gear ratio applied). */
  public double getPositionRotations() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  /** Motor stator current in amps. */
  public double getStatorAmps() {
    return pivotMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Absolute arm shaft position from the Through Bore encoder in rotations. Applies offset and
   * inversion from Constants. 0.0 = fully retracted.
   */
  // public double getAbsolutePositionRotations() {
  //   double raw = throughBoreEncoder.get();

  //   if (Constants.IntakeArmConstants.ENCODER_INVERTED) raw = 1.0 - raw;

  //   double adjusted = raw - Constants.IntakeArmConstants.ENCODER_OFFSET;

  //   // Normalize to handle wrap-around at the 0/1 boundary
  //   while (adjusted < -0.5) adjusted += 1.0;
  //   while (adjusted > 0.5) adjusted -= 1.0;

  //   return adjusted;
  // }

  // /** Returns true if the encoder is connected and reporting a valid value. */
  // public boolean isEncoderConnected() {
  //   return throughBoreEncoder.isConnected();
  // }

  /**
   * Returns the current state of the intake arm (DEPLOYED, RETRACTED, DEPLOYING, or RETRACTING).
   */
  public ArmState getCurrentState() {
    return currentState;
  }

  /** Returns true if the arm is currently in the DEPLOYED state. */
  public boolean isDeployed() {
    return currentState == ArmState.DEPLOYED;
  }

  /** Returns true if the arm is currently in the RETRACTED state. */
  public boolean isRetracted() {
    return currentState == ArmState.RETRACTED;
  }

  // -----------------------------------------------------------------------
  // Periodic
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm/IsDeployed", isDeployed());
    SmartDashboard.putBoolean("Arm/IsRetracted", isRetracted());
    SmartDashboard.putBoolean("Arm/Stalled", isStalled());
    SmartDashboard.putString("Arm/State", currentState.name());
    SmartDashboard.putNumber("Arm/PositionRot", getPositionRotations());
    SmartDashboard.putNumber("Arm/VelocityRPS", getVelocityRPS());
    SmartDashboard.putNumber("Arm/StatorAmps", getStatorAmps());
    // SmartDashboard.putNumber("Arm/ThroughBoreRaw", throughBoreEncoder.get());
    // SmartDashboard.putNumber("Arm/ThroughBoreAdjusted", getAbsolutePositionRotations());
    // SmartDashboard.putBoolean("Arm/EncoderConnected", isEncoderConnected());
  }
}

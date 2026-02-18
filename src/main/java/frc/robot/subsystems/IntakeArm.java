package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * IntakeArmSubsystem
 *
 * <p>MotionMagic position control for the intake arm pivot. REV Through Bore Encoder on a RoboRIO
 * DIO port provides absolute position. The TalonFX internal encoder is seeded from the Through Bore
 * on startup, then used for closed-loop — no drift, no homing sequence.
 *
 * <p>CURRENT LIMITS: Stator: 60A — arm needs meaningful torque to move and hold against gravity,
 * but 60A prevents sustained overheating if the arm is stalled against a hard stop (soft limits
 * should prevent this anyway) Supply: 40A — sized above the stator limit to give headroom during
 * fast moves
 *
 * <p>POSITION UNITS: mechanism rotations of the ARM SHAFT (not motor shaft). SensorToMechanismRatio
 * in the config divides out the gearbox automatically. With a Through Bore on the arm shaft
 * directly, values are small fractions of 1.0 — expect your full range to be something like 0.0 to
 * 0.30.
 *
 * <p>TUNING ORDER: 1. Watch Arm/ThroughBoreRaw — move arm, verify it changes 0.0–1.0 If backwards,
 * set ENCODER_INVERTED = true in Constants 2. Stow arm, read Arm/ThroughBoreRaw, paste as
 * ENCODER_OFFSET 3. Move arm to DEPLOY and GROUND positions, read Arm/ThroughBoreAdjusted, paste
 * those values into Constants as DEPLOY_ROTATIONS / GROUND_ROTATIONS 4. Set MIN/MAX soft limits
 * with ~0.02 rot margin inside physical hard stops 5. Tune kG: hold arm horizontal, raise until it
 * holds without drifting 6. Tune MotionMagic cruise/accel/jerk: start conservative, speed up 7.
 * Tune kP: raise until position error is small without oscillation
 */
public class IntakeArm extends SubsystemBase {

  // -----------------------------------------------------------------------
  // Hardware
  // -----------------------------------------------------------------------
  private final TalonFX m_IntakeArm = new TalonFX(Constants.IntakeArm.M_Intake_Arm_ID);
  private final DutyCycleEncoder throughBoreEncoder =
      new DutyCycleEncoder(Constants.IntakeArm.THROUGH_BORE_DIO_PORT);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0) /*.withEnableFOC(true)*/;

  // -----------------------------------------------------------------------
  // Arm position presets
  // -----------------------------------------------------------------------
  public enum ArmPosition {
    STOW(Constants.IntakeArm.STOW_ROTATIONS),
    DEPLOY(Constants.IntakeArm.DEPLOY_ROTATIONS),
    GROUND(Constants.IntakeArm.GROUND_ROTATIONS);

    public final double rotations;

    ArmPosition(double r) {
      this.rotations = r;
    }
  }

  private ArmPosition currentTarget = ArmPosition.STOW;

  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------
  public IntakeArm() {
    configurePivotMotor();
    seedMotorFromEncoder();
  }

  // -----------------------------------------------------------------------
  // Configuration
  // -----------------------------------------------------------------------
  private void configurePivotMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // --- Slot 0: MotionMagic PID + Gravity Feedforward ---
    cfg.Slot0.kP = Constants.IntakeArm.kP;
    cfg.Slot0.kI = Constants.IntakeArm.kI;
    cfg.Slot0.kD = Constants.IntakeArm.kD;
    cfg.Slot0.kS = Constants.IntakeArm.kS;
    cfg.Slot0.kG = Constants.IntakeArm.kG;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // --- MotionMagic Profile ---
    cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeArm.MM_CRUISE_VEL;
    cfg.MotionMagic.MotionMagicAcceleration = Constants.IntakeArm.MM_ACCELERATION;
    cfg.MotionMagic.MotionMagicJerk = Constants.IntakeArm.MM_JERK;

    // --- Feedback: TalonFX internal encoder, seeded from Through Bore ---
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    cfg.Feedback.SensorToMechanismRatio = Constants.IntakeArm.GEAR_RATIO;

    // --- Current Limits ---
    // Higher than kicker/hopper because the arm needs real torque
    cfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeArm.STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeArm.SUPPLY_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // --- Soft Limits ---
    // VERIFY these on the real robot before running closed-loop!
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.IntakeArm.MAX_ROTATIONS;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.IntakeArm.MIN_ROTATIONS;

    // Brake: holds position when idle, resists gravity
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_IntakeArm.getConfigurator().apply(cfg);
  }

  // -----------------------------------------------------------------------
  // Encoder seeding
  // -----------------------------------------------------------------------

  /**
   * Seeds the TalonFX internal encoder from the Through Bore absolute position. Called once in the
   * constructor. Can also be bound to a button for field-side re-zeroing after a hard impact.
   */
  public void seedMotorFromEncoder() {
    m_IntakeArm.setPosition(getAbsolutePositionRotations());
  }

  // -----------------------------------------------------------------------
  // Public API
  // -----------------------------------------------------------------------

  /**
   * Move the arm to a preset position using MotionMagic. Generates a smooth S-curve profile
   * automatically.
   */
  public void setPosition(ArmPosition position) {
    currentTarget = position;
    m_IntakeArm.setControl(motionMagicRequest.withPosition(position.rotations));
  }

  /**
   * True when arm is within tolerance of its target. Use this in command sequences to detect when
   * motion is complete.
   */
  public boolean atSetpoint() {
    return Math.abs(m_IntakeArm.getClosedLoopError().getValueAsDouble())
        < Constants.IntakeArm.TOLERANCE_ROT;
  }

  /** Arm position in mechanism rotations (what MotionMagic uses). */
  public double getPositionRotations() {
    return m_IntakeArm.getPosition().getValueAsDouble();
  }

  /**
   * Absolute position from the Through Bore in mechanism rotations, with offset and inversion
   * applied. 0.0 = stow.
   */
  public double getAbsolutePositionRotations() {
    double raw = throughBoreEncoder.get(); // 0.0–1.0 over one rotation of the arm shaft

    if (Constants.IntakeArm.ENCODER_INVERTED) raw = 1.0 - raw;

    double adjusted = raw - Constants.IntakeArm.ENCODER_OFFSET;

    // Normalize around 0 to handle wrap-around at the 0/1 boundary
    while (adjusted < -0.5) adjusted += 1.0;
    while (adjusted > 0.5) adjusted -= 1.0;

    return adjusted;
  }

  public boolean isEncoderConnected() {
    return throughBoreEncoder.isConnected();
  }

  public ArmPosition getCurrentTarget() {
    return currentTarget;
  }

  public void stop() {
    m_IntakeArm.stopMotor();
  }

  // -----------------------------------------------------------------------
  // Periodic
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/PositionRot", getPositionRotations());
    SmartDashboard.putNumber("Arm/TargetRot", currentTarget.rotations);
    SmartDashboard.putNumber(
        "Arm/ClosedLoopError", m_IntakeArm.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putBoolean("Arm/AtSetpoint", atSetpoint());
    SmartDashboard.putString("Arm/Target", currentTarget.name());
    SmartDashboard.putNumber("Arm/ThroughBoreRaw", throughBoreEncoder.get());
    SmartDashboard.putNumber("Arm/ThroughBoreAdjusted", getAbsolutePositionRotations());
    SmartDashboard.putBoolean("Arm/EncoderConnected", isEncoderConnected());
    SmartDashboard.putNumber("Arm/StatorAmps", m_IntakeArm.getStatorCurrent().getValueAsDouble());
  }
}

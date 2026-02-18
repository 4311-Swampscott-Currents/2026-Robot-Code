// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // -----------------------------------------------------------------------
  // Shooter
  // -----------------------------------------------------------------------
  public static final class Shooter {
    public static final int M_RIGHT_SHOOTER_ID = 10;
    public static final int M_LEFT_SHOOTER_ID = 11;

    // Velocity PID (Phoenix 6 Slot 0) — units: rot/sec
    // kV is the most critical gain. Characterize first via SysId or manual sweep.
    public static final double kP = 0.30;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kS = 0.05; // static friction (volts)
    public static final double kV = 0.12; // volts per rot/sec — TUNE THIS

    // "At speed" tolerance (~50 RPM expressed in rot/sec)
    public static final double TOLERANCE_RPS = 50.0 / 60.0;

    // Current limits
    // Flywheels can draw high current during spin-up; 80A stator keeps them safe
    // during sustained operation. Supply limit protects the 60A PDH breaker.
    public static final double STATOR_CURRENT_LIMIT = 80.0; // amps
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // amps
  }

  // -----------------------------------------------------------------------
  // Kicker
  // -----------------------------------------------------------------------
  public static final class Kicker {
    public static final int M_KICKER_ID = 12;
    public static final double KICK_PERCENT = 1.0;

    // Current limits
    // Kicker runs in short bursts — 40A stator is plenty, 30A supply
    // keeps draw reasonable alongside the shooter motors
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
  }

  // -----------------------------------------------------------------------
  // Hopper / Conveyor
  // -----------------------------------------------------------------------
  public static final class Hopper {
    public static final int M_HOPPER_ID = 13;
    public static final double FORWARD_PERCENT = 0.6;
    public static final double REVERSE_PERCENT = -0.4;

    // Current limits
    // Light conveyor load, but may spike during ball jams
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
  }

  // -----------------------------------------------------------------------
  // Intake Arm (pivot)
  // -----------------------------------------------------------------------
  public static final class IntakeArm {
    public static final int M_Intake_Arm_ID = 20;

    // REV Through Bore Encoder → RoboRIO DIO port
    public static final int THROUGH_BORE_DIO_PORT = 0; // change to match your wiring

    // Set ENCODER_INVERTED = true if the encoder reads backwards
    // (decreases when arm deploys toward ground)
    public static final boolean ENCODER_INVERTED = false;

    // Offset: the raw Through Bore value when the arm is fully stowed.
    // Stow arm physically → read Arm/ThroughBoreRaw → paste here.
    public static final double ENCODER_OFFSET = 0.0; // TUNE!

    // Mechanism gear ratio: motor rotations per 1 arm shaft rotation
    public static final double GEAR_RATIO = 60.0;

    // Arm setpoint positions in ARM SHAFT rotations (not motor rotations).
    // Move arm to each position → read Arm/ThroughBoreAdjusted → paste here.
    // Expect small numbers (0.0–0.35 range for most FRC arms).
    public static final double STOW_ROTATIONS = 0.0;
    public static final double DEPLOY_ROTATIONS = 0.18; // TUNE!
    public static final double GROUND_ROTATIONS = 0.27; // TUNE!

    // Soft limits: set ~0.02 rot INSIDE the physical hard stops
    public static final double MIN_ROTATIONS = -0.02; // TUNE!
    public static final double MAX_ROTATIONS = 0.30; // TUNE!

    // MotionMagic profile — start conservative, increase until smooth
    public static final double MM_CRUISE_VEL = 80.0; // rot/s
    public static final double MM_ACCELERATION = 160.0; // rot/s^2
    public static final double MM_JERK = 1600.0; // rot/s^3

    // PID Slot 0
    public static final double kP = 2.4;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kG = 0.30; // TUNE FIRST — gravity comp

    // At-setpoint tolerance in mechanism rotations
    public static final double TOLERANCE_ROT = 0.02; // tighter now that units are right

    // Current limits
    // Arm needs real torque to move + hold against gravity; 60A stator
    // gives headroom without risking motor damage if stalled against soft limit
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  }

  // -----------------------------------------------------------------------
  // Intake Roller
  // -----------------------------------------------------------------------
  public static final class IntakeRoller {
    public static final int M_Intake_Roller_ID = 22;
    public static final double INTAKE_PERCENT = 0.8;
    public static final double EJECT_PERCENT = -0.5;

    // Current limits
    // Roller is lightly loaded but can spike on ball contact
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
  }

  // -----------------------------------------------------------------------
  // Operator Interface
  // -----------------------------------------------------------------------
  public static final class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double STICK_DEADBAND = 0.1;
  }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.TreeMap;

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

  // canBus's
  public static class CANBus {
    // public static final String RIO_CANBUS = "";
    // public static final CANBus Rio_CANBUS = new CANBus("rio", CANBus.roboRIO("rio"));
    public static final String DRIVETRAIN_CANBUS = "Default Name";
  }

  // -----------------------------------------------------------------------
  // Shooter
  // -----------------------------------------------------------------------
  public static final class ShooterConstants {
    public static final int M_RIGHT_SHOOTER_ID = 10;
    public static final int M_LEFT_SHOOTER_ID = 11;

    // Velocity PID (Phoenix 6 Slot 0) — units: rot/sec
    // kV is the most critical gain. Characterize first via SysId or manual sweep.
    public static final double kP = 0.50;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kS = 0.2; // static friction (volts)
    public static final double kV = 0.12643; // volts per rot/sec — TUNE THIS

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
  public static final class KickerConstants {
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
  public static final class HopperConstants {
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
  public static final class IntakeArmConstants {
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
  public static final class IntakeRollerConstants {
    public static final int M_Intake_Roller_ID = 22;
    public static final double INTAKE_PERCENT = 0.8;
    public static final double EJECT_PERCENT = -0.5;

    // Current limits
    // Roller is lightly loaded but can spike on ball contact
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
  }

  // -----------------------------------------------------------------------
  // Vision
  // -----------------------------------------------------------------------
  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";

    // Camera physical properties — measure on the real robot
    public static final double CAMERA_HEIGHT_METERS =
        0.4; // meters off ground *needs to be updated!
    public static final double CAMERA_PITCH_DEGREES = 15.0; // upward tilt — MEASURE!

    // Height of the scoring target center in meters off the ground.
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(44.25);

    public static final double HUB_HALF_DEPTH_METERS = 1.194 / 2.0; // 0.597m

    // ── Alliance-specific AprilTag IDs ───────────────────────────────
    // Update these from the 2026 field layout in the game manual.
    // Each alliance's scoring target has a unique tag ID.
    public static final int RED_ALLIANCE_TARGET_TAG_ID = 10; // double-check id
    public static final int BLUE_ALLIANCE_TARGET_TAG_ID = 26; // double-check id

    // ── Alliance-specific target field positions ─────────────────────
    // WPILib field coordinate system: origin = blue alliance corner.
    // x = toward red alliance wall, y = toward blue alliance left wall.
    // Update these from the 2026 game manual field layout.
    public static final Translation2d RED_TARGET_FIELD_POSITION =
        new Translation2d(11.915, 4.035); // double check
    public static final Translation2d BLUE_TARGET_FIELD_POSITION =
        new Translation2d(4.625, 4.035); // double check

    public static final int[] ALL_TAG_IDS = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
      27, 28, 29, 30, 31, 32
    };

    // Kalman filter standard deviations for MegaTag 2 vision updates
    // in Drive.updateOdometry(). Lower = trust vision more.
    // Tune by watching pose on AdvantageScope — if it jumps, increase these.
    // public static final double XY_STD_DEV_BASE  = 0.5;  // meters
    // public static final double ROT_STD_DEV_BASE = 0.9;  // radians
  }

  // -----------------------------------------------------------------------
  // Auto Aim
  // -----------------------------------------------------------------------
  public static final class AutoAim {

    // Rotation ProfiledPID gains
    // Input: tx in degrees. Output: degrees/sec.
    // Start with kP = 0.05, increase until robot aims without oscillating.
    // Not needed since using Drive's built-in heading lock, but you could use these to add a
    // separate rotation command if you wanted.
    public static final double ROTATION_kP = 0.05;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.004;

    // Motion profile constraints for rotation
    // Not needed since using Drive's built-in heading lock, but you could use these to add a
    // separate rotation command if you wanted.
    public static final double MAX_ROTATION_VEL_DEG_PER_SEC = 360.0; // deg/s
    public static final double MAX_ROTATION_ACCEL_DEG_PER_SEC2 = 720.0; // deg/s^2

    // How close tx must be to 0 for isHeadingLocked() to return true
    // Not needed since using Drive's built-in heading lock, but you could use these to add a
    // separate rotation command if you wanted.
    public static final double HEADING_TOLERANCE_DEGREES = 1.5;

    // Max translation speed passed through from the driver during auto-aim
    // Set this to your drive subsystem's max speed (typically 4.5–5.0 m/s)
    // Not needed since using Drive's built-in heading lock, but you could use these to add a
    // separate rotation command if you wanted.
    public static final double MAX_DRIVE_SPEED_METERS_PER_SEC = 4.5;

    // Default flywheel speed if shot map is empty or distance is invalid
    public static final double DEFAULT_RPS = 70.0;

    // Distance (meters) → flywheel speed (rot/sec) lookup table.
    // Measure real shot data on the field and fill this in.
    // Format: put(distanceMeters, rotationsPerSecond)
    // Needs to be filled in with real data...
    public static final TreeMap<Double, Double> SHOT_MAP = new TreeMap<>();

    static {
      SHOT_MAP.put(1.5, 55.0);
      SHOT_MAP.put(2.0, 63.0);
      SHOT_MAP.put(2.5, 70.0);
      SHOT_MAP.put(3.0, 78.0);
      SHOT_MAP.put(3.5, 85.0);
      SHOT_MAP.put(4.0, 92.0);
    }
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

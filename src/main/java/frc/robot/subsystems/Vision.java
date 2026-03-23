package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem
 *
 * <p>Lightweight read-only wrapper around LimelightHelpers. Provides target detection,
 * angle-to-target, and distance for AutoAimCommand.
 *
 * <p>AIMING STRATEGY — pose based: Instead of using tx (camera pixel angle), we compute the
 * field-relative angle from the robot's MegaTag 2 pose to the known target position:
 *
 * <p>angleToTarget = atan2(target.y - robot.y, target.x - robot.x)
 *
 * <p>This is more accurate than tx because: - No camera offset error (angle is from robot center,
 * not camera) - Works even if the tag briefly leaves frame mid-rotation - Distance and angle come
 * from the same pose source — always consistent - More tags visible = better MegaTag 2 accuracy, so
 * no filtering needed
 *
 * <p>ALLIANCE AWARENESS: hasTarget(), getAngleToTarget(), and getDistanceMeters() all use the
 * correct alliance's tag ID and field position automatically. The Limelight sees all tags freely —
 * MegaTag 2 uses all of them for the most accurate pose estimate possible.
 *
 * <p>POSE FUSION: MegaTag 2 addVisionMeasurement() lives in Drive.updateOdometry(), not here. Drive
 * owns the pose estimator and gyro it depends on.
 *
 * <p>METHOD REFERENCE (LimelightHelpers v1.14): LimelightHelpers.getTV(name) → boolean, true if any
 * target visible LimelightHelpers.getTX(name) → double, horizontal angle error (degrees)
 * LimelightHelpers.getTY(name) → double, vertical angle error (degrees)
 * LimelightHelpers.getTA(name) → double, target area (0–100%) LimelightHelpers.getFiducialID(name)
 * → double (cast to int), tracked tag ID
 */
public class Vision extends SubsystemBase {

  private final Drive drive;

  public Vision(Drive drive) {
    this.drive = drive;
    String limelightURL = "http://10.43.11.11:5800/stream.mjpg";
    SmartDashboard.putString("Limelight Stream", limelightURL);
  }

  // -----------------------------------------------------------------------
  // Alliance helpers
  // -----------------------------------------------------------------------

  /**
   * Returns the AprilTag ID of the scoring target for the current alliance. Defaults to blue if
   * Driver Station connection isn't established yet — safe since alliance is always known before a
   * real match starts.
   */
  public int getTargetTagID() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return Constants.VisionConstants.RED_ALLIANCE_TARGET_TAG_ID;
    }
    return Constants.VisionConstants.BLUE_ALLIANCE_TARGET_TAG_ID;
  }

  /**
   * Returns the field-relative position of the scoring target for the current alliance. Used for
   * both angle and distance calculations.
   */
  private Translation2d getTargetFieldPosition() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return Constants.VisionConstants.RED_TARGET_FIELD_POSITION;
    }
    return Constants.VisionConstants.BLUE_TARGET_FIELD_POSITION;
  }

  // -----------------------------------------------------------------------
  // Tag filter — used by TX strategy only
  // -----------------------------------------------------------------------

  /**
   * Restricts the Limelight to only track the current alliance's target tag. Called by
   * AutoAimCommand.buildTX() on activation.
   *
   * <p>Without this, tx is pulled toward whichever tag the Limelight happens to select when
   * multiple tags are visible, causing inaccurate aiming.
   */
  public void enableTargetTagFilter() {
    LimelightHelpers.SetFiducialIDFiltersOverride(
        Constants.VisionConstants.LIMELIGHT_NAME, new int[] {getTargetTagID()});
  }

  /**
   * Restores full tag visibility so MegaTag 2 can use all field tags for the most accurate pose
   * estimate. Called by AutoAimCommand.buildTX() on deactivation.
   */
  public void disableTargetTagFilter() {
    LimelightHelpers.SetFiducialIDFiltersOverride(
        Constants.VisionConstants.LIMELIGHT_NAME, Constants.VisionConstants.ALL_TAG_IDS);
  }

  // -----------------------------------------------------------------------
  // Target detection
  // -----------------------------------------------------------------------

  /**
   * Returns true when the Limelight sees the current alliance's target tag.
   *
   * <p>Checks both: 1. tv = 1 (Limelight has a valid target) 2. Tracked tag ID matches the alliance
   * target ID
   *
   * <p>AutoAimCommand gates all aiming logic behind this. If false, the robot holds its current
   * heading rather than rotating blindly.
   */
  public boolean hasTarget() {
    if (!LimelightHelpers.getTV(Constants.VisionConstants.LIMELIGHT_NAME)) return false;
    int trackedID = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_NAME);
    return trackedID == getTargetTagID();
  }

  /** Returns true when the robot is inside the alliance zone and can shoot. */
  public boolean inAllianceZone() {
    var alliance = DriverStation.getAlliance();
    var blue = DriverStation.Alliance.Blue;
    var red = DriverStation.Alliance.Red;
    // checks if connected to driver station
    if (alliance.isEmpty()) return false;
    if (alliance.get() == blue
        && drive.getPose().getX() < Constants.VisionConstants.BLUE_ALLIANCE_ZONE_X) {
      return true;
    }
    if (alliance.get() == red
        && drive.getPose().getX() > Constants.VisionConstants.RED_ALLIANCE_ZONE_X) {
      return true;
    }
    return false; // nutmeg
  }

  /**
   * Returns the horizontal angle error to the primary target in degrees. Kept for logging and
   * diagnostics — aiming uses getAngleToTarget() instead.
   */
  public double getTX() {
    return LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Returns the vertical angle error to the primary target in degrees. Used only for the ty-based
   * distance fallback.
   */
  public double getTY() {
    return LimelightHelpers.getTY(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Returns the ID of the tag currently being tracked. Logged for diagnostics — verify this matches
   * getTargetTagID() during testing.
   */
  public int getTrackedTagID() {
    return (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  // -----------------------------------------------------------------------
  // Pose-based angle to target
  // -----------------------------------------------------------------------

  /**
   * Returns the field-relative angle the robot needs to face in order to point directly at the
   * alliance's scoring target.
   *
   * <p>Computed as: atan2(target.y - robot.y, target.x - robot.x)
   *
   * <p>This is the value passed directly to joystickDriveAtAngle's rotation supplier — no tx, no
   * camera offset correction needed.
   *
   * <p>If no target has been seen and the pose is untrustworthy, returns the robot's current
   * heading so it holds direction rather than spinning.
   *
   * @return Rotation2d — absolute field angle pointing from robot to target
   */
  public Rotation2d getAngleToTarget() {

    Pose2d robotPose = drive.getPose();

    // If pose is unavailable or robot is not in alliance zone
    if (robotPose == null || !inAllianceZone()) {
      return drive.getRotation();
    }

    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d targetTranslation = getTargetFieldPosition();

    // getAngle() on a Translation2d returns the direction of that vector
    // relative to the positive x-axis — exactly the field-relative heading
    // the robot needs to face
    return targetTranslation.minus(robotTranslation).getAngle();
  }

  // -----------------------------------------------------------------------
  // Angle to target — TX BASED (fallback)
  // -----------------------------------------------------------------------

  /**
   * Returns the field-relative angle the robot needs to face in order to aim at the CENTER of the
   * HUB, using the Limelight's raw tx as a base.
   *
   * <p>WHY AN OFFSET IS NEEDED: tx = 0 means the tag face is centered in the camera, but the HUB
   * opening is behind the tag face by half the HUB depth (0.597m). Aiming purely at tx = 0 would
   * aim at the tag face, not the opening.
   *
   * <p>OFFSET CALCULATION: The HUB center is HUB_HALF_DEPTH_METERS (0.597m) behind the tag. At
   * shooting distance d, the angular correction needed is:
   *
   * <p>offsetDegrees = atan(HUB_HALF_DEPTH / distance)
   *
   * <p>This offset shrinks as distance increases: At 1.5m → ~21.7° At 3.0m → ~11.3° At 2.0m →
   * ~16.6° At 4.0m → ~8.5°
   *
   * <p>If distance is unavailable (<0.1m), no offset is applied and the robot aims directly at the
   * tag face as a safe fallback.
   *
   * <p>SIGN CONVENTION: The offset is added to tx (positive = rotate further in the same direction
   * tx is already pulling). This assumes the robot approaches the HUB roughly head-on. At extreme
   * approach angles the simple atan model is slightly off, but acceptable for a fallback strategy.
   *
   * <p>IMPORTANT: Requires the tag filter to be active via enableTargetTagFilter() so tx only
   * reflects the alliance target tag. AutoAimCommand.buildTX() manages the filter lifecycle
   * automatically.
   *
   * @return Rotation2d — desired field heading aimed at the HUB center
   */
  public Rotation2d getAngleToTargetTX() {
    if (!hasTarget()) {
      return drive.getRotation();
    }

    double tx = getTX();

    // Compute angular offset to account for HUB center being behind the tag.
    // Uses current distance estimate — falls back to 0 if distance unknown.
    double offsetDegrees = 0.0;
    double distance = getDistanceMeters();
    if (distance > 0.1) {
      offsetDegrees =
          Math.toDegrees(Math.atan(Constants.VisionConstants.HUB_HALF_DEPTH_METERS / distance));
    }

    // Apply tx + depth offset to current heading.
    // tx pulls toward the tag; offset nudges further to the HUB center.
    return drive.getRotation().plus(Rotation2d.fromDegrees(tx + offsetDegrees));
  }

  // -----------------------------------------------------------------------
  // Distance estimation
  // -----------------------------------------------------------------------

  /**
   * Returns estimated distance from the robot to the alliance target in meters.
   *
   * <p>PRIMARY — pose based: Uses drive.getPose() (refined by MegaTag 2) and the known field
   * position of the alliance target. Most accurate option.
   *
   * <p>FALLBACK — ty geometry: distance = (targetHeight - cameraHeight) / tan(cameraPitch + ty)
   * Used only if pose is unavailable.
   *
   * <p>Returns 0.0 if distance cannot be determined.
   */
  public double getDistanceMeters() {

    // Primary: field pose distance
    Pose2d robotPose = drive.getPose();
    if (robotPose != null) {
      return robotPose.getTranslation().getDistance(getTargetFieldPosition());
    }

    if (!hasTarget()) return 0.0;

    // Fallback: ty-based geometry
    double ty = getTY();
    double angleRadians = Math.toRadians(Constants.VisionConstants.CAMERA_PITCH_DEGREES + ty);
    if (Math.abs(angleRadians) < 1e-6) return 0.0;

    return (Constants.VisionConstants.TARGET_HEIGHT_METERS
            - Constants.VisionConstants.CAMERA_HEIGHT_METERS)
        / Math.tan(angleRadians);
  }

  // -----------------------------------------------------------------------
  // Periodic
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    double distance = getDistanceMeters();
    double txOffsetDegrees =
        (distance > 0.1)
            ? Math.toDegrees(Math.atan(Constants.VisionConstants.HUB_HALF_DEPTH_METERS / distance))
            : 0.0; // if else statement

    // Update SmartDashboard with vision status
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());
    SmartDashboard.putNumber("Vision/DistanceMeters", getDistanceMeters());
    SmartDashboard.putNumber("Vision/TrackedTagID", getTrackedTagID());
    SmartDashboard.putNumber("Vision/ExpectedTagID", getTargetTagID());
    SmartDashboard.putNumber("Vision/AngleToTargetDeg", getAngleToTarget().getDegrees());
    SmartDashboard.putNumber("Vision/AngleTXDeg", getAngleToTargetTX().getDegrees());
    SmartDashboard.putNumber(
        "Vision/TXHubOffsetDeg", txOffsetDegrees); // how much TX is being corrected
    SmartDashboard.putBoolean("Vision/InsideAllianceZone", inAllianceZone());

    // Log the same data to the data logger for offline analysis
    Logger.recordOutput("Vision/HasTarget", hasTarget());
    Logger.recordOutput("Vision/TX", getTX());
    Logger.recordOutput("Vision/TY", getTY());
    Logger.recordOutput("Vision/DistanceMeters", getDistanceMeters());
    Logger.recordOutput("Vision/TrackedTagID", getTrackedTagID());
    Logger.recordOutput("Vision/ExpectedTagID", getTargetTagID());
    Logger.recordOutput("Vision/AngleToTargetDeg", getAngleToTarget().getDegrees());
    Logger.recordOutput("Vision/AngleTXDeg", getAngleToTargetTX().getDegrees());
    Logger.recordOutput("Vision/TXHubOffsetDeg", txOffsetDegrees);
    Logger.recordOutput("Vision/InsideAllianceZone", inAllianceZone());
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class AutoAimCommands {

  private AutoAimCommands() {}

  /**
   * Builds the pose-based auto-aim command. Uses vision.getAngleToTarget() — no tag filtering
   * required.
   *
   * @param drive ADK Drive subsystem
   * @param vision VisionSubsystem
   * @param shooter ShooterSubsystem
   * @param xSupplier Driver left stick Y (negate for FRC convention)
   * @param ySupplier Driver left stick X (negate for FRC convention)
   */
  public static Command autoAimPoseBased(
      Drive drive,
      Vision vision,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    Command headingLock =
        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, vision::getAngleToTarget);

    return Commands.parallel(headingLock, buildFlywheelPrep(vision, shooter))
        .beforeStarting(() -> SmartDashboard.putString("AutoAim/ActiveStrategy", "POSE"));
  }

  /**
   * Builds the tx-based auto-aim command. Uses vision.getAngleToTargetTX() — manages tag filter
   * lifecycle.
   *
   * <p>Tag filter lifecycle: Activate → vision.enableTargetTagFilter() (alliance tag only) Release
   * → vision.disableTargetTagFilter() (all tags restored)
   *
   * @param drive ADK Drive subsystem
   * @param vision VisionSubsystem
   * @param shooter ShooterSubsystem
   * @param xSupplier Driver left stick Y (negate for FRC convention)
   * @param ySupplier Driver left stick X (negate for FRC convention)
   */
  public static Command autoAimTXbased(
      Drive drive,
      Vision vision,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    Command headingLock =
        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, vision::getAngleToTargetTX);

    return Commands.sequence(
            // On activate: filter Limelight to alliance target tag only
            // so tx reflects only the correct tag, not an adjacent one
            Commands.runOnce(
                () -> {
                  vision.enableTargetTagFilter();
                  SmartDashboard.putString("AutoAim/ActiveStrategy", "TX");
                }),
            Commands.parallel(headingLock, buildFlywheelPrep(vision, shooter))

            // On release or interrupt: restore full tag visibility so
            // MegaTag 2 resumes using all field tags for pose accuracy
            )
        .finallyDo(() -> vision.disableTargetTagFilter());
  }

  /**
   * Reads distance every loop, interpolates RPS from SHOT_MAP, commands shooter. Logs both angle
   * methods to dashboard so you can compare them side by side. finallyDo stops the shooter cleanly
   * on button release.
   */
  private static Command buildFlywheelPrep(Vision vision, Shooter shooter) {
    return Commands.run(
            () -> {
              double distance = vision.getDistanceMeters();
              if (distance > 0.1) {
                shooter.setVelocity(interpolateRPS(distance));
              }
              SmartDashboard.putNumber("AutoAim/DistanceMeters", distance);
              SmartDashboard.putBoolean("AutoAim/HasTarget", vision.hasTarget());
              SmartDashboard.putBoolean("AutoAim/FlywheelsReady", shooter.atSetpoint());
              SmartDashboard.putNumber("AutoAim/TX", vision.getTX());
              SmartDashboard.putNumber(
                  "AutoAim/PoseAngleDeg", vision.getAngleToTarget().getDegrees());
              SmartDashboard.putNumber(
                  "AutoAim/TXAngleDeg", vision.getAngleToTargetTX().getDegrees());
            },
            shooter)
        .finallyDo(() -> shooter.stop());
  }

  /**
   * Linearly interpolates flywheel RPS from SHOT_MAP for a given distance. Clamps to nearest entry
   * if distance is outside the mapped range.
   */
  private static double interpolateRPS(double distanceMeters) {
    var map = Constants.AutoAim.SHOT_MAP;

    if (map.isEmpty()) return Constants.AutoAim.DEFAULT_RPS;
    if (distanceMeters <= map.firstKey()) return map.firstEntry().getValue();
    if (distanceMeters >= map.lastKey()) return map.lastEntry().getValue();

    double lowerDist = map.floorKey(distanceMeters);
    double upperDist = map.ceilingKey(distanceMeters);
    if (lowerDist == upperDist) return map.get(lowerDist);

    double t = (distanceMeters - lowerDist) / (upperDist - lowerDist);
    return map.get(lowerDist) + t * (map.get(upperDist) - map.get(lowerDist));
  }
}

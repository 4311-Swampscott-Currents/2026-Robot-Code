// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAimCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final IntakeArm intakeArm;
  private final IntakeRoller intakeRoller;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Vision vision;
  // private static Pose2d robotPose;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    // Instantiate subsystems
    shooter = new Shooter();
    intakeArm = new IntakeArm();
    intakeRoller = new IntakeRoller();
    hopper = new Hopper();
    kicker = new Kicker();
    vision = new Vision(drive);

    // pathplanner commands
    NamedCommands.registerCommand("Deploy Intake", intakeArm.deployCommand());
    NamedCommands.registerCommand("Retract Intake", intakeArm.retractCommand());
    // NamedCommands.registerCommand(
    //     "Intake Balls", Commands.runOnce(() -> intakeRoller.intake(), intakeRoller));
    // NamedCommands.registerCommand(
    //     "Stop Intaking", Commands.runOnce(() -> intakeRoller.stop(), intakeRoller));
    NamedCommands.registerCommand("Spin Shooter", shooter.shootWithRPS(vision));
    NamedCommands.registerCommand("Stop Shooting", Commands.runOnce(() -> shooter.stop(), shooter));
    NamedCommands.registerCommand(
        "Spin Kicker When Shooting", Commands.waitUntil(shooter::atSetpoint).andThen(kicker::kick));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Auto-aim and shoot while holding left bumper
    controller
        .leftBumper()
        .whileTrue(
            AutoAimCommands.autoAimPoseBased(
                drive, vision, shooter, () -> -controller.getLeftY(), () -> -controller.getLeftX())
            /* .alongWith(shooter.shootWithRPS(vision))*/ );

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Shoot while holding right bumper by enabling the kicker and hopper, shooter needs to already
    // be running
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                    Commands.run(() -> kicker.kick(), kicker),
                    Commands.run(() -> hopper.runForward(), hopper))
                .finallyDo(
                    () -> {
                      kicker.stop();
                      hopper.stop();
                    }));

    // Un jam, run everything in reverse
    controller
        .povLeft()
        .whileTrue(
            Commands.parallel(
                    Commands.run(() -> kicker.runPercent(-1), kicker),
                    Commands.run(() -> hopper.runReverse(), hopper))
                .finallyDo(
                    () -> {
                      kicker.stop();
                      hopper.stop();
                    }));

    // Move intake arm either deploy or retract when left trigger is pressed, depending on current
    // state
    // controller
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.either(
    //             intakeArm.retractCommand(),
    //             intakeArm.deployCommand(),
    //             () ->
    //                 (intakeArm.isDeployed() || intakeArm.getCurrentState() ==
    // ArmState.DEPLOYING)));

    // controller.leftTrigger().onTrue(intakeArm.toggleDeploy());

    // tests deploy and retract commands by themselves
    // controller.povUp().onTrue(intakeArm.retractCommand());
    // controller.povDown().onTrue(intakeArm.deployCommand());

    controller.povUp().whileTrue(intakeArm.manualCommand(0.2));
    controller.povDown().whileTrue(intakeArm.manualCommand(-0.2));

    //  runs the kicker
    controller
        .start()
        .whileTrue(
            Commands.run(() -> kicker.runPercent(0.5), kicker)
                .finallyDo(() -> kicker.stop())); // change this value if you want

    // runs hopper
    // controller
    //     .povRight()
    //     .whileTrue(
    //         Commands.run(() -> hopper.runPercent(0.5), hopper).finallyDo(() -> hopper.stop()));
    // runs intakeroller
    controller
        .leftTrigger()
        .whileTrue(
            Commands.run(() -> intakeRoller.runPercent(0.75), intakeRoller)
                .finallyDo(() -> intakeRoller.stop()));

    // runs shooter
    controller
        .rightTrigger()
        .whileTrue(
            Commands.run(
                    () -> shooter.setVelocity(SmartDashboard.getNumber("Shooter/TestRPS", 0.0)),
                    shooter)
                .finallyDo(() -> shooter.stop()));
  }

  public Pose2d updatePose() {
    return drive.getPose();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

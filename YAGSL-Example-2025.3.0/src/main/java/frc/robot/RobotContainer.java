// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import swervelib.SwerveInputStream;
// import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Coral.intakeCoral;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.Configs.CoralSubsystem;
import frc.robot.Constants.ElevatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Coral Coral = new Coral();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final Elevator    elevatorsub           = new Elevator();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    // NamedCommands.registerCommand("drive in auto", drivebase.driveForAuton());
    NamedCommands.registerCommand("L2", elevatorsub.setSetpointCommandPP(Setpoint.kLevel2).until(elevatorsub.L2Trigger()));
    NamedCommands.registerCommand("shoot", elevatorsub.runIntakeCommand().unless(elevatorsub.CoralTrigger()));
    NamedCommands.registerCommand("L0", elevatorsub.setSetpointCommand(Setpoint.kFeederStation).until(elevatorsub.L0Trigger()));
    NamedCommands.registerCommand("dealgea", elevatorsub.DeAlgea());
    NamedCommands.registerCommand("Intake", elevatorsub.runIntakeCommand().until(elevatorsub.CoralTrigger()));
    NamedCommands.registerCommand("LockDrive", Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.runOnce(coral::intake));
      // driverXbox.rightBumper().onTrue(Commands.runOnce(coral::scoreL1));
    } else
    {
      // Start button -> Zero gyro
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      // D pad up -> Set elevator to feeder station height
      driverXbox.povUp().onTrue(elevatorsub.setSetpointCommand(Setpoint.kFeederStation));
      // D pad right -> Set elevator to level 1 height
      driverXbox.povRight().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel1));
      // D pad down -> Set elevator to level 2 height
      driverXbox.povDown().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel2));
      // D pad left -> Set elevator to level 3 height
      driverXbox.povLeft().onTrue(elevatorsub.setSetpointCommand(Setpoint.kLevel3));
      // Right Trigger -> Run intake to apropiate speed or intake depending on elevator height
      driverXbox.rightTrigger(OperatorConstants.TRIGGER_DEADBAND).whileTrue(elevatorsub.runIntakeCommand().until(elevatorsub.CoralTrigger()));
      // Left Trigger -> Run intake in reverse
      driverXbox.leftTrigger(OperatorConstants.TRIGGER_DEADBAND).whileTrue(elevatorsub.reverseIntakeCommand());
      // Right Bumper -> force intake forwards
      driverXbox.rightBumper().whileTrue(elevatorsub.runIntakeCommand());
      // Left Bumper -> force intake backwards
      // driverXbox.b().onTrue(Commands.runOnce(() -> drivebase.driveToLeftCoral()));
      // driverXbox.b().onFalse(Commands.runOnce(() -> drivebase.stopCoralpathing()));
      // driverXbox.a().onTrue(Commands.runOnce(() -> drivebase.driveToRightCoral()));
      // driverXbox.a().onFalse(Commands.runOnce(() -> drivebase.stopCoralpathing()));
      driverXbox.a().whileTrue(elevatorsub.runstickCommand());
      
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("1meter");
    // return drivebase.driveForAuton();
    return new WaitCommand(0.5).deadlineFor(drivebase.drivedistanceForAuton(new Pose2d(-0.17 , 0, Rotation2d.fromDegrees(0)), .25))
    .andThen(new WaitCommand(2.9))
    .andThen(Commands.runOnce(drivebase::lock, drivebase))
    .andThen(new WaitCommand(1.5).deadlineFor(elevatorsub.setSetpointCommand(Setpoint.kLevel3)))
    .andThen(new WaitCommand(1.5).deadlineFor(elevatorsub.runIntakeCommand()))
    .andThen(elevatorsub.setSetpointCommand(Setpoint.kFeederStation));
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  
  // public void getCoralScoreLEFTpose()
  // {
  //   double heading = drivebase.getHeading().getDegrees();
  //     Pose2d CoralScorePoseLEFT;
  //     if (heading >= 30 && heading < 90) {
  //       CoralScorePoseLEFT = new Pose2d(3.668, 2.916, Rotation2d.fromDegrees(60));
  //       System.out.println("60");
  //     } else if (heading >= 90 && heading < 150) {
  //       CoralScorePoseLEFT = new Pose2d(5.035, 2.772, Rotation2d.fromDegrees(120));
  //       System.out.println("120");
  //     } else if (heading >= 150 && heading < 210) {
  //       CoralScorePoseLEFT = new Pose2d(5.826, 3.875, Rotation2d.fromDegrees(180));
  //       System.out.println("180");
  //     } else if (heading >= 210 && heading < 270) {
  //       CoralScorePoseLEFT = new Pose2d(5.323, 5.122, Rotation2d.fromDegrees(240));
  //       System.out.println("240");
  //     } else if (heading >= 270 && heading < 330) {
  //       CoralScorePoseLEFT = new Pose2d(3.944, 5.266, Rotation2d.fromDegrees(300));
  //       System.out.println("300");
  //     } else {
  //       CoralScorePoseLEFT = new Pose2d(3.105, 4.175, Rotation2d.fromDegrees(0));
  //       System.out.println("0");
          
  //     }

  //     if (drivebase.isRedAlliance()) {
  //       CoralScorePoseLEFT = new Pose2d(
  //       -CoralScorePoseLEFT.getX(),
  //       -CoralScorePoseLEFT.getY(),
  //       CoralScorePoseLEFT.getRotation().plus(Rotation2d.fromDegrees(180))
  //       );
  //     }
  // }

}

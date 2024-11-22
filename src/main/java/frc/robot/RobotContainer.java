// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Photonvision;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final SendableChooser<Command> m_chooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  private final DriveRobot m_drive = new DriveRobot(m_drivetrain, m_driverController);

  private final UsbCamera frontCamera;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_drive);

    NamedCommands.registerCommand("Lower Intake", m_intake.moveIntakeOut().until(m_intake::getIntakeArrived));
    NamedCommands.registerCommand("Raise Intake", m_intake.moveIntakeIn().until(m_intake::getIntakeArrived));
    NamedCommands.registerCommand("Toggle Intaking", m_intake.toggleIntaking());

    NamedCommands.registerCommand("Spin Wheels", m_shooter.autoShoot(5));
    NamedCommands.registerCommand("Eject Note", m_intake.eject(1));
    NamedCommands.registerCommand("Zero Pose", m_drivetrain.runOnce(() -> m_drivetrain.resetOdometry(new Pose2d())));

    m_chooser = AutoBuilder.buildAutoChooser("1M Forward");
    SmartDashboard.putData("Auto Chooser", m_chooser);
    // m_chooser.addOption("Single Note", m_shooter.autoShoot(5).alongWith());
    // Configure the trigger bindings
    configureBindings();
    frontCamera = CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture("camera", 0);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_drivetrain.goToOrigin());
    // m_driverController.a().onTrue(m_drivetrain.runOnce(() ->
    // m_drivetrain.resetWheels()));
    m_driverController.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetGyro()));
    m_driverController.back().onTrue(m_intake.zeroIntake());
    m_operatorController.button(3).onTrue(m_intake.moveIntakeOut());
    m_operatorController.button(4).onTrue(m_intake.moveIntakeIn());
    m_operatorController.button(1).onTrue(m_intake.toggleIntaking());
    m_operatorController.button(2).onTrue(m_intake.toggleOuttaking());
    m_operatorController.button(6).whileTrue(m_shooter.shoot(0.75));
    m_operatorController.button(5).whileTrue(m_shooter.shoot(1));
    m_operatorController.button(5).whileTrue(m_shooter.shoot(1));
    m_operatorController.axisLessThan(1, 0.0).whileTrue(m_intake.increasePosition());
    m_operatorController.axisGreaterThan(1, 0.0).whileTrue(m_intake.decreasePosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

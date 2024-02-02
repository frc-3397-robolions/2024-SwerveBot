// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.TimerTask;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SpinInPlace extends Command {
  /** Creates a new SpinInPlace. */
  private PIDController m_pidController = new PIDController(0.1, 0, 0);
  private final Drivetrain m_drivetrain;
  private double m_startAngle;
  private Timer starttime = new Timer();

  public SpinInPlace(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startAngle = m_drivetrain.getGyro().getDegrees();

    starttime.start();
    m_drivetrain.drive(0, 0, 0.5, false, false);
    // starttime.schedule(new TimerTask() {
    // @Override
    // public void run() {
    // // Put the code that stops the robot here

    // }
    // }, /*/ put delay here in milliseconds /*/ 1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (starttime.get() > 1) {
      double output = m_pidController.calculate(m_drivetrain.getGyro().getDegrees(), m_startAngle) * 0.5;
      m_drivetrain.drive(0, 0, output, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

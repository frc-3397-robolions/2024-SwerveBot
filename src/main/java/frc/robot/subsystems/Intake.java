// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.utilities.MathUtils;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax angleMotor;
  private final CANSparkMax driveMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkPIDController anglePID;
  private boolean intakeDesiredOut = false;
  private boolean intakeArrived = false;
  private boolean intaking = false;
  private boolean outtaking = false;

  public Intake() {
    // The motor controlling the angle of the assembly
    angleMotor = new CANSparkMax(kAngle, CANSparkMax.MotorType.kBrushless);
    angleMotor.setSmartCurrentLimit(CurrentLimit.kDrive);
    angleMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    angleMotor.setInverted(false);
    angleMotor.setIdleMode(IdleMode.kBrake);

    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(kAnglePositionFactor);
    anglePID = angleMotor.getPIDController();
    anglePID.setP(kP);
    anglePID.setI(kI);
    anglePID.setD(kD);
    anglePID.setFF(kFF);
    angleMotor.burnFlash();

    // The motor driving the intake wheels
    driveMotor = new CANSparkMax(kDrive, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // Set the PID Controller to the corresponding angle for in/out
    anglePID.setReference(intakeStates.get(intakeDesiredOut), ControlType.kPosition);

    // If the intake is within tolerance of it's desired angle, set this variable to
    // true so other files can act
    if (MathUtils.angleInRange(angleEncoder.getPosition(), intakeStates.get(intakeDesiredOut), kPositionTolerance))
      intakeArrived = true;
    else
      intakeArrived = false;

    // Drive intake wheels based on the desired state
    if (intaking)
      driveMotor.set(kIntakePower);
    else if (outtaking)
      driveMotor.set(kOuttakePower);
    else
      driveMotor.set(0);

    // Putting data on SmartDashboard
    SmartDashboard.putNumber("Intake Angle", angleEncoder.getPosition());
    SmartDashboard.putNumber("Intake Desired Angle", intakeStates.get(intakeDesiredOut));
    SmartDashboard.putBoolean("Intake Arrived", intakeArrived);
    SmartDashboard.putBoolean("Intaking", intaking);
    SmartDashboard.putBoolean("Outtaking", outtaking);
  }

  public boolean getIntakeArrived() {
    return intakeArrived;
  }

  public Command lowerIntake() {
    return runOnce(() -> {
      outtaking = false;
      intakeDesiredOut = true;
      intaking = true;
    });
  }

  public Command raiseIntake() {
    return runOnce(() -> {
      intaking = false;
      intakeDesiredOut = false;
    });
  }

  public Command eject() {
    return runEnd(
        () -> {
          outtaking = true;
        },
        () -> {
          outtaking = false;
        });
  }
}

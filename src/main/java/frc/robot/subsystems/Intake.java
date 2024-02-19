// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;
  private final SparkPIDController anglePID;
  private boolean intakeDesiredOut = false;
  private boolean intakeArrived = false;
  private boolean intaking = false;
  private boolean outtaking = false;

  public Intake() {
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
    driveMotor = new CANSparkMax(kDrive, CANSparkMax.MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    anglePID.setReference(intakeStates.get(intakeDesiredOut), ControlType.kPosition);

    if (MathUtils.angleInRange(angleEncoder.getPosition(), intakeStates.get(intakeDesiredOut), kPositionTolerance))
      intakeArrived = true;
    else
      intakeArrived = false;

    if (intaking)
      driveMotor.set(kIntakePower);
    else if (outtaking)
      driveMotor.set(kOuttakePower);
    else
      driveMotor.set(0);
  }

  public Command lowerIntake() {
    return run(() -> {
      outtaking = false;
      intakeDesiredOut = true;
      intaking = true;
    });
  }

  public Command raiseIntake() {
    return run(() -> {
      intaking = false;
      intakeDesiredOut = false;
    });
  }

  public Command shoot() {
    return runEnd(
        () -> {
          outtaking = true;
        },
        () -> {
          outtaking = false;
        });
  }
}

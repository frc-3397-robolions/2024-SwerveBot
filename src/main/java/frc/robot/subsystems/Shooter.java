// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkPIDController leftPID;
  private SparkPIDController rightPID;
  private SimpleMotorFeedforward shooterFF;
  private double desiredVelocity = 0;

  public Shooter() {
    leftMotor = new CANSparkMax(kLeft, MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(CurrentLimit.kShooter);
    leftMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    leftMotor.setInverted(true);
    leftMotor.setIdleMode(IdleMode.kCoast);
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setVelocityConversionFactor(1);
    leftPID = leftMotor.getPIDController();
    leftPID.setP(0);
    leftPID.setFF(0.1);

    rightMotor = new CANSparkMax(kRight, MotorType.kBrushless);
    rightMotor.setSmartCurrentLimit(CurrentLimit.kShooter);
    rightMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    rightMotor.setInverted(false);
    rightMotor.setIdleMode(IdleMode.kCoast);
    rightEncoder = rightMotor.getEncoder();
    rightEncoder.setVelocityConversionFactor(1);
    rightPID = rightMotor.getPIDController();
    rightPID.setP(0);
    rightPID.setFF(0.1);

    shooterFF = new SimpleMotorFeedforward(0.01, kv);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Desired Shooter Speed", desiredVelocity);
    SmartDashboard.putNumber("Shooter Speed", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Pos", leftEncoder.getPosition());
    double ff = shooterFF.calculate(desiredVelocity);
    leftPID.setReference(desiredVelocity, ControlType.kVelocity, 0);
    rightPID.setReference(desiredVelocity, ControlType.kVelocity, 0);
  }

  public Command shoot(double percent) {
    return runEnd(() -> {
      desiredVelocity = 5820 * percent;
    }, () -> {
      desiredVelocity = 0;
    });
  }
  // public Command shoot(double percent) {
  // return runEnd(() -> {
  // leftMotor.set(kPower);
  // rightMotor.set(kPower);
  // }, () -> {
  // leftMotor.set(0);
  // rightMotor.set(0);
  // });
  // }

  public Command autoShoot(double time) {
    return runEnd(() -> {
      leftMotor.set(kPower);
      rightMotor.set(kPower);
    }, () -> {
      leftMotor.set(0);
      rightMotor.set(0);
    }).withTimeout(time);
  }
}

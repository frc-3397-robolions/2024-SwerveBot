// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private boolean intakeDesiredOut = false;
  private boolean intakeArrived = false;
  private boolean intaking = false;
  private boolean outtaking = false;

  public Intake() {
    // The motor controlling the angle of the assembly
    angleMotor = new CANSparkMax(kAngle, CANSparkMax.MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();
    // angleMotor.setSmartCurrentLimit(CurrentLimit.kIntakeAngle);
    angleMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    angleMotor.setInverted(false);
    angleMotor.setSoftLimit(SoftLimitDirection.kForward, intakeStates.get(true).floatValue());
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, intakeStates.get(false).floatValue());
    angleMotor.setIdleMode(IdleMode.kCoast);

    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(kAnglePositionFactor);
    angleEncoder.setVelocityConversionFactor(kAnglePositionFactor);
    anglePID = angleMotor.getPIDController();
    anglePID.setP(kP);
    anglePID.setI(kI);
    anglePID.setD(kD);
    anglePID.setFF(kFF);
    angleMotor.burnFlash();

    // The motor driving the intake wheels
    driveMotor = new CANSparkMax(kDrive, CANSparkMax.MotorType.kBrushless);
    driveMotor.setSmartCurrentLimit(CurrentLimit.kIntakeWheels);
    driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    m_goal = new TrapezoidProfile.State(intakeStates.get(intakeDesiredOut).floatValue(), 0);
    m_setpoint = profile.calculate(0.02, m_setpoint, m_goal);
    // Set the PID Controller to the corresponding angle for in/out
    anglePID.setReference(m_setpoint.position, ControlType.kPosition);

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
    SmartDashboard.putNumber("Intake Power", angleMotor.getAppliedOutput());
  }

  public boolean getIntakeArrived() {
    return intakeArrived;
  }

  public Command zeroIntake() {
    return runOnce(() -> {
      angleEncoder.setPosition(0);
    });
  }

  public Command eject(double time) {
    return runEnd(
        () -> {
          outtaking = true;
        },
        () -> {
          outtaking = false;
        }).withTimeout(time);
  }

  public Command moveIntakeOut() {
    return runOnce(() -> {
      intakeDesiredOut = true;
    });
  }

  public Command moveIntakeIn() {
    return runOnce(() -> {
      intakeDesiredOut = false;
    });
  }

  public Command toggleIntaking() {
    return runOnce(() -> {
      outtaking = false;
      intaking = !intaking;
    });
  }

  public Command toggleOuttaking() {
    return runOnce(() -> {
      outtaking = !outtaking;
      intaking = false;
    });
  }
}

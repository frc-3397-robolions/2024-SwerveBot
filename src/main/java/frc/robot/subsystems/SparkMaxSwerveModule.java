package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Azimuth;
import frc.robot.Constants.ModuleConstants.Drive;

/**
 * FRC 1706 Class for creating a swerve module with 2 SparkMax motor controllers
 * and an analog azimuth Encoder
 */
public class SparkMaxSwerveModule extends SubsystemBase {
    private final CANSparkMax m_azimuthMotor;
    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_azimuthEnc;
    private final RelativeEncoder m_driveEnc;
    private EncoderSim m_azimuthEncSim;
    private EncoderSim m_driveEncSim;
    private final CANcoder m_absEncoder;
    private CANcoderSimState m_absEncoderSimState;
    private final SparkPIDController m_drivePID;
    private final SparkPIDController m_azimuthPID;
    private final SimpleMotorFeedforward m_driveFF;
    private double m_referenceAngleRadians = 0;
    private double m_offset;

    /**
     * Create a new FRC 1706 SparkMaxSwerveModule Object
     *
     * @param drive   The drive SparkMax CAN ID.
     * @param azimuth The azimuth SparkMax CAN ID.
     * @param absEnc  The analog encoder port for the absolute encoder.
     * @param offset  The offset for the analog encoder.
     */
    public SparkMaxSwerveModule(int driveID, int azimuthID, int absEncID, double offset) {
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setSmartCurrentLimit(CurrentLimit.kDrive);
        m_driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_driveMotor.setInverted(true);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_driveEnc = m_driveMotor.getEncoder();
        m_driveEnc.setPositionConversionFactor(Drive.kPositionFactor);
        m_driveEnc.setVelocityConversionFactor(Drive.kVelocityFactor);
        m_driveEnc.setAverageDepth(4);
        m_driveEnc.setMeasurementPeriod(16);

        m_drivePID = m_driveMotor.getPIDController();
        m_drivePID.setP(Drive.kp);

        m_driveFF = new SimpleMotorFeedforward(Drive.ks, Drive.kv);

        m_driveMotor.burnFlash();

        m_azimuthMotor = new CANSparkMax(azimuthID, MotorType.kBrushless);
        m_azimuthMotor.restoreFactoryDefaults();
        m_azimuthMotor.setSmartCurrentLimit(CurrentLimit.kAzimuth);
        m_azimuthMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_azimuthMotor.setInverted(false);
        m_azimuthMotor.setIdleMode(IdleMode.kBrake);

        m_azimuthEnc = m_azimuthMotor.getEncoder();
        m_azimuthEnc.setPositionConversionFactor(Azimuth.kPositionFactor);
        m_azimuthEnc.setVelocityConversionFactor(Azimuth.kVelocityFactor);
        m_azimuthEnc.setAverageDepth(4);
        m_azimuthEnc.setMeasurementPeriod(16);

        m_azimuthPID = m_azimuthMotor.getPIDController();
        m_azimuthPID.setP(Azimuth.kp);

        m_absEncoder = new CANcoder(absEncID);

        m_azimuthMotor.burnFlash();
        m_offset = offset;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEnc.getVelocity(), new Rotation2d(getStateAngle()));
    }

    /**
     * 
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEnc.getPosition(), new Rotation2d(getStateAngle()));
    }

    public void resetAzimuth() {
        m_azimuthEnc.setPosition(getAbsEncoder());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Reset the relative to the absolute encoder to account for value drift.
        // Attempt to find source later.
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
        // Calculate the drive output from the drive PID controller.
        // final double driveOutput =
        // m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
        // state.speedMetersPerSecond);
        // Calculates the desired feedForward motor % from the current desired velocity
        // and the static and feedforward gains
        final double driveFF = m_driveFF.calculate(state.speedMetersPerSecond);

        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0,
                driveFF * GlobalConstants.kVoltCompensation);
        setReferenceAngle(state.angle.getRadians());
    }

    /**
     * Sets the reference angle for the azimuth.
     *
     * @param referenceAngleRadians Desired reference angle.
     */
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_azimuthEnc.getPosition();

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        m_referenceAngleRadians = referenceAngleRadians;
        m_azimuthPID.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
    }

    /**
     * Gets the reference angle for the azimuth.
     *
     * @return reference angle.
     */
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    public double getStateAngle() {
        double motorAngleRadians = m_azimuthEnc.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    public double getAbsEncoder() {

        return 69;

    }

    public void enableBrake(boolean brake) {
        if (brake) {
            m_driveMotor.setIdleMode(IdleMode.kBrake);
        } else {
            m_driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void stop() {
        m_driveMotor.set(0.0);
        m_azimuthMotor.set(0.0);
    }

}

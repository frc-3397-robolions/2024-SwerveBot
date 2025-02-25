package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.beans.DesignMode;
import java.io.PipedInputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.swing.text.html.Option;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.FrontLeft;
import frc.robot.Constants.DriveConstants.FrontRight;
import frc.robot.Constants.DriveConstants.KeepAngle;
import frc.robot.Constants.DriveConstants.RearLeft;
import frc.robot.Constants.DriveConstants.RearRight;
import frc.robot.Constants.ModuleConstants.Drive;
import frc.robot.utilities.Photonvision;
import frc.robot.Robot;

/**
 * Implements a swerve Drivetrain Subsystem for the Robot
 */
public class Drivetrain extends SubsystemBase {
  private boolean fieldOriented = false;
  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;
  private double desiredAngle = 0.0;

  private final PIDController m_keepAnglePID = new PIDController(KeepAngle.kp, KeepAngle.ki, KeepAngle.kd);

  private final Timer m_keepAngleTimer = new Timer();

  private SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTransSlewRate);
  private SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTransSlewRate);
  private SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotSlewRate);

  private final SparkMaxSwerveModule m_FLModule = new SparkMaxSwerveModule(FrontLeft.kDrive, FrontLeft.kAzimuth,
      FrontLeft.kAbsEnc, 0);
  private final SparkMaxSwerveModule m_FRModule = new SparkMaxSwerveModule(FrontRight.kDrive, FrontRight.kAzimuth,
      FrontRight.kAbsEnc, 0);
  private final SparkMaxSwerveModule m_RLModule = new SparkMaxSwerveModule(RearLeft.kDrive, RearLeft.kAzimuth,
      RearLeft.kAbsEnc, -Math.PI / 2);
  private final SparkMaxSwerveModule m_RRModule = new SparkMaxSwerveModule(RearRight.kDrive, RearRight.kAzimuth,
      RearRight.kAbsEnc, -Math.PI / 2);

  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics,
      getGyro(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics,
      getGyro(), getModulePositions());

  private final double[] m_latestSlew = { 0.0, 0.0, 0.0 };

  private SwerveModuleState[] m_desStates = DriveConstants.kSwerveKinematics
      .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));

  private final Field2d m_field = new Field2d();

  private Pose2d simOdometryPose = new Pose2d();

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kSwerveKinematics,
      getGyro(),
      getModulePositions(),
      new Pose2d());

  /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    m_keepAngleTimer.reset();
    m_keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    m_odometry.resetPosition(getGyro(), getModulePositions(), new Pose2d());

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setModuleStates, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            0.5 * Math.sqrt(Math.pow(DriveConstants.kWheelBaseLength, 2) + Math.pow(DriveConstants.kWheelBaseWidth, 2)), // Drive
                                                                                                                         // base
                                                                                                                         // radius
                                                                                                                         // in
                                                                                                                         // meters.
                                                                                                                         // Distance
                                                                                                                         // from
                                                                                                                         // robot
                                                                                                                         // center
                                                                                                                         // to
                                                                                                                         // furthest
                                                                                                                         // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this::onBlueSide,
        this // Reference to this subsystem to set requirements
    );
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle,
      boolean noteLockOn, CommandXboxController m_controller) {

    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);

    m_latestSlew[0] = xSpeed;
    m_latestSlew[1] = ySpeed;
    m_latestSlew[2] = rot;

    // SmartDashboard.putNumber("Current Gyro Angle", getGyro().getRadians());

    SmartDashboard.putBoolean("Keep Angle", keepAngle);

    if (Math.sqrt((m_controller.getRightX() * m_controller.getRightX())
        + (m_controller.getRightY() * m_controller.getRightY())) >= DriveConstants.kRotTolerance) {
      desiredAngle = getDesiredAngle(m_controller);
    }

    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    double currentAngleRad = getGyro().getRadians() % (2 * Math.PI);
    SmartDashboard.putNumber("Current Angle", currentAngleRad);

    m_keepAnglePID.enableContinuousInput(0, (2 * Math.PI));
    rot = m_keepAnglePID.calculate(currentAngleRad, desiredAngle); // Set output command

    /*
     * if (keepAngle) {
     * rot = performKeepAngle(xSpeed, ySpeed, rot, m_controller); // Calls the keep
     * angle function to update the keep
     * // angle or rotate
     * } else if (noteLockOn) {
     * rot = performNoteLockOn();
     * }
     */

    if (Math.abs(rot) < 0.02) {
      rot = 0.0;
    }
    if (Math.abs(xSpeed) < 0.02) {
      xSpeed = 0.0;
    }
    if (Math.abs(ySpeed) < 0.02) {
      ySpeed = 0.0;
    }
    SmartDashboard.putNumber("DesiredXSpeed", xSpeed);
    SmartDashboard.putNumber("DesiredYSpeed", ySpeed);
    SmartDashboard.putNumber("DesiredRotation", rot);

    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
          getGyro()));
    } else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    double[] states = new double[8];
    for (int i = 0; i < m_desStates.length; i++) {
      states[i * 2] = m_desStates[i].angle.getDegrees();
      states[i * 2 + 1] = m_desStates[i].speedMetersPerSecond;
    }

    SmartDashboard.putNumberArray("States", states);

    if (Robot.isSimulation()) {
      ChassisSpeeds speeds = DriveConstants.kSwerveKinematics.toChassisSpeeds(m_desStates);
      simOdometryPose = simOdometryPose.exp(
          new Twist2d(
              speeds.vxMetersPerSecond * .02,
              speeds.vyMetersPerSecond * .02,
              speeds.omegaRadiansPerSecond * .02));
      m_field.setRobotPose(simOdometryPose);
    }

    updateOdometry();

    getPose();
  }

  public double getDesiredAngle(CommandXboxController m_controller) {
    double desiredAngleRad = 0;

    SmartDashboard.putNumber("Controller X", m_controller.getRightX());
    SmartDashboard.putNumber("Controller Y", m_controller.getRightY());
    if (m_controller.getRightX() > 0) {
      desiredAngleRad = (Math.PI / 2) - Math.atan((-1 * m_controller.getRightY()) /
          m_controller.getRightX());
    } else if (m_controller.getRightX() < 0) {
      desiredAngleRad = (3 * (Math.PI / 2)) - Math.atan((-1 * m_controller.getRightY()) /
          m_controller.getRightX());
    }

    return desiredAngleRad;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Encoder", m_FLModule.getStateAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Front Right Encoder", m_FRModule.getStateAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Rear Left Encoder", m_RLModule.getStateAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Rear Right Encoder", m_RRModule.getStateAngle() * 180 / Math.PI);

    SmartDashboard.putNumber("Front Left Ref", m_FLModule.getReferenceAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Front Right Ref", m_FRModule.getReferenceAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Rear Left Ref", m_RLModule.getReferenceAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("Rear Right Ref", m_RRModule.getReferenceAngle() * 180 / Math.PI);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_FLModule.setDesiredState(desiredStates[0]);
    m_FRModule.setDesiredState(desiredStates[1]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  public void resetWheels() {
    m_FLModule.resetAzimuth();
    m_FRModule.resetAzimuth();
    m_RLModule.resetAzimuth();
    m_RRModule.resetAzimuth();
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kSwerveKinematics
        .toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_desStates = desiredStates;
    // @TODO UNDO THIS
    m_FLModule.setDesiredState(desiredStates[1]);
    m_FRModule.setDesiredState(desiredStates[0]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0))
        .times(chassisSpeeds.omegaRadiansPerSecond * DriveConstants.kRotTransFactor);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  public void stop() {
    m_FLModule.stop();
    m_FRModule.stop();
    m_RLModule.stop();
    m_RRModule.stop();
  }

  public void brakeMode(boolean enabled) {
    m_FLModule.enableBrake(enabled);
    m_FRModule.enableBrake(enabled);
    m_RLModule.enableBrake(enabled);
    m_RRModule.enableBrake(enabled);
  }

  public double getTilt() {
    return ahrs.getRoll();
    // return MathUtils.pythagorean(ahrs.getRoll(), ahrs.getPitch());
  }

  public double getTiltVel() {
    return ahrs.getRawGyroY();
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */
  public void updateOdometry() {
    m_odometry.update(getGyro(), getModulePositions());
    // Optional<EstimatedRobotPose> result =
    // Photonvision.instance.getEstimatedGlobalPose();
    // if (result.isPresent()) {
    // EstimatedRobotPose pose = result.get();
    // addVisionMeasurement(
    // pose.estimatedPose.toPose2d(),
    // pose.timestampSeconds,
    // Photonvision.instance.getEstimationStdDevs(pose.estimatedPose.toPose2d()));
    // }
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * 
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return ahrs.getRotation2d().times(-1);
  }

  /**
   * Function created to retreieve and push the robot pose to the SmartDashboard
   * for diagnostics
   * 
   * @return Pose2d object containing the X and Y position and the heading of the
   *         robot.
   */
  public Pose2d getPose() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    Translation2d position = pose.getTranslation();

    if (Robot.isSimulation())
      pose = simOdometryPose;

    return pose;
  }

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    ahrs.reset();
    ahrs.setAngleAdjustment(pose.getRotation().getDegrees());
    updateKeepAngle();
    m_poseEstimator.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle.getDegrees());
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    updateKeepAngle();
    m_poseEstimator.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  public void resetGyro() {
    ahrs.reset();
  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kSwerveKinematics.toChassisSpeeds(m_FLModule.getState(), m_FRModule.getState(),
        m_RLModule.getState(),
        m_RRModule.getState());
  }

  public ChassisSpeeds getCorDesChassisSpeed() {
    return DriveConstants.kSwerveKinematics.toChassisSpeeds(m_desStates[0], m_desStates[1], m_desStates[2],
        m_desStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { m_FLModule.getPosition(), m_FRModule.getPosition(), m_RLModule.getPosition(),
        m_RRModule.getPosition() };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { m_FLModule.getState(), m_FRModule.getState(), m_RLModule.getState(),
        m_RRModule.getState() };
  }

  public boolean onBlueSide() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      return alliance.get() == Alliance.Blue;
    return true;
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Keep angle function is performed to combat drivetrain drift without the need
   * of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the
   * keepAngle value. This value is updated when the robot
   * is rotated manually by the driver input
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot, CommandXboxController m_controller) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= 0.01) { // If the driver commands the robot to rotate set the
                                 // last rotate time to the current time
      lastRotTime = m_keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= 0.01
        || Math.abs(ySpeed) >= 0.01) { // if driver commands robot to translate set the
                                       // last drive time to the current time
      lastDriveTime = m_keepAngleTimer.get();
    }
    timeSinceRot = m_keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = m_keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive
                                                             // time
    if (timeSinceRot < 0.25) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                               // move to finish
      double desiredAngleRad = Math.PI - Math.atan(m_controller.getRightY() / m_controller.getRightX());
      ;

      if (m_controller.getRightX() > 0) {
        desiredAngleRad = Math.PI - Math.atan(m_controller.getRightY() /
            m_controller.getRightX());
      } else if (m_controller.getRightX() < 0) {
        desiredAngleRad = ((3 / 2) * Math.PI) - Math.atan(m_controller.getRightY() /
            m_controller.getRightX());
      }

      SmartDashboard.putNumber("Desired Angle", desiredAngleRad);
      // keepAngle = getGyro().getRadians();
      keepAngle = desiredAngleRad;
      // } else if (Math.abs(rot) <= 0.01 && timeSinceDrive < 0.25) { // Run Keep
      // angle pid
      // until 0.75s after drive
      // command stops to combat
      // decel drift
      double currentAngleRad = getGyro().getRadians() % (2 * Math.PI);
      output = m_keepAnglePID.calculate(currentAngleRad, keepAngle); // Set output command to the result of the
                                                                     // Keep Angle PID

      m_keepAnglePID.enableContinuousInput(0, (2 * Math.PI));
    }
    return output;
  }

  public double performNoteLockOn() {
    var result = Photonvision.instance.getLatestIntakeResult();
    double output = 0;
    if (result.hasTargets()) {
      output = -m_keepAnglePID.calculate(result.getBestTarget().getYaw(), 0);
    }
    return output;
  }

  public double rotateToJoystickPos(XboxController xb) {
    double output = 0;
    // TODO might not work, maybe adjust
    output = -m_keepAnglePID.calculate(getGyro().getRadians(), Math.atan(xb.getRightY() / xb.getRightX()));
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getGyro().getRadians();
  }

  public void changeSlewRate(double translation, double rotation) {
    m_slewX = new SlewRateLimiter(translation, -translation, m_latestSlew[0]);
    m_slewY = new SlewRateLimiter(translation, -translation, m_latestSlew[1]);
    m_slewRot = new SlewRateLimiter(rotation, -rotation, m_latestSlew[2]);
  }

  // public Command goToOrigin() {
  // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  // getPose(),
  // new Pose2d(2, 5, Rotation2d.fromDegrees(0)));
  // // Create the path using the bezier points created above
  // PathPlannerPath path = new PathPlannerPath(
  // bezierPoints,
  // new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints
  // for this path. If using a
  // // differential drivetrain, the angular constraints
  // // have no effect.
  // new GoalEndState(3.0, Rotation2d.fromDegrees(0)) // Goal end state. You can
  // set a holonomic rotation here. If
  // // using a differential drivetrain, the rotation will have no
  // // effect.
  // );

  // // Prevent the path from being flipped if the coordinates are already correct
  // path.preventFlipping = true;
  // return new FollowPathHolonomic(
  // path,
  // this::getPose, // Robot pose supplier
  // this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  // this::setModuleStates, // Method that will drive the robot given ROBOT
  // RELATIVE ChassisSpeeds
  // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
  // likely live in your Constants class
  // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
  // 4.5, // Max module speed, in m/s
  // 0.4, // Drive base radius in meters. Distance from robot center to furthest
  // module.
  // new ReplanningConfig() // Default path replanning config. See the API for the
  // options here
  // ),
  // () -> {
  // return false;
  // },
  // this // Reference to this subsystem to set requirements
  // );
  // }

  public Command goToOrigin() {
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(2, 5, Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    return pathfindingCommand;
  }
}

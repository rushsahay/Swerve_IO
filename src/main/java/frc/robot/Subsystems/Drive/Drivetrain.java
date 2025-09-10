// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private Gyro gyro;

  private Module leftFront;
  private Module rightFront;
  private Module leftBack;
  private Module rightBack;

  private SlewRateLimiter frontLimiter =
      new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter =
      new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter =
      new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private RobotConfig config;
  private SwerveDrivePoseEstimator poseEstimator;

  public Drivetrain(
      GyroIO gyroIO,
      ModuleIO leftFrontModuleIO,
      ModuleIO rightFrontModuleIO,
      ModuleIO leftBackModuleIO,
      ModuleIO rightBackModuleIO) {
    gyro = new Gyro(gyroIO);
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                gyro.zero();
              } catch (Exception e) {
              }
            })
        .start();

    leftFront = new Module(leftFrontModuleIO);
    rightFront = new Module(rightFrontModuleIO);
    leftBack = new Module(leftBackModuleIO);
    rightBack = new Module(rightBackModuleIO);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.DRIVE_KINEMATICS, getHeadingRotation2d(), getPositions(), new Pose2d());

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::setPose2d, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        SwerveConstants.pid_controls,
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(getHeadingRotation2d(), getPositions());
    Logger.recordOutput("Drivetrain/Pose 2D", getPose());
    Logger.recordOutput("Drivetrain/Module Positions", getPositions());
    Logger.recordOutput("Drivetrain/Module States", getModuleStates());
  }

  public void zeroHeading() {
    gyro.zero();
    poseEstimator.resetRotation(getHeadingRotation2d());
  }

  public void setHeading(double headingDegrees) {
    gyro.setYawDegrees(0);
  }

  public void stopModules() {
    leftFront.stop();
    rightFront.stop();
    leftBack.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public void swerveDrive(
      double frontSpeed,
      double sideSpeed,
      double turnSpeed,
      boolean fieldOriented,
      Translation2d centerOfRotation,
      boolean deadband) {
    // Drive with rotational speed control w/ joystick
    if (deadband) {
      frontSpeed =
          Math.abs(frontSpeed) > SwerveConstants.DriverConstants.xDeadband ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > SwerveConstants.DriverConstants.yDeadband ? sideSpeed : 0;
      turnSpeed =
          Math.abs(turnSpeed) > SwerveConstants.DriverConstants.turnDeadband ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates =
        SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
    // m_ModulePublisherIn.set(moduleStates); TURN INTO LOGGER
    setModuleStates(moduleStates);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading() {
    return (Math.IEEEremainder(gyro.getYaw(), 360));
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    };
  }

  public void setPose2d(Pose2d pose) {
    double gyroAngle = isRedAlliance()?pose.getRotation().getDegrees()+180:pose.getRotation().getDegrees();
    gyro.setYawDegrees(gyroAngle);
    poseEstimator.resetPosition(Rotation2d.fromDegrees(gyroAngle), getPositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      leftFront.getState(), rightFront.getState(), leftBack.getState(), rightBack.getState()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getBlueAbsoluteHeading() {
    double relativeHeading = gyro.getYaw();
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      relativeHeading += 180;
    }
    return relativeHeading;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates =
        SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public boolean isRedAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public void setVisionMeasurementStdDevs(double... numbers) {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(numbers[0], numbers[1], numbers[2]));
  }

  public void addVisionMeasurement(Pose2d visionPoseEstimate, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionPoseEstimate, timestampSeconds);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLooop) {
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getHeadingRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] moduleStates =
        SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);

    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);

    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }
}

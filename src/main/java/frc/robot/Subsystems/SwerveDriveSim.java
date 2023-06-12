package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSim extends SubsystemBase {
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    private double maxSpeed = Constants.maxSpeed;
    // private double maxAngularRate = Constants.moduleMaxAngularVelocity;
  
    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.swerveModuleDistanceFromOtherModule, Constants.swerveModuleDistanceFromOtherModule);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.swerveModuleDistanceFromOtherModule, -Constants.swerveModuleDistanceFromOtherModule);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.swerveModuleDistanceFromOtherModule, Constants.swerveModuleDistanceFromOtherModule);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.swerveModuleDistanceFromOtherModule, -Constants.swerveModuleDistanceFromOtherModule);
  
    private final SwerveModuleSim m_frontLeft = new SwerveModuleSim();
    private final SwerveModuleSim m_frontRight = new SwerveModuleSim();
    private final SwerveModuleSim m_backLeft = new SwerveModuleSim();
    private final SwerveModuleSim m_backRight = new SwerveModuleSim();

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        }
    );

    public SwerveDriveSim() {
        m_gyroSim.setInitialized(true);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        double angleDelta = m_kinematics.toTwist2d(new SwerveModulePosition[] {
            m_frontLeft.getPositionDelta(),
            m_frontRight.getPositionDelta(),
            m_backLeft.getPositionDelta(),
            m_backRight.getPositionDelta()
            }).dtheta;

        
        m_gyroSim.setRate(angleDelta * 180 / Math.PI);
        m_gyroSim.setAngle((m_gyroSim.getRate() + m_gyroSim.getAngle()) % 360);
      }
    
    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
    }

    public SwerveDriveOdometry getOdometry() {
        return m_odometry;
    }

    public void resetGyro() {
        m_gyroSim.setAngle(0);
        m_gyroSim.setRate(0);
    }

    public double getGyroAngle() {
        return m_gyroSim.getAngle();
    }

    public AnalogGyro getRawGyro() {
        return m_gyro;
    }

    public void setGyroAngle(double value) {
        m_gyroSim.setAngle(value);
    }

    public double getXDelta() {
        return m_kinematics.toTwist2d(new SwerveModulePosition[] {
            m_frontLeft.getPositionDelta(),
            m_frontRight.getPositionDelta(),
            m_backLeft.getPositionDelta(),
            m_backRight.getPositionDelta()
            }).dx;    
    }

    public double getYDelta() {
        return m_kinematics.toTwist2d(new SwerveModulePosition[] {
            m_frontLeft.getPositionDelta(),
            m_frontRight.getPositionDelta(),
            m_backLeft.getPositionDelta(),
            m_backRight.getPositionDelta()
            }).dy;    
    }

    public double getThetaDelta() {
        return m_kinematics.toTwist2d(new SwerveModulePosition[] {
            m_frontLeft.getPositionDelta(),
            m_frontRight.getPositionDelta(),
            m_backLeft.getPositionDelta(),
            m_backRight.getPositionDelta()
            }).dtheta;    
    }
}
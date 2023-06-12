package frc.robot.SimUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveDriveSim;

public class FieldSim extends SubsystemBase {
    private SwerveDriveSim m_swerveDrive;
    private XboxController m_controller;

    private boolean driveFieldRelative = SmartDashboard.putBoolean("Drive Field Relative", true);
    private final Field2d m_field = new Field2d();

    public FieldSim(SwerveDriveSim m_swerveDrive, XboxController m_controller) {
        this.m_swerveDrive = m_swerveDrive;
        this.m_controller = m_controller;

        Shuffleboard.getTab("Simulation")
        .add("Field", m_field)
        .withWidget(BuiltInWidgets.kField);

        SmartDashboard.putBoolean("Drive Field Relative", false);
    }

    public void initalize() {
        m_swerveDrive.setGyroAngle(0);
        m_field.setRobotPose(Constants.initFieldPosition);
        m_swerveDrive.updateOdometry();
    }

    public void driveWithJoystick(boolean fieldRelative) {
        final double xSpeed =
            MathUtil.clamp((MathUtil.applyDeadband(m_controller.getLeftX(), Constants.deadBand))
                * Constants.moduleMaxSpeedSim, -1, 1);
    
        final double ySpeed = -1 *
            MathUtil.clamp((MathUtil.applyDeadband(m_controller.getLeftY(), Constants.deadBand))
                * Constants.moduleMaxSpeedSim, -1, 1);
    
        final double rot =
            MathUtil.clamp((MathUtil.applyDeadband(m_controller.getRightX(), Constants.deadBand)), -1, 1);
    
        m_swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative);

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("Z Rotation", rot);
      }

      @Override
      public void simulationPeriodic() {
        if (RobotState.isTeleop()) {
            driveFieldRelative = SmartDashboard.getBoolean("Drive Field Relative", true);
            driveWithJoystick(driveFieldRelative);
            m_swerveDrive.updateOdometry();

            SwerveDriveOdometry odometry = m_swerveDrive.getOdometry();
            m_field.getRobotObject().setPose(new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), odometry.getPoseMeters().getRotation()));

            SmartDashboard.putNumber("Simulation X", (odometry.getPoseMeters().getX()));
            SmartDashboard.putNumber("Simulation Y", (odometry.getPoseMeters().getY()));
            SmartDashboard.putNumber("Simulation Theta", (odometry.getPoseMeters().getRotation().getDegrees()));
        }
    }
}

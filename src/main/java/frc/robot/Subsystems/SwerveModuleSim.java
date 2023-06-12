package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SimUtil.RelativeEncoderSim;

public class SwerveModuleSim extends SubsystemBase {
    private static final double moduleMaxAngularVelocity = Constants.moduleMaxAngularVelocity;
    private static final double moduleMaxAngularAcceleration = Constants.moduleMaxAngularAcceleration;
  
    private DCMotorSim m_driveMotor;
    private DCMotorSim m_turningMotor;

    private DCMotor m_driveMotorSource;
    private DCMotor m_turningMotorSource;

    private double lastDistanceDrive = 0;
    private double lastDistanceTurning = 0;
  
    private RelativeEncoderSim m_driveEncoder;
    private RelativeEncoderSim m_turningEncoder;

    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);
    private final PIDController m_turningPIDController = new PIDController(1, 0, 0);

    public SwerveModuleSim() {
        m_turningMotorSource = DCMotor.getNeo550(1);

        m_driveMotorSource = DCMotor.getNeo550(1);

        m_driveMotor = new DCMotorSim(m_driveMotorSource, Constants.gearBoxRatio, Constants.momentaryInertia);
        m_turningMotor = new DCMotorSim(m_turningMotorSource, Constants.gearBoxRatio, Constants.momentaryInertia);
        
        double driveDistancePerPulse  = 2 * Math.PI * Constants.wheelRadius / Constants.encoderResolution;
        double turningDistancePerPulse = 2 * Math.PI / Constants.encoderResolution;


        m_driveEncoder = new RelativeEncoderSim(m_driveMotor, driveDistancePerPulse);
        m_turningEncoder = new RelativeEncoderSim(m_turningMotor, turningDistancePerPulse);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    public SwerveModulePosition getPositionDelta() {
        return new SwerveModulePosition(lastDistanceDrive - m_driveEncoder.getDistance(), new Rotation2d(lastDistanceTurning - m_turningEncoder.getDistance()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));
    
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);    
        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

        if (turnOutput == 0) {
            m_turningPIDController.reset();
        }
        
        m_driveMotor.setInputVoltage(driveOutput + 0);
        m_turningMotor.setInputVoltage(turnOutput + 0);

        lastDistanceDrive = m_driveEncoder.getDistance();
        lastDistanceTurning = m_turningEncoder.getDistance();

        m_driveMotor.update(Constants.updateTime);
        m_driveEncoder.update();

        m_turningMotor.update(Constants.updateTime);
        m_turningEncoder.update();
    }

    public double getDriveEncoderDistance() {
        return m_driveEncoder.getDistance();
    }

    public double getTurningEncoderDistance() {
        return m_turningEncoder.getDistance();
    }

    public double getDriveMotorOutput() {
        return MathUtil.clamp(m_driveMotor.getCurrentDrawAmps(), 0, Constants.maxMotorAmpage) / Constants.maxMotorAmpage;
    }

    public double getTurningMotorOutput() {
        return MathUtil.clamp(m_turningMotor.getCurrentDrawAmps(), 0, Constants.maxMotorAmpage) / Constants.maxMotorAmpage;
    }
}

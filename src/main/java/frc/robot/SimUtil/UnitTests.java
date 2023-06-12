package frc.robot.SimUtil;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveDriveSim;
import frc.robot.Subsystems.SwerveModuleSim;

public class UnitTests {
    public static void testAll(SwerveDriveSim swerveDrive) {
        // SwerveModuleSim swerveModuleSim = makeSwerveModule();
        // RelativeEncoderSim relativeEncoderSim = makeRelativeEncoder();
        // SwerveModulePosition swerveModulePosition = getSwerveModulePosition();
        // SwerveModulePosition swerveModulePositionFromSettingPosition = setSwerveModulePosition();
        // DriveSim swerveField = makeField();

        makeSwerveModule();
        makeRelativeEncoder();
        getSwerveModulePosition();
        setSwerveModulePosition();

        testRelativeEncoderSetAndGet(Math.PI);
        testEncoderSimRotation();

        testSwerveDriveOdometry(swerveDrive);
        testSwerveDrive(swerveDrive, 1, 1, 1, false);
        testSwerveDriveOdometryAndDrive(swerveDrive, 1, 1, 1, false);
    }

    public static SwerveModuleSim makeSwerveModule() {
        return new SwerveModuleSim();
    }

    public static RelativeEncoderSim makeRelativeEncoder() {
        DCMotor motor = DCMotor.getNEO(1);
        DCMotorSim motorSim = new DCMotorSim(motor, 1, 1);
        return new RelativeEncoderSim(motorSim, 1);
    }

    public static SwerveDriveSim makeSwerveDrive() {
        return new SwerveDriveSim();
    }

    public static SwerveModulePosition getSwerveModulePosition() {
        SwerveModuleSim swerveModule = new SwerveModuleSim();
        return swerveModule.getPosition();
    }

    public static SwerveModulePosition setSwerveModulePosition() {
        SwerveModuleSim swerveModule = new SwerveModuleSim();
        SwerveModuleState swerveModuleState = new SwerveModuleState();
        swerveModule.setDesiredState(swerveModuleState);
        return swerveModule.getPosition();
    }

    public static void testRelativeEncoderSetAndGet(double setDistance) {
        DCMotor motor = DCMotor.getNEO(1);
        DCMotorSim motorSim = new DCMotorSim(motor, 1, 1);
        RelativeEncoderSim relativeEncoder = new RelativeEncoderSim(motorSim, 1);
        relativeEncoder.setDistance(setDistance);
        if (relativeEncoder.getDistance() != setDistance) {
            throw new Error("RelativeEncoderSim class does get function does not return proper value that was set");
        }
    }

    public static void testEncoderSimRotation() {
        DCMotor motor = DCMotor.getNEO(1);
        DCMotorSim motorSim = new DCMotorSim(motor, 1, 1);
        RelativeEncoderSim relativeEncoder = new RelativeEncoderSim(motorSim, 1);
        motorSim.setInputVoltage(Constants.nominalVoltage);
        motorSim.update(Constants.updateTime);
        relativeEncoder.update();
        if (relativeEncoder.getDistance() == 0) {
            throw new Error("RelativeEncoderSim class does not update cummulative rotations.");
        }
    }

    public static void testSwerveDrive(SwerveDriveSim swerveDriveSim, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        swerveDriveSim.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    public static void testSwerveDriveOdometry(SwerveDriveSim swerveDriveSim) {
        swerveDriveSim.updateOdometry();
    }

    public static void testSwerveDriveOdometryAndDrive(SwerveDriveSim swerveDriveSim, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        swerveDriveSim.drive(xSpeed, ySpeed, rot, fieldRelative);
        swerveDriveSim.updateOdometry();
    }
}

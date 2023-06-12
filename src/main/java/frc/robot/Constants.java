package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final double wheelRadius = 0.5;
    public static final double encoderResolution = 4096;


    public static final double moduleMaxAngularVelocity = 2 * Math.PI;
    public static final double moduleMaxAngularSpeed = Math.PI;
    public static final double moduleMaxAngularAcceleration = 2 * Math.PI;
    public static final double maxSpeed = 1;
    public static final double momentaryInertia = 0.0005;
    public static final Pose2d initFieldPosition = new Pose2d(new Translation2d(1, 3), new Rotation2d(0));

    public static final double gearBoxRatio = 10;
    public static final double slewRate = 3;
    public static final double maxMotorAmpage = 2;

    public static final double swerveModuleDistanceFromOtherModule = 1;

    public static final int nominalVoltage = 12;
    public static final double updateTime = 0.02;

    public static final int conrollerPort = 0;
    public static final double deadBand = 0.25;

    public static final double moduleMaxAngularSpeedSim = 1;
    public static final double moduleMaxSpeedSim = 0.2;
}

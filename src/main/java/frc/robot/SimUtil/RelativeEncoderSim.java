package frc.robot.SimUtil;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RelativeEncoderSim {
    private DCMotorSim m_motor;
    private double m_rotations = 0;
    private double m_distancePerPulse;

    public RelativeEncoderSim(DCMotorSim m_motor, double m_distancePerPulse) {
        if (m_distancePerPulse == 0) {
            throw new Error("distancePerPulse paramater must not be equal to zero.");
        }

        this.m_motor = m_motor;
        this.m_distancePerPulse = m_distancePerPulse;
    }

    public void update() {
        m_rotations = m_motor.getAngularPositionRad();
    }

    public double getDistance() {
        return m_rotations;
    }

    public void setDistance(double value) {
        m_rotations = value;
    }

    public double getRawRate() {
        return m_motor.getAngularVelocityRadPerSec();
    }

    public double getRate() {
        return m_motor.getAngularVelocityRadPerSec() * m_distancePerPulse;
    }
}

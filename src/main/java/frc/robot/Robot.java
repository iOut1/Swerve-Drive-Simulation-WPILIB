// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SimUtil.FieldSim;
import frc.robot.SimUtil.UnitTests;
import frc.robot.Subsystems.SwerveDriveSim;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(Constants.conrollerPort);
  private final SwerveDriveSim m_swerveDrive = new SwerveDriveSim();
  private FieldSim m_fieldSim = new FieldSim(m_swerveDrive, m_controller);

  @Override
  public void robotInit() {
      SmartDashboard.putBoolean("Drive Field Relative", true);
      UnitTests.testAll(m_swerveDrive);
      m_fieldSim.initalize();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    // m_fieldSim.simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putBoolean("Drive Field Relative", true);
  }

  @Override
  public void disabledInit() {
    m_fieldSim.initalize();
  }
}

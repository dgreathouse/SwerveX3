// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.k;
import frc.robot.subsystems.Chassis;

public class ChassisDefaultCommand extends CommandBase {
  Chassis m_chassis;
  private SlewRateLimiter m_speedLimiterX = new SlewRateLimiter(3);
  private SlewRateLimiter m_speedLimiterY = new SlewRateLimiter(3);
  private SlewRateLimiter m_speedLimiterRot = new SlewRateLimiter(3);

  /** Creates a new ChassisDefaultCommand. */

  public ChassisDefaultCommand(Chassis _subsystem) {
    addRequirements(_subsystem);
    m_chassis = _subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -m_speedLimiterX.calculate(MathUtil.applyDeadband(RobotContainer.m_controllerDriver.getLeftY(), 0.02)) * k.SWERVE.driveMax_MPS;
    double ySpeed = -m_speedLimiterY.calculate(MathUtil.applyDeadband(RobotContainer.m_controllerDriver.getLeftX(), 0.02)) * k.SWERVE.driveMax_MPS;
    double rotSpeed = -m_speedLimiterRot.calculate(MathUtil.applyDeadband(RobotContainer.m_controllerDriver.getRightX(), 0.02)) * k.SWERVE.steerMax_RadPS;

    RobotContainer.chassis.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

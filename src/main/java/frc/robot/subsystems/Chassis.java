// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
import frc.robot.lib.SwerveModule;


public class Chassis extends SubsystemBase {
  private SwerveModule m_LF;
  private SwerveModule m_RF;
  private SwerveModule m_B;

 private AHRS m_gyyo_1;
 private ADXRS450_Gyro m_gyro_2;

 public int m_defaultGyro = 1;

 private final SwerveDriveOdometry m_odometry;

  public Chassis() {
    
    m_LF = new SwerveModule(k.SWERVE.LFData);
    m_RF = new SwerveModule(k.SWERVE.RFData);
    m_B = new SwerveModule(k.SWERVE.BData);

   m_gyyo_1 = new AHRS(Port.kMXP);
   m_gyro_2 = new ADXRS450_Gyro(Port.kOnboardCS0);

   m_gyyo_1.reset();
   m_gyro_2.reset();

   m_odometry = new SwerveDriveOdometry(k.CHASSIS.kinematics, getGyroAngle(), 
   new SwerveModulePosition[] {
     m_LF.getPosition(),
     m_RF.getPosition(),
     m_B.getPosition()
   });
   
   // TODO: May have to reset the odometry since the wheels are not set at this point of first creation. 
  }
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative){
    var swerveModleStates = k.CHASSIS.kinematics.toSwerveModuleStates(
      _fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, getGyroAngle())
      : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModleStates,k.SWERVE.driveMax_MPS);
    m_LF.setDesiredState(swerveModleStates[0]);
    m_RF.setDesiredState(swerveModleStates[1]);
    m_B.setDesiredState(swerveModleStates[2]);
  }
  private Rotation2d getGyroAngle(){
    return m_defaultGyro == 1 ? m_gyyo_1.getRotation2d() : m_gyro_2.getRotation2d();
  }
  @Override
  public void periodic() {
    m_odometry.update(getGyroAngle(), new SwerveModulePosition[] {
      m_LF.getPosition(),
      m_RF.getPosition(),
      m_B.getPosition()});
  }
}

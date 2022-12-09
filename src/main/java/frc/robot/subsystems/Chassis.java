// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
  /** Creates a new Chassis. */
  public Chassis() {
    m_LF = new SwerveModule(k.SWERVE.LFData);
    m_RF = new SwerveModule(k.SWERVE.RFData);
    m_B = new SwerveModule(k.SWERVE.BData);

   m_gyyo_1 = new AHRS(Port.kMXP);
   m_gyro_2 = new ADXRS450_Gyro(Port.kOnboardCS0);

   m_gyyo_1.calibrate();
   m_gyro_2.calibrate();


  }
  public void drive(double _xSpeed, double _ySpeed, double _rot){
    
  }
  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}

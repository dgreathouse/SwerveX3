// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.k;

/** Add your docs here. */
public class SwerveModule {

    WPI_TalonFX m_driveMot;
    WPI_TalonFX m_steerMot;
    CANCoder m_swerveEncoder;
    SwerveData m_data;
    private PIDController m_drivePidController = new PIDController(1, 0, 0);
    private ProfiledPIDController m_steerPIDController = 
        new ProfiledPIDController(0, 0, 0, 
        new TrapezoidProfile.Constraints(k.CHASSIS.angularMax_RadPS, k.CHASSIS.angularMax_RadPSS));

    private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3, 0);
    private SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(1, 0.5,0);

    public SwerveModule(SwerveData _data){
        m_data = _data;
        m_driveMot = new WPI_TalonFX(_data.driveCANID);
        m_steerMot = new WPI_TalonFX(_data.steerCANID);
        m_swerveEncoder = new CANCoder(_data.canCoderCANID);

        m_driveMot.configVoltageCompSaturation(12.0, 20);
        m_driveMot.enableVoltageCompensation(true);

        m_steerMot.configVoltageCompSaturation(12.0, 20);
        m_steerMot.enableVoltageCompensation(true);

        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_swerveEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);


    }
    /**
     * 
     * @return Disitance in Meters
     */
    public double getDriveDistance(){
        final double dis = m_driveMot.getSelectedSensorPosition();
        final double meters = dis / k.SWERVE.driveDistanceCntsPMeter;
        return meters;
    }
    /**
     * 
     * @return Velocity in Meters/s
     */
    public double getDriveVelocity(){
        // Get the FalconFX velocity in raw units /100 ms
        final double vel1 = m_driveMot.getSelectedSensorVelocity();
        // Convert to Meters/s
        final double velocity = vel1 / k.SWERVE.driveRawVelocityToMPS;
        return velocity;
    }
    public double getSteerAngle(){
        return m_steerMot.getSelectedSensorPosition() / k.SWERVE.steer_CntsPRad;
    }
    public double getSteerVelocity(){
        return m_steerMot.getSelectedSensorVelocity() / k.SWERVE.steerVelRatio;
        
    }
    
    /**
     * 
     * @return Angle in Radians of steer motor
     */
    public double getSwerveAngle(){
        return Math.toRadians(m_swerveEncoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getSteerAngle()));
    }
    public void setDesiredState(SwerveModuleState _desiredState){
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(_desiredState, new Rotation2d(getSteerAngle()));
        // Calculate the drive output from the drive PID controller.
        double driveOutput = m_drivePidController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double steerOutput = m_steerPIDController.calculate(getSteerAngle(), state.angle.getRadians());
        double steerFeedforward = m_steerFeedforward.calculate(m_steerPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber(m_data.name + "_SFF",steerFeedforward);
        SmartDashboard.putNumber(m_data.name + "_DFF",driveFeedforward);
        SmartDashboard.putNumber(m_data.name + "_SPIDOut",steerOutput);
        SmartDashboard.putNumber(m_data.name + "_DPIDOut",driveOutput);
        SmartDashboard.putNumber("Batt", RobotController.getBatteryVoltage());
        
        m_driveMot.setVoltage(driveOutput + driveFeedforward);
        m_steerMot.setVoltage(steerOutput + steerFeedforward);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class BallShooter extends SubsystemBase {
  /** Creates a new BallShooter. */

  
  private static double period = 0.1;
  private static double wheelCircumference = 0.1; //in meters

  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.flyWheelID);
  //private WPI_TalonSRX rightFlyWheel = new WPI_TalonSRX(Constants.rightFlyWheelID);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(Constants.feedWheelID);
  private Trigger breakBeam = new Trigger();
  private PIDController pid = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
  private double speed; 
  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  //private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  public BallShooter() {
    flyWheel.configFactoryDefault();
    //rightFlyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.setInverted(false);
    //rightFlyWheel.configFactoryDefault();
    feedWheel.setInverted(false);

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //rightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    pid.setSetpoint(0.0);

    pid.setTolerance(0.05);
  }

  public double getRPM(){ 
    return ((flyWheel.getSelectedSensorVelocity() * 10)/4096.0)*60;
    //Converts ticks per 0.1 seconds to RPM
  }
  /*public double getRightRPM(){ //Among Us In Real Life!
    return ((rightFlyWheel.getSelectedSensorVelocity() * 10)/4096.0)*wheelCircumference;
  }*/
  
 /* public void setFlySpeed(double targetSpeed){
    //pid.setSetpoint(targetSpeed);
    flyWheel.setVoltage((leftFlywheelFF.calculate(targetSpeed))/12.0 + pid.calculate(getRPM(), targetSpeed));
  //flyWheel.set(ControlMode.PercentOutput, pid.calculate(getRPM(), targetSpeed));
  //rightFlyWheel.setVoltage((rightFlywheelFF.calculate(targetSpeed))/12.0 + pid.calculate(getRightRPM(), targetSpeed));
}*/
  public void spin(double speed){
    flyWheel.set(ControlMode.PercentOutput, speed);
  }
  public void stopFlywheel(){
    flyWheel.set(ControlMode.PercentOutput, 0);
    //rightFlyWheel.set(ControlMode.PercentOutput, 0);
  }
  public void setFeedOnOff(boolean onOrOff){
    if (onOrOff) {
      feedWheel.set(ControlMode.PercentOutput, Constants.feedSpeed);
    }
    else {
      feedWheel.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
    //rightFlyWheel.setSelectedSensorPosition(0);
  }

  public void setMode(double newSpeed){
    speed = newSpeed;
  }
  @Override
  public void periodic() {
    // assume period is 0.1 seconds
    // Take care of Fly Wheel first
    //breakBeam.whenActive(timer.start());
    SmartDashboard.putNumber("rpm", getRPM());
    SmartDashboard.putNumber("Voltage set", leftFlywheelFF.calculate(Constants.midSpeed)/12.0 + pid.calculate(getRPM(), Constants.midSpeed));
    //double drivePowerLevel = pid.calculate(getRPM());
    //flyWheel.set(ControlMode.PercentOutput, drivePowerLevel);

    // Take care of Feed Wheel second
    
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
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

  private static double kp = 0.07;
  private static double kd = 0.0;
  private static double ki = 0.0;
  private static double period = 0.1;

  private Timer timer = new Timer();
  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.flyWheelID);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(Constants.feedWheelID);
  private Trigger breakBeam = new Trigger();
  private PIDController pid = new PIDController(kp, ki, kd, period);
  private double ticksToRPM = (Constants.encoderTicks)/1000;
  private double speed = Constants.midSpeed;
  public BallShooter() {
    flyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.setInverted(false);
    feedWheel.setInverted(false);

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    pid.setSetpoint(0.0);
  }

  public double getRPM(){
    return(flyWheel.getSelectedSensorVelocity());
  }

  public void setFlySpeed(double targetSpeed){
    //pid.setSetpoint(targetSpeed);
  flyWheel.set(ControlMode.PercentOutput, pid.calculate(getRPM(), targetSpeed));
  }
  public void setFeedOnOff(boolean onOrOff){
    if (onOrOff) {
      feedWheel.set(ControlMode.PercentOutput, 0.0);
    }
    else {
      feedWheel.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
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
    if(RobotContainer.getJoystick().getRawButtonPressed(Constants.speedUpButton)){
      setFlySpeed(speed);
    }
    else if(RobotContainer.getJoystick().getRawButtonPressed(Constants.stopButton)){
      flyWheel.set(ControlMode.PercentOutput, 0);
    }
    //double drivePowerLevel = pid.calculate(getRPM());
    //flyWheel.set(ControlMode.PercentOutput, drivePowerLevel);

    // Take care of Feed Wheel second
    if(RobotContainer.getJoystick().getTriggerPressed()){
      setFeedOnOff(false);
    }
    else{
      setFeedOnOff(true);
    }
    
  }
}
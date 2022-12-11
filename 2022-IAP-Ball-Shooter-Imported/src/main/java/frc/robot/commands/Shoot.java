// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.leftFlywheelFF;
import frc.robot.subsystems.BallShooter;

public class Shoot extends CommandBase {
  private BallShooter shooter = new BallShooter();
  private double speed;
  private PIDController pid;
  private SimpleMotorFeedforward leftFlywheelFF;
  /** Creates a new Shoot. */
  public Shoot(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
    pid = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  public double getSpeed(){
    return speed;
  }
  @Override
  public void initialize() {
    shooter.resetEncoder();
    pid.setSetpoint(speed*2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //shooter.setFlySpeed((leftFlywheelFF.calculate(speed))/12.0 + pid.calculate(shooter.getRPM(), speed));
      shooter.spin(pid.calculate(shooter.getRPM()));
      SmartDashboard.putNumber("Speed Diff", pid.calculate(shooter.getRPM()));
      //feedForward divided by 12 due to 12 volt battery
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.getJoystick().getRawButtonPressed(Constants.stopButton);
  }
}

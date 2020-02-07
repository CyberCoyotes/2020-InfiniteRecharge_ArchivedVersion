/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Robot extends TimedRobot {
  WPI_TalonFX left = new WPI_TalonFX(1);
  WPI_TalonFX right = new WPI_TalonFX(2);
  WPI_TalonFX leftshoot = new WPI_TalonFX(3);
  WPI_TalonFX rightshoot = new WPI_TalonFX(4);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);
  
  Limelight limelight = new Limelight();

  Joystick driver = new Joystick(0);

  //6ft  0.3
  //7ft 0.35
  //8ft 0.39
  //9ft 0.47

  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController vision_rot = new PIDController(0.0333, 0, 0);
  PIDController shooterSpeedPID = new PIDController(0.0001, 0, 0);

  TalonFXSensorCollection leftShootEnc = leftshoot.getSensorCollection();

  @Override
  public void robotInit() {
    leftShootEnc.getIntegratedSensorVelocity();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    double x = driver.getRawAxis(0);
    double y = driver.getRawAxis(1);
    if(!driver.getRawButton(1) && (Math.abs(x)>= 0.075 || Math.abs(y)>= 0.075)) {
      mainDrive.curvatureDrive(x, y,false); //TODO: add quickturn logic
    } else if(limelight.hasValidTarget()) {
      double vision_x = vision_rot.calculate(limelight.getX());
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity(), getSpeed(0));

      mainDrive.arcadeDrive(0, vision_x);

    } else { 
      mainDrive.curvatureDrive(0, 0, false);
    }

    if(driver.getRawButton(2)) {
      leftshoot.set(0.5);
      SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());
    } else {
      leftshoot.set(0);
    }

    SmartDashboard.putNumber("Limelight width", limelight.getWidth());
  }

  @Override
  public void testPeriodic() {
  }

  public double getSpeed(double distance) {
    return 0.047*distance + 0.072;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {
  WPI_TalonFX left = new WPI_TalonFX(1);
  WPI_TalonFX right = new WPI_TalonFX(2);
  WPI_TalonFX leftshoot = new WPI_TalonFX(3);
  WPI_TalonFX rightshoot = new WPI_TalonFX(4);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);
  
  Limelight limelight = new Limelight();

  Joystick driver = new Joystick(0);

  final I2C.Port i2cPort = I2C.Port.kOnboard;//No touch
  final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);//No touch
  final ColorMatch colorMatcher = new ColorMatch();//No touch

  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); //Yes touch!
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController vision_rot = new PIDController(0.0333, 0, 0);
  PIDController shooterSpeedPID = new PIDController(0.0001, 0, 0);

  TalonFXSensorCollection leftShootEnc = leftshoot.getSensorCollection();

  DigitalInput proximity_sensor = new DigitalInput(0);

  @Override
  public void robotInit() {
    leftShootEnc.getIntegratedSensorVelocity();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 
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
      mainDrive.curvatureDrive(x, y, Math.abs(y) < 0.1); 
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

    Color detectedColor = colorSensor.getColor(); //No touch
    String colorString; //No touch
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);//No touch

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);//No touch
    SmartDashboard.putNumber("Green", detectedColor.green);//No touch
    SmartDashboard.putNumber("Blue", detectedColor.blue);//No touch

    SmartDashboard.putNumber("Confidence", match.confidence);//No touch
    SmartDashboard.putString("Detected Color", colorString);//No touch
    SmartDashboard.putNumber("Limelight width", limelight.getWidth());
    SmartDashboard.putBoolean("Proximity Sensor", proximity_sensor.get());
  }

  @Override
  public void testPeriodic() {
  }

  public double getSpeed(double distance) {
    return 0.047*distance + 0.072;
  }
}

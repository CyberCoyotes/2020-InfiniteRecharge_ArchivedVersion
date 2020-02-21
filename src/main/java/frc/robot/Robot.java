/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {
  WPI_TalonFX left = new WPI_TalonFX(1);
  WPI_TalonFX right = new WPI_TalonFX(2);
  WPI_TalonFX leftshoot = new WPI_TalonFX(8);
  WPI_TalonFX rightshoot = new WPI_TalonFX(7);
  WPI_TalonFX advanceBelt = new WPI_TalonFX(3);
  WPI_TalonFX advanceThing = new WPI_TalonFX(4);
  WPI_VictorSPX intake = new WPI_VictorSPX(5);

  DifferentialDrive mainDrive = new DifferentialDrive(left, right);
  SpeedControllerGroup shooter = new SpeedControllerGroup(leftshoot, rightshoot);
  Limelight limelight = new Limelight();

  Joystick driver = new Joystick(0);

  final I2C.Port i2cPort = I2C.Port.kOnboard;//This tells the RoboRIO where it can communicate with the color sensor
  final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);//This is the actual object that senses colors
  final ColorMatch colorMatcher = new ColorMatch();//This is basically an object that does a lot of math to calculate what color is seen

  //                                              Red   Green   Blue
  //Every visible color can be described in different proportions of red, green and blue.
  //These objects defines these proportions from a scale of 0 to 1, with 0 being it does
  //not use that part (like red) or 1, meaning LOTS of red. Theoretically, the color yellow
  //would be "1.0, 1.0, 0.0", but unfortunately things don't work that way.
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); //Yes touch!
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController vision_rot = new PIDController(0.0333, 0, 0);
  PIDController shooterSpeedPID = new PIDController(0.0001, 0, 0); //Tune me!

  TalonFXSensorCollection leftShootEnc = leftshoot.getSensorCollection();
  ////Encoder rotenc = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
  

  DigitalInput proximity_sensor = new DigitalInput(0);//Infrared sensor

  @Override
  public void robotInit() {
    ////rotenc.setDistancePerPulse(1./2048);
    rightshoot.setInverted(true);
    leftShootEnc.getIntegratedSensorVelocity();

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 

  }

  @Override
  public void robotPeriodic() {
    ////SmartDashboard.putNumber("shooter rotations", rotenc.getDistance());
  
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    final double x = driver.getRawAxis(0);
    final double y = driver.getRawAxis(1);

    //This part represents the drive code. The first part does the threshholding as
    //normal, and includes a get raw button thing that will override the manual drive
    //and switch to vision-targeting mode.
    if (!driver.getRawButton(1) && (Math.abs(x) >= 0.075 || Math.abs(y) >= 0.075)) { 
      mainDrive.curvatureDrive(x, y, Math.abs(y) < 0.1);
    } else if (driver.getRawButton(1) && limelight.hasValidTarget()) { //If the driver is pulling the trigger and the limelight has a target, go into vision-targeting mode
      final double vision_x = vision_rot.calculate(limelight.getX()); //Calculate how fast the
      final double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity(), getSpeed(0));

      //shooter.set(shooterSpeed);

      mainDrive.arcadeDrive(0, vision_x);

    } else {
      mainDrive.curvatureDrive(0, 0, false);
    }

    if (driver.getRawButton(1)) {
      shooter.set(0.80);
    } else {
      shooter.set(0);
    }

    final Color detectedColor = colorSensor.getColor(); // No touch
    String colorString; // No touch
    final ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);// No touch

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

    SmartDashboard.putNumber("Red", detectedColor.red);// No touch
    SmartDashboard.putNumber("Green", detectedColor.green);// No touch
    SmartDashboard.putNumber("Blue", detectedColor.blue);// No touch

    SmartDashboard.putNumber("Confidence", match.confidence);// No touch
    SmartDashboard.putString("Detected Color", colorString);// No touch
    SmartDashboard.putNumber("Limelight width", limelight.getWidth());
    SmartDashboard.putBoolean("Proximity Sensor", proximity_sensor.get());
    
    SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());
  }

  @Override
  public void testPeriodic() {

    if (driver.getRawButton(1)) {
      shooter.set(1.0);
    } else {
      shooter.set(0);
    }

    SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());

    /**
    double measuredSpeed = ......;
    if(driver.getRawButton(1)) {
      shooterSpeedPID.setSetpoint(measuredSpeed/2);
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity());

      shooter.set(shooterSpeed);
    } else {
      shooter.set(0);
    }

    SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());
    */
  }

  public double getSpeed(final double distance) {
    return 0.047*distance + 0.072;
  }
}

/**
 * 1) Shooting code: Set the shooter motors to 1.0 speed and look at the
 *    encoder velocity on the SmartDashboard. This will essentially be
 *    the fastest encoder velocity possible. Use this function- 
 *    "shooterSpeedPID.setSetpoint(measuredSpeed/2.);". You can then begin
 *    the PID tuning process. Use the SmartDashboard to see if the speed
 *    is crazy or stable. If it's crazy, change the value of P in the PID
 *    thing at the top of the code. If the motor seems like it isn't 
 *    moving as fast as it should, increase P.
 */
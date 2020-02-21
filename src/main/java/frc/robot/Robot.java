/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

  WPI_TalonFX left = new WPI_TalonFX(1);
  WPI_TalonFX right = new WPI_TalonFX(2);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_TalonFX advanceBelt = new WPI_TalonFX(3);
  WPI_TalonFX advanceThing = new WPI_TalonFX(4);
  WPI_VictorSPX intake = new WPI_VictorSPX(5);
  WPI_TalonFX leftshooter = new WPI_TalonFX(8);
  WPI_TalonFX rightshooter = new WPI_TalonFX(7);
  SpeedControllerGroup shooter = new SpeedControllerGroup(leftshooter, rightshooter);

  DoubleSolenoid shifter = new DoubleSolenoid(3, 4); //Flip the order of the numbers if this is backwards
  DoubleSolenoid intakePiston = new DoubleSolenoid(2, 5);
  DoubleSolenoid angler = new DoubleSolenoid(1, 6); 
  Value out = Value.kForward;
  Value in = Value.kReverse;

  final I2C.Port i2cPort = I2C.Port.kOnboard;//This tells the RoboRIO where it can communicate with the color sensor
  final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);//This is the actual object that senses colors
  final ColorMatch colorMatcher = new ColorMatch();//This is basically an object that does a lot of math to calculate what color is seen

  /*Every visible color can be described in different proportions of red, green and blue.
  These objects defines these proportions from a scale of 0 to 1, with 0 being it does
  not use that part (like red) or 1, meaning LOTS of red. Theoretically, the color yellow
  would be "1.0, 1.0, 0.0", but unfortunately things don't work that way. */
  //                                              Red   Green   Blue
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  Limelight limelight = new Limelight();
  AHRS gyro = new AHRS(Port.kMXP); //NavX
  TalonFXSensorCollection leftDriveEnc = left.getSensorCollection();
  TalonFXSensorCollection leftShootEnc = leftshooter.getSensorCollection();
  Joystick driver = new Joystick(0);
  ////Encoder rotenc = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController vision_rot = new PIDController(0.0333, 0, 0);
  PIDController shooterSpeedPID = new PIDController(0.0001, 0, 0); //Tune me!
  PIDController strPID = new PIDController(0.05, 0, 0);
  
  DigitalInput proximity_sensor = new DigitalInput(0);//Infrared sensor
  DigitalInput slot1 = new DigitalInput(2); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(3);
  DigitalInput slot3 = new DigitalInput(4);
  AutonType autonMode; //This is used to select which auton mode the robot must do
  int step; //This keeps track of which step of the auton sequence it is in

  final static double lowGear = Math.PI*4.0/30680.0;

  @Override
  public void robotInit() {
    ////rotenc.setDistancePerPulse(1./2048);
    rightshooter.setInverted(true);

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
		if(slot1.get()) { //If the first position of the auton switch is selected, choose the leftSide auton
			autonMode = AutonType.leftSide;
		} else if(slot2.get()) { //If the second position of the auton switch is selected, chose the middle auton
			autonMode = AutonType.middle;
		} else if(slot3.get()) { //If the third position of the auton switch is selected, choose the right auton
			autonMode = AutonType.rightSide;
		} else { //If the fourth position of the auton switch is selected, choose to just drive straight
			autonMode = AutonType.straight;
    }

    if(autonMode == null) { //This is an emergency backup operation in case the auton selection failed
			autonMode = AutonType.straight;
			System.out.println("ERROR: autonMode is null. Defaulting to cross auto line.");
    }

    strPID.setSetpoint(0.0); //Set the drive-straight PID setpoint to 0 degrees
  }

  @Override
  public void autonomousPeriodic() {
    switch(autonMode) { //This takes the auton mode selected and executes its corresponding function
      case leftSide:
        leftSide();
      break;
      case middle:
        middle();
      break;
      case rightSide:
        rightSide();
      break;
      case straight:
        straight();
      break;
    }
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
      //final double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity(), getSpeed(0));
      //shooter.set(shooterSpeed);
      mainDrive.arcadeDrive(0, vision_x);

    } else {
      mainDrive.curvatureDrive(0, 0, false);
    }

    if(driver.getRawButton(2)) {
      shifter.set(out);
    } else {
      shifter.set(in);
    }

    if (driver.getRawButton(1)) {
      shooter.set(0.80);
    } else {
      shooter.set(0);
    }

    read(); //No touch, this puts data onto the SmartDashboard
  }

  void read() { //Put any SmartDashboard stuff here!
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
    SmartDashboard.putNumber("Drive Encoder", leftDriveEnc.getIntegratedSensorPosition());

    if(driver.getRawButtonPressed(12)) {
      limelight.setPipeline(2);
    }
    if(driver.getRawButton(12)) {
      SmartDashboard.putNumber("Limelight distance", limelight.getDistance());
    }
    if(driver.getRawButtonReleased(12)) {
      limelight.setPipeline(1);
    }
  }

  @Override
  public void testPeriodic() {

    if (driver.getRawButton(1)) {
      shooter.set(1.0);
    } else {
      shooter.set(0);
    }

    /**
    double measuredSpeed = ......;
    if(driver.getRawButton(1)) {
      shooterSpeedPID.setSetpoint(measuredSpeed/2);
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity());

      shooter.set(shooterSpeed);
    } else {
      shooter.set(0);
    }

    */

    read(); //No touch, this puts data onto the SmartDashboard
  }

  public double getSpeed(final double distance) {
    return 0.047*distance + 0.072;
  }

  void leftSide() {
    switch(step) {
      case 1:
        if(true) {

        } else {
          step = 2;
        }
      break;
      case 2:
        if(true) {

        } else {
          step = 3;
        }
      break;
      case 3:
        if(true) {

        } else {
          step = 4;
        }
    }
  }

  void middle() {
    switch(step) {
      case 1:
        if(true) {

        } else {
          step = 2;
        }
      break;
    }
  }

  void rightSide() {
    switch(step) {
      case 1:
        if(true) {

        } else {
          step = 2;
        }
      break;
    }
  }

  void straight() { //Drive [leaveDistance] inches - figure out how far the robot must move to leave the line (you get points)
    double leaveDistance = 30.0;
    switch(step) { //This takes in which step of the auton sequence and executes its correlated actions
      case 1: //Step 1:
        if(getDriveDistance() < leaveDistance) { //If the robot has driven less than [leaveDistance]...
          double turnSpeed = strPID.calculate(gyro.getAngle()); //Use the drive-straight PID to calculate its turn speed
          mainDrive.arcadeDrive(0.3, turnSpeed); //Drive at 0.3 speed forwards and correct its heading with the turnSpeed
        } else {//If the robot has driven past [leaveDistance]
          step = 2; //Set the step to 2. Because there is no step 2 programmed, this will stop the auton (which is a good thing)
          mainDrive.arcadeDrive(0, 0); //Stop the robot
        }
      break;
    }
  }

  double getDriveDistance() { //This function uses the drive encoder to calculate how far the robot has driven
    return leftDriveEnc.getIntegratedSensorPosition() * lowGear;
  }

  enum AutonType {//A list of different auton types
		rightSide, middle, leftSide, straight
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
 * 
 * 2) Auton code: We need to know the gear ratio of the drive motors. This
 *    is important because the robot needs to be able to calculate how many
 *    inches it has driven. You will also need to know the diameter of the
 *    drive wheels for the same purpose. Write them here:
 *    Gear ratio:              (Note: this should look like 1:50 [or whatever number it is])
 *    Wheel diameter:
 */
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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  WPI_TalonFX left1 = new WPI_TalonFX(1);
  WPI_TalonFX left2 = new WPI_TalonFX(2);
  WPI_TalonFX right1 = new WPI_TalonFX(3);
  WPI_TalonFX right2 = new WPI_TalonFX(4);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_TalonFX lifter = new WPI_TalonFX(10);
  WPI_TalonFX advanceBelt = new WPI_TalonFX(5);
  WPI_TalonFX accelerator = new WPI_TalonFX(6);
  WPI_VictorSPX intake = new WPI_VictorSPX(7);
  WPI_VictorSPX hopper = new WPI_VictorSPX(11);
  WPI_TalonFX leftShooter = new WPI_TalonFX(8);
  WPI_TalonFX rightShooter = new WPI_TalonFX(9);
  SpeedControllerGroup shooter = new SpeedControllerGroup(leftShooter, rightShooter);

  DoubleSolenoid shifter = new DoubleSolenoid(4, 3); //Flip the order of the numbers if this is backwards
  DoubleSolenoid intakePiston = new DoubleSolenoid(2, 5);
  DoubleSolenoid angler = new DoubleSolenoid(1, 6); 
  Value out = Value.kForward;
  Value in = Value.kReverse;

  Limelight limelight = new Limelight();
  AHRS gyro = new AHRS(Port.kMXP); //NavX
  //CTREEncoder leftDriveEnc = new CTREEncoder(left1, false);
  //CTREEncoder leftShootEnc = new CTREEncoder(leftShooter, false);
  TalonFXSensorCollection leftDriveEnc = left1.getSensorCollection();
  TalonFXSensorCollection leftShootEnc = leftShooter.getSensorCollection();
  Joystick driver = new Joystick(0);
  Joystick manip = new Joystick(1);
  ////Encoder rotenc = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController turnPID = new PIDController(0.02, 0.05, 0);
  PIDController shooterSpeedPID = new PIDController(0.00003, 0.0002, 0);
  PIDController strPID = new PIDController(0.05, 0, 0);
  
  DigitalInput proximity_sensor = new DigitalInput(0);//Infrared sensor
  DigitalInput slot1 = new DigitalInput(2); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(3);
  DigitalInput slot3 = new DigitalInput(4);
  AutonType autonMode; //This is used to select which auton mode the robot must do
  int step; //This keeps track of which step of the auton sequence it is in

  final static double lowGear = Math.PI*4.0/30680.0;
  boolean onTarget = false;
  double lastTurn = 1;

  @Override
  public void robotInit() {
    rightShooter.setInverted(true);
    accelerator.setInverted(true);
    intake.setInverted(true);
  }

  @Override
  public void disabledPeriodic() {
    read();
  }

  @Override
  public void robotPeriodic() {
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

    //DRIVER CONTROLS//
    final double rot = driver.getRawAxis(2);
    final double y = driver.getRawAxis(1);

    if(!driver.getRawButton(1) && Math.abs(rot) >= 0.09 || Math.abs(y) >= 0.09) {
      mainDrive.curvatureDrive(-y, -rot, Math.abs(y) < 0.1);
      shooter.set(0.0);
    } else if(driver.getRawButton(1) && limelight.hasValidTarget()) { //If the driver is pulling the trigger and the limelight has a target, go into vision-targeting mode
      
      turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
      double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading

      if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
        if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
          turnPID.reset(); //If the robot must change its direction, reset its PID
        }
      }
      mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
      lastTurn = rotationSpeed; //Record the current speed into the previous speed

      shooterSpeedPID.setSetpoint(5000); //TODO: make this change based on distance
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
      shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
      shooter.set(shooterSpeed); //Power the flywheel

      double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
      double turnError = Math.abs(limelight.getX()); //Calculate the error
      if(shooterError < 100.0 && turnError < 1.0) { //If both are in range, signal the drivers
        onTarget = true;
      } else {
        onTarget = false;
      }
    } else {
      mainDrive.arcadeDrive(0, 0);
      shooter.set(0.0);
    }
    if(driver.getRawButtonReleased(1)) { //If the driver no longer wants to target...
      turnPID.reset(); //Reset the PIDs and turn the onTarget indicator to false
      shooterSpeedPID.reset();
      onTarget = false;
    }

    if(driver.getRawButton(2)) {
      shifter.set(out);
    } else {
      shifter.set(in);
    }


      //MANIP CONTROLS//
    double acceleratorwheel = manip.getRawAxis(2);
    double shoot = manip.getRawAxis(2);
    double advancer = manip.getRawAxis(1);
    double hoppermove = manip.getRawAxis(1);
    if(Math.abs(shoot) >= 0.075) {
      accelerator.set(acceleratorwheel/2);
    } else {
      accelerator.set(0);
    }

    if(Math.abs(advancer) >= 0.075) {
      advanceBelt.set(advancer);
      hopper.set(hoppermove);
    } else {
      advanceBelt.set(0);
      hopper.set(hoppermove);
    }
    if(manip.getRawButton(4)) {
      angler.set(in);
    } else {
      angler.set(out);
    }
    if(manip.getRawButton(1)) {
      intakePiston.set(out);
      intake.set(.5);
    } else {
      intakePiston.set(in);
      intake.set(0);
    }

    read(); //Put data onto the SmartDashboard
  }


  void read() {
    SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Drive Encoder", leftDriveEnc.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Limelight X Angle", limelight.getX());
    SmartDashboard.putNumber("Distance", (91.0-42.75) / Math.tan(limelight.getY() * Math.PI/180.));
    SmartDashboard.putBoolean("On Target", onTarget);
  }
  
  @Override
  public void testPeriodic() {

    final double rot = driver.getRawAxis(2);
    final double y = driver.getRawAxis(1);
    if(!driver.getRawButton(1) && Math.abs(rot) >= 0.09 || Math.abs(y) >= 0.09) {
      mainDrive.arcadeDrive(-y, -rot);
    } else if (driver.getRawButton(1) && limelight.hasValidTarget()) { //If the driver is pulling the trigger and the limelight has a target, go into vision-targeting mode
      turnPID.setSetpoint(0.0);
      double rotationSpeed = turnPID.calculate(limelight.getX());
      if(rotationSpeed/Math.abs(rotationSpeed) != lastTurn/Math.abs(lastTurn)) {
        turnPID.reset();
      }
      mainDrive.arcadeDrive(0.0, rotationSpeed);
      System.out.println(limelight.getX());
      lastTurn = rotationSpeed;
    } else {
      turnPID.reset();
      mainDrive.arcadeDrive(0, 0);
    }
    if(driver.getRawButtonReleased(1)) {
      turnPID.reset();
    }
    /*
    double lift = manip.getRawAxis(5);
    if(Math.abs(lift) >= 0.75) {
      lifter.set(-lift/2);
    } else {
      lifter.set(0);
    }
     */
    /*******
    double measuredSpeed = 10000.0;
    if(driver.getRawButton(1)) {
      shooterSpeedPID.setSetpoint(measuredSpeed);
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity());
      shooterSpeed = Math.abs(shooterSpeed);

      shooter.set(shooterSpeed);
      System.out.println(leftShootEnc.getIntegratedSensorVelocity());
    } else {
      shooter.set(0);
    }
    */

    read(); //Put data onto the SmartDashboard
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
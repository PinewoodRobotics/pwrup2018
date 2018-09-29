/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4765.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	public TalonSRX rightMaster = new TalonSRX(3);
	public TalonSRX leftMaster = new TalonSRX(8);
	public TalonSRX frontElevator = new TalonSRX(1);
	public TalonSRX backElevator = new TalonSRX(9);
	public TalonSRX bottomRoller = new TalonSRX(2);
	public TalonSRX topRoller = new TalonSRX(4);
	
	private static final double gripVoltage=0.5;
	private static final double steadyVoltage=0.25;
	private static final double releaseVoltage=-0.25;
	private static final double intakeLiftCurrent=6;
	private static final double intakeLiftHeight = 2000;
	private static int counter =100;
	
	public Joystick stick = new Joystick(0);
	public Joystick opTable = new Joystick(1);
	
	public Boolean currentHeightIsSet = false;
	public double currentHeight = 0;


	public Boolean amInClimbMode;
	public double frontHeightWhenReset = 0;
	public double backHeightWhenReset = 0;

	public int gripState;

	public AHRS ahrs = new AHRS(SPI.Port.kMXP); 

	public enum OwnedSide {
		LEFT, RIGHT, UNKNOWN
	}

	public enum GameFeature {
		SWITCH_NEAR, SCALE, SWITCH_FAR
	}

	public enum AutonSetting {
		STAY, MOVE_ONLY, RIGHT_CORNER_SCORE_SAME, LEFT_CORNER_SCORE_SAME, RIGHT_CORNER_SCORE_OPPO, LEFT_CORNER_SCORE_OPPO, MIDDLE_SCORE_RIGHT, MIDDLE_SCORE_LEFT
	}


	public OwnedSide nearSwitch;
	public OwnedSide scale;
	public OwnedSide farSwitch;

	public double currentDistance;
	public int autonStep;
	public double targetDistance;
	public Boolean didSetTargetDistance = false;

	public Boolean autonInitHasRun = false;
	
	/**
	 * JOYSTICK CONTROLS:
	 * 
	 * Changing throttle: changes robot throttle (up 0%, down 100%)
	 * Twist joy stick: rotate robot (clockwise goes clockwise, counterclockwise goes counterclockwise)
	 * Push joy stick forward: changes robot speed (up 100%, down -100%)
	 * Button 1: release block
	 * Button 2: intake block
	 * Button 3: lift after intake
	 * Button 4: 
	 * Button 5: 
	 * Button 6: 
	 * Button 7: climb mode on
	 * Button 8: climb mode off
	 * POV Up: raise elevators
	 * POV Down: lower elevators
	 * 
	 * OPERATOR CONTROLS:
	 * 
	 * Button 1: auton stay
	 * Button 2: auton move
	 * Button 3:
	 * Button 4: 
	 * Button 5: 
	 * Button 6: climb mode off
	 * Button 7: 
	 * Button 8: lower elevator
	 * Button 9: raise elevator
	 * Button 10: lift after intake
	 * Button 11: release block
	 * Button 12: intake block
	 */
	
	private static final int RELEASE_BLOCK_JOYSTICK = 1;
	private static final int INTAKE_BLOCK_JOYSTICK = 2;
	private static final int LIFT_INTAKE_JOYSTICK = 3;
	private static final int CLIMB_ON_JOYSTICK = 7;
	private static final int CLIMB_OFF_JOYSTICK = 8;
	
	private static final int CLIMB_OFF_OPERATOR = 6;
	private static final int LOWER_OPERATOR = 8;
	private static final int RAISE_OPERATOR = 9;
	private static final int LIFT_INTAKE_OPERATOR = 10;
	private static final int RELEASE_BLOCK_OPERATOR = 11;
	private static final int INTAKE_BLOCK_OPERATOR = 12;
	
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		amInClimbMode = false;
		
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);


		leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 0);

		//to reverse sensor use .reverseSensor() method
		frontElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		backElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		frontElevator.setSelectedSensorPosition(0, 0, 0);
		backElevator.setSelectedSensorPosition(0, 0, 0);

		//PID Constant Config Here
		frontElevator.selectProfileSlot(0, 0);
		frontElevator.config_kF(0, 0, 0);
		frontElevator.config_kP(0, 1.0, 0);
		frontElevator.config_kD(0, 0, 0);
		frontElevator.config_kI(0, 0, 0);
		frontElevator.config_IntegralZone(0, 100, 0);

		backElevator.selectProfileSlot(0, 0);
		backElevator.config_kF(0, 0, 0);
		backElevator.config_kP(0, 1.5, 0);
		backElevator.config_kD(0, 0, 0);
		backElevator.config_kI(0, 0, 0);
		backElevator.config_IntegralZone(0, 100, 0);

		leftMaster.selectProfileSlot(0, 0);
		leftMaster.config_kF(0, 0.75, 0);
		leftMaster.config_kP(0, 1.5, 0);
		leftMaster.config_kD(0, 0.2, 0);
		leftMaster.config_kI(0, 0.02, 0);
		leftMaster.config_IntegralZone(0, 100, 0);

		rightMaster.selectProfileSlot(0, 0);
		rightMaster.config_kF(0, 0.75, 0);
		rightMaster.config_kP(0, 1.5, 0);
		rightMaster.config_kD(0, 0.2, 0);
		rightMaster.config_kI(0, 0.02, 0);
		rightMaster.config_IntegralZone(0, 100, 0);

		leftMaster.setSelectedSensorPosition(0, 0, 0);
		rightMaster.setSelectedSensorPosition(0, 0, 0);

		gripState=0;
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		//compressor.start();
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		nearSwitch = getOwnedSide(GameFeature.SWITCH_NEAR, gameData);
		scale = getOwnedSide(GameFeature.SCALE, gameData);
		farSwitch = getOwnedSide(GameFeature.SWITCH_FAR, gameData);
		autonStep = -1;
		leftMaster.setSelectedSensorPosition(0, 0, 0);
		didSetTargetDistance = false;
		currentDistance = 0;
		autonInitHasRun = true;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if(autonInitHasRun) {
			autonInitHasRun = false;
		}
		
		if(autonStep == -1) {
			if(finishedElevatingFrontDistanceInInchesTo(1)) {
				autonStep = 0;
			}
		}
		if(opTable.getRawButton(1)) {
			//STAY
		} else if(opTable.getRawButton(2)) {
			//MOVE
			autonCornerMoveOnly();
		}
	}

	@Override
	public void disabledInit() {
		autonInitHasRun = false;
	}

	public void autonCornerMoveOnly() {
		if(autonStep == 0) {
			if(finishedDrivingDistanceInInches(95)) {
				autonStep = 1;
			}
		}
	}

	public Boolean finishedDrivingDistanceInInches(double distance) {
		currentDistance = leftMaster.getSelectedSensorPosition(0);
		if(!didSetTargetDistance) {
			targetDistance = currentDistance + (distance * 76.336);
			didSetTargetDistance = true;
		}
		if(targetDistance - Math.abs(currentDistance) < 0) {
			driveCalculate(0, 0, 0);
			leftMaster.setSelectedSensorPosition(0, 0, 0);
			didSetTargetDistance = false;
			return true;
		} else {
			driveCalculate(0.2, 0.5, 0);
			return false;
		}
	}
	
	public OwnedSide getOwnedSide(GameFeature feature, String gsm) {

		int index = feature.ordinal();

		if (gsm == null)
			return OwnedSide.UNKNOWN;

		// If the length is less than 3, it's not valid. Longer than 3 is permitted, but only
		// the first 3 characters are taken.
		if (gsm.length() < 3)
			return OwnedSide.UNKNOWN;

		// Theoretically this should never happen, but it's good to be safe.
		if (index >= 3 || index < 0)
			return OwnedSide.UNKNOWN;

		char ownedObj = gsm.charAt(index);
		switch (ownedObj) {
		case 'L':
		case 'l':
			return OwnedSide.LEFT;
		case 'R':
		case 'r':
			return OwnedSide.RIGHT;
		default:
			return OwnedSide.UNKNOWN;
		}
	}

	@Override
	public void teleopInit() {
		amInClimbMode = false;
		gripState=0;
	}

	/**
	 * This function is called periodically during operator control.
	 */

	@Override
	public void teleopPeriodic() {
		//DRIVE lOGIC
		double throttle = stick.getThrottle();
		double forward = -stick.getY();
		double twist = stick.getZ();

		if (Math.abs(throttle) < 0.15) {
			throttle = 0;
		}
		if (Math.abs(twist) < 0.3) {
			twist = 0;
		}
		if (Math.abs(forward) < 0.1) {
			forward = 0;
		}

		twist = twist * 0.75;

		driveCalculate(throttle, forward, twist);

		//ELEVATOR LOGIC
		//climb mode

		if(!(opTable.getRawButton(CLIMB_OFF_OPERATOR))) {
			toggleClimbMode(false);
		} else {
			toggleClimbMode(true);
		}
		
		//holding elevate button
		if(stick.getPOV() == 0 || opTable.getRawButton(RAISE_OPERATOR)) {
			elevate();
			//holding deElevate button
		} else if(stick.getPOV() == 180 || opTable.getRawButton(LOWER_OPERATOR)) {
			deElevate();
		} else {
			if (currentHeightIsSet == false) {
				currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
				currentHeightIsSet = true;
			}
			keepElevatorSteady(currentHeight);
		}

		//GRIPPER LOGIC
		//way out of all states if something's wrong
		if (gripState==0 || gripState==1 || gripState==2) {
			if (stick.getRawButtonPressed(RELEASE_BLOCK_JOYSTICK) || opTable.getRawButtonPressed(RELEASE_BLOCK_OPERATOR)) {
				gripState=4;
				counter=100;
			}
		}
		
		//intake if thumb or operator button 12
		if (gripState==0) {
			System.out.println("inState0");
			if (stick.getRawButtonPressed(INTAKE_BLOCK_JOYSTICK) || opTable.getRawButtonPressed(INTAKE_BLOCK_OPERATOR)) {
				gripState=1;
			} else {
				stopBlock();
			}
		}
		//lift if current gets to limit or button
		//fix all of this; buttons and volts
		else if (gripState==1) {
			System.out.println("inState1");
			if (stick.getRawButtonPressed(LIFT_INTAKE_JOYSTICK) || opTable.getRawButtonPressed(LIFT_INTAKE_OPERATOR) ||	topRoller.getOutputCurrent()>=intakeLiftCurrent){
				gripState=2;
			} else {
				gripBlock();
			}
		}
		//if lift to height where box isn't dragging then switch to hold
		else if (gripState==2){
			System.out.println ("inState2");
			currentHeight=2000;
			currentHeightIsSet=true;
			gripState=3;
		}
		//trigger to release
		else if (gripState==3) {
			System.out.println ("inState3");
			if (stick.getRawButtonPressed(RELEASE_BLOCK_JOYSTICK) || opTable.getRawButtonPressed(RELEASE_BLOCK_OPERATOR)) {
				gripState=4;
				counter=100;
			} else{
				keepBlockSteady();
			}
		}
		//stop after release
		else if (gripState==4) {
			System.out.println ("inState4");
		/*	if ((stick.getRawButtonPressed(RELEASE_BLOCK_JOYSTICK)==false) && (opTable.getRawButtonPressed(RELEASE_BLOCK_OPERATOR)==false)) {
				gripState=0;
			}else {
				releaseBlock();
				*/
			if (counter>0) {
				counter --;
				releaseBlock();
			} else {
				gripState=0;
			}
		}
		System.out.print("Top Current: " + topRoller.getOutputCurrent());
		System.out.print("\tTop Voltage: " + topRoller.getMotorOutputVoltage());
		System.out.print("\tBottom Current: " + bottomRoller.getOutputCurrent());
		System.out.print("\tBottm Voltage: " + bottomRoller.getMotorOutputVoltage());
		System.out.println("\tCurrent Height: " + currentHeight);
	}

	public void stopBlock() {
		topRoller.set(ControlMode.Current, 0);
		bottomRoller.set(ControlMode.Current, 0);
	}
	
	public void gripBlock() {
		//setting bottom and top rollers at current required for grip
		//topRoller.set(ControlMode.Current, gripCurrent);
		//bottomRoller.set(ControlMode.Current, gripCurrent);
		topRoller.set(ControlMode.PercentOutput, gripVoltage);
		bottomRoller.set(ControlMode.PercentOutput, gripVoltage);
	}

	public void releaseBlock() {
		//setting bottom and top rollers at current required for release
		topRoller.set(ControlMode.PercentOutput,releaseVoltage);
		bottomRoller.set(ControlMode.PercentOutput, releaseVoltage);
	}

	public void keepBlockSteady() {
		//setting bottom and top rollers at current required for hold
		topRoller.set(ControlMode.PercentOutput, steadyVoltage);
		bottomRoller.set(ControlMode.PercentOutput, steadyVoltage);
	}
	
	public void driveCalculate(double currentThrottle, double currentForward, double currentTwist) {
		double throttle = (currentThrottle + 1)/2;
		double forward = currentForward * throttle;
		double twist = currentTwist * throttle;
		double leftSpeed = forward + twist;
		double rightSpeed = -forward + twist;

		double max = Math.abs(leftSpeed);
		if (Math.abs(rightSpeed) > max) {
			max = Math.abs(rightSpeed);
		}
		if (max > 1) {
			rightSpeed = rightSpeed/max;
			leftSpeed = leftSpeed/max;
		}
		rightSpeed = rightSpeed * 1100;
		leftSpeed = leftSpeed * 1100;
		//		//some conversion there
		rightMaster.set(ControlMode.Velocity, rightSpeed);
		leftMaster.set(ControlMode.Velocity, leftSpeed);
		System.out.println(rightMaster.getSelectedSensorVelocity(0));
		System.out.println(leftMaster.getSelectedSensorVelocity(0));
	}
	
	public void liftAfterIntake() {
		/*currentHeightIsSet = false;
		frontElevator.config_kP(0, 1.5, 0);
		backElevator.config_kP(0, 1.5, 0);
		{
			double currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight + 728; // TODO: check this position
			double frontHeight;
			double backHeight;
			if (newHeight > 28500) {
				frontHeight = 28500;
				backHeight = newHeight - 28500;
				if (backHeight > 25400) {
					backHeight = 25400;
				}
			} else {
				frontHeight = newHeight;			
				backHeight = 0;
			}
			frontElevator.set(ControlMode.Position, frontHeight);
			backElevator.set(ControlMode.Position, backHeight);
			*/
		currentHeight=2000;
		currentHeightIsSet=true;
		}
	

	public void elevate() {
		currentHeightIsSet = false;
		frontElevator.config_kP(0, 1.5, 0);
		backElevator.config_kP(0, 1.5, 0);
		if(amInClimbMode == true) {
			double currentHeight = backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight + 728;
			double backHeight;
			backHeight = newHeight;
			if (backHeight > 21760) {
				backHeight = 21760;
			}
			backElevator.set(ControlMode.Position, backHeight);
		} else {
			double currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight + 728;
			double frontHeight;
			double backHeight;
			if (newHeight > 28500) {
				frontHeight = 28500;
				backHeight = newHeight - 28500;
				if (backHeight > 25400) {
					backHeight = 25400;
				}
			} else {
				frontHeight = newHeight;			
				backHeight = 0;
			}
			frontElevator.set(ControlMode.Position, frontHeight);
			backElevator.set(ControlMode.Position, backHeight);
		}
	}

	public void deElevate() {
		currentHeightIsSet = false;
		frontElevator.config_kP(0, 1.0, 0);
		backElevator.config_kP(0, 1.0, 0);
		if(amInClimbMode == true) {
			double currentHeight = backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight - 728;
			double backHeight;
			backHeight = newHeight;
			if (backHeight < 91) {
				backHeight = 91;
			}
			backElevator.set(ControlMode.Position, backHeight);
		} else {
			double currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight - 728;
			double frontHeight;
			double backHeight;
			if (newHeight > 28500) {
				frontHeight = 28500;
				backHeight = newHeight - 28500;
				if (backHeight > 25400) {
					backHeight = 25400;
				}
			} else {
				if (newHeight < 182) {
					newHeight = 182;
				}
				frontHeight = newHeight;			
				backHeight = 0;
			}
			frontElevator.set(ControlMode.Position, frontHeight);
			backElevator.set(ControlMode.Position, backHeight);
		}
	}

	public void keepElevatorSteady(double height) {
		if(amInClimbMode == true) {
			double newHeight = height;
			double backHeight;
			backHeight = newHeight;
			backElevator.set(ControlMode.Position, backHeight);
		} else {
			double newHeight = height;
			double frontHeight;
			double backHeight;
			if (newHeight > 28500) {
				frontHeight = 28500;
				backHeight = newHeight - 28500;
				if (backHeight > 25400) {
					backHeight = 25400;
				}
			} else {
				frontHeight = newHeight;
				backHeight = 0;
			}
			frontElevator.set(ControlMode.Position, frontHeight);
			backElevator.set(ControlMode.Position, backHeight);
		}
	}



	public void toggleClimbMode(Boolean toggleOn) {
		if(amInClimbMode == toggleOn) {
			/*System.out.println("Not Doing Anything");
			System.out.println("ToggleOn:" + toggleOn);
			System.out.println("climbMode:" + amInClimbMode);*/
			return;
		} else {
			if (toggleOn) {
				frontHeightWhenReset = frontElevator.getSelectedSensorPosition(0);
				backHeightWhenReset = backElevator.getSelectedSensorPosition(0);
				frontElevator.set(ControlMode.Position, 91);
				backElevator.set(ControlMode.Position, 728);
				amInClimbMode = true;
				System.out.println("Toggling On");
				System.out.println("ToggleOn:" + toggleOn);
				System.out.println("climbMode:" + amInClimbMode);
			} else {
				frontElevator.set(ControlMode.Position, frontHeightWhenReset);
				backElevator.set(ControlMode.PercentOutput, backHeightWhenReset);
				amInClimbMode = false;
				System.out.println("Toggling Off");
				System.out.println("ToggleOn:" + toggleOn);
				System.out.println("climbMode:" + amInClimbMode);
			}
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}

	public Boolean finishedElevatingFrontDistanceInInchesTo(double height) {
		frontElevator.config_kP(0, 1.5, 0);
		double newHeight = height * 728;
		if(newHeight > 28242) {
			newHeight = 28242;
		}
		if(frontElevator.getSelectedSensorPosition(0) < newHeight - 364) {
			frontElevator.set(ControlMode.Position, newHeight);
			return false;
		} else {
			frontElevator.set(ControlMode.Position, newHeight);
			return true;
		}
	}

	//front elevator max is 28500
	//back elevator max is 25400
	public void elevateTest(TalonSRX elevator, double max) {
		currentHeightIsSet = false;
		elevator.config_kP(0, 0.75, 0);
		double currentHeight = elevator.getSelectedSensorPosition(0);
		double height = currentHeight + 728;
		if (height > max) {
			height = max;
		}
		elevator.set(ControlMode.Position, height);
	}

	public void deElevateTest(TalonSRX elevator) {
		currentHeightIsSet = false;
		elevator.config_kP(0, 0.75, 0);
		double currentHeight = elevator.getSelectedSensorPosition(0);
		double height = currentHeight - 728;
		if (height < 91) {
			height = 91;
		}
		elevator.set(ControlMode.Position, height);
	}

	public void keepElevatorSteadyTest(TalonSRX elevator, double height) {
		elevator.set(ControlMode.Position, height);
	}
}

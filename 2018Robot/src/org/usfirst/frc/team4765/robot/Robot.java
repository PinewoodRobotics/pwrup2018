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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
	public TalonSRX rightSlave = new TalonSRX(2);
	public TalonSRX leftMaster = new TalonSRX(8);
	public TalonSRX leftSlave = new TalonSRX(4);
	//public Compressor compressor = new Compressor(32);
	//public Solenoid grip = new Solenoid(32, 2);
	//public Solenoid release = new Solenoid(32, 3);
	//public Solenoid forwardPush = new Solenoid(32, 0);
	//public Solenoid backwardPull = new Solenoid(32, 1);
	public TalonSRX frontElevator = new TalonSRX(1);
	public TalonSRX backElevator = new TalonSRX(9);
	public TalonSRX bottomRoller = new TalonSRX(99);
	public TalonSRX topRoller = new TalonSRX(98);
	private static final double gripCurrent=0;
	private static final double steadyCurrent=0;
	private static final double releaseCurrent=0;
	
	public Joystick stick = new Joystick(0);
	public Joystick opTable = new Joystick(1);
	
	public double maxCurrent = 0;
	
	public Boolean currentHeightIsSet = false;
	public double currentHeight = 0;
	
	public DigitalInput autoPickupSensor = new DigitalInput(0);
	
	public Boolean amInClimbMode;
	public double frontHeightWhenReset = 0;
	public double backHeightWhenReset = 0;
	
	public Boolean autoPickupEnabled;
	public Boolean amSupposedToBeGripping;
	public Boolean amSupposedToBePushing;
	public int gripState;
	
	public AHRS ahrs = new AHRS(SPI.Port.kMXP); 
	
	double maxLeftV = 0;
	double maxRightV = 0;
	double maxBackP = 0;
	double maxFrontP = 0;

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
	public double currentAngle;
	public int autonStep;
	public double targetAngle;
	public Boolean didSetTargetAngle = false;
	public double targetDistance;
	public Boolean didSetTargetDistance = false;
	
	public int stallingMS;
	
	public Boolean autonInitHasRun = false;
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
		autoPickupEnabled = false;
		amSupposedToBeGripping = false;
		amSupposedToBePushing = false;
		
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		

		leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 0);
		
		//to change direction use .setInverted() method
		rightSlave.follow(rightMaster);
		leftSlave.follow(leftMaster);
		
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
		
		stallingMS = 0;
		//gripState=0;
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
		didSetTargetAngle = false;
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
		if(leftMaster.getSelectedSensorVelocity(0) * 10/76.4 < 2) {
			stallingMS += 1;
		} else {
			stallingMS = 0;
		}
//		switch (m_autoSelected) {
//		case kCustomAuto:
//		// Put custom auto code here
//		break;
//	case kDefaultAuto:
//	default:
//		// Put default auto code here
//		break;
	if(autonStep == -1) {
		if(finishedElevatingFrontDistanceInInchesTo(1)) {
			gripBlock();
			amSupposedToBeGripping = true;
			autonStep = 0;
		}
	}
	if(opTable.getRawButton(1)) {
	    //STAY
	} else if(opTable.getRawButton(2)) {
	    //MOVE
	    autonCornerMoveOnly();
	} else if(opTable.getRawButton(3)) {
	    //START RIGHT
	    if(nearSwitch == OwnedSide.LEFT) {
	    	autonCornerScoreSame(true);
	    } else {
	        autonCornerScoreOppo(true);
	    }
	} else if(opTable.getRawButton(4)) {
	    //START LEFT
	    if(nearSwitch == OwnedSide.LEFT) {
	    	autonCornerScoreOppo(false);
	    } else {
	        autonCornerScoreSame(false);
	    }
	} else if(opTable.getRawButton(5)) {
	    //START MIDDLE
	    if(nearSwitch == OwnedSide.LEFT) {
	        autonMiddleScoreLeft();
	    } else {
	        autonMiddleScoreRight();
	    }
	}
	}
	
	@Override
	public void disabledInit() {
		autonInitHasRun = false;
	}
	
	 public void autonCornerScoreSame(Boolean amStartingInLeftCorner) {
	        double turnDegrees = 90;
	        if(amStartingInLeftCorner == false) {
	            turnDegrees = -90;
	        }
	        if(autonStep == 0) {
	        	stallingMS = 0;
	            gripBlock();
	            autonStep = 1;
	        } else if(autonStep == 1) {
	            if(finishedDrivingDistanceInInches(145) || stallingMS > 2000) {
	            	stallingMS = 0;
	                autonStep = 2;
	            }
	        } else if(autonStep == 2) {
	            if(finishedTurningAngleInDegrees(turnDegrees)) {
	            	stallingMS = 0;
	                autonStep = 3;
	            }
	        } else if(autonStep == 3) {
	            if(finishedElevatingFrontDistanceInInchesTo(30)) {
	            	stallingMS = 0;
	            	autonStep = 4;
	            }
	        } else if(autonStep == 4) {
	            if(finishedDrivingDistanceInInches(8) || stallingMS > 2000) {
	            	stallingMS = 0;
	                autonStep = 5;
	            }
	        } else if(autonStep == 5) {
	            pushBlockForward();
	            Timer.delay(1);
	            releaseBlock();
	            autonStep = 6;
	        }
	    }
	    //
	        public void autonCornerScoreOppo(Boolean amStartingInLeftCorner) {
	            double turnDegrees = 90;
	            if(amStartingInLeftCorner == false) {
	                turnDegrees = -90;
	            }

	            if(autonStep == 0) {
	                if(finishedDrivingDistanceInInches(216) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 1;
	                }
	            } else if(autonStep == 1) {
	                if(finishedTurningAngleInDegrees(turnDegrees)) {
	                	stallingMS = 0;
	                    autonStep = 2;
	                }
	            } else if(autonStep == 2) {
	                if(finishedDrivingDistanceInInches(180) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 3;
	                }
	            } else if(autonStep == 3) {
	                if(finishedTurningAngleInDegrees(turnDegrees)) {
	                	stallingMS = 0;
	                    autonStep = 4;
	                }
	            } else if(autonStep == 4) {
	                if(finishedElevatingFrontDistanceInInchesTo(30)) {
	                	stallingMS = 0;
	                    autonStep = 5;
	                }
	            } else if(autonStep == 5) {
	            	if(finishedDrivingDistanceInInches(12) || stallingMS > 2000) {
	            		stallingMS = 0;
	                    autonStep = 6;
	                }
	            } else if(autonStep == 7) {
	                pushBlockForward();
	                Timer.delay(1);
	                releaseBlock();
	                autonStep = 8;
	            }
	        }
	    
	        public void autonMiddleScoreRight() {
	            if(autonStep == 0) {
	                if(finishedDrivingDistanceInInches(50) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 1;
	                }
	            } else if(autonStep == 1) {
	                if(finishedTurningAngleInDegrees(90)) {
	                	stallingMS = 0;
	                    autonStep = 2;
	                }
	            } else if(autonStep == 2) {
	                if(finishedDrivingDistanceInInches(46) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 3;
	                }
	            } else if(autonStep == 3) {
	                if(finishedTurningAngleInDegrees(-90)) {
	                	stallingMS = 0;
	                    autonStep = 4;
	                }
	            } else if(autonStep == 4) {
	                if(finishedElevatingFrontDistanceInInchesTo(30)) {
	                	stallingMS = 0;
	                    autonStep = 5;
	                }
	            } else if(autonStep == 5) {
	                if(finishedDrivingDistanceInInches(44) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 6;
	                }
	            } else if(autonStep == 6) {
	                pushBlockForward();
	                Timer.delay(1);
	                releaseBlock();
	                autonStep = 7;
	            }
	        }
	    
	        public void autonMiddleScoreLeft() {
	            if(autonStep == 0) {
	                if(finishedDrivingDistanceInInches(50) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 1;
	                }
	            } else if(autonStep == 1) {
	                if(finishedTurningAngleInDegrees(-90)) {
	                	stallingMS = 0;
	                    autonStep = 2;
	                }
	            } else if(autonStep == 2) {
	                if(finishedDrivingDistanceInInches(60) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 3;
	                }
	            } else if(autonStep == 3) {
	                if(finishedTurningAngleInDegrees(90)) {
	                	stallingMS = 0;
	                    autonStep = 4;
	                }
	            } else if(autonStep == 4) {
	                if(finishedElevatingFrontDistanceInInchesTo(30)) {
	                	stallingMS = 0;
	                    autonStep = 5;
	                }
	            } else if(autonStep == 5) {
	                if(finishedDrivingDistanceInInches(48) || stallingMS > 2000) {
	                	stallingMS = 0;
	                    autonStep = 6;
	                }
	            } else if(autonStep == 6) {
	                pushBlockForward();
	                Timer.delay(1);
	                releaseBlock();
	                autonStep = 7;
	            }
	        }

	    public void autonCornerMoveOnly() {
	        if(autonStep == 0) {
	            if(finishedDrivingDistanceInInches(95)) {
	                autonStep = 1;
	            }
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
		//compressor.start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	
	/**
	 * JOYSTICK CONTROLS:
	 * 
	 * Changing throttle: changes robot throttle (up 0%, down 100%)
	 * Twist joy stick: rotate robot (clockwise goes clockwise, counterclockwise goes counterclockwise)
	 * Push joy stick forward: changes robot speed (up 100%, down -100%)
	 * Button 1: toggle grip on
	 * Button 2: toggle grip off
	 * Button 3: toggle push off
	 * Button 4: disable auto pickup
	 * Button 5: toggle push on
	 * Button 6: enable auto pickup
	 * Button 7: climb mode on
	 * Button 8: climb mode off
	 * POV Up: raise elevators
	 * POV Down: lower elevators
	 * 
	 */
	
	@Override
	public void teleopPeriodic() {
		
		if(opTable.getRawButton(6)) {
			toggleClimbMode(false);
		} else {
			toggleClimbMode(true);
		}
		
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
//		
//		System.out.println(leftMaster.getSelectedSensorPosition(0));
//		System.out.println(rightMaster.getSelectedSensorPosition(0));
//		
//		if (stick.getRawButtonPressed(7) || opTable.getRawButton(6) == false) {
//			toggleClimbMode(true);
//		} else if(stick.getRawButtonPressed(8) || opTable.getRawButton(6)) {
//			toggleClimbMode(false);
//		}
//		
//		if(Math.abs(leftMaster.getSelectedSensorVelocity(0)) > Math.abs(maxLeftV)) {
//			maxLeftV = leftMaster.getSelectedSensorVelocity(0);
//		}
//		if(Math.abs(rightMaster.getSelectedSensorVelocity(0)) > Math.abs(maxRightV)) {
//			maxRightV = rightMaster.getSelectedSensorVelocity(0);
//		}
//		if(Math.abs(frontElevator.getSelectedSensorPosition(0)) > Math.abs(maxFrontP)) {
//			maxFrontP = frontElevator.getSelectedSensorPosition(0);
//		}
//		if(Math.abs(backElevator.getSelectedSensorPosition(0)) > Math.abs(maxBackP)) {
//			maxBackP = backElevator.getSelectedSensorPosition(0);
//		}
//		
//		System.out.println("Left Velocity:" + maxLeftV);
//		System.out.println("Right Velocity:" + maxRightV);
//		System.out.println("Front Position:" + maxFrontP);
//		System.out.println("Back Position" + maxBackP);
		
		//MARK: ELEVATOR LOGIC
		
		//holding elevate button
		if(stick.getPOV() == 0 || opTable.getRawButton(9)) {
			elevate();
			//holding deElevate button
		} else if(stick.getPOV() == 180 || opTable.getRawButton(8)) {
			deElevate();
		} else {
			if (currentHeightIsSet == false) {
				currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
				currentHeightIsSet = true;
			}
			keepElevatorSteady(currentHeight);
		}
				
		//MARK: GRIPPER LOGIC
		
		if (stick.getRawButtonPressed(1) || opTable.getRawButtonPressed(12)) {
			amSupposedToBeGripping = true;
		}
		if (stick.getRawButtonPressed(2) || opTable.getRawButtonPressed(11)) {
			amSupposedToBeGripping = false;
		}
				
		//gripping toggled on
		if(amSupposedToBeGripping) {
			gripBlock();
		} else {
			releaseBlock();
		}
		
		/*NEW GRIPPER LOGIC
		//intake if thumb or operator button 12
		if gripState=0 {
			System.out.Println("State0");
			if (stick.getRawButtonPressed(1) || opTable.getRawButtonPressed(12)) {
			gripState=1;
		}}
	
		//lift if current gets to limit or button
		 //fix all of this; buttons and volts
			if gripState=1 {
				System.out.println("inState1");
				if (stick.getRawButtonPressed(??) || opTable.getRawButtonPressed(??)) ||	topRoller.getMotorOutputVoltage()>=FIND THIS{
			gripState=2;
			}}
		
		//if lift to height where box isn't dragging then switch to hold
		if gripState=2{
		System.out.println ("inState2");
			{if frontElevator.getSelectedSensorPosition>=10; 
			//fixme
				gripState=3;
			}}
			

			//trigger to release
			 if gripState=3 {
			 System.out.println ("inState3");
			 			if (stick.getRawButtonPressed(2) || opTable.getRawButtonPressed(11)) {
							gripState=4;
			}}
			
			//stop after release
			 if gripState=4 {
			 System.out.println ("inState4");
			 		if (stick.getRawButtonPressed(2) || opTable.getRawButtonPressed(11)) {
			 		gripState=0;
			 		}
			 		}
			 		
			 		if gripState=0 {
			 		System.out.println ("inState0");
			 			topRoller.set(ControlMode.Current, 0);
			 			bottomRoller.set(ControlMode.Current, 0);
			 			}
			 			
			 			if gripState=1 {
			 			gripBlock;
			 			}
			 			
			 			if gripState=2{
			 			liftAfterIntake;
			 			}
			 			
			 			if gripState=3 {
			 			keepblockSteady;
			 			}
			 			
			 			if gripState=4 {
			 			releaseBlock;
			 			}
			 			
			 //way out of all states if something's wrong
			if gripState=0 || gripState=1 ||gripState=2 {
				if (stick.getRawButtonPressed(2) || opTable.getRawButtonPressed(11)) {
					gripState=4;
					}}
			 		
		*/
		//MARK: PUSHER LOGIC
		
		if (stick.getRawButtonPressed(5) || opTable.getRawButton(10)) {
			amSupposedToBePushing = true;
		}
		if (stick.getRawButtonPressed(3) || opTable.getRawButton(10) == false) {
			amSupposedToBePushing = false;
		}
		
		//holding push button
		if(amSupposedToBePushing) {
			pushBlockForward();
		} else {
			pullBlockBackward();
		}
		
		//MARK: AUTOPICKUP LOGIC
		
		if (stick.getRawButtonPressed(6)) {
			autoPickupEnabled = true;
		}
		if (stick.getRawButton(4)) {
			autoPickupEnabled = false;
		}
		
		//auto pickup toggled
		if(autoPickupEnabled) {
			if(!autoPickupSensor.get()) {
				amSupposedToBeGripping = true;
				autoPickupEnabled = false;
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
	
	//clockwise is positive, counterclockwise is negative
	public Boolean finishedTurningAngleInDegrees(double angle) {
		currentAngle = ahrs.getYaw();
		if(!didSetTargetAngle) {
			targetAngle = ((currentAngle + angle)%180);
			didSetTargetAngle = true;
		}
		double turnScale = 0;
		double error = Math.abs(Math.abs(turnScale) - Math.abs(currentAngle));
		if((angle) > 0) {
			if(error < 25) {
				turnScale = (0.05 + (error * 0.02));
			} else {
				turnScale = 0.35;
			}
		} else if((angle) < 0) {
			if(error < 25) {
				turnScale = -(0.05 + (error * 0.02));
			} else {
				turnScale = -0.35;
			}
		}
		if(Math.abs(targetAngle - currentAngle) < 2.5) {
			driveCalculate(0, 0, 0);
			leftMaster.setSelectedSensorPosition(0, 0, 0);
			didSetTargetAngle = false;
			return true;
		} else {
			driveCalculate(1, 0, turnScale);
			return false;
		}
	}
	
	public void gripBlock() {
		//grip.set(true);
		//release.set(false);
		//setting bottom and top rollers at current required for grip
		topRoller.set(ControlMode.Current, gripCurrent);
		bottomRoller.set(ControlMode.Current, gripCurrent);
	}
	
	public void releaseBlock() {
	//	release.set(true);
	//	grip.set(false);
		//setting bottom and top rollers at current required for release
		topRoller.set(ControlMode.Current, releaseCurrent);
		bottomRoller.set(ControlMode.Current, releaseCurrent);
	}
	
	
	/* public void liftAfterIntake() {
		currentHeightIsSet = false;
		frontElevator.config_kP(0, 1.5, 0);
		backElevator.config_kP(0, 1.5, 0);
		{
			double currentHeight = frontElevator.getSelectedSensorPosition(0) + backElevator.getSelectedSensorPosition(0);
			double newHeight = currentHeight + 728;
			//fixme, 728
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
	
	 
	 */
	
	/*public void pushBlockForward() {
		//forwardPush.set(true);
		//backwardPull.set(false);
	}
	
	public void pullBlockBackward() {
		//backwardPull.set(true);
		//forwardPush.set(false);
	}*/
	
	public void keepBlockSteady() {
		//backwardPull.set(false);
		//forwardPush.set(false);
		//setting bottom and top rollers at current required for hold
		topRoller.set(ControlMode.Current, steadyCurrent);
		bottomRoller.set(ControlMode.Current, steadyCurrent);
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
				if (newHeight < 728) {
					newHeight = 728;
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
	
	public void toggleClimbMode(Boolean toggleOn) {
		if(amInClimbMode == toggleOn) {
			System.out.println("Not Doing Anything");
			System.out.println("ToggleOn:" + toggleOn);
			System.out.println("climbMode:" + amInClimbMode);
			return;
		} else {
			if (toggleOn) {
				//compressor.stop();
				//grip.set(false);
				//release.set(true);
				frontHeightWhenReset = frontElevator.getSelectedSensorPosition(0);
				backHeightWhenReset = backElevator.getSelectedSensorPosition(0);
				frontElevator.set(ControlMode.Position, 91);
				backElevator.set(ControlMode.Position, 728);
				//forwardPush.set(false);
			//	backwardPull.set(true);
				amInClimbMode = true;
				System.out.println("Toggling On");
				System.out.println("ToggleOn:" + toggleOn);
				System.out.println("climbMode:" + amInClimbMode);
			} else {
				//compressor.start();
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

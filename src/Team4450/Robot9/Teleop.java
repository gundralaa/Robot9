
package Team4450.Robot9;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;

class Teleop
{
	private final Robot 		robot;
	public  JoyStick			rightStick, leftStick, utilityStick;
	public  LaunchPad			launchPad;
	private final FestoDA		shifterValve = new FestoDA(2);
	private final FestoDA		ptoValve = new FestoDA(0);
	private final FestoDA		tiltValve = new FestoDA(1, 0);
	private final FestoDA		climberArmsValve = new FestoDA(1, 2);
	private final FestoDA		defenseArmsValve = new FestoDA(1, 4);
	private boolean				ptoMode = false, invertDrive = false, limitSwitchEnabled = false;
	private boolean				autoTarget = false, climbPrepEnabled = false, climbPrepInProgress = false;
	private double				shooterPower = Shooter.SHOOTER_HIGH_POWER;
	private Relay				headLight = new Relay(0, Relay.Direction.kForward);
	//private final RevDigitBoard	revBoard = RevDigitBoard.getInstance();
	//private final DigitalInput	hallEffectSensor = new DigitalInput(0);
	private final Shooter		shooter;
	private final DigitalInput	climbUpSwitch = new DigitalInput(3);

	private Vision2016					vision = new Vision2016();
	private Vision2016.ParticleReport 	par;
	

	// Encoder is plugged into dio port 1 - orange=+5v blue=signal, dio port 2 black=gnd yellow=signal. 
	private Encoder				encoder = new Encoder(1, 2, true, EncodingType.k4X);
	
	// Encoder ribbon cable to dio ports: ribbon wire 2 = orange, 5 = yellow, 7 = blue, 10 = black

	// Constructor.
	
	Teleop(Robot robot)
	{
		Util.consoleLog();

		this.robot = robot;
		
		shooter = new Shooter(robot, this);
	}

	// Free all objects that need it.
	
	void dispose()
	{
		Util.consoleLog();
		
		if (leftStick != null) leftStick.dispose();
		if (rightStick != null) rightStick.dispose();
		if (utilityStick != null) utilityStick.dispose();
		if (launchPad != null) launchPad.dispose();
		if (shifterValve != null) shifterValve.dispose();
		if (ptoValve != null) ptoValve.dispose();
		if (tiltValve != null) tiltValve.dispose();
		if (climberArmsValve != null) climberArmsValve.dispose();
		if (defenseArmsValve != null) defenseArmsValve.dispose();
		if (shooter != null) shooter.dispose();
		//if (armMotor != null) armMotor.free();
		if (climbUpSwitch != null) climbUpSwitch.free();
		if (encoder != null) encoder.free();
		if (headLight != null) headLight.free();
		//if (revBoard != null) revBoard.dispose();
		//if (hallEffectSensor != null) hallEffectSensor.free();
	}

	void OperatorControl()
	{
		double	rightY, leftY, utilY;
        
        // Motor safety turned off during initialization.
        robot.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();
		
		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, robot.ds.isFMSAttached());
		
		// Initial setting of air valves.

		shifterLow();
		ptoDisable();
		tiltUp();
		climberArmsUp();
		defenseArmsUp();
		shooter.HoodDown();
		
		// Configure LaunchPad and Joystick event handlers.
		
		launchPad = new LaunchPad(robot.launchPad, LaunchPadControlIDs.BUTTON_BLACK, this);
		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_FRONT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;
		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_BACK);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;
		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_RIGHT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_YELLOW);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_GREEN);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED_RIGHT);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE_RIGHT);
        launchPad.addLaunchPadEventListener(new LaunchPadListener());
        launchPad.Start();

		leftStick = new JoyStick(robot.leftStick, "LeftStick", JoyStickButtonIDs.TRIGGER, this);
        leftStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		leftStick.addJoyStickEventListener(new LeftStickListener());
        leftStick.Start();
        
		rightStick = new JoyStick(robot.rightStick, "RightStick", JoyStickButtonIDs.TOP_LEFT, this);
        rightStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		//rightStick.AddButton(JoyStickButtonIDs.TRIGGER);
        rightStick.addJoyStickEventListener(new RightStickListener());
        rightStick.Start();
        
		utilityStick = new JoyStick(robot.utilityStick, "UtilityStick", JoyStickButtonIDs.TOP_LEFT, this);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_RIGHT);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		utilityStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		utilityStick.AddButton(JoyStickButtonIDs.TRIGGER);
        utilityStick.addJoyStickEventListener(new UtilityStickListener());
        utilityStick.Start();
        
        // Set gyro to heading 0.
        robot.gyro.reset();
        
        // Motor safety turned on.
        robot.robotDrive.setSafetyEnabled(true);
        
		// Driving loop runs until teleop is over.

		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			if (ptoMode)
			{
				rightY = utilityStick.GetY();
				
				if (rightY > 0 && climbUpSwitch.get() && limitSwitchEnabled) rightY = 0;

				leftY = rightY;
			} 
//			else if (invertDrive)
//			{
//    			rightY = rightStick.GetY() * -1.0;		// fwd/back right
//    			leftY = leftStick.GetY() * -1.0;		// fwd/back left
//			}
			else
			{
//				rightY = rightStick.GetY();				// fwd/back right
//    			leftY = leftStick.GetY();				// fwd/back left
				rightY = stickLogCorrection(rightStick.GetY());	// fwd/back right
    			leftY = stickLogCorrection(leftStick.GetY());	// fwd/back left
			}
			
			LCD.printLine(3, "encoder=%d  climbUp=%b", encoder.get(), climbUpSwitch.get());
			LCD.printLine(4, "leftY=%.4f  rightY=%.4f", leftY, rightY);
			LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());
			// encoder rate is revolutions per second.
			LCD.printLine(6, "encoder=%d rpm=%.0f pwr=%.2f pwrR=%.2f", shooter.shooterSpeedSource.get(), 
							shooter.shooterSpeedSource.getRate() * 60, shooter.shooterMotorControl.get(),
							shooter.shooterMotorControl.get());
			//LCD.printLine(7, "shooterspeedsource=%.0f", shooter.shooterSpeedSource.pidGet());

			// This corrects stick alignment error when trying to drive straight. 
			//if (Math.abs(rightY - leftY) < 0.2) rightY = leftY;
			
			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			if (!autoTarget && !climbPrepInProgress) robot.robotDrive.tankDrive(leftY, rightY);

			// End of driving loop.
			
			Timer.delay(.020);	// wait 20ms for update from driver station.
		}
		
		// End of teleop mode.
		
		Util.consoleLog("end");
	}

	// Map joystick y value of 0.0-1.0 to the motor working power range of approx 0.5-1.0
	
	private double stickCorrection(double joystickValue)
	{
		if (joystickValue != 0)
		{
			if (joystickValue > 0)
				joystickValue = joystickValue / 1.5 + .4;
			else
				joystickValue = joystickValue / 1.5 - .4;
		}
		
		return joystickValue;
	}
	
	// Custom base logrithim.
	// Returns logrithim base of the value.
	
	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0-1.0 to the motor working power range of approx 0.5-1.0 using
	// logrithmic curve.
	
	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);
		
		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);
			
		return joystickValue;
	}
	
	// Transmission control functions.
	
	//--------------------------------------
	void shifterLow()
	{
		Util.consoleLog();
		
		shifterValve.SetA();

		SmartDashboard.putBoolean("Low", true);
		SmartDashboard.putBoolean("High", false);
	}

	void shifterHigh()
	{
		Util.consoleLog();
		
		shifterValve.SetB();

		SmartDashboard.putBoolean("Low", false);
		SmartDashboard.putBoolean("High", true);
	}
	
	//--------------------------------------
	void ptoDisable()
	{
		Util.consoleLog();
		
		ptoMode = false;
		
		ptoValve.SetA();

		SmartDashboard.putBoolean("PTO", false);
	}
	
	void ptoEnable()
	{
		Util.consoleLog();
		
		ptoValve.SetB();

		ptoMode = true;
		
		SmartDashboard.putBoolean("PTO", true);
	}
	
	// Misc control functions.
	
	//--------------------------------------
	void tiltUp()
	{
		Util.consoleLog();
		
		tiltValve.SetA();
	}

	void tiltDown()
	{
		Util.consoleLog();
		
		tiltValve.SetB();
	}
	
	//--------------------------------------
	void climberArmsUp()
	{
		Util.consoleLog();
		
		climberArmsValve.SetB();
	}

	void climberArmsDown()
	{
		Util.consoleLog();
		
		climberArmsValve.SetA();
	}
	
	//--------------------------------------
	void defenseArmsUp()
	{
		Util.consoleLog();
		
		defenseArmsValve.SetB();
	}

	void defenseArmsDown()
	{
		Util.consoleLog();
		
		defenseArmsValve.SetA();
	}

	//--------------------------------------
	// Automatically prepare for climb. Drive on batter press black button.
	// We deploy kicker, shift to PTO, disable joystick control of motors,
	// jog motors backwards a bit to facilitate shifting to PTO. Then run
	// climber arms up to near max height. Reenable joystick control of motors.
	void climbPrep()
	{
		Util.consoleLog();
		
		tiltDown();		// deploy foot to hold position.
		
		launchPad.FindButton(LaunchPadControlIDs.BUTTON_GREEN).latchedState = true;	// reset button state for tilt.

		climbPrepInProgress = true;
		
		ptoEnable();	// shift to pto mode.
		
		launchPad.FindButton(LaunchPadControlIDs.BUTTON_BLUE).latchedState = true;	// reset button state for pto.

		robot.robotDrive.setSafetyEnabled(false);

		robot.robotDrive.tankDrive(-.4, -.4);	// jog motors.
		
		Timer.delay(.3);

		robot.robotDrive.tankDrive(.8, .8);		// raise arms.
		
		Timer.delay(.6);
		
		robot.robotDrive.tankDrive(0, 0);		// stop arms.

		robot.robotDrive.setSafetyEnabled(true);

		climbPrepInProgress = false;
	}
	
	//--------------------------------------
//	void lightOn()
//	{
//		headLight.set(Relay.Value.kOn);
//		SmartDashboard.putBoolean("Light", true);
//	}
//	
//	void lightOff()
//	{
//		headLight.set(Relay.Value.kOff);
//		rightStick.FindButton(JoyStickButtonIDs.TRIGGER).latchedState = false;
//		SmartDashboard.putBoolean("Light", false);
//	}
	
	/**
	 * Rotate the robot by bumping appropriate motors based on the X offset
	 * from center of camera image. 
	 * @param value Target offset. + value means target is right of center so
	 * run right side motors backwards. - value means target is left of ceneter so run
	 * left side motors backwards.
	 */
	void bump(int value)
	{
		Util.consoleLog("%d", value);

		if (value > 0)
			robot.robotDrive.tankDrive(.60, 0);
		else
			robot.robotDrive.tankDrive(0, .60);
			
		Timer.delay(.10);
	}
	
    /**
     * Check current camera image for the target. 
     * @return A particle report for the target.
     */
	Vision2016.ParticleReport findTarget()
	{
		Util.consoleLog();
		
		par = vision.CheckForTarget(robot.cameraThread.CurrentImage());
		
		if (par != null) Util.consoleLog("Target=%s", par.toString());
		
		return par;
	}
	
	/**
	 * Loops checking camera images for target. Stops when no target found.
	 * If target found, check target X location and if needed bump the bot
	 * in the appropriate direction and then check target location again.
	 */
	void seekTarget()
	{
		Vision2016.ParticleReport par;
		
		Util.consoleLog();

		SmartDashboard.putBoolean("TargetLocked", false);
		SmartDashboard.putBoolean("AutoTarget", true);

		par = findTarget();

		autoTarget = true;
		robot.robotDrive.setSafetyEnabled(false);
		
		while (robot.isEnabled() && autoTarget && par != null)
		{
			if (Math.abs(320 - par.CenterX) > 10)
			{
				bump(320 - par.CenterX);
				
				par = findTarget();
			}
			else
			{
				SmartDashboard.putBoolean("TargetLocked", true);
				par = null;
			}
		}
		
		autoTarget = false;
		robot.robotDrive.setSafetyEnabled(true);

		SmartDashboard.putBoolean("AutoTarget", false);
	}
	
	// Handle LaunchPad control events.
	
	public class LaunchPadListener implements LaunchPadEventListener 
	{
	    public void ButtonDown(LaunchPadEvent launchPadEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.id.name(),  launchPadEvent.control.latchedState);
			
			if (launchPadEvent.control.id.equals(LaunchPad.LaunchPadControlIDs.BUTTON_YELLOW))
				if (launchPadEvent.control.latchedState)
					shooter.HoodUp();
				else
					shooter.HoodDown();
			
			if (launchPadEvent.control.id.equals(LaunchPad.LaunchPadControlIDs.BUTTON_BLACK))
				if (climbPrepEnabled)
					climbPrep();
				else
				{
    				if (launchPadEvent.control.latchedState)
    					shooter.PickupArmDown();
    				else
    					shooter.PickupArmUp();
				}
			
			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_BLUE)
			{
				if (launchPadEvent.control.latchedState)
    				shifterHigh();
    			else
    				shifterLow();
			}

			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_GREEN)
			{
				if (launchPadEvent.control.latchedState)
    				tiltDown();
    			else
    				tiltUp();
			}

			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_BLUE)
			{
				if (launchPadEvent.control.latchedState)
				{
					shifterLow();
					ptoEnable();
				}
    			else
    				ptoDisable();
			}

			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_BLUE_RIGHT)
			{
				if (launchPadEvent.control.latchedState)
					climberArmsDown();
				else
    				climberArmsUp();
				
				// Start auto targeting on button push, stop on next button push.
//				if (!autoTarget)
//					seekTarget();
//				else
//					autoTarget = false;
			}

			if (launchPadEvent.control.id == LaunchPadControlIDs.BUTTON_RED_RIGHT)
			{
//				limitSwitchEnabled = !limitSwitchEnabled;
//				SmartDashboard.putBoolean("LSOverride", !limitSwitchEnabled);

//				if (launchPadEvent.control.latchedState)
//					lightOn();
//				else
//					lightOff();

				if (launchPadEvent.control.latchedState)
					shooter.StartAutoBallSpit();
				else
					shooter.StopAutoBallSpit();
			}
			
			if (launchPadEvent.control.id.equals(LaunchPadControlIDs.BUTTON_RED))
			{
				if (launchPadEvent.control.latchedState)
    				defenseArmsDown();
    			else
    				defenseArmsUp();
			}
	    }
	    
	    public void ButtonUp(LaunchPadEvent launchPadEvent) 
	    {
	    	//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
	    }

	    public void SwitchChange(LaunchPadEvent launchPadEvent) 
	    {
	    	Util.consoleLog("%s", launchPadEvent.control.id.name());

	    	// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (launchPadEvent.control.id.equals(LaunchPadControlIDs.ROCKER_LEFT_FRONT))
				if (launchPadEvent.control.latchedState)
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam2);
				else
					robot.cameraThread.ChangeCamera(robot.cameraThread.cam1);
			
			if (launchPadEvent.control.id.equals(LaunchPadControlIDs.ROCKER_LEFT_BACK))
				if (launchPadEvent.control.latchedState)
				climbPrepEnabled = true;
			else
				climbPrepEnabled = false;
			
//				if (launchPadEvent.control.latchedState)
//					robot.SetCANTalonNeutral(false);	// coast
//				else
//					robot.SetCANTalonNeutral(true);		// brake
			
			if (launchPadEvent.control.id.equals(LaunchPadControlIDs.ROCKER_RIGHT))
				if (launchPadEvent.control.latchedState)
				{
					shooterPower = Shooter.SHOOTER_LOW_POWER;
					SmartDashboard.putBoolean("ShooterLowPower", true);
				}
				else
				{
					shooterPower = Shooter.SHOOTER_HIGH_POWER;
					SmartDashboard.putBoolean("ShooterLowPower", false);
				}
	    }
	}

	// Handle Right JoyStick Button events.
	
	private class RightStickListener implements JoyStickEventListener 
	{
		
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			// Change which USB camera is being served by the RoboRio when using dual usb cameras.
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT))
				if (joyStickEvent.button.latchedState)
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam2);
				else
					((CameraFeed) robot.cameraThread).ChangeCamera(((CameraFeed) robot.cameraThread).cam1);			
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
				shooter.StopShoot();
			
//			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TRIGGER))
//				//invertDrive = joyStickEvent.button.latchedState;
//				if (joyStickEvent.button.latchedState)
//					lightOn();
//				else
//					lightOff();
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.name());
	    }
	}

	// Handle Left JoyStick Button events.
	
	private class LeftStickListener implements JoyStickEventListener 
	{
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TRIGGER))
			{
				if (joyStickEvent.button.latchedState)
    				shifterHigh();
    			else
    				shifterLow();
			}
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
			{
				if (joyStickEvent.button.latchedState)
    				climberArmsDown();
    			else
    				climberArmsUp();
			}
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.name());
	    }
	}

	// Handle Utility JoyStick Button events.
	
	private class UtilityStickListener implements JoyStickEventListener 
	{
	    public void ButtonDown(JoyStickEvent joyStickEvent) 
	    {
			Util.consoleLog("%s, latchedState=%b", joyStickEvent.button.id.name(),  joyStickEvent.button.latchedState);
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TRIGGER))
			{
				//lightOff();
				shooter.StartShoot(false);
			}
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_RIGHT))
				if (joyStickEvent.button.latchedState)
					shooter.StartAutoPickup();
				else
					shooter.StopAutoPickup();
					
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT))
				if (joyStickEvent.button.latchedState)
					shooter.ShooterMotorStart(shooterPower);
				else
					shooter.ShooterMotorStop();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
				if (joyStickEvent.button.latchedState)
					shooter.PickupMotorIn(1.0);
				else
					shooter.PickupMotorStop();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_BACK))
				if (joyStickEvent.button.latchedState)
					shooter.PickupMotorOut(1.0);
				else
					shooter.PickupMotorStop();
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.id.name());
	    }
	}
}

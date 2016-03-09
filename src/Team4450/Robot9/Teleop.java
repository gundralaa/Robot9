
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

class Teleop
{
	private final Robot 		robot;
	private JoyStick			rightStick, leftStick, utilityStick;
	private LaunchPad			launchPad;
	private final FestoDA		shifterValve = new FestoDA(2);
	private final FestoDA		ptoValve = new FestoDA(0);
	private final FestoDA		tiltValve = new FestoDA(1, 0);
	private final FestoDA		armsValve = new FestoDA(1, 2);
	private boolean				ptoMode = false, invertDrive = false;
	//private final RevDigitBoard	revBoard = new RevDigitBoard();
	//private final DigitalInput	hallEffectSensor = new DigitalInput(0);
	public  final Shooter		shooter;
	//private final Talon			armMotor;
	private final DigitalInput	climbUpSwitch = new DigitalInput(3);
	
	// encoder is plugged into dio port 1 - orange=+5v blue=signal, dio port 2 black=gnd yellow=signal. 
	private Encoder				encoder = new Encoder(1, 2, true, EncodingType.k4X);

	// Constructor.
	
	Teleop(Robot robot)
	{
		Util.consoleLog();

		this.robot = robot;
		
		//armMotor = new Talon(2);
		shooter = new Shooter(robot);
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
		if (armsValve != null) armsValve.dispose();
		if (shooter != null) shooter.dispose();
		//if (armMotor != null) armMotor.free();
		if (climbUpSwitch != null) climbUpSwitch.free();
		if (encoder != null) encoder.free();
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
		armsUp();
		
		// Configure LaunchPad and Joystick event handlers.
		
		launchPad = new LaunchPad(robot.launchPad, LaunchPadControlIDs.BUTTON_BLACK, this);
		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_FRONT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_YELLOW);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_GREEN);
		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED);
        launchPad.addLaunchPadEventListener(new LaunchPadListener());
        launchPad.Start();

		leftStick = new JoyStick(robot.leftStick, "LeftStick", JoyStickButtonIDs.TRIGGER, this);
        leftStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		leftStick.addJoyStickEventListener(new LeftStickListener());
        leftStick.Start();
        
		rightStick = new JoyStick(robot.rightStick, "RightStick", JoyStickButtonIDs.TOP_LEFT, this);
        rightStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		rightStick.AddButton(JoyStickButtonIDs.TRIGGER);
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
				
				if (rightY > 0 && climbUpSwitch.get()) rightY = 0;

				leftY = rightY;
			} 
			else if (invertDrive)
			{
    			rightY = rightStick.GetY() * -1.0;		// fwd/back right
    			leftY = leftStick.GetY() * -1.0;		// fwd/back left
			}
			else
			{
				rightY = rightStick.GetY();				// fwd/back right
    			leftY = leftStick.GetY();				// fwd/back left
			}
			
			LCD.printLine(3, "encoder=%d  climbUp=%b", encoder.get(), climbUpSwitch.get());
			LCD.printLine(4, "leftY=%.4f  rightY=%.4f", leftY, rightY);
			LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());

			// This corrects stick alignment error when trying to drive straight. 
			//if (Math.abs(rightY - leftY) < 0.2) rightY = leftY;
			
			// Set wheel motors.

			robot.robotDrive.tankDrive(leftY, rightY);

			// End of driving loop.
			
			Timer.delay(.020);	// wait 20ms for update from driver station.
		}
		
		// End of teleop mode.
		
		Util.consoleLog("end");
	}

	// Transmission control functions.
	
	void shifterLow()
	{
		Util.consoleLog();
		
		shifterValve.SetA();

		SmartDashboard.putBoolean("Low", true);
		SmartDashboard.putBoolean("High", false);
	}
	//--------------------------------------
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
	//--------------------------------------
	void ptoEnable()
	{
		Util.consoleLog();
		
		ptoValve.SetB();

		ptoMode = true;
		
		SmartDashboard.putBoolean("PTO", true);
	}
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
	void armsUp()
	{
		Util.consoleLog();
		
		armsValve.SetB();
	}

	void armsDown()
	{
		Util.consoleLog();
		
		armsValve.SetA();
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
			
			if (launchPadEvent.control.id.equals(LaunchPad.LaunchPadControlIDs.BUTTON_RED))
				if (launchPadEvent.control.latchedState)
					shooter.PickupArmDown();
				else
					shooter.PickupArmUp();
	
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
					robot.SetCANTalonNeutral(false);	// coast
				else
					robot.SetCANTalonNeutral(true);		// brake
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
				shooter.StopAutoShoot();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TRIGGER))
				invertDrive = joyStickEvent.button.latchedState;
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
    				armsDown();
    			else
    				armsUp();
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
				shooter.StartShoot(false);
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_RIGHT))
				if (joyStickEvent.button.latchedState)
					shooter.StartAutoPickup();
				else
					shooter.StopAutoPickup();
					
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_LEFT))
				if (joyStickEvent.button.latchedState)
					shooter.ShooterMotorStart(1);
				else
					shooter.ShooterMotorStop();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_MIDDLE))
				if (joyStickEvent.button.latchedState)
					shooter.PickupMotorIn(1);
				else
					shooter.PickupMotorStop();
			
			if (joyStickEvent.button.id.equals(JoyStickButtonIDs.TOP_BACK))
				if (joyStickEvent.button.latchedState)
					shooter.PickupMotorOut(1);
				else
					shooter.PickupMotorStop();
	    }

	    public void ButtonUp(JoyStickEvent joyStickEvent) 
	    {
	    	//Util.consoleLog("%s", joyStickEvent.button.id.name());
	    }
	}
}

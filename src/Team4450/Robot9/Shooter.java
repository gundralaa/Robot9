package Team4450.Robot9;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter
{
	private final Robot	robot;
	
	//private final CANTalon		pickupMotor = new CANTalon(7);
	private final SpeedController	pickupMotor;
	private final Talon				shooterMotor1 = new Talon(0);
	private final Talon				shooterMotor2 = new Talon(1);
	private final FestoDA			pickupCylinder = new FestoDA(6);
	private final FestoDA			hoodCylinder = new FestoDA(4);
	private final DigitalInput		pickupSwitch = new DigitalInput(0);
	
	private Thread					autoPickupThread, autoShootThread;

	Shooter(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;

		// Handle the fact that the pickup motor is a CANTalon on competition robot
		// and a pwm Talon on clone.
		
		if (robot.robotProperties.getProperty("RobotId").equals("comp")) 
		{
			pickupMotor = new CANTalon(7);
			robot.InitializeCANTalon((CANTalon) pickupMotor);
		}
		else
			pickupMotor = new Talon(7);
		
		PickupArmUp();
		HoodDown();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		if (autoPickupThread != null) autoPickupThread.interrupt();
		if (autoShootThread != null) autoShootThread.interrupt();

		if (pickupMotor != null)
		{
			if (robot.robotProperties.getProperty("RobotId").equals("comp"))
				((CANTalon) pickupMotor).delete();
			else
				((Talon) pickupMotor).free();
		}
		
		if (shooterMotor1 != null) shooterMotor1.free();
		if (shooterMotor2 != null) shooterMotor2.free();
		if (pickupCylinder != null) pickupCylinder.dispose();
		if (hoodCylinder != null) hoodCylinder.dispose();
		if (pickupSwitch != null) pickupSwitch.free();
	}

	public void PickupArmUp()
	{
		Util.consoleLog();
		
		pickupCylinder.SetA();
	}
	//---------------------------------------
	public void PickupArmDown()
	{
		Util.consoleLog();
		
		pickupCylinder.SetB();
	}
	//---------------------------------------
	public void HoodDown()
	{
		Util.consoleLog();
		
		hoodCylinder.SetB();
	}
	//----------------------------------------
	public void HoodUp()
	{
		Util.consoleLog();
		
		hoodCylinder.SetA();
	}
	//----------------------------------------
	public void PickupMotorIn(double speed)
	{
		Util.consoleLog("%f", speed);

		pickupMotor.set(speed);
	}
	//----------------------------------------
	public void PickupMotorOut(double speed)
	{
		Util.consoleLog("%f", speed);
		
		pickupMotor.set(Math.abs(speed) * -1);
	}
	//----------------------------------------
	public void PickupMotorStop()
	{
		Util.consoleLog();
		
		pickupMotor.set(0);
	}
	//----------------------------------------
	public void ShooterMotorStart(double speed)
	{
		Util.consoleLog("%f", speed);
	
		shooterMotor1.set(speed);
		shooterMotor2.set(speed);
	}
	//----------------------------------------
	public void ShooterMotorStop()
	{
		Util.consoleLog();
		
		shooterMotor1.set(0);
		shooterMotor2.set(0);
	}
	//----------------------------------------
	public void StartAutoPickup()
	{
		Util.consoleLog();
		
		if (autoPickupThread != null) return;

		autoPickupThread = new AutoPickup();
		autoPickupThread.start();
	}
	//----------------------------------------
	public void StopAutoPickup()
	{
		Util.consoleLog();

		if (autoPickupThread != null) autoPickupThread.interrupt();
		
		autoPickupThread = null;
	}

	// Automatic ball pickup thread.
	
	private class AutoPickup extends Thread
	{
		AutoPickup()
		{
			Util.consoleLog();
			
			this.setName("AutoPickup");
	    }
		
	    public void run()
	    {
	    	Util.consoleLog();
	    	
	    	try
	    	{
	    		PickupArmDown();
	    		sleep(250);
	    		PickupMotorIn(1.0);
	    		
    	    	while (!isInterrupted() && !pickupSwitch.get())
    	    	{
    	            // We sleep since JS updates come from DS every 20ms or so. We wait 50ms so this thread
    	            // does not run at the same time as the teleop thread.
    	            sleep(50);
    	    	}
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}

	    	PickupMotorStop();
			PickupArmUp();
			
			autoPickupThread = null;
	    }
	}	// end of AutoPickup thread class.
	
	public void StartAutoShoot(boolean hoodUp)
	{
		Util.consoleLog();

//		if (hoodUp)
//			HoodUp();
//		else
//			HoodDown();
		
		if (autoShootThread != null) return;
		
		autoShootThread = new AutoShoot();
		autoShootThread.start();
	}

	public void StopAutoShoot()
	{
		Util.consoleLog();

		if (autoShootThread != null) autoShootThread.interrupt();
		
		autoShootThread = null;
	}

	// Automatic ball shooting thread.
	
	private class AutoShoot extends Thread
	{
		AutoShoot()
		{
			Util.consoleLog();
			
			this.setName("AutoShoot");
	    }
		
	    public void run()
	    {
	    	Util.consoleLog();
	    	
	    	try
	    	{
	    		//ShooterMotorStart(1.0);

	    		//sleep(2500);
	    		
	    		PickupMotorIn(1.0);
	    	
	    		sleep(1000);
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}

	    	PickupMotorStop();
    		ShooterMotorStop();
    		
    		autoShootThread = null;
	    }
	}	// end of AutoShoot thread class.
}

package Team4450.Robot9;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.JoyStickButtonIDs;
import Team4450.Lib.LaunchPad.LaunchPadControlIDs;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter
{
	private final Robot		robot;
	private final Teleop	teleop;
	
	//private final CANTalon		pickupMotor = new CANTalon(7);
	private final SpeedController	pickupMotor;
	private final Talon				shooterMotor1 = new Talon(0);
	private final Talon				shooterMotor2 = new Talon(1);
	private final FestoDA			pickupCylinder = new FestoDA(6);
	private final FestoDA			hoodCylinder = new FestoDA(4);
	private final DigitalInput		pickupSwitch = new DigitalInput(0);

	// encoder is plugged into dio port 4 - orange=+5v blue=signal, dio port 5 black=gnd yellow=signal. 
	public Encoder					encoder = new Encoder(4, 5, true, EncodingType.k4X);

	// Competition robot PID defaults. These look like static constants but you must instantiate
	// this class to set up these items for comp or clone robot before accessing them.
	public double					SHOOTER_LOW_POWER, SHOOTER_HIGH_POWER;
	public double					SHOOTER_LOW_RPM, SHOOTER_HIGH_RPM;
	public double					PVALUE, IVALUE, DVALUE; 

	private final PIDController		shooterPidController;
	public ShooterSpeedController	shooterMotorControl = new ShooterSpeedController();
	public ShooterSpeedSource		shooterSpeedSource = new ShooterSpeedSource(encoder);
	
	private Thread					autoPickupThread, autoSpitBallThread, shootThread;

	Shooter(Robot robot, Teleop teleop)
	{
		Util.consoleLog();
		
		this.robot = robot;
		this.teleop = teleop;
		
		// This is distance per pulse and our distance is 1 revolution since we want to measure
		// rpm. We determined there are 1024 pulses in a rev so 1/1024 = .000976 rev per pulse.
		encoder.setDistancePerPulse(.000976);
		
		// Tells encoder to supply the rate as the input to any PID controller source.
		encoder.setPIDSourceType(PIDSourceType.kRate);

		// Invert encoder on clone since the encoder is mounted opposite and returns negative values.
		// Encoder.setReverseDirection() does not seem to work...so ShooterSpeedSource takes care of it.
		if (robot.isClone) shooterSpeedSource.setInverted(true);
		
		// Create PIDController using our custom PIDSource and SpeedController classes.
		shooterPidController = new PIDController(0.0, 0.0, 0.0, shooterSpeedSource, shooterMotorControl);

		// Handle the fact that the pickup motor is a CANTalon on competition robot
		// and a pwm Talon on clone. Set PID defaults. Note that this sets the default
		// values which are reflected on the DS. Changes there change the values in use
		// by the running program.
		
		if (robot.isComp) 
		{
			pickupMotor = new CANTalon(7);
			robot.InitializeCANTalon((CANTalon) pickupMotor);

			// Competition robot PID defaults.
			SHOOTER_LOW_POWER = .45;
			SHOOTER_HIGH_POWER = .70;
			SHOOTER_LOW_RPM = 4900;
			SHOOTER_HIGH_RPM = 9000;	//7800;

			PVALUE = .0025;
			IVALUE = .0025;
			DVALUE = .003; 
		}
		else
		{
			pickupMotor = new Talon(7);

			// Clone robot PID defaults.
			SHOOTER_LOW_POWER = .45;
			SHOOTER_HIGH_POWER = 1.0;
			SHOOTER_LOW_RPM = 4900;
			SHOOTER_HIGH_RPM = 9000;

			PVALUE = .002; 
			IVALUE = .002;
			DVALUE = .005; 
		}
		
		PickupArmUp();
		HoodDown();
	}

	public void dispose()
	{
		Util.consoleLog();
		
		if (autoPickupThread != null) autoPickupThread.interrupt();
		if (shootThread != null) shootThread.interrupt();

		if (pickupMotor != null)
		{
			if (robot.robotProperties.getProperty("RobotId").equals("comp"))
				((CANTalon) pickupMotor).delete();
			else
				((Talon) pickupMotor).free();
		}

		if (shooterPidController != null)
		{
			shooterPidController.disable();
			shooterPidController.free();
		}
		
		if (shooterMotor1 != null) shooterMotor1.free();
		if (shooterMotor2 != null) shooterMotor2.free();
		if (pickupCylinder != null) pickupCylinder.dispose();
		if (hoodCylinder != null) hoodCylinder.dispose();
		if (pickupSwitch != null) pickupSwitch.free();
		if (encoder != null) encoder.free();
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
	/**
	 * Starts pickup motor in intake direction.
	 * @param power Power level to use 0.0 to +1.0.
	 */
	public void PickupMotorIn(double power)
	{
		Util.consoleLog("%f", power);

		pickupMotor.set(power);
	}
	//----------------------------------------
	/**
	 * Starts pickup motor in output direction.
	 * @param power Power level to use 0.0 to +1.0.
	 */
	public void PickupMotorOut(double power)
	{
		Util.consoleLog("%f", power);
		
		pickupMotor.set(Math.abs(power) * -1);
		SmartDashboard.putBoolean("PickupMotor", true);
	}
	//----------------------------------------
	/**
	 * Stop pickup motor.
	 */
	public void PickupMotorStop()
	{
		Util.consoleLog();
		
		pickupMotor.set(0);
		if (teleop != null) teleop.launchPad.FindButton(LaunchPadControlIDs.BUTTON_RED_RIGHT).latchedState = false;
		SmartDashboard.putBoolean("PickupMotor", false);
	}
	//----------------------------------------
	/**
	 * Start shooter motors.
	 * @param power Power level to use 0.0 to +1.0. If PID is enabled, and the power is equal to
	 * the low or high power constant, PID will be used to hold the low or high RPM as set on the
	 * driver station and with the P I D values set on the DS. If any other value or PID is off,
	 * the power value is used directly on the motors.
	 */
	public void ShooterMotorStart(double power)
	{
		Util.consoleLog("%.2f", power);
	
		if (SmartDashboard.getBoolean("PIDEnabled", true))
		{
    		if (power == SHOOTER_LOW_POWER)
    			// When shooting at low power, we will attempt to maintain a constant wheel speed (rpm)
    			// using pid controller measuring rpm via the encoder. RPM determined experimentally
    			// by setting motors to the low power value and seeing what rpm results.
    			// This call starts the pid controller and turns shooter motor control over to it.
    			// The pid will run the motors on its own until disabled.
    			holdShooterRPM(SmartDashboard.getNumber("LowSetting", SHOOTER_LOW_RPM));
    		else if (power == SHOOTER_HIGH_POWER)
    			// We later decided to use pid for high power shot when high power was reduced from 100%.
    			holdShooterRPM(SmartDashboard.getNumber("HighSetting", SHOOTER_HIGH_RPM));
    		else
    			// Set power directly for any value other than the defined high and low values.
    			shooterMotorControl.set(power);
		}
		else
			shooterMotorControl.set(power);
			
		SmartDashboard.putBoolean("ShooterMotor", true);
	}
	//----------------------------------------
	/**
	 * Stops shooter motors.
	 */
	public void ShooterMotorStop()
	{
		Util.consoleLog();
		
		shooterPidController.disable();

		shooterMotorControl.set(0);
		
		if (teleop != null) teleop.utilityStick.FindButton(JoyStickButtonIDs.TOP_LEFT).latchedState = false;
		SmartDashboard.putBoolean("ShooterMotor", false);
	}
	//----------------------------------------
	/**
	 * Start auto pickup thread.
	 */
	public void StartAutoPickup()
	{
		Util.consoleLog();
		
		if (autoPickupThread != null) return;

		autoPickupThread = new AutoPickup();
		autoPickupThread.start();
	}
	//----------------------------------------
	/**
	 * Stops auto pickup thread.
	 */
	public void StopAutoPickup()
	{
		Util.consoleLog();

		if (autoPickupThread != null) autoPickupThread.interrupt();
		
		autoPickupThread = null;
	}

	//----------------------------------------
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

	//----------------------------------------
	/**
	 * Start auto ball spitting thread.
	 */
	public void StartAutoBallSpit()
	{
		Util.consoleLog();
		
		if (autoSpitBallThread != null) return;

		autoSpitBallThread = new AutoSpitBall();
		autoSpitBallThread.start();
	}
	//----------------------------------------
	/**
	 * Stops auto ball spitting thread.
	 */
	public void StopAutoBallSpit()
	{
		Util.consoleLog();

		if (autoSpitBallThread != null) autoSpitBallThread.interrupt();
		
		autoSpitBallThread = null;
	}
	//----------------------------------------
	// Automatic ball spitting thread.
	
	private class AutoSpitBall extends Thread
	{
		AutoSpitBall()
		{
			Util.consoleLog();
			
			this.setName("AutoSpitBall");
	    }
		
	    public void run()
	    {
	    	Util.consoleLog();
	    	
	    	try
	    	{
	    		PickupArmDown();
    		
	    		sleep(250);
    		
	    		PickupMotorOut(1.0);
	    		
	    		while (!isInterrupted()) sleep(1000);
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}

	    	PickupMotorStop();
			PickupArmUp();
			
			autoSpitBallThread = null;
	    }
	}	// end of AutoSpitBall thread class.

	//----------------------------------------
	/**
	 * Start auto shoot thread.
	 * @param spinUpMotors True to handle shooter motor spinup, false to leave spinup to driver.
	 */
	public void StartShoot(boolean spinUpMotors, double power)
	{
		Util.consoleLog();
		
		if (shootThread != null) return;
		
		shootThread = new Shoot(spinUpMotors, power);
		shootThread.start();
	}
	//----------------------------------------
	/**
	 * Stop auto shoot thread.
	 */
	public void StopShoot()
	{
		Util.consoleLog();

		if (shootThread != null) shootThread.interrupt();
		
		shootThread = null;
	}

	// Ball shooting thread.
	
	private class Shoot extends Thread
	{
		boolean			spinUpMotors;
		double			power;
		
		Shoot(boolean spinUpMotors, double power)
		{
			Util.consoleLog();
			
			this.setName("Shoot");
			this.spinUpMotors = spinUpMotors;
			this.power = power;
		}
		
	    public void run()
	    {
	    	Util.consoleLog();
	    	
	    	try
	    	{
	    		if (spinUpMotors)
	    		{
	    			ShooterMotorStart(power);

	    			sleep(6000);
	    		}
	    		
	    		// Pull defense arms back while shooting when shooter motor power is LOW.
	    		
	    		if (teleop != null)
	    		{
	    			if (teleop.launchPad.FindButton(LaunchPadControlIDs.ROCKER_RIGHT).latchedState)	teleop.defenseArmsDown();
	    		}
	    		
	    		PickupMotorIn(1.0);
	    		
	    		if (teleop != null)
	    		{
	    			if (teleop.launchPad.FindButton(LaunchPadControlIDs.ROCKER_RIGHT).latchedState)	
	    			{
	    	    		sleep(550);
	    				teleop.defenseArmsUp();
	    	    		sleep(600);
	   			}
	    			else
	    				sleep(1000);
	    		}
	    		else
	    			sleep(1000);
	    	}
	    	catch (InterruptedException e) {}
	    	catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}

	    	PickupMotorStop();
    		ShooterMotorStop();
    		
    		shootThread = null;
	    }
	}	// end of Shoot thread class.
	
	/**
	 * Automatically hold shooter motor speed (rpm). Starts PID controller to
	 * manage motor power to maintain rpm target.
	 * @param rpm RPM to hold.
	 */
	void holdShooterRPM(double rpm)
	{
		double pValue = SmartDashboard.getNumber("PValue", PVALUE);
		double iValue = SmartDashboard.getNumber("IValue", IVALUE);
		double dValue = SmartDashboard.getNumber("DValue", DVALUE);

		Util.consoleLog("%.0f  p=%.4f  i=%.4f  d=%.4f", rpm, pValue, iValue, dValue);
		
		// p,i,d values are a guess.
		// f value is the base motor speed, which is where (power) we start.
		// setpoint is target rpm converted to rev/sec.
		// The idea is that we apply power to get rpm up to set point and then maintain.
		//shooterPidController.setPID(0.001, 0.001, 0.0, 0.0); 
		shooterPidController.setPID(pValue, iValue, dValue, 0.0); 
		shooterPidController.setSetpoint(rpm / 60);		// setpoint is revolutions per second.
		shooterPidController.setPercentTolerance(5);	// 5% error.
		shooterPidController.setToleranceBuffer(4096);	// 4 seconds of averaging.
		shooterSpeedSource.reset();
		shooterPidController.enable();
	}
	
	// Ecapsulate the two shooter motors in a single speed controller which
	// can be passed into the PID controller.
	public class ShooterSpeedController implements SpeedController
	{
		private boolean	inverted, disabled;
	
		@Override
		public void pidWrite(double output)
		{
			this.set(output);
		}

		@Override
		public double get()
		{
			return shooterMotor1.get();
		}

		@Override
		public void set(double speed, byte syncGroup)
		{
			this.set(speed);
		}

		@Override
		public void set(double speed)
		{
			if (!disabled)
			{
    			shooterMotor1.set(speed);
    			shooterMotor2.set(speed);
			}
		}

		@Override
		public void setInverted(boolean isInverted)
		{
			inverted = isInverted;
		}

		@Override
		public boolean getInverted()
		{
			return inverted;
		}

		public void enable()
		{
			disabled = false;
		}
		
		@Override
		public void disable()
		{
			disabled = true;
		}

		@Override
		public void stopMotor()
		{
			this.set(0);
		}
	}
	
	// Encapsulate the encoder so we could modify the rate returned to
	// the PID controller.
	public class ShooterSpeedSource implements PIDSource
	{
		private Encoder	encoder;
		private int		inversion = 1;
		private double	rpmAccumulator, rpmSampleCount;
		
		public ShooterSpeedSource(Encoder encoder)
		{
			this.encoder = encoder;
		}
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource)
		{
			encoder.setPIDSourceType(pidSource);
		}

		@Override
		public PIDSourceType getPIDSourceType()
		{
			return encoder.getPIDSourceType();
		}
		
		public void setInverted(boolean inverted)
		{
			if (inverted)
				inversion = -1;
			else
				inversion = 1;
		}

		public int get()
		{
			return encoder.get() * inversion;
		}
		
		public double getRate()
		{
			// TODO: Some sort of smoothing could be done to damp out the
			// fluctuations in encoder rate.
			
//			if (rpmSampleCount > 2048) rpmAccumulator = rpmSampleCount = 0;
//			
//			rpmAccumulator += encoder.getRate();
//			rpmSampleCount += 1;
//			
//			return rpmAccumulator / rpmSampleCount;

			return encoder.getRate() * inversion;
		}
		
		/**
		 * Return the current rotational rate of the encoder or current value (count) to PID controllers.
		 * @return Encoder revolutions per second or current count.
		 */
		@Override
		public double pidGet()
		{
			if (encoder.getPIDSourceType() == PIDSourceType.kRate)
				return getRate();
			else
				return get();
		}
		
		public void reset()
		{
			rpmAccumulator = rpmSampleCount = 0;
			
			encoder.reset();
		}
	}
}

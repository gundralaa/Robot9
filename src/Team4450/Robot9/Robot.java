// 2016 competition robot code.
// Cleaned up and reorganized in preparation for 2016.
// For Robot "USS Kelvin" built for FRC game "First Stronghold".

package Team4450.Robot9;

import java.io.IOException;
import java.util.Properties;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file.
 */

public class Robot extends SampleRobot 
{
  static final String  	PROGRAM_NAME = "RAC9-06.09.16-01";

  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
  CANTalon				LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon, LSlaveCanTalon, RSlaveCanTalon;
  Talon					LFPwmTalon, LRPwmTalon, RFPwmTalon, RRPwmTalon;
  RobotDrive      		robotDrive;
  
  final Joystick        utilityStick = new Joystick(2);	// 0 old ds configuration
  final Joystick        leftStick = new Joystick(0);	// 1
  final Joystick        rightStick = new Joystick(1);	// 2
  final Joystick		launchPad = new Joystick(3);
  
  final Compressor		compressor = new Compressor(0);	// Compressor class represents the PCM.
  final Compressor		compressor1 = new Compressor(1);
  final AnalogGyro		gyro = new AnalogGyro(0);		// gyro must be plugged into analog port 0 or 1.
  
  public Properties		robotProperties;
  
  public boolean		isClone = false, isComp = false;
  
  PowerDistributionPanel PDP = new PowerDistributionPanel();
  
  //AxisCamera			camera = null;
  CameraServer			usbCameraServer = null;

  DriverStation         ds = null;
    	
  DriverStation.Alliance	alliance;
  int                       location;
    
  Thread               	monitorBatteryThread, monitorDistanceThread, monitorCompressorThread;
  CameraFeed2			cameraThread;
    
  static final String  	CAMERA_IP = "10.44.50.11";
  static final int	   	USB_CAMERA = 2;
  static final int     	IP_CAMERA = 3;
 
  // PWM port assignments:
  // 0 - shooter motor
  // 1 - shooter motor
  // 2 - defense arm motor
  // 3 - wheel motor lf
  // 4 - wheel motor lr
  // 5 - wheel motor rf
  // 6 - wheel motor rr
  // 7 - pickup motor
  
  public Robot() throws IOException
  {	
	// Set up our custom logger.
	 
	try
	{
		Util.CustomLogger.setup();
    }
    catch (Throwable e) {Util.logException(e);}
      
    try
    {
    	Util.consoleLog(PROGRAM_NAME);
    
        ds = DriverStation.getInstance();

        // IP Camera object used for vision processing.
        //camera = AxisCamera.getInstance(CAMERA_IP);
      	
        Util.consoleLog("%s %s", PROGRAM_NAME, "end");
    }
    catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }
    
  public void robotInit()
  {
   	try
    {
   		Util.consoleLog();

   		LCD.clearAll();
   		LCD.printLine(1, "Mode: RobotInit");
      
   		// Read properties file from RoboRio "disk".
      
   		robotProperties = Util.readProperties();
      
   		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;

   		SmartDashboard.putString("Program", PROGRAM_NAME);
   		
   		//SmartDashboard.putBoolean("CompressorEnabled", false);
   		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

   		SmartDashboard.putBoolean("PIDEnabled", true);
   		SmartDashboard.putNumber("PValue", Shooter.PVALUE);
   		SmartDashboard.putNumber("IValue", Shooter.IVALUE);
   		SmartDashboard.putNumber("DValue", Shooter.DVALUE);
   		SmartDashboard.putNumber("LowSetting", Shooter.SHOOTER_LOW_RPM);
   		SmartDashboard.putNumber("HighSetting", Shooter.SHOOTER_HIGH_RPM);
   		
   		// Reset PDB & PCM sticky faults.
      
   		PDP.clearStickyFaults();
   		compressor.clearAllPCMStickyFaults();
   		compressor1.clearAllPCMStickyFaults();

   		// Configure motor controllers and RobotDrive.
        // Competition robot uses CAN Talons clone uses PWM Talons.
   		
		if (robotProperties.getProperty("RobotId").equals("comp")) 
			InitializeCANTalonDrive();
		else
			InitializePWMTalonDrive();
		
        robotDrive.stopMotor();
        robotDrive.setSafetyEnabled(false);
        robotDrive.setExpiration(0.1);
        
        // Reverse motors so they all turn on the right direction to match "forward"
        // as we define it for the robot.

        robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
     
        // calibrate the gyro.
        
        //gyro.initGyro();
        //gyro.calibrate();
        
   		// Set starting camera feed mode on driver station to USB-HW
        // and IP address of system hosting the camera.
      
   		SmartDashboard.putNumber("CameraSelect", USB_CAMERA);
   		SmartDashboard.putString("RobotCameraIP", CAMERA_IP);

   		// Start usb camera feed server on roboRIO. usb camera name is set by roboRIO.
   		// If camera feed stops working, check roboRIO name assignment.
   		// Note this is not used if we do dual usb cameras, which are handled by the
   		// cameraFeed class. The function below is the standard WpiLib server which
   		// can be used for a single usb camera.
      
   		//StartUSBCameraServer("cam0");
      
   		// Start the battery, compressor, camera feed and distance monitoring Tasks.

   		monitorBatteryThread = new MonitorBattery(ds);
   		monitorBatteryThread.start();

   		monitorCompressorThread = new MonitorCompressor();
   		monitorCompressorThread.start();

   		// Start camera server using our class for usb cameras.
   		// Not used at this time as we are feeding the DS from the Raspberry Pi by
   		// setting the DS camera IP address to .11 which is assigned to the Pi. In 
   		// this case the usb camera is plugged into the Pi and the Pi is running Grip
   		// feeding images to Grip and Grip provides an MJpeg image stream to the DS.
      
   		//cameraThread = new CameraFeed2(this);
   		//cameraThread.start();

   		// Start Grip and suspend it when running it on the RoboRio.
        //Grip.suspendGrip(true)
   		//Grip.startGrip();
   		
   		// Start thread to monitor distance sensor.
   		
   		//monitorDistanceThread = new MonitorDistanceMBX(this);
   		//monitorDistanceThread.start();
   		
   		Util.consoleLog("end");
    }
    catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }
    
  public void disabled()
  {
	  try
	  {
		  Util.consoleLog();

		  LCD.printLine(1, "Mode: Disabled");

		  // Reset driver station LEDs.

		  SmartDashboard.putBoolean("Disabled", true);
		  SmartDashboard.putBoolean("Auto Mode", false);
		  SmartDashboard.putBoolean("Teleop Mode", false);
		  SmartDashboard.putBoolean("PTO", false);
		  SmartDashboard.putBoolean("FMS", ds.isFMSAttached());
		  SmartDashboard.putBoolean("ShooterMotor", false);
		  SmartDashboard.putBoolean("PickupMotor", false);
		  SmartDashboard.putBoolean("LSOverride", false);
		  SmartDashboard.putBoolean("ShooterLowPower", false);
		  SmartDashboard.putBoolean("AutoTarget", false);
		  SmartDashboard.putBoolean("TargetLocked", false);

		  //Grip.suspendGrip(true);
		  
		  Util.consoleLog("end");
	  }
	  catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }
    
  public void autonomous() 
  {
      try
      {
    	  Util.consoleLog();

    	  LCD.clearAll();
    	  LCD.printLine(1, "Mode: Autonomous");
            
    	  SmartDashboard.putBoolean("Disabled", false);
    	  SmartDashboard.putBoolean("Auto Mode", true);
        
    	  // Make available the alliance (red/blue) and staring position as
    	  // set on the driver station or FMS.
        
    	  alliance = ds.getAlliance();
    	  location = ds.getLocation();

    	  // This code turns off the automatic compressor management if requested by DS.
    	  compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));

    	  PDP.clearStickyFaults();
    	  compressor.clearAllPCMStickyFaults();
    	  compressor1.clearAllPCMStickyFaults();
             
    	  // Start autonomous process contained in the MyAutonomous class.
        
    	  Autonomous autonomous = new Autonomous(this);
        
    	  autonomous.execute();
        
    	  autonomous.dispose();
    	  
    	  SmartDashboard.putBoolean("Auto Mode", false);
    	  Util.consoleLog("end");
      }
      catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }

  public void operatorControl() 
  {
      try
      {
    	  Util.consoleLog();

    	  LCD.clearAll();
      	  LCD.printLine(1, "Mode: Teleop");
            
      	  SmartDashboard.putBoolean("Disabled", false);
      	  SmartDashboard.putBoolean("Teleop Mode", true);
        
      	  alliance = ds.getAlliance();
      	  location = ds.getLocation();
        
          Util.consoleLog("Alliance=%s, Location=%d, FMS=%b", alliance.name(), location, ds.isFMSAttached());

          PDP.clearStickyFaults();
          compressor.clearAllPCMStickyFaults();
       	  compressor1.clearAllPCMStickyFaults();

          // This code turns off the automatic compressor management if requested by DS.
          compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));
          
          //Grip.suspendGrip(false);
        
          // Start operator control process contained in the MyTeleop class.
        
          Teleop teleOp = new Teleop(this);
       
          teleOp.OperatorControl();
        
          teleOp.dispose();
        	
          Util.consoleLog("end");
       }
       catch (Throwable e) {e.printStackTrace(Util.logPrintStream);}
  }
    
  public void test() 
  {
  }

  // Start WpiLib usb camera server for single selected camera.
  
  public void StartUSBCameraServer(String cameraName)
  {
	  Util.consoleLog(cameraName);

	  usbCameraServer = CameraServer.getInstance();
      usbCameraServer.setQuality(30);
      usbCameraServer.startAutomaticCapture(cameraName);
  }

  private void InitializeCANTalonDrive()
  {
	  Util.consoleLog();

	  LFCanTalon = new CANTalon(1);
	  LRCanTalon = new CANTalon(2);
	  RFCanTalon = new CANTalon(3);
	  RRCanTalon = new CANTalon(4);
	  LSlaveCanTalon = new CANTalon(5);
	  RSlaveCanTalon = new CANTalon(6);
	  
	  robotDrive = new RobotDrive(LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon);

      // Initialize CAN Talons and write status to log so we can verify
      // all the talons are connected.
      InitializeCANTalon(LFCanTalon);
      InitializeCANTalon(LRCanTalon);
      InitializeCANTalon(RFCanTalon);
      InitializeCANTalon(RRCanTalon);
      InitializeCANTalon(LSlaveCanTalon);
      InitializeCANTalon(RSlaveCanTalon);
      
      // Configure slave CAN Talons to follow the front L & R Talons.
      LSlaveCanTalon.changeControlMode(TalonControlMode.Follower);
      LSlaveCanTalon.set(LFCanTalon.getDeviceID());

      RSlaveCanTalon.changeControlMode(TalonControlMode.Follower);
      RSlaveCanTalon.set(RFCanTalon.getDeviceID());
      
      // Turn on brake mode for CAN Talons.
      SetCANTalonNeutral(true);
  }

  private void InitializePWMTalonDrive()
  {
	  Util.consoleLog();

	  LFPwmTalon = new Talon(3);
	  LRPwmTalon = new Talon(4);
	  RFPwmTalon = new Talon(5);
	  RRPwmTalon = new Talon(6);
	 
	  robotDrive = new RobotDrive(LFPwmTalon, LRPwmTalon, RFPwmTalon, RRPwmTalon);
	  
	  Util.consoleLog("end");
  }
  
  // Initialize and Log status indication from CANTalon. If we see an exception
  // or a talon has low voltage value, it did not get recognized by the RR on start up.
  
  public void InitializeCANTalon(CANTalon talon)
  {
	  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

	  talon.clearStickyFaults();
	  talon.enableControl();
	  talon.changeControlMode(TalonControlMode.PercentVbus);
  }
  
  // Set neutral behavior of CAN Talons. True = brake mode, false = coast mode.

  public void SetCANTalonNeutral(boolean brakeMode)
  {
	  Util.consoleLog("brakes on=%b", brakeMode);
	  
	  LFCanTalon.enableBrakeMode(brakeMode);
	  LRCanTalon.enableBrakeMode(brakeMode);
	  RFCanTalon.enableBrakeMode(brakeMode);
	  RRCanTalon.enableBrakeMode(brakeMode);
	  LSlaveCanTalon.enableBrakeMode(brakeMode);
	  RSlaveCanTalon.enableBrakeMode(brakeMode);
  }
  
  // Set CAN Talon voltage ramp rate. Rate is volts/sec and can be 2-12v.
  
  public void SetCANTalonRampRate(double rate)
  {
	  Util.consoleLog("%f", rate);
	  
	  LFCanTalon.setVoltageRampRate(rate);
	  LRCanTalon.setVoltageRampRate(rate);
	  RFCanTalon.setVoltageRampRate(rate);
	  RRCanTalon.setVoltageRampRate(rate);
	  LSlaveCanTalon.setVoltageRampRate(rate);
	  RSlaveCanTalon.setVoltageRampRate(rate);
  }
  
  // Return voltage and current draw for each CAN Talon.
  
  public String GetCANTalonStatus()
  {
	  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
			  LFCanTalon.getOutputVoltage(), LFCanTalon.getOutputCurrent(),
			  LRCanTalon.getOutputVoltage(), LRCanTalon.getOutputCurrent(),
			  RFCanTalon.getOutputVoltage(), RFCanTalon.getOutputCurrent(),
			  RRCanTalon.getOutputVoltage(), RRCanTalon.getOutputCurrent(),
			  LSlaveCanTalon.getOutputVoltage(), LSlaveCanTalon.getOutputCurrent(),
			  RSlaveCanTalon.getOutputVoltage(), RSlaveCanTalon.getOutputCurrent());
  }
}

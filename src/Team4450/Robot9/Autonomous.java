
package Team4450.Robot9;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous
{
	private final Robot	robot;
	private final int	program = (int) SmartDashboard.getNumber("AutoProgramSelect");
	
	// encoder is plugged into dio port 2 - orange=+5v blue=signal, dio port 3 black=gnd yellow=signal. 
	private Encoder		encoder = new Encoder(1, 2, true, EncodingType.k4X);

	Autonomous(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
	}

	public void dispose()
	{
		Util.consoleLog();
		
		encoder.free();
	}

	public void execute()
	{
		Util.consoleLog("Alliance=%s, Location=%d, Program=%d, FMS=%b", robot.alliance.name(), robot.location, program, robot.ds.isFMSAttached());
		LCD.printLine(2, "Alliance=%s, Location=%d, FMS=%b, Program=%d", robot.alliance.name(), robot.location, robot.ds.isFMSAttached(), program);

		robot.robotDrive.setSafetyEnabled(false);

		// Initialize encoder.
		encoder.reset();
        
        // Set gyro to heading 0.
        robot.gyro.reset();

		switch (program)
		{
			case 0:		// No auto program.
				break;
				
			case 1:		// Drive forward to defense and stop.
				robot.robotDrive.tankDrive(-.64, -.60);
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 350) 
				{
					LCD.printLine(3, "encoder=%d", encoder.get());
					LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(0, 0, true);				
				break;

			case 2:		// Drive forward to and cross the rough ground then stop.
				robot.robotDrive.tankDrive(-.94, -.90);
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 1600)	 
				{
					LCD.printLine(3, "encoder=%d", encoder.get());
					LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(0, 0, true);				
				break;

			case 3:		// Drive forward to and cross the rock wall then stop.
				robot.robotDrive.tankDrive(-.64, -.60);
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 350) 
				{
					LCD.printLine(3, "encoder=%d", encoder.get());
					LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(-.94, -.90);
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 400)	// 1600 
				{
					LCD.printLine(3, "encoder=%d", encoder.get());
					LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(0, 0, true);				
				break;

			case 4:		// Auto shoot from spybot position.
				Shooter shoot = new Shooter(robot);
				
				shoot.StartShoot(true);
				
				Timer.delay(4000);
				
				shoot.dispose();
				break;
				
			case 5:		// Drive forward to test gyro then stop.
				double left = -.60, right = -.60, gain = 1.0;
				
				while (robot.isAutonomous() && Math.abs(encoder.get()) < 3000) 
				{
					LCD.printLine(3, "encoder=%d", encoder.get());
					LCD.printLine(5, "gyroAngle=%d, gyroRate=%d", (int) robot.gyro.getAngle(), (int) robot.gyro.getRate());

					left = left + (robot.gyro.getAngle() * gain);
					right = right - (robot.gyro.getAngle() * gain);

					robot.robotDrive.tankDrive(left, right);
					Timer.delay(.020);
				}

				robot.robotDrive.tankDrive(0, 0, true);				
				break;
		}
		
		Util.consoleLog("end");
	}

	public void executeWithCamera()
	{
		Util.consoleLog();
    
		robot.robotDrive.setSafetyEnabled(false);
    
		Util.consoleLog("end");
	}
}
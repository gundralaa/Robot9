package Team4450.Robot9;

import java.lang.Math;
import java.util.Comparator;
import java.util.Vector;

import Team4450.Lib.Util;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Example of finding target with green light shined on retroreflective tape.
 * This example utilizes an image file, which you need to copy to the roboRIO
 * To use a camera you will have to integrate the appropriate camera details with this example.
 * To use a USB camera instead, see the SimpelVision and AdvancedVision examples for details
 * on using the USB camera. To use an Axis Camera, see the AxisCamera example for details on
 * using an Axis Camera.
 *
 * Sample images can found here: http://wp.wpi.edu/wpilib/2015/01/16/sample-images-for-vision-projects/ 
 */

public class Vision2016
{
	// A structure to hold measurements of a particle
	private class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>
	{
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int) (r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int) (r1.Area - r2.Area);
		}
	};

	// Structure to represent the scores for the various tests used for target identification
	private class Scores 
	{
		double Area;
		double Aspect;
	};

	// Images
	Image 	frame, binaryFrame;
	int 	imaqError;

	// Constants
	NIVision.Range HUE_RANGE = new NIVision.Range(105, 37);		//Default hue range for green reflection
	NIVision.Range SAT_RANGE = new NIVision.Range(230, 255);	//Default saturation range for green reflection
	NIVision.Range VAL_RANGE = new NIVision.Range(133, 183);	//Default value range for green reflection
	
	double AREA_MINIMUM = 0.5; 	//Default Area minimum for particle as a percentage of total image area
	//double LONG_RATIO = 2.22; 	//Target long side = 26.9 / Target height = 12.1 = 2.22
	//double SHORT_RATIO = 1.4; 	//Target short side = 16.9 / Target height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  	//Minimum score to be considered a target
	double VIEW_ANGLE = 52; 	//View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 
								//52 for HD3000 square, 60 for HD3000 640x480

	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2  filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	
	Scores scores = new Scores();

	public Vision2016()
	{
	    // create images
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);
	}

	public boolean CheckTarget(Image image) 
	{
		boolean isTarget = false;
		
		frame = image;
		
		// read file in from disk. For this example to run you need to copy image.jpg from the SampleImages folder to the
		// directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
		//NIVision.imaqReadFile(frame, "/home/lvuser/SampleImages/image.jpg");

		// Threshold the image looking for green
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, HUE_RANGE, SAT_RANGE, VAL_RANGE);

		// Send particle count to log
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		Util.consoleLog("Masked particles=%d", numParticles);

		// filter out small particles
		float areaMin = (float) AREA_MINIMUM;
		criteria[0].lower = areaMin;
		
		imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

		// Send particle count after filtering to log
		numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		Util.consoleLog("Filtered particles=%d", numParticles);

		if (numParticles > 0)
		{
			// Measure particles and sort by particle size
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				ParticleReport par = new ParticleReport();
				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
				par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
				par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				
				particles.add(par);
			}
			
			particles.sort(null);

			// This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
			// for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
			// about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
			scores.Aspect = AspectScore(particles.elementAt(0));
			scores.Area = AreaScore(particles.elementAt(0));
			isTarget = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

			// Log distance and target status. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote

			Util.consoleLog("Aspect=%f  Area=%f  IsTarget=%b  Dist=%f", scores.Aspect, scores.Area, isTarget,
							computeDistance(binaryFrame, particles.elementAt(0)));
		} 

		Util.consoleLog("IsTarget", false);
		
		return (isTarget);
	}

	// Comparator function for sorting particles. Returns true if particle 1 is larger
	@SuppressWarnings("unused")
	private boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		// we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	private double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
	}

	private double AreaScore(ParticleReport report)
	{
		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
		// Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
		//return ratioToScore((49 / 24) * report.Area / boundingArea);
		// For 2016, the target is 20w x 12h. So bounding rect is 240". With 2" tape, I guess the coverage is 160".
		return ratioToScore((240 / 160) * report.Area / boundingArea);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 7"x7" so aspect should be 1
	 */
	private double AspectScore(ParticleReport report)
	{
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop)));
	}

	/**
	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 *
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @return The estimated distance to the target in feet.
	 */
	private double computeDistance (Image image, ParticleReport report) 
	{
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width;
		targetWidth = 20;	// was 7 for 2015.

		return (targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2))));
	}
}

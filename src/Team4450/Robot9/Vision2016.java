package Team4450.Robot9;

import java.lang.Math;
import java.util.Comparator;
import java.util.Vector;

import Team4450.Lib.Util;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
//import com.ni.vision.NIVision.Rect;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.ShapeMode;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.image.ColorImage;
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
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>
	{
		double PercentAreaToImageArea;
		double Area;					// Area inside the tape borders.
		int BoundingRectLeft;			// Full target bounding box.
		int BoundingRectTop;
		int BoundingRectRight;
		int BoundingRectBottom;
		int Height;
		int Width;
		int CenterX;					// Center of mass location.
		int CenterY;
		double AreaScore, AspectScore, Distance;
		
		public boolean IsTarget()
		{
			if (AreaScore >= SCORE_MIN && AspectScore >= SCORE_MIN) return true;
			
			return false;
		}

		public int Top()
		{
			return BoundingRectTop;
		}
		
		public int Left()
		{
			return BoundingRectLeft;
		}
		
//		public int height()
//		{
//			return BoundingRectBottom - BoundingRectTop;
//		}
//		
//		public int width()
//		{
//			return BoundingRectRight - BoundingRectLeft;
//		}
		
		public NIVision.Rect rect()
		{
			return new NIVision.Rect(Top(), Left(), Height, Width);
		}
		
		public int compareTo(ParticleReport r)
		{
			return (int) (r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int) (r1.Area - r2.Area);
		}
		
		public String toString()
		{
			return String.format("\n--------------------------------------------------\n" + 
					"area=%.3f pctAreaToImage=%.2f\ntop=%d left=%d bottom=%d right=%d\nh=%d w=%d x=%d y=%d\narea score=%.2f  aspect score=%.2f  is target=%b  dist=%.2f", 
					Area, PercentAreaToImageArea, BoundingRectTop, BoundingRectLeft, BoundingRectBottom, BoundingRectRight,
					Height, Width, CenterX, CenterY, AreaScore, AspectScore, IsTarget(), Distance);
		}
	};

	// Images
	Image 	frame, binaryFrame;
	int 	imaqError;

	// Constants
//	NIVision.Range HUE_RANGE = new NIVision.Range(37, 105);		
//	NIVision.Range SAT_RANGE = new NIVision.Range(230, 255);	
//	NIVision.Range VAL_RANGE = new NIVision.Range(133, 183);	
	
//	NIVision.Range HUE_RANGE = new NIVision.Range(36, 104);		
//	NIVision.Range SAT_RANGE = new NIVision.Range(0, 77);		
//	NIVision.Range VAL_RANGE = new NIVision.Range(240, 255);	
	
	NIVision.Range HUE_RANGE = new NIVision.Range(59, 137);		
	NIVision.Range SAT_RANGE = new NIVision.Range(64, 255);		
	NIVision.Range VAL_RANGE = new NIVision.Range(224, 255);	

//	NIVision.Range HUE_RANGE = new NIVision.Range(140, 228);		
//	NIVision.Range SAT_RANGE = new NIVision.Range(0, 12);		
//	NIVision.Range VAL_RANGE = new NIVision.Range(98, 100);	
	
	double AREA_MIN = 0.25; 		//Area minimum for particle as a percentage of total image area
	double AREA_MAX = 0.50; 		//Area maximum for particle as a percentage of total image area
	double SCORE_MIN = 70.0;  		//Minimum score to be considered a target
	double VIEW_ANGLE = 60; 		//View angle for camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 
									//52 for HD3000 square, 60 for HD3000 640x480

	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2  filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	
	public Vision2016()
	{
	    // create images
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MIN, AREA_MAX, 0, 0);
	}

	/**
	 * Check a camera image for the 2016 target. Returns particle report for target if found.
	 * @param image Camera image to examine.
	 * @return If target found, particle report describing the target. Not found, returns null.
	 */
	public ParticleReport CheckForTarget(Image image) 
	{
		ParticleReport par, tpar = null;
		
		// read file in from disk. For this example to run you need to copy image.jpg from the SampleImages folder to the
		// directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
		//NIVision.imaqReadFile(frame, "/home/lvuser/SampleImages/image.jpg");

		// Save passed in frame for processing.
		frame = image;

		NIVision.imaqWriteJPEGFile(frame, "/home/lvuser/SampleImages/capture.jpg", 1000, null);

		// Threshold the image looking for color.
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, HUE_RANGE, SAT_RANGE, VAL_RANGE);
		
		NIVision.imaqWriteJPEGFile(binaryFrame, "/home/lvuser/SampleImages/threshold.jpg", 1000, null); // new NIVision.RawData());
		
		// Send particle count to log.
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		
		Util.consoleLog("Masked particles=%d", numParticles);

		// filter out small particles.
		//criteria[0].lower = (float) AREA_MIN;
		//Util.consoleLog("criterialower=%f", criteria[0].lower);
		
		imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

		NIVision.imaqWriteJPEGFile(binaryFrame, "/home/lvuser/SampleImages/filtered.jpg", 1000, null); // new NIVision.RawData());

		// Send particle count after filtering to log.
		numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		
		Util.consoleLog("Filtered particles=%d", numParticles);

		if (numParticles > 0)
		{
			// Measure particles and sort by particle size.
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				par = new ParticleReport();
				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
				par.BoundingRectTop = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				par.BoundingRectLeft = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectBottom = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
				par.BoundingRectRight = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				par.CenterX = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CENTER_OF_MASS_X);
				par.CenterY = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CENTER_OF_MASS_Y);
				par.Height = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_HEIGHT);
				par.Width = (int) NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_WIDTH);

				par.AreaScore = AreaScore(par);
				par.AspectScore = AspectScore(par);
				par.Distance = computeDistance(binaryFrame, par);
				
				particles.add(par);
			}
			
			particles.sort(null);	// Largest particle first.

			// Examine particles for largest that scores above the threshold and select it as the target.
			
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				par = particles.elementAt(particleIndex);
				
				Util.consoleLog("<%d>%s", particleIndex, par.toString());
				
				if (tpar == null && par.IsTarget()) tpar = par;
			}

			// If target particle located, draw a box around it on original image and save.
			
			if (tpar != null)
			{
    			NIVision.imaqDrawShapeOnImage(frame, frame, tpar.rect(), DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
    
//    			NIVision.Rect rect = new NIVision.Rect(tpar.CenterY, tpar.CenterX, 10, 10);
//    			NIVision.imaqDrawShapeOnImage(frame, frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
//    			rect.free();
    			
    			NIVision.imaqWriteJPEGFile(frame, "/home/lvuser/SampleImages/shapes.jpg", 1000, null);

    			//Util.consoleLog("Target=%s", tpar.toString());
			}
		} 
		
		return (tpar);
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

	/**
	 * Computes the ratio of the target area inside the tape lines to the bounding area of the target.
	 * Then compares the targets ration to the ideal ratio and converts that to a percentage of 
	 */
	private double AreaScore(ParticleReport report)
	{
		//double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
		double boundingArea = (report.Height * report.Width);
		// Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
		//return ratioToScore((49 / 24) * report.Area / boundingArea);
		// For 2016, the target is 20w x 12h. So bounding rect is 240". With 2" tape, the tape covers 80".
		return ratioToScore((240 / 80) * report.Area / boundingArea);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 20"x12" so aspect should be 1.6
	 */
	private double AspectScore(ParticleReport report)
	{
		//return ratioToScore(report.BoundingRectBottom - report.BoundingRectTop) / (report.BoundingRectRight - report.BoundingRectLeft));
		return ratioToScore(report.Width / report.Height);
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
		//normalizedWidth = 2 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width;
		normalizedWidth = 2 * report.Width / size.width;
		targetWidth = 20;	// was 7 for 2015.

		return (targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2))));
	}
}

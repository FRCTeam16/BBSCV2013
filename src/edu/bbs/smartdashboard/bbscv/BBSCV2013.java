
package edu.bbs.smartdashboard.bbscv;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.*;
import com.googlecode.javacv.cpp.opencv_imgproc.*;
import com.googlecode.javacv.cpp.opencv_imgproc;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.wpijavacv.CVHelper;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import javax.imageio.ImageIO;
import java.util.List;
import javax.swing.WindowConstants;

public class BBSCV2013 extends WPICameraExtension {
    private boolean _debugMode = false;
    
    private IntegerProperty _hue;
    private IntegerProperty _sat;
    private IntegerProperty _satInv;
    private IntegerProperty _val;
    private IntegerProperty _areaMin;
    private IntegerProperty _areaMax;
    private BooleanProperty _showBinaryImg;
    private IntegerProperty _dilate;
    private IntegerProperty _erode;
    private IntegerProperty _approxPoly;
    private IntegerProperty _vertices;
    private IntegerProperty _ratioHigh;
    private IntegerProperty _ratioLow;
    private IntegerProperty _blur;
    private IntegerProperty _closingIntensity;
    private BooleanProperty _showCenterLine;
    private IntegerProperty _linePos;
    
    //private IplConvKernel _morphKernel;
    private IplImage _binImg;
    private IplImage _hsvImg;
    private IplImage _hueImg;
    private IplImage _satImg;
    private IplImage _valImg;
    private CvSize _size = null;
    
    public BBSCV2013()
    {
        this(false);
    }

    public BBSCV2013(boolean debug)
    {
        _debugMode = debug;
        
        _hue = new IntegerProperty(this, "Hue", 50); 
        _sat = new IntegerProperty(this, "Sat", 210);
        _satInv = new IntegerProperty(this, "Sat Inv", 255);
        _val = new IntegerProperty(this, "Val", 210);
        _areaMin = new IntegerProperty(this, "AreaMin", 900);
        _areaMax = new IntegerProperty(this, "AreaMax", 5000000);
        _showBinaryImg = new BooleanProperty(this, "Show Binary", false);
        _dilate = new IntegerProperty(this, "Dilate", 3);
        _erode = new IntegerProperty(this, "Erode", 6);
        _approxPoly = new IntegerProperty(this, "Approximate Polygon", 5);
        _vertices = new IntegerProperty(this, "Vertices", 5);
        _ratioHigh = new IntegerProperty(this, "Ratio High", 4);
        _ratioLow = new IntegerProperty(this, "Ratio Low", 2);
        _blur = new IntegerProperty(this, "Blur", 1);
        _closingIntensity = new IntegerProperty(this, "Hole Closing Intensity", 10);
        _showCenterLine = new BooleanProperty(this, "Show Line", true);
        _linePos = new IntegerProperty(this, "Line Position (px)");
        //_morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_ELLIPSE, null);
        
        CVHelper.init();
    }
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage)
    {        
        _size = opencv_core.cvSize(rawImage.getWidth(),rawImage.getHeight());
        _binImg = IplImage.create(_size, 8, 1);
        _hsvImg = IplImage.create(_size, 8, 3);
        _hueImg = IplImage.create(_size, 8, 1);
        _satImg = IplImage.create(_size, 8, 1);
        _valImg = IplImage.create(_size, 8, 1);
        
        IplImage input = CVHelper.getIplImage(rawImage);
        
       // opencv_imgproc.cvSmooth(input, input, opencv_imgproc.CV_BLUR, _blur.getValue());
        
        opencv_imgproc.cvCvtColor(input, _hsvImg, opencv_imgproc.CV_BGR2HSV);
        
        opencv_core.cvSplit(_hsvImg, _hueImg, _satImg, _valImg, null);

        //////////////////////opencv_imgproc.cvThreshold(hue, bin, _hue.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY);
        //////////////////////opencv_imgproc.cvThreshold(_hueImg, _hueImg, 100, 255, opencv_imgproc.CV_THRESH_BINARY);
        
        //CanvasFrame result = new CanvasFrame("hue");
        //result.showImage(_hueImg.getBufferedImage());

        // Saturation
        opencv_imgproc.cvThreshold(_satImg, _satImg, _sat.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(_satImg, _satImg, _satInv.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        //CanvasFrame results = new CanvasFrame("sat");
        //results.showImage(_satImg.getBufferedImage());
        
        // Value
        opencv_imgproc.cvThreshold(_valImg, _valImg, _val.getValue(), 255, opencv_imgproc.CV_THRESH_BINARY);

        opencv_core.cvAnd(_satImg, _valImg, _binImg, null);
        //opencv_core.cvAnd(_binImg, _valImg, _binImg, null);        
        //opencv_core.cvAnd(hue, bin, bin, null);
        //opencv_core.cvAnd(bin, sat, bin, null);
        //opencv_core.cvAnd(bin, val, bin, null);
        int filterSize = 3;
        IplConvKernel convKernel = opencv_imgproc.cvCreateStructuringElementEx(filterSize, filterSize, (filterSize - 1) / 2, (filterSize - 1) / 2, opencv_imgproc.CV_SHAPE_ELLIPSE, null);
        opencv_imgproc.cvMorphologyEx(_binImg, _binImg, null, convKernel, opencv_imgproc.CV_MOP_CLOSE, _closingIntensity.getValue());
        
        opencv_imgproc.cvErode(_binImg, _binImg, null, _erode.getValue());
        opencv_imgproc.cvDilate(_binImg, _binImg, null, _dilate.getValue());
        
        CanvasFrame result = new CanvasFrame("binary");
        result.showImage(_binImg.getBufferedImage());
       
        opencv_imgproc.cvCanny(_binImg, _binImg, 50, 150, 3);
       
        WPIBinaryImage binWpi = CVHelper.makeWPIBinaryImage(_binImg);
       
        if (_showBinaryImg.getValue())
        {
            return binWpi;
        }
        
        WPIContour[] contours = CVHelper.findConvexContours(binWpi, _debugMode);

        List<WPIPolygon> polys = new ArrayList<WPIPolygon>();

        for (WPIContour c : contours) {
            WPIPolygon p = c.approxPolygon(_approxPoly.getValue());
            if (_debugMode) {
                System.out.println("pi:"+p.getPerimeter()/p.getWidth()+"|area:"+p.getArea()+"|Perimeter:"+p.getPerimeter()+"|width:"+p.getWidth()+"|isconvex:"+p.isConvex()+"|vert#:"+p.getNumVertices()+"|ratio:"+p.getPerimeter()/p.getWidth()+"|x:"+p.getX()+"|y:"+p.getY());
            }
          
            if (p.isConvex() && p.getArea() >= _areaMin.getValue() && p.getArea() <= _areaMax.getValue() && p.getNumVertices() >= _vertices.getValue() && p.getPerimeter() > 0 && p.getWidth() > 0) {
                if (p.getPerimeter()/p.getWidth() >= _ratioLow.getValue() && p.getPerimeter()/p.getWidth() <= _ratioHigh.getValue()) {
                    polys.add(p);
               }
            }
        }
        
        if (_debugMode) {
            System.out.println("poly count (before filter):"+polys.size());
        }
        
        if (polys.size() > 2)
        {
            polys = CVHelper.filterSimilar(polys);
            //filter out 2 closest to center
            polys = CVHelper.getCenterClosest(polys, rawImage.getWidth()/2);
            
        }
       
        if (_debugMode) {
            System.out.println("poly count (after filter):"+polys.size());
        }
        
        if (_showCenterLine.getValue())
        {
            WPIPoint p1 = new WPIPoint(0, rawImage.getHeight()/2);
            WPIPoint p2 = new WPIPoint(rawImage.getWidth(), rawImage.getHeight()/2);
            rawImage.drawLine(p1, p2, WPIColor.BLUE, 2);
        }
        
        if (polys.isEmpty())
        {
            Robot.getTable().putNumber("AvgXCenter", 0);
            Robot.getTable().putNumber("AvgYCenter", 0);
            Robot.getTable().putNumber("AvgArea", 0);
            Robot.getTable().putNumber("FoundDisc", 0);
            return rawImage;
        }
        
        int avgxCenter = 0;
        int xcenter1 = 0;
        int xcenter2 = 0;
        int avgArea = 0;
        int area1 = 0;
        int area2 = 0;
        int avgyCenter = 0;
        int ycenter1 = 0;
        int ycenter2 = 0;
        for (WPIPolygon p : polys) {
            if (_debugMode) {
                System.out.println("pi:"+p.getPerimeter()/p.getWidth()+"|area:"+p.getArea()+"|Perimeter:"+p.getPerimeter()+"|width:"+p.getWidth()+"|isconvex:"+p.isConvex()+"|vert#:"+p.getNumVertices()+"|ratio:"+p.getPerimeter()/p.getWidth()+"|x:"+p.getX()+"|y:"+p.getY());
            }
            int cx = p.getX() + (p.getWidth() / 2);
            int cy = p.getY() + (p.getHeight() / 2);
            
                
            opencv_core.CvPoint center = new opencv_core.CvPoint(Math.round(cx), Math.round(cy));

            opencv_core.cvCircle(CVHelper.getIplImage(rawImage), center, 5, CvScalar.RED, -1, 8, 0);
            opencv_core.cvCircle(CVHelper.getIplImage(rawImage), center, p.getWidth()/2, CvScalar.BLUE, 3, 8, 0);
            
            if (xcenter1 == 0) {
                xcenter1 = cx;
            }
            else {
                xcenter2 = cx;
            }
            
            if (ycenter1 == 0) {
                ycenter1 = cy;
            }
            else {
                ycenter2 = cy;
            }
            
            if (area1 == 0){
                area1 = p.getArea();
            }
            else {
                area2 = p.getArea();
            }
        }
        
        avgxCenter = (xcenter1 + xcenter2) / polys.size();
        avgyCenter = (ycenter1 + ycenter2) / polys.size();
        avgArea = (area1 + area2) / polys.size(); 

        
        if (polys.size() > 0) {
            opencv_core.CvFont font = new opencv_core.CvFont(opencv_core.CV_FONT_HERSHEY_PLAIN, 1, 1);
            opencv_core.cvPutText(CVHelper.getIplImage(rawImage), "AvgXCenter: " + avgxCenter, opencv_core.cvPoint(10, 14), font, CvScalar.RED);
            opencv_core.cvPutText(CVHelper.getIplImage(rawImage), "AvgYCenter: " + avgyCenter, opencv_core.cvPoint(10, 40), font, CvScalar.RED);
            opencv_core.cvPutText(CVHelper.getIplImage(rawImage), "AvgArea: " + avgArea, opencv_core.cvPoint(10, 66), font, CvScalar.RED);
                    
            Robot.getTable().putNumber("AvgXCenter", avgxCenter);
            Robot.getTable().putNumber("AvgYCenter", avgyCenter);
            Robot.getTable().putNumber("AvgArea", avgArea);
            Robot.getTable().putNumber("FoundDisc", 1);
        }
        else
        {           
            Robot.getTable().putNumber("AvgXCenter", 0);
            Robot.getTable().putNumber("AvgYCenter", 0);
            Robot.getTable().putNumber("AvgArea", 0);
            Robot.getTable().putNumber("FoundDisc", 0);
        }
        
        if (_debugMode) {
            System.out.println("AvgXCenter:"+avgxCenter+"|AvgYCenter:"+avgyCenter+"|AvgArea:"+avgArea+"|Found:"+!polys.isEmpty());
        }
       
        
        return rawImage;
        
    }

    public static void main(String[] args) 
    {
        if (args.length == 0)
        {
            System.out.println("Usage: Arguments are paths to image files to test the program on");
            return;
        }

        // Create the widget
        BBSCV2013 widget = new BBSCV2013(true);

        long totalTime = 0;
        for (int i = 0; i < args.length; i++)
        {
            // Load the image
            WPIColorImage rawImage = null;
            try
            {
                rawImage = new WPIColorImage(ImageIO.read(new File(args[i%args.length])));
            } catch (IOException e)
            {
                System.err.println("Could not find file!");
                return;
            }
            
            //shows the raw image before processing to eliminate the possibility
            //that both may be the modified image.
           // CanvasFrame original = new CanvasFrame("Raw");
           // original.showImage(rawImage.getBufferedImage());

            WPIImage resultImage = null;

            // Process image
            long startTime, endTime;
            startTime = System.nanoTime();
            resultImage = widget.processImage(rawImage);
            endTime = System.nanoTime();

            // Display results
            totalTime += (endTime - startTime);
            double milliseconds = (double) (endTime - startTime) / 1000000.0;
            System.out.format("Processing took %.2f milliseconds%n", milliseconds);
            System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
            
            CanvasFrame result = new CanvasFrame("Result");
            result.showImage(resultImage.getBufferedImage());
            result.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
 
            System.out.println("Waiting for ENTER to continue to next image or exit...");
            Scanner console = new Scanner(System.in);
            console.nextLine();

            /*if (original.isVisible())
            {
                original.setVisible(false);
                original.dispose();
            }*/
            
            if (result.isVisible())
            {
                result.setVisible(false);
                result.dispose();
            }
            
           
        }
    }
}

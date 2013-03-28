
package edu.wpi.first.wpijavacv;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import java.util.ArrayList;
import java.util.List;
import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacpp.Loader;

public class CVHelper
{
    public static CvSeq getCvSeq(WPIContour contour)
    {
        return contour.getCVSeq();
    }

    public static WPIContour makeWPIContour(CvSeq seq)
    {
        return new WPIContour(seq);
    }

    public static WPIGrayscaleImage makeWPIGrayscaleImage(IplImage arr)
    {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        opencv_core.cvCopy(arr, tempImage);
        return new WPIGrayscaleImage(tempImage);
    }

    public static WPIColorImage makeWPIColorImage(IplImage arr)
    {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        opencv_core.cvCopy(arr, tempImage);
        return new WPIColorImage(tempImage);
    }

    public static WPIBinaryImage makeWPIBinaryImage(IplImage arr)
    {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        opencv_core.cvCopy(arr, tempImage);
        return new WPIBinaryImage(tempImage);
    }

    public static IplImage getIplImage(WPIImage image)
    {
        return image.image;
    }

    private static CvMemStorage storage;

    public static void init()
    {
        storage = CvMemStorage.create();
    }

    public static WPIContour[] findConvexContours(WPIBinaryImage image, boolean isDebug)
    {
        image.validateDisposed();
        IplImage tempImage = IplImage.create(image.image.cvSize(), image.image.depth(), 1);

        opencv_core.cvCopy(image.image, tempImage);

        CvSeq contours = new CvSeq();
        opencv_imgproc.cvFindContours(tempImage, storage, contours, Loader.sizeof(opencv_core.CvContour.class), opencv_imgproc.CV_RETR_LIST, opencv_imgproc.CV_CHAIN_APPROX_TC89_KCOS);
       //cvDrawContours(color_img, c, CV_RGB(255,255,255), CV_RGB(255,255,255), 0, -1, 8); 
        //opencv_core.cvDrawContours(tempImage, contours, opencv_core.cvScalarAll(255), opencv_core.cvScalarAll(255), 0, -1, 8);
        //System.out.println("c tot:"+ contours.total());
        ArrayList<WPIContour> results = new ArrayList();
        while (!WPIDisposable.isNull(contours)) {
            if (contours.total() >= 6) {
                //opencv_core.cvDrawContours(tempImage, contours, opencv_core.cvScalarAll(255), opencv_core.cvScalarAll(255), 0, -1, 8);
                opencv_core.CvBox2D box = opencv_imgproc.cvFitEllipse2(contours);
                opencv_core.cvEllipseBox(tempImage, box, opencv_core.cvScalar(255, 255, 255, 0), 1, 8, 0);
                //System.out.println("box:" +box.angle());
              
                float ratioOfWidthToHeight = 0;
                if (box.size().width() > box.size().height())
                {
                    ratioOfWidthToHeight = (float)box.size().width()/box.size().height();
                }
                else
                {
                    ratioOfWidthToHeight = (float)box.size().height()/box.size().width();
                }

                CvSeq convexContour = opencv_imgproc.cvConvexHull2(contours, storage, opencv_imgproc.CV_CLOCKWISE, 1);
               
                if (ratioOfWidthToHeight > .5 && ratioOfWidthToHeight < 3) {
                    WPIContour contour = new WPIContour(opencv_core.cvCloneSeq(convexContour, storage));

                    results.add(contour); 
                }
            }
            contours = contours.h_next();
        }
        //opencv_core.cvDrawContours(tempImage, contours, opencv_core.cvScalarAll(255), opencv_core.cvScalarAll(255), 0, 1, 8);

        if (isDebug) {
            CanvasFrame resultv = new CanvasFrame("contour2");
            resultv.showImage(tempImage.getBufferedImage());
        }
        
        tempImage.release();
        WPIContour[] array = new WPIContour[results.size()];
        return results.toArray(array);
    }
    
    public static List<WPIPolygon> filterSimilar(List<WPIPolygon> polys) {
        List<WPIPolygon> ret = new ArrayList<>();
        ret.addAll(polys);

        List<WPIPolygon> removalQueue = new ArrayList<>();

        for (WPIPolygon p : polys) {
            // find all polygons with similar center points
            List<WPIPolygon> similar = getSimilar(p, polys);

            // find the largest of the similar polygons
            WPIPolygon largest = getLargest(similar);

            // remove the largest poly
            similar.remove(largest);

            // queue the smaller polygons for removal
            removalQueue.addAll(similar);
        }

        // remove everything in the removal queue
        for (WPIPolygon p : removalQueue) {
            removeAll(p, ret);
        }

        return ret;
    }
    
    private static List<WPIPolygon> getSimilar(WPIPolygon poly, List<WPIPolygon> pool) {
        List<WPIPolygon> ret = new ArrayList<>();

        int xCenter = poly.getX() + (poly.getWidth() / 2);
        int yCenter = poly.getY() + (poly.getHeight() / 2);

        for (WPIPolygon p : pool) {
            int pcx = p.getX() + (p.getWidth() / 2);
            int pcy = p.getY() + (p.getHeight() / 2);

            int dx = Math.abs(pcx - xCenter);
            int dy = Math.abs(pcy - yCenter);

            int distSquared = (dx * dx) + (dy * dy);

            if (distSquared < 20 * 20) {
                    ret.add(p);
            } // ignore this rect if it's too small
        }

        return ret;
    }

    private static WPIPolygon getLargest(List<WPIPolygon> pool) {
        WPIPolygon ret = null;

        for (WPIPolygon p : pool) {
            if (ret == null || p.getArea() > ret.getArea()) {
                ret = p;
            }
        }

        return ret;
    }
    
    public static List<WPIPolygon> getCenterClosest(List<WPIPolygon> pool, int center) {
        List<WPIPolygon> ret = new ArrayList<>();
        
        WPIPolygon p1 = null;
        WPIPolygon p2 = null;
        
        for (WPIPolygon p : pool) {
            int temp = Math.abs(center - (p.getX()+(p.getWidth()/2)));
            if (p1 == null || temp < Math.abs(center - (p1.getX()+(p1.getWidth()/2))))
            {
                p1 = p;
            }
            //System.out.println(p.getX()+(p.getWidth()/2) + "-" + Math.abs(center - (p.getX()+(p.getWidth()/2))));
        }
        
        for (WPIPolygon p : pool) {
            int temp = Math.abs(center - (p.getX()+(p.getWidth()/2)));
            if ((p2 == null || temp < Math.abs(center - (p2.getX()+(p2.getWidth()/2)))) && p != p1)
            {
                p2 = p;
            }
        }
        
        ret.add(p1);
        ret.add(p2);
        
        return ret;
    }

    private static <T> void removeAll(T obj, List<T> pool) {
        while (pool.contains(obj)) {
            pool.remove(obj);
        }
    }

    public static void releaseMemory()
    {
        opencv_core.cvClearMemStorage(storage);
    }
    
}

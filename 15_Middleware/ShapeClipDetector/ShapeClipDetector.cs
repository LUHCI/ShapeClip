using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

class ShapeClipDetector
{

    private static double ComputeOrientationFromVerts(CvPoint[] points)
    {
        CvPoint p1 = points[0];
        CvPoint p2 = points[1];

        return Math.Atan((float)(p2.Y - p1.Y) / (float)(p2.X - p1.X));
    }

    /// <summary>
    /// The resolution of the screen to map the surface image into correct pixel space
    /// </summary>
    private Vec2F screenResolution;

    public ShapeClipDetector(float screenWidth, float screenHeight)
    {
        screenResolution = new Vec2F(screenWidth, screenHeight);
    }

    /// <summary>
    /// Computes the image projection and clusters that projection
    /// </summary>
    /// <param name="region">The image to perform this operation on</param>
    /// <param name="horizontal">If true the clustering is performed on a horizontal projection; on a vertical one otherwise</param>
    /// <returns>The number of clusters found and the index of the largest cluster</returns>
    private int[] ComputeClusters(IplImage region, bool horizontal, out double[] projectionOut)
    {
        int dim = horizontal ? region.Width : region.Height;

        double[] projection = new double[dim];
        for (int i = 0; i < projection.Length; i++)
            projection[i] = ((horizontal ? region.GetCol(i) : region.GetRow(i)).Sum().Val0 / 255) / dim;
            
        int inCluster = 0, clusters = 0;
        int lastCluster = 0, lastClusterCount = 0;
        for (int i = 0; i < projection.Length - 1; i++)
        {
            if (projection[i] > 0.03 && inCluster == 0)
            {
                inCluster++;
            }
            else if (projection[i] < 0.01 && inCluster > 0)
            {
                clusters++;
                if (lastClusterCount < inCluster)
                {
                    lastCluster = i;
                    lastClusterCount = inCluster;
                }
                inCluster = 0;
            }
        }

        projectionOut = projection;
        return new int[] { clusters, lastCluster };
    }

    private ShapeClip DetectClip(CvSeq<CvPoint> contour, IplImage image)
    {
        // Approximate contours to rectange.
        CvMemStorage cstorage = new CvMemStorage();
        CvSeq<CvPoint> verts = contour.ApproxPoly(CvContour.SizeOf, cstorage, ApproxPolyMethod.DP, contour.ContourPerimeter() * 0.05);
        CvRect rect = Cv.BoundingRect(verts);

        // scale BB
        CvSize originalSize = rect.Size;
        CvSize size = new CvSize((int)(rect.Width * 1.5), (int)(rect.Height * 1.5));
        CvSize sizeDist = new CvSize(rect.Width - size.Width, rect.Height - size.Height);
        rect = new CvRect(
            Math.Max(rect.Location.X + sizeDist.Width / 2, 0),
            Math.Max(rect.Location.Y + sizeDist.Height / 2, 0), size.Width, size.Height);

        // If rect, convert to region of interest and approximate top.
        if (verts.Total >= 4 && new CvRect(0, 0, image.Width, image.Height).Contains(rect))
        {
            DetectionState detectionState = verts.Total == 4 ? DetectionState.SemiOriented : DetectionState.Candidate;
            double angle = (180.0 / Math.PI) * ComputeOrientationFromVerts(verts.ToArray());

            using (IplImage region = image.Clone(rect))
            using (IplImage finalRegion = image.Clone(rect))
            using (IplImage colorRegion = new IplImage(region.Size.Width, region.Size.Height, BitDepth.U8, 3))
            using (IplImage debug = new IplImage(region.Size.Width + 20, region.Size.Height + 20, BitDepth.U8, 3))
            {
                // Rotate into position based on the line angle estimate
                Cv.WarpAffine(region, region, Cv.GetRotationMatrix2D(new CvPoint2D32f(rect.Width / 2, rect.Height / 2), angle, 1));
                Cv.FloodFill(region, new CvPoint(0, 0), 255, 0, 150);

                // Project image and find clusters
                region.Not(region);
                double[] horizontalProjection, verticalProjection;
                int[] horizontalPrjClusters = ComputeClusters(region, true, out horizontalProjection);
                int horizontalClusters = horizontalPrjClusters[0], lastHorizontalCluster = horizontalPrjClusters[1];
                int[] verticalPrjClusters = ComputeClusters(region, false, out verticalProjection);
                int verticalClusters = verticalPrjClusters[0], lastVerticalCluster = verticalPrjClusters[1];



                // Correct the orientation based on the clusters found
                bool foundLDRs = false;
                if (verticalClusters > horizontalClusters)
                {
                    // 90 deg

                    if (lastHorizontalCluster < region.Width / 2)
                    {
                        // 90deg
                        angle += 90;
                        foundLDRs = true;
                    }
                    else
                    {
                        // 270 deg
                        angle += 270;
                        foundLDRs = true;
                    }
                }
                else if (verticalClusters < horizontalClusters)
                {
                    // 0 deg
                    if (lastVerticalCluster < region.Height / 2)
                    {
                        // 0deg
                        foundLDRs = true;
                    }
                    else
                    {
                        // 180 deg
                        angle += 180;
                        foundLDRs = true;
                    }
                }
                else
                {
                    // something went wrong with our initial alignment
                    //    NO proper orientation found - could not identify the LDRs
                }

                #region DEBUG
                //debug.Zero();
                //Cv.CvtColor(finalRegion, colorRegion, ColorConversion.GrayToRgb);
                //debug.DrawImage(20, 0, region.Width, region.Height, colorRegion);

                //for (int i = 0; i < region.Width / 2; i++)
                //    debug.DrawRect(20 + i, debug.Height - (int)(horizontalProjection[i] * 100), 20 + i, debug.Height, CvColor.Red, 1);
                //for (int i = 0; i < region.Height / 2; i++)
                //    debug.DrawRect(0, i, (int)(verticalProjection[i] * 100), i, CvColor.Red, 1);
                //debugWindow.ShowImage(debug);
                #endregion

                if (foundLDRs) detectionState = DetectionState.FullyOriented;
            }

            // Compute pixel space mapping
            Vec2F scale = new Vec2F(screenResolution.X / image.Width, screenResolution.Y / image.Height);

            return new ShapeClip(
                detectionState,
                new Vec2F(rect.Location.X + 0.5f * rect.Width, rect.Location.Y + 0.5f * rect.Height).Scale(scale),
                new Vec2F(originalSize).Scale(scale), 
                angle);
        }
        else
        {
            return null;
        }
    }

    public ShapeClip[] DetectClips(IplImage copy, int threshold = 230)
    {
        //using (IplImage copy = image.Clone())
        {
            Cv.Threshold(copy, copy, threshold, 255, ThresholdType.Binary);

            // find contours
            CvSeq<CvPoint> contours;
            CvMemStorage storage = new CvMemStorage();
            using (IplImage contourImage = copy.Clone())
                Cv.FindContours(contourImage, storage, out contours, CvContour.SizeOf, ContourRetrieval.Tree, ContourChain.ApproxSimple);

            List<ShapeClip> result = new List<ShapeClip>();
            for (CvSeq<CvPoint> contour = contours; contour != null; contour = contour.HNext)
            {
                ShapeClip clip = DetectClip(contour, copy);
                if (clip != null) result.Add(clip);
            }
            return result.ToArray();
        }
    }

    public IplImage DrawShapeClips(IplImage source, ShapeClip[] clips)
    {
        IplImage result = source;// new IplImage(source.Size, BitDepth.U8, 3);
        //Cv.CvtColor(source, result, ColorConversion.GrayToRgb);
        foreach (var clip in clips)
        {
            CvScalar color = CvColor.Red;
            if (clip.DetectionState == DetectionState.SemiOriented)
            {
                color = CvColor.Yellow;
            }
            else if (clip.DetectionState == DetectionState.FullyOriented)
            {
                color = CvColor.Green;
            }

            result.DrawCircle((int)clip.Position.X, (int)clip.Position.Y, (int)clip.BoundingBox.Length, color, 1);
            result.DrawLine(
                (int)clip.Position.X, (int)clip.Position.Y,
                (int)(clip.Position.X + (clip.BoundingBox.Length * Math.Cos((Math.PI / 180.0) * clip.Angle))), 
                (int)(clip.Position.Y + (clip.BoundingBox.Length * Math.Sin((Math.PI / 180.0) * clip.Angle))),
                color, 1);
        }
        return result;
    }

}

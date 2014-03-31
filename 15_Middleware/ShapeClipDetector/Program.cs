using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using OpenCvSharp;
using System.Diagnostics;

namespace SCD
{
    static class Program
    {

        static void Main()
        {
            int threshold = 200;
            SurfaceImageSource source = new SurfaceImageSource();
            source.Start();
            ShapeClipDetector detector = new ShapeClipDetector(1920, 1080);
            //ShapeClipDetector detector = new ShapeClipDetector(960, 520);

            #region GUI
            //IplImage dbglrg = new IplImage(1920, 1080, BitDepth.U8, 3);
            //IplImage grsrfc = new IplImage(960, 520, BitDepth.U8, 3);

            //using (CvWindow win = new CvWindow())
            //{
            //    win.CreateTrackbar("Threshold", threshold, 255, (v) => threshold = v);

            //    source.OnNewImage += (img) =>
            //    {
            //        Cv.CvtColor(img, grsrfc, ColorConversion.GrayToRgb);
            //        Cv.Resize(grsrfc, dbglrg);

            //        var clips = detector.DetectClips(img, threshold);
            //        detector.DrawShapeClips(dbglrg, clips);
            //        win.ShowImage(dbglrg);
            //    };

            //    CvWindow.WaitKey(0);
            //    win.Close();
            //}
            #endregion

            #region SERVER
            Stopwatch watch = new Stopwatch();
            watch.Start();
            ArrangementWebsocketServer server = new ArrangementWebsocketServer(8889);
            server.Start();
            Console.WriteLine("Server running - waiting for connections");

            int pos = 0;
            char[] animation = "|/-\\".ToCharArray();

            while (true)
            {
                if (watch.ElapsedMilliseconds > (1000 / 10.0))
                {
                    watch.Restart();
                    var clips = detector.DetectClips(source.GetImage(), threshold);

                    if (clips.Length == 9)
                    {
                        server.BroadcastArrangement(clips);

                        Console.SetCursorPosition(0, 1);
                        Console.WriteLine("{0}\tBroadcasting {1} shape clips", animation[pos++ % animation.Length], clips.Length);
                    }
                    else
                    {
                        Console.SetCursorPosition(0, 1);
                        Console.WriteLine("NOT\tBroadcasting {0} shape clips", clips.Length);
                    }
                }
            }
            #endregion

            source.Stop();
        }
    }
}

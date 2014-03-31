using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Surface.Core;
using OpenCvSharp;
using System.Threading;
using System.Windows.Forms;
using Emgu.CV.Structure;
using System.Runtime.InteropServices;

namespace SCD
{
    class SurfaceImageSource
    {
        public delegate void OnNewImageHandler(IplImage image);


        private byte[] _surfaceImage;

        private ImageMetrics _surfaceMetrics;

        private long _currentTimeStamp;

        private long _lastTimeStamp;

        private TouchTarget _touchTarget;

        private IplImage _emguCvImage;

        private AutoResetEvent _imageEvent = new AutoResetEvent(false);

        private AutoResetEvent _startEvent = new AutoResetEvent(false);

        public event OnNewImageHandler OnNewImage;

        public bool IsRunning { get; private set; }

        public IplImage GetImage()
        {
            lock (this)
            {
                // block until started and new image is available
                while (!IsRunning || _lastTimeStamp == _currentTimeStamp)
                    _imageEvent.WaitOne();
                _lastTimeStamp = _currentTimeStamp;
                //Console.WriteLine(_lastTimeStamp);
            }
            return _emguCvImage;
        }

        public void Start()
        {
            lock (this)
            {
                if (!IsRunning)
                {
                    (new Thread(new ThreadStart(delegate
                    {
                        _touchTarget = new TouchTarget((new Form()).Handle, EventThreadChoice.OnBackgroundThread);
                        _touchTarget.EnableInput();
                        _touchTarget.EnableImage(ImageType.Normalized);
                        _touchTarget.FrameReceived += OnTouchTargetFrameReceived;

                        _startEvent.Set();
                    }))).Start();

                    // block until started
                    _startEvent.WaitOne();
                    IsRunning = true;
                }
            }
        }

        public void Stop()
        {
            lock (this)
            {
                if (IsRunning)
                {
                    _touchTarget.FrameReceived -= OnTouchTargetFrameReceived;
                    _touchTarget.DisableImage(ImageType.Normalized);
                    _touchTarget.Dispose();

                    IsRunning = false;
                }
            }
        }

        private void OnTouchTargetFrameReceived(object sender, FrameReceivedEventArgs e)
        {
            //lock (this)
            //{
            // get image from Surface
            if (_surfaceImage == null)
            {
                e.TryGetRawImage(
                    ImageType.Normalized,
                    0, 0,
                    Microsoft.Surface.Core.InteractiveSurface.PrimarySurfaceDevice.WorkingAreaWidth,
                    Microsoft.Surface.Core.InteractiveSurface.PrimarySurfaceDevice.WorkingAreaHeight,
                    out _surfaceImage,
                    out _surfaceMetrics);
            }
            else
            {
                e.UpdateRawImage(
                    ImageType.Normalized,
                    _surfaceImage,
                    0, 0,
                    Microsoft.Surface.Core.InteractiveSurface.PrimarySurfaceDevice.WorkingAreaWidth,
                    Microsoft.Surface.Core.InteractiveSurface.PrimarySurfaceDevice.WorkingAreaHeight);
            }

            // create EmguCV image and fire event
            if (_emguCvImage == null) _emguCvImage = new IplImage(_surfaceMetrics.Width, _surfaceMetrics.Height, BitDepth.U8, 1);
            UpdateCvImage(_emguCvImage, _surfaceImage);
            _currentTimeStamp = DateTime.Now.Ticks;
            _imageEvent.Set();
            if (OnNewImage != null) OnNewImage(_emguCvImage);
            //}
        }

        private void UpdateCvImage(IplImage image, byte[] update)
        {
            Marshal.Copy(update, 0, image.ImageData, update.Length);
        }

        //// Surface image (byte array) to EmguCV image
        //private IplImage CreateEmguCvImage(byte[] image, ImageMetrics metrics)
        //{
        //    return Cv.GetImage(new CvMat(metrics.Height, metrics.Width, MatrixType.U8C1, image, true));
        //}

    }
}

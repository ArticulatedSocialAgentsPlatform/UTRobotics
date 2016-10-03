// Configuration
//#define DRAW_FACE_POINTS

namespace KinectSkeleton {

  using Microsoft.Kinect;
  using Microsoft.Kinect.Toolkit.FaceTracking;
  using System;
  using System.Diagnostics;
  using System.Collections.Generic;
  using System.Linq;
  using System.Runtime.InteropServices;
  using System.Windows.Forms;

  using BitMap = System.Drawing.Bitmap;
  using Brush = System.Drawing.Brush;
  using Color = System.Drawing.Color;
  using Pen = System.Drawing.Pen;
  using Point = System.Windows.Point;
  using SolidBrush = System.Drawing.SolidBrush;

  public partial class KinectTracker : Form {

    // Apache Apollo
    private StompHandler apollo;
    private Form settingsForm;

    // Kinect variables
    private KinectSensor kinect;
    private ColorImageFrame colorImageFrame;
    private DepthImageFrame depthImageFrame;
    private SkeletonFrame skeletonFrame;
    private Skeleton[] skeletonData { get; set; }
    private readonly Dictionary<int, SkeletonFaceTracker> trackedSkeletons = new Dictionary<int, SkeletonFaceTracker>();
    private ColorImageFormat colorStreamFormat = ColorImageFormat.RgbResolution640x480Fps30;
    private DepthImageFormat depthStreamFormat = DepthImageFormat.Resolution320x240Fps30;

    // Define default pens
    private Pen redPen = new Pen(Color.Red, 1);
    private Pen bluePen = new Pen(Color.Blue, 2);
    private Pen yellowPen = new Pen(Color.Yellow, 3);
    private Pen greenPen = new Pen(Color.Green, 3);
    private Pen orangePen = new Pen(Color.Orange, 2);
    private Brush orangeBrush = new SolidBrush(Color.Orange);

    // Image processing variables
    private byte[] colorData;
    private short[] depthData;
    private BitMap kinectVideoBitmap;
    private IntPtr colorPtr;

    // Audio processing variables
    private int audioDirection = -100;
    private double audioAngle = 0;
    private double audioConfidence = 0;

    // Output
    private SceneUpdate sceneUpdate;

    public KinectTracker() {

      // Init form
      InitializeComponent();

      // Init STOMP connection
      this.apollo = new StompHandler();
      this.sceneUpdate = new SceneUpdate();
    }

    private void KinectTracker_Load(object sender, EventArgs e) {
      try {
        this.initializeKinect();
      } catch {
        MessageBox.Show("Initialisation failed", "Kinect error");
        Application.Exit();
      }
    }

    private double calculateAudioLikeliness(int headX) {
      // Check how near the audio is to the headLocation
      double diff;
      if (headX > this.audioDirection) {
        diff = (double)headX - (double)this.audioDirection;
      } else {
        diff = (double)this.audioDirection - (double)headX;
      }

      if (diff == 0.0) return this.audioConfidence;

      // Calculate likeliness
      return this.audioConfidence / (diff / (this.kinectVideoBitmap.Width / 40.0));

    }

    private void fillDebugLabels() {
      if (this.debugOutputCheckbox.Checked) return;

      var person1 = this.sceneUpdate.getScenePerson(1);
      var person2 = this.sceneUpdate.getScenePerson(2);
      var format = "Person {0}";

      this.groupBox1.Text = String.Format(format, person1.TrackingId);
      this.skeletonHeadX1.Text = person1.SkeletonHeadX;
      this.skeletonHeadY1.Text = person1.SkeletonHeadY;
      this.faceHeadX1.Text = person1.FaceHeadX;
      this.faceHeadY1.Text = person1.FaceHeadY;
      this.depth1.Text = person1.SkeletonDepth;
      this.speech1.Text = person1.AudioLikeliness.ToString("0.00");

      this.groupBox2.Text = String.Format(format, person2.TrackingId);
      this.skeletonHeadX2.Text = person2.SkeletonHeadX;
      this.skeletonHeadY2.Text = person2.SkeletonHeadY;
      this.faceHeadX2.Text = person2.FaceHeadY;
      this.faceHeadY2.Text = person2.FaceHeadY;
      this.depth2.Text = person2.SkeletonDepth;
      this.speech2.Text = person2.AudioLikeliness.ToString("0.00");
    }

    private Tuple<int, int> convertSkeletonPoint(SkeletonPoint point) {

      ColorImagePoint jointPoint =
        kinect.CoordinateMapper.MapSkeletonPointToColorPoint(point, colorStreamFormat);

      float xd = Math.Max(0, Math.Min(jointPoint.X, this.kinectVideoBitmap.Width));
      float yd = Math.Max(0, Math.Min(jointPoint.Y, this.kinectVideoBitmap.Height));

      return Tuple.Create(
        (int)Math.Ceiling(xd),
        (int)Math.Ceiling(yd)
        );
    }

    private void DrawAudioDirection() {
      // Paint audio direction to image
      if (this.audioTrackingCheckbox.Checked) {
        var height = this.kinectVideoBitmap.Height;
        using (var graphics = System.Drawing.Graphics.FromImage(this.kinectVideoBitmap)) {
          graphics.DrawLine(this.orangePen, this.audioDirection - 5, height - 5, this.audioDirection + 5, height - 5);
          graphics.DrawLine(this.orangePen, this.audioDirection, height - 10, this.audioDirection, height);

          int drawTextAt = 0;
          if (this.audioDirection > (this.kinectVideoBitmap.Width - 50)) {
            drawTextAt = this.kinectVideoBitmap.Width - 50;
          } else {
            drawTextAt = this.audioDirection;
          }
          graphics.DrawString(String.Format("{0} - ({1})", this.audioAngle.ToString("0.0"), this.audioConfidence.ToString("0.0")),
            System.Drawing.SystemFonts.DefaultFont,
            this.orangeBrush,
            drawTextAt, height - 20);
        }
      }
    }

    private void DrawTrackedSkeletonJoints(JointCollection jointCollection) {
      // Render Head and Shoulders
      if (this.headTrackingCheckbox.Checked) {
        DrawHead(jointCollection[JointType.Head].Position);
      }

      if (this.skeletonTrackingCheckbox.Checked) {
        DrawBone(jointCollection[JointType.Head], jointCollection[JointType.ShoulderCenter]);
        DrawBone(jointCollection[JointType.ShoulderCenter], jointCollection[JointType.ShoulderLeft]);
        DrawBone(jointCollection[JointType.ShoulderCenter], jointCollection[JointType.ShoulderRight]);

        // Render Left Arm
        DrawBone(jointCollection[JointType.ShoulderLeft], jointCollection[JointType.ElbowLeft]);
        DrawBone(jointCollection[JointType.ElbowLeft], jointCollection[JointType.WristLeft]);
        DrawBone(jointCollection[JointType.WristLeft], jointCollection[JointType.HandLeft]);

        // Render Right Arm
        DrawBone(jointCollection[JointType.ShoulderRight], jointCollection[JointType.ElbowRight]);
        DrawBone(jointCollection[JointType.ElbowRight], jointCollection[JointType.WristRight]);
        DrawBone(jointCollection[JointType.WristRight], jointCollection[JointType.HandRight]);
      }
    }

    private void DrawSkeletonPosition(SkeletonPoint position) {

      // Convert the point to image location
      var convertedPoint = this.convertSkeletonPoint(position);

      // Draw circle to screen
      using (var graphics = System.Drawing.Graphics.FromImage(this.kinectVideoBitmap)) {
        graphics.DrawEllipse(yellowPen, convertedPoint.Item1 - 5, convertedPoint.Item2 - 5, 10, 10);
      }
    }

    private void DrawBone(Joint jointFrom, Joint jointTo) {
      if (jointFrom.TrackingState == JointTrackingState.NotTracked ||
      jointTo.TrackingState == JointTrackingState.NotTracked) {
        return; // nothing to draw, one of the joints is not tracked
      }

      if (jointFrom.TrackingState == JointTrackingState.Inferred ||
      jointTo.TrackingState == JointTrackingState.Inferred) {
        // Draw thin lines if either one of the joints is inferred
        DrawBoneLine(jointFrom.Position, jointTo.Position, false);
      }

      if (jointFrom.TrackingState == JointTrackingState.Tracked &&
      jointTo.TrackingState == JointTrackingState.Tracked) {
        // Draw bold lines if the joints are both tracked
        DrawBoneLine(jointFrom.Position, jointTo.Position, true);
      }
    }

    private void DrawBoneLine(SkeletonPoint start, SkeletonPoint end, bool tracked) {
      // Select pen for the line
      System.Drawing.Pen pen;
      if (tracked) {
        pen = bluePen;
      } else {
        pen = redPen;
      }

      // Convert points
      var startPoint = this.convertSkeletonPoint(start);
      var endPoint = this.convertSkeletonPoint(end);

      // Draw line to screen.
      using (var graphics = System.Drawing.Graphics.FromImage(this.kinectVideoBitmap)) {
        graphics.DrawLine(pen, startPoint.Item1, startPoint.Item2, endPoint.Item1, endPoint.Item2);
      }
    }

    private void DrawHead(SkeletonPoint head) {
      // Convert the point to image location
      var convertedPoint = this.convertSkeletonPoint(head);

      // Draw circle to screen
      using (var graphics = System.Drawing.Graphics.FromImage(this.kinectVideoBitmap)) {
        graphics.DrawEllipse(greenPen, convertedPoint.Item1 - 5, convertedPoint.Item2 - 5, 10, 10);
      }
    }

    private void fillUpdateTrackingId(int person, int val) {
      this.sceneUpdate.getScenePerson(person).TrackingId = val.ToString();
    }

    private void fillUpdateHeadX(int person, bool face, int val) {
      if (face) {
        this.sceneUpdate.getScenePerson(person).FaceHeadX = val.ToString();
      } else {
        this.sceneUpdate.getScenePerson(person).SkeletonHeadX = val.ToString();
      }
    }

    private void fillUpdateHeadY(int person, bool face, int val) {
      if (face) {
        this.sceneUpdate.getScenePerson(person).FaceHeadY = val.ToString();
      } else {
        this.sceneUpdate.getScenePerson(person).SkeletonHeadY = val.ToString();
      }
    }

    private void fillUpdateDepth(int person, int val) {
      this.sceneUpdate.getScenePerson(person).SkeletonDepth = val.ToString();
    }

    private void fillUpdateAudio(int person, double val) {
      this.sceneUpdate.getScenePerson(person).AudioLikeliness = val;
    }

    private void initializeKinect() {
      // Check for Kinect sensors
      if (KinectSensor.KinectSensors.Count == 0) {
        throw new Exception("Kinect device not detected. Is it connected and running?");
      }

      try {
        // Get first Kinect Sensor
        Debug.WriteLine("Finding Kinect sensor...");
        this.kinect = KinectSensor.KinectSensors.FirstOrDefault(s => s.Status == KinectStatus.Connected);

        // Enable color stream and skeleton tracking
        this.kinect.ColorStream.Enable(colorStreamFormat);
        this.kinect.DepthStream.Enable(depthStreamFormat);
        this.kinect.SkeletonStream.Enable();

        // Set some audio properties
        //this.kinect.AudioSource.NoiseSuppression = true;
        //this.kinect.AudioSource.EchoCancellationMode = EchoCancellationMode.CancellationAndSuppression;

        // Depth in near range enabled
        this.kinect.DepthStream.Range = DepthRange.Default;
        // Enable returning skeletons while depth is in Near Range
        this.kinect.SkeletonStream.EnableTrackingInNearRange = true;
        // Use seated tracking
        this.kinect.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;

        // Allocate Skeleton tracking data
        this.skeletonData = new Skeleton[this.kinect.SkeletonStream.FrameSkeletonArrayLength];

        // Connect the event listeners
        this.kinect.AllFramesReady +=
          new EventHandler<AllFramesReadyEventArgs>(this.kinect_AllFramesReady);
        this.kinect.AudioSource.SoundSourceAngleChanged +=
          new EventHandler<SoundSourceAngleChangedEventArgs>(this.kinect_AudioSourceSoundSourceAngleChanged);


        // Start the kinect sensor and the audiostream
        this.kinect.Start();
        this.kinect.AudioSource.Start();
        Debug.WriteLine("Kinect started!");
      } catch (Exception e) {
        throw new Exception("Kinect initialisation failed", e);
      }
    }

    private void kinect_AudioSourceSoundSourceAngleChanged(object sender, SoundSourceAngleChangedEventArgs e) {

      // Convert the angle to pixel (image width)
      // Positive angles -> right (0 - 50)
      // Negative angles -> left (-50 - 0)
      // 0 == center
      if (this.kinectVideoBitmap == null) return;
      var halfWidth = this.kinectVideoBitmap.Width / 2;

      // Determine location
      if (e.Angle > 0) {
        this.audioDirection = (int)Math.Ceiling((e.Angle / 50) * halfWidth) + halfWidth;
      } else if (e.Angle < 0) {
        this.audioDirection = halfWidth - (int)Math.Ceiling((Math.Abs(e.Angle) / 50) * halfWidth);
      } else {
        this.audioDirection = halfWidth;
      }

      // Check bounds
      this.audioDirection = Math.Min(this.audioDirection, this.kinectVideoBitmap.Width);
      this.audioDirection = Math.Max(this.audioDirection, 0);

      // Save values
      this.audioAngle = e.Angle;
      this.audioConfidence = e.ConfidenceLevel;
    }

    private void kinect_AllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs) {

      try {
        this.colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame();
        this.depthImageFrame = allFramesReadyEventArgs.OpenDepthImageFrame();
        this.skeletonFrame = allFramesReadyEventArgs.OpenSkeletonFrame();

        if (this.colorImageFrame == null
          || this.depthImageFrame == null
          || this.skeletonFrame == null) {
          return;
        }

        // Clear current state
        this.sceneUpdate.clear();

        // Get the skeletal information in this frame
        this.skeletonFrame.CopySkeletonDataTo(this.skeletonData);

        // If no data, fill it with black pixels
        if (this.colorData == null) this.colorData = new byte[this.colorImageFrame.PixelDataLength];
        if (this.depthData == null) this.depthData = new short[this.depthImageFrame.PixelDataLength];

        // Copy the pixel data 
        this.colorImageFrame.CopyPixelDataTo(this.colorData);
        this.depthImageFrame.CopyPixelDataTo(this.depthData);

        // Do some magic, as found on stackoverflow
        // Shamelessly stolen from http://stackoverflow.com/questions/12577935/how-can-i-show-kinect-video-stream-in-windows-forms-picturebox-appropriately
        Marshal.FreeHGlobal(this.colorPtr);
        this.colorPtr = Marshal.AllocHGlobal(this.colorData.Length);
        Marshal.Copy(this.colorData, 0, this.colorPtr, this.colorData.Length);

        // Create the video bitmap
        kinectVideoBitmap = new BitMap(
            this.colorImageFrame.Width,
            this.colorImageFrame.Height,
            this.colorImageFrame.Width * this.colorImageFrame.BytesPerPixel,
            System.Drawing.Imaging.PixelFormat.Format32bppRgb,
            colorPtr);

        // Draw the audio direction on the image
        this.DrawAudioDirection();

        // Draw the skeletons on the image
        this.processSkeletons();

        // Write the image to the videobox
        this.kinectVideoBox.Image = this.kinectVideoBitmap;

        // Process the updates to the view and message broker
        this.fillDebugLabels();
        this.apollo.sendTextMessage(this.sceneUpdate.serialize());

      } finally {
        if (this.colorImageFrame != null) {
          this.colorImageFrame.Dispose();
        }

        if (this.depthImageFrame != null) {
          this.depthImageFrame.Dispose();
        }

        if (this.skeletonFrame != null) {
          this.skeletonFrame.Dispose();
        }
      }
    }

    private void processSkeletons() {
      // Loop the skeleton data to draw the wanted lines
      var counter = 0;
      var counterSkeletonId = new Dictionary<int, int>();
      foreach (Skeleton skeleton in this.skeletonData) {

        // Sanity check
        if (skeleton == null) continue;

        // Check if tracked
        if (skeleton.TrackingState == SkeletonTrackingState.Tracked) {
          if (this.skeletonTrackingCheckbox.Checked || this.headTrackingCheckbox.Checked) {
            counter++;
            counterSkeletonId.Add(skeleton.TrackingId, counter);
            DrawTrackedSkeletonJoints(skeleton.Joints);

            // Add debug output
            var headLocation = this.convertSkeletonPoint(skeleton.Joints[JointType.Head].Position);
            this.fillUpdateTrackingId(counter, skeleton.TrackingId);
            this.fillUpdateHeadX(counter, false, headLocation.Item1);
            this.fillUpdateHeadY(counter, false, headLocation.Item2);

            var depthInfo = this.kinect.CoordinateMapper.MapSkeletonPointToDepthPoint(
              skeleton.Joints[JointType.Head].Position,
              this.depthStreamFormat
            );
            this.fillUpdateDepth(counter, depthInfo.Depth);

            // Check for audio likeliness
            var likeliness = this.calculateAudioLikeliness(headLocation.Item1);
            this.fillUpdateAudio(counter, likeliness);
          }
        } else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly) {
          if (this.skeletonTrackingCheckbox.Checked) {
            DrawSkeletonPosition(skeleton.Position);
          }
        }

        // We want keep a record of any skeleton, tracked or untracked.
        if (!this.trackedSkeletons.ContainsKey(skeleton.TrackingId)) {
          this.trackedSkeletons.Add(skeleton.TrackingId, new SkeletonFaceTracker());
        }

        // Give each tracker the upated frame.
        SkeletonFaceTracker skeletonFaceTracker;
        if (this.trackedSkeletons.TryGetValue(skeleton.TrackingId, out skeletonFaceTracker)) {
          skeletonFaceTracker.OnFrameReady(this.kinect, this.colorStreamFormat, this.colorData, this.depthStreamFormat, this.depthData, skeleton);
          skeletonFaceTracker.LastTrackedFrame = this.skeletonFrame.FrameNumber;

          // Draw face model on the image
          var location = skeletonFaceTracker.DrawFaceModel(this.kinectVideoBitmap, this.faceTrackingCheckbox.Checked);

          // Record the output
          if (location != null && counterSkeletonId.ContainsKey(skeleton.TrackingId)) {
            this.fillUpdateHeadX(counterSkeletonId[skeleton.TrackingId], true, (location.Item1 + location.Item2) / 2);
            this.fillUpdateHeadY(counterSkeletonId[skeleton.TrackingId], true, (location.Item3 + location.Item4) / 2);
          }
        }
      }

      // Clear SkeletonFaceTrackers
      var keys = this.trackedSkeletons.Keys;
      List<int> remove = new List<int>();
      for (var x = 0; x < keys.Count; x++) {
        if (counterSkeletonId.ContainsKey(keys.ElementAt(x))) {
          continue;
        }
        remove.Add(keys.ElementAt(x));
      }
      for (var i = 0; i < remove.Count; i++) {
        this.trackedSkeletons[remove[i]].Dispose();
        this.trackedSkeletons.Remove(remove[0]);
      }
    }

    private class SkeletonFaceTracker : IDisposable {

      private Pen lightYellowPen = new Pen(Color.LightYellow, 1);

      private static FaceTriangle[] faceTriangles;

      // More information about the facepoints
      // https://blogs.msdn.microsoft.com/kinectforwindows/2014/01/31/mysteries-of-kinect-for-windows-face-tracking-output-explained/
      private EnumIndexableCollection<FeaturePoint, Microsoft.Kinect.Toolkit.FaceTracking.PointF> facePoints;

      private FaceTracker faceTracker;

      private bool lastFaceTrackSucceeded;

      private SkeletonTrackingState skeletonTrackingState;

      public int LastTrackedFrame { get; set; }

      public void Dispose() {
        if (this.faceTracker != null) {
          this.faceTracker.Dispose();
          this.faceTracker = null;
        }
      }

      public Tuple<int, int, int, int> DrawFaceModel(BitMap kinectVideoBitmap, bool draw) {
        if (!this.lastFaceTrackSucceeded || this.skeletonTrackingState != SkeletonTrackingState.Tracked) {
          return null;
        }

        var faceModelPts = new List<Point>();
        int maxX = 0, maxY = 0, minX = kinectVideoBitmap.Width, minY = kinectVideoBitmap.Height;

        using (var graphics = System.Drawing.Graphics.FromImage(kinectVideoBitmap)) {
          for (int i = 0; i < this.facePoints.Count; i++) {
            Point point = new Point(this.facePoints[i].X + 0.5f, this.facePoints[i].Y + 0.5f);
            faceModelPts.Add(point);

            // Recalculate the rectangle dimensions
            var x = (int)Math.Ceiling(point.X);
            var y = (int)Math.Ceiling(point.Y);
            if (x > maxX) maxX = x;
            if (x < minX) minX = x;
            if (y > maxY) maxY = y;
            if (y < minY) minY = y;

            // Draw points
#if DRAW_FACE_POINTS
            if(draw) graphics.DrawEllipse(lightYellowPen, x - 1, y - 1, 2, 2);
#endif
          }

          // Draw rectangle around the head
          if (draw) graphics.DrawRectangle(lightYellowPen, new System.Drawing.Rectangle(minX, minY, maxX - minX, maxY - minY));
        }

        return Tuple.Create(
            minX,
            maxX,
            minY,
            maxY
          );

        // This code can be used to create triangles
        //var faceModel = new List<FaceModelTriangle>();
        //foreach (var t in faceTriangles) {
        //  var triangle = new FaceModelTriangle();
        //  triangle.P1 = faceModelPts[t.First];
        //  triangle.P2 = faceModelPts[t.Second];
        //  triangle.P3 = faceModelPts[t.Third];
        //  faceModel.Add(triangle);
        //}

        //var faceModelGroup = new GeometryGroup();
        //for (int i = 0; i < faceModel.Count; i++) {
        //  var faceTriangle = new GeometryGroup();
        //  faceTriangle.Children.Add(new LineGeometry(faceModel[i].P1, faceModel[i].P2));
        //  faceTriangle.Children.Add(new LineGeometry(faceModel[i].P2, faceModel[i].P3));
        //  faceTriangle.Children.Add(new LineGeometry(faceModel[i].P3, faceModel[i].P1));
        //  faceModelGroup.Children.Add(faceTriangle);
        //}

      }

      /// <summary>
      /// Updates the face tracking information for this skeleton
      /// </summary>
      internal void OnFrameReady(KinectSensor kinectSensor, ColorImageFormat colorImageFormat, byte[] colorImage, DepthImageFormat depthImageFormat, short[] depthImage, Skeleton skeletonOfInterest) {
        this.skeletonTrackingState = skeletonOfInterest.TrackingState;

        if (this.skeletonTrackingState != SkeletonTrackingState.Tracked) {
          // nothing to do with an untracked skeleton.
          return;
        }

        if (this.faceTracker == null) {
          try {
            this.faceTracker = new FaceTracker(kinectSensor);
          } catch (InvalidOperationException) {
            // During some shutdown scenarios the FaceTracker
            // is unable to be instantiated.  Catch that exception
            // and don't track a face.
            Debug.WriteLine("AllFramesReady - creating a new FaceTracker threw an InvalidOperationException");
            this.faceTracker = null;
          }
        }

        if (this.faceTracker != null) {
          FaceTrackFrame frame = this.faceTracker.Track(
              colorImageFormat, colorImage, depthImageFormat, depthImage, skeletonOfInterest);

          this.lastFaceTrackSucceeded = frame.TrackSuccessful;
          if (this.lastFaceTrackSucceeded) {
            if (faceTriangles == null) {
              // only need to get this once.  It doesn't change.
              faceTriangles = frame.GetTriangles();
            }

            this.facePoints = frame.GetProjected3DShape();
          }
        }
      }

      //private struct FaceModelTriangle {
      //  public Point P1;
      //  public Point P2;
      //  public Point P3;
      //}
    }
    
    private void updateApollo_Click(object sender, EventArgs e) {
      this.settingsForm = new ApolloSettings();
      this.settingsForm.Show();
      this.settingsForm.FormClosed += this.updateApollo_Closed;
    }

    private void updateApollo_Closed(object sender, EventArgs e) {
      this.settingsForm.Dispose();
      this.sceneUpdate.updateSettings();
      this.apollo.reconnect();
    }
  }
}

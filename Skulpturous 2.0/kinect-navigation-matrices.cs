using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using System;

#pragma warning disable 1591

namespace KinectNavigation
{
  public class Camera : IDisposable
  {
    // Members

    private Transaction _tr = null;
    private Document _doc = null;

    private Point3d _location = Point3d.Origin;
    private Point3d _target = Point3d.Origin;
    private Matrix3d _locTrans = Matrix3d.Identity;
    private Matrix3d _trgTrans = Matrix3d.Identity;

    private double _lensLength = 1.8;
    private double _zoom = 1.0;

    // Properties

    public double LensLength
    {
      get { return _lensLength; }
      set { _lensLength = value; }
    }

    public Point3d Location
    {
      get { return _location; }
      set { _location = value; }
    }

    public Point3d Target
    {
      get { return _target; }
      set { _target = value; }
    }

    public Matrix3d LocTransform
    {
      get { return _locTrans; }
      set { _locTrans = value; }
    }

    public Matrix3d TargTransform
    {
      get { return _trgTrans; }
      set { _trgTrans = value; }
    }

    public double Zoom
    {
      get { return _zoom; }
      set { _zoom = value; }
    }

    // Start a transaction on construction

    public Camera(Document doc)
    {
      _doc = doc;
      _tr = _doc.TransactionManager.StartTransaction();
    }

    // Commit and dispose of the transaction on Dispose

    public void Dispose()
    {
      if (_tr != null)
      {
        _tr.Commit();
        _tr.Dispose();
        _tr = null;
      }
    }

    // Apply our existing settings to the current view

    public void ApplyToCurrentView()
    {
      Editor ed = _doc.Editor;

      ViewportTableRecord vptr =
        (ViewportTableRecord)_tr.GetObject(
          ed.ActiveViewportId, OpenMode.ForRead
        );

      // Adjust the target

      if (_trgTrans != Matrix3d.Identity)
        _target = _target.TransformBy(_trgTrans);

      // Adjust the camera location

      if (_locTrans != Matrix3d.Identity)
        _location = _location.TransformBy(_locTrans);

      // Set up a view for the current settings

      ViewTableRecord vtr = new ViewTableRecord();

      vtr.CenterPoint = Point2d.Origin;
      vtr.ViewTwist = 0.0;
      vtr.PerspectiveEnabled = true;
      vtr.IsPaperspaceView = false;
      vtr.Height = vptr.Height;
      vtr.Width = vptr.Width;
      vtr.ViewDirection =
        Target.GetVectorTo(Location).MultiplyBy(Zoom);
      vtr.Target = Target;
      vtr.LensLength = LensLength;

      // Set it as the current view

      ed.SetCurrentView(vtr);
    }
  }

  public class Commands
  {
    // Flags for navigation modes

    bool _finished = false;
    bool _reset = false;
    bool _navigating = false;
    bool _orbiting = false;

    // The direction we're navigating in

    Vector3d _direction;

    double _zoomDist = 0.0;

    void OnSkeletonFrameReady(
      object sender, SkeletonFrameReadyEventArgs e
    )
    {
      using (SkeletonFrame s = e.OpenSkeletonFrame())
      {
        if (s != null)
        {
          Skeleton[] skels = new Skeleton[s.SkeletonArrayLength];
          s.CopySkeletonDataTo(skels);

          foreach (Skeleton data in skels)
          {
            if (data.TrackingState == SkeletonTrackingState.Tracked)
            {
              // Get the positions of joints we care about

              Point3d leftShoulder =
                PointFromVector(
                  data.Joints[JointType.ShoulderLeft].Position
                );
              Point3d rightShoulder =
                PointFromVector(
                  data.Joints[JointType.ShoulderRight].Position
                );
              Point3d leftHand =
                PointFromVector(
                  data.Joints[JointType.HandLeft].Position
                );
              Point3d rightHand =
                PointFromVector(
                  data.Joints[JointType.HandRight].Position
                );
              Point3d centerShoulder =
                PointFromVector(
                  data.Joints[JointType.ShoulderCenter].Position
                );

              // Make sure our hands are non-zero

              if (NonZero(leftHand) && NonZero(rightHand))
              {
                // We're finished if our hands are close together
                // and near the centre of our shoulders

                _finished =
                  CloseTo(leftHand, rightHand, 0.1) &&
                  CloseTo(leftHand, centerShoulder, 0.4);

                // Reset the view if our hands are at about the
                // same level but further apart

                _reset =
                  leftHand.DistanceTo(rightHand) > 0.5 &&
                  Math.Abs(leftHand.Y - rightHand.Y) < 0.1 &&
                  Math.Abs(leftHand.Z - rightHand.Z) < 0.1;

                // If neither of these modes is set...

                if (!_finished && !_reset)
                {
                  // .. we may still be navigating or orbiting

                  _navigating = false;
                  _orbiting = false;

                  if (CloseTo(leftHand, rightHand, 0.05))
                  {
                    // Hands are close together, but not near
                    // the chest which means we're navigating

                    _navigating = true;
                    _direction =
                      GetDirection(
                        leftHand + ((rightHand - leftHand) / 2),
                        centerShoulder
                      );

                    // Normalize to unit length

                    _direction = _direction / _direction.Length;
                  }
                  else if (
                    (
                      CloseTo(leftHand, leftShoulder, 0.3) &&
                      rightHand.DistanceTo(rightShoulder) > 0.4
                    ) ||
                    (
                      CloseTo(rightHand, rightShoulder, 0.3) &&
                      leftHand.DistanceTo(leftShoulder) > 0.4
                    )
                  )
                  {
                    // One hand is near its shoulder, and the other
                    // is pointed outwards, which means we're
                    // orbiting

                    _orbiting = true;
                    _direction =
                      (CloseTo(leftHand, leftShoulder) ?
                        GetDirection(rightHand, rightShoulder) :
                        GetDirection(leftHand, leftShoulder)
                      );

                    // Normalize to unit length

                    _direction = _direction / _direction.Length;
                  }
                }
              }
              break;
            }
          }
        }
      }
    }

    // Is the Point3d non-zero?

    internal static bool NonZero(Point3d pt)
    {
      return
        !CloseTo(
          pt, Point3d.Origin, Tolerance.Global.EqualPoint
        );
    }

    // Are two points within a certain distance?

    private static bool CloseTo(
      Point3d first, Point3d second, double dist = 0.1
    )
    {
      return first.DistanceTo(second) < dist;
    }

    // Get a Point3d from a Kinect Vector

    private static Point3d PointFromVector(SkeletonPoint p)
    {
      return new Point3d(p.X, p.Y, p.Z);
    }

    // Get the vector from the shoulder to the hand

    private static Vector3d GetDirection(
      Point3d hand, Point3d shoulder
    )
    {
      return shoulder - hand;
    }

    [CommandMethod("ADNPLUGINS", "KINNAV", CommandFlags.Modal)]
    public void NavigateWithKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Database db = doc.Database;
      Editor ed = doc.Editor;

      // As the user to select the camera and target locations

      PromptPointResult camRes =
        ed.GetPoint("\nPick camera location");
      if (camRes.Status != PromptStatus.OK)
        return;

      PromptPointResult tgtRes =
        ed.GetPoint("\nPick target location");
      if (tgtRes.Status != PromptStatus.OK)
        return;

      // And also the height from ground level

      PromptDoubleOptions pdo =
        new PromptDoubleOptions("\nEnter height from ground");
      pdo.UseDefaultValue = true;

      // Default height of 6' or 2m

      pdo.DefaultValue = (db.Lunits > 2 ? 6 : 2);

      PromptDoubleResult hgtRes = ed.GetDouble(pdo);
      if (hgtRes.Status != PromptStatus.OK)
        return;

      _zoomDist = hgtRes.Value / 10.0;

      // We need a Kinect object

      KinectSensor kinect = null;

      // Make sure we dispose of our Camera (which will
      // commit the transaction)

      Camera camera = new Camera(doc);
      using (camera)
      {
        // Set the height from the ground as a vector

        Vector3d height = new Vector3d(0, 0, hgtRes.Value);
        camera.Location = camRes.Value + height;
        camera.Target = tgtRes.Value + height;

        camera.ApplyToCurrentView();

        // Unset the loop termination flag

        _finished = false;
        _reset = true;

        if (KinectSensor.KinectSensors.Count <= 0)
        {
          ed.WriteMessage(
            "\nUnable to start Kinect sensor - " +
            "are you sure it's plugged in?"
          );
          return;
        }
                      
        // We need our Kinect sensor

        kinect = KinectSensor.KinectSensors[0];

        // We only care about skeleton information

        kinect.SkeletonFrameReady +=
          new EventHandler<SkeletonFrameReadyEventArgs>(
            OnSkeletonFrameReady
          );

        kinect.SkeletonStream.Enable();

        kinect.Start();

        // Loop until user terminates or cancels

        while (
          !_finished &&
          !HostApplicationServices.Current.UserBreak()
        )
        {
          // Direction from Kinect is:
          //
          // 0, 0, 1  - arm pointing directly at the center
          // 1, 0, 0  - arm pointing directly left
          // -1, 0, 0 - arm pointing directly right
          // 0, 1, 0  - arm pointing directly down
          // 0, -1, 0 - arm pointing directly up

          const double distFac = 0.001;
          const double angFac = 0.003;

          if (_reset)
          {
            // Reset to the initial view parameters

            camera.Location = camRes.Value + height;
            camera.Target = tgtRes.Value + height;

            camera.ApplyToCurrentView();

            _reset = false;
          }
          else if (_orbiting || _navigating)
          {
            Matrix3d locMat = Matrix3d.Identity;

            // We only orbit if not too close to the centre

            if (Math.Abs(_direction.X) > 0.2)
            {
              locMat =
                Matrix3d.Rotation(
                  -_direction.X * angFac,
                  Vector3d.ZAxis, camera.Target
                ) * locMat;
            }
            
            // And the same for upwards/downwards tilt

            if (Math.Abs(_direction.Y) > 0.2)
            {
              // To tilt upwards/downwards, we need the right axis
              // of rotation. Use a coordinate system object taking
              // the view direction as the Y axis and the Z axis of
              // WCS as the X axis, leaving the Z axis of the
              // coordinate system as our axis of rotation

              Vector3d newYaxis =
                camera.Location - camera.Target;
              newYaxis = newYaxis / newYaxis.Length;

              CoordinateSystem3d cs =
                new CoordinateSystem3d(
                  camera.Target,
                  Vector3d.ZAxis,
                  newYaxis
                );

              locMat =
                Matrix3d.Rotation(
                  -_direction.Y * angFac,
                  cs.Zaxis, camera.Target
                ) * locMat;
            }

            // If we're zooming, we adjust the camera and target
            // locations by a displacement along the viewing
            // direction

            if (_navigating)
            {
              Matrix3d trgMat =
                Matrix3d.Displacement(
                  (camera.Target - camera.Location) * distFac
                );
              locMat = trgMat * locMat;
              camera.TargTransform = trgMat;
            }
            camera.LocTransform = locMat;

            camera.ApplyToCurrentView();
          }

          // Reset the transformation values

          camera.LocTransform = Matrix3d.Identity;
          camera.TargTransform = Matrix3d.Identity;

          System.Windows.Forms.Application.DoEvents();
        }
      }

      kinect.Stop();

      kinect.SkeletonFrameReady -=
        new EventHandler<SkeletonFrameReadyEventArgs>(
          OnSkeletonFrameReady
        );
    }
  }
}
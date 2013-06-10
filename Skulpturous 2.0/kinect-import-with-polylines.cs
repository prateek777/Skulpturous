using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using Autodesk.AutoCAD.GraphicsInterface;
using Microsoft.Kinect;
using System.Collections.Generic;
using System;

#pragma warning disable 1591

namespace KinectSamples
{
  public class KinectPolyJig : KinectPointCloudJig
  {
    // A transaction and database to add polylines

    private Transaction _tr;
    private Document _doc;

    // A list of vertices to draw between
    // (we use this for the final polyline creation)

    private Point3dCollection _vertices;

    // The most recent vertex being captured/drawn

    private Point3d _curPt;
    private Entity _cursor;

    // A list of line segments being collected
    // (pass these as AcGe objects as they may
    // get created on a background thread)

    private List<LineSegment3d> _lineSegs;

    // The database lines we use for temporary
    // graphics (that need disposing afterwards)

    private DBObjectCollection _lines;

    // Flags to indicate Kinect gesture modes

    private bool _calibrating; // First skeleton callback
    private bool _drawing;     // Drawing mode active

    public KinectPolyJig(Document doc, Transaction tr)
    {
      // Initialise the various members

      _doc = doc;
      _tr = tr;
      _vertices = new Point3dCollection();
      _lineSegs = new List<LineSegment3d>();
      _lines = new DBObjectCollection();
      _cursor = null;
      _calibrating = true;
      _drawing = false;
    }

    public override void OnSkeletonFrameReady(
      object sender, SkeletonFrameReadyEventArgs e
    )
    {
      using (SkeletonFrame s = e.OpenSkeletonFrame())
      {
        Skeleton[] skels = new Skeleton[s.SkeletonArrayLength];
        s.CopySkeletonDataTo(skels);

        if (_calibrating)
        {
          DrawSkeleton(skels);
        }
        else
        {
          if (!Finished)
          {
            foreach (Skeleton data in skels)
            {
              if (
                data.TrackingState == SkeletonTrackingState.Tracked
              )
              {
                Point3d leftHip =
                  PointFromVector(
                    data.Joints[JointType.HipLeft].Position, false
                  );
                Point3d leftHand =
                  PointFromVector(
                    data.Joints[JointType.HandLeft].Position, false
                  );
                Point3d rightHand =
                  PointFromVector(
                    data.Joints[JointType.HandRight].Position, false
                  );

                _drawing = (leftHand.Z < leftHip.Z);

                if (
                  leftHand.DistanceTo(Point3d.Origin) > 0 &&
                  rightHand.DistanceTo(Point3d.Origin) > 0 &&
                  leftHand.DistanceTo(rightHand) < 0.1)
                {
                  _drawing = false;
                  Finished = true;
                }

                if (_drawing)
                {
                  // If we have at least one prior vertex...

                  if (_vertices.Count > 0)
                  {
                    // ... connect them together with
                    // a temp LineSegment3d

                    Point3d lastVert =
                      _vertices[_vertices.Count - 1];
                    if (lastVert.DistanceTo(rightHand) >
                        Tolerance.Global.EqualPoint)
                    {
                      _lineSegs.Add(
                        new LineSegment3d(lastVert, rightHand)
                      );
                    }
                  }

                  // Add the new vertex to our list

                  _vertices.Add(rightHand);
                }
                break;
              }
            }
          }
        }
      }
    }

    private void GetBodySegment(
      JointCollection joints, List<LineSegment3d> segs,
      params JointType[] ids
    )
    {
      // Get the sequence of segments connecting the
      // locations of the joints passed in

      Point3d start, end;
      start = PointFromVector(joints[ids[0]].Position);
      for (int i = 1; i < ids.Length; ++i)
      {
        end = PointFromVector(joints[ids[i]].Position);
        segs.Add(new LineSegment3d(start, end));
        start = end;
      }
    }

    private void DrawSkeleton(Skeleton[] skels)
    {
      // We won't simply append but replace what's there

      _lineSegs.Clear();

      foreach (Skeleton data in skels)
      {
        if (data.TrackingState == SkeletonTrackingState.Tracked)
        {
          // Draw dem bones, dem bones, dem dry bones...

          // Hip bone's conected to the spine bone,
          // Spine bone's connected to the shoulders bone,
          // Shoulders bone's connected to the head bone

          GetBodySegment(
            data.Joints, _lineSegs,
            JointType.HipCenter, JointType.Spine,
            JointType.ShoulderCenter, JointType.Head
          );

          // Middle of the shoulders connected to the
          // left shoulder,
          // Left shoulder's connected to the left elbow,
          // Left elbow's connected to the left wrist,
          // Left wrist's connected to the left hand

          GetBodySegment(
            data.Joints, _lineSegs,
            JointType.ShoulderCenter, JointType.ShoulderLeft,
            JointType.ElbowLeft, JointType.WristLeft,
            JointType.HandLeft
          );

          // Middle of the shoulders is connected to the
          // right shoulder,
          // Right shoulder's connected to the right elbow,
          // Right elbow's connected to the right wrist,
          // Right wrist's connected to the right hand

          GetBodySegment(
            data.Joints, _lineSegs,
            JointType.ShoulderCenter, JointType.ShoulderRight,
            JointType.ElbowRight, JointType.WristRight,
            JointType.HandRight
          );

          // Middle of the hips is connected to the left hip,
          // Left hip's connected to the left knee,
          // Left knee's connected to the left ankle,
          // Left ankle's connected to the left foot

          GetBodySegment(
            data.Joints, _lineSegs,
            JointType.HipCenter, JointType.HipLeft,
            JointType.KneeLeft, JointType.AnkleLeft,
            JointType.FootLeft
          );

          // Middle of the hips is connected to the right hip,
          // Right hip's connected to the right knee,
          // Right knee's connected to the right ankle,
          // Right ankle's connected to the right foot

          GetBodySegment(
            data.Joints, _lineSegs,
            JointType.HipCenter, JointType.HipRight,
            JointType.KneeRight, JointType.AnkleRight,
            JointType.FootRight
          );
        }
      }
    }

    protected override SamplerStatus SamplerData()
    {
      if (!_drawing && _lines.Count > 0)
      {
        AddPolylines();
      }

      // Generate a point cloud

      SamplerStatus res = SamplerStatus.Cancel;

      try
      {
        res = base.SamplerData();
      }
      catch { }

      return res;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!base.WorldDrawData(draw))
        return false;

      TransientManager ctm =
        TransientManager.CurrentTransientManager;
      IntegerCollection ints = new IntegerCollection();

      // Draw any outstanding segments (and do so only once)

      bool wasCalibrating = _calibrating;

      while (_lineSegs.Count > 0)
      {
        // Get the line segment and remove it from the list

        LineSegment3d ls = _lineSegs[0];
        _lineSegs.RemoveAt(0);

        // Create an equivalent, red, database line
        // (or yellow, if calibrating)

        Line ln = new Line(ls.StartPoint, ls.EndPoint);
        ln.ColorIndex = (wasCalibrating ? 2 : 1);
        _lines.Add(ln);

        // Draw it as transient graphics

        ctm.AddTransient(
          ln, TransientDrawingMode.DirectShortTerm,
          128, ints
        );

        _calibrating = false;
      }

      if (_drawing)
      {
        if (_cursor == null)
        {
          if (_vertices.Count > 0)
          {
            // Clear our skeleton

            ClearTransients();

            _curPt = _vertices[_vertices.Count - 1];

            // Make our sphere 10cm in diameter (5cm radius)

            Solid3d sol = new Solid3d();
            sol.CreateSphere(0.05);
            _cursor = sol;
            _cursor.TransformBy(
              Matrix3d.Displacement(_curPt - Point3d.Origin)
            );

            _cursor.ColorIndex = 2;

            ctm.AddTransient(
              _cursor, TransientDrawingMode.DirectShortTerm,
              128, ints
            );
          }
        }
        else
        {
          if (_vertices.Count > 0)
          {
            Point3d newPt = _vertices[_vertices.Count - 1];
            _cursor.TransformBy(
              Matrix3d.Displacement(newPt - _curPt)
            );
            _curPt = newPt;

            ctm.UpdateTransient(_cursor, ints);
          }
        }
      }
      else // !_drawing
      {
        if (_cursor != null)
        {
          ctm.EraseTransient(_cursor, ints);
          _cursor.Dispose();
          _cursor = null;
        }
      }

      return true;
    }

    public void AddPolylines()
    {
      ClearTransients();

      // Dispose of the database objects

      foreach (DBObject obj in _lines)
      {
        obj.Dispose();
      }
      _lines.Clear();

      // Create a true database-resident 3D polyline
      // (and let it be green)

      if (_vertices.Count > 1)
      {
        BlockTableRecord btr =
          (BlockTableRecord)_tr.GetObject(
            _doc.Database.CurrentSpaceId,
            OpenMode.ForWrite
          );

        Polyline3d pl =
          new Polyline3d(
            Poly3dType.SimplePoly, _vertices, false
          );
        pl.ColorIndex = 3;

        btr.AppendEntity(pl);
        _tr.AddNewlyCreatedDBObject(pl, true);
      }
      _vertices.Clear();
    }

    public void ClearTransients()
    {
      TransientManager ctm =
        TransientManager.CurrentTransientManager;

      // Erase the various transient graphics

      ctm.EraseTransients(
        TransientDrawingMode.DirectShortTerm, 128,
        new IntegerCollection()
      );
    }
  }

  public class KinectPolyCommands
  {
    [CommandMethod("ADNPLUGINS", "KINPOLY", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      Transaction tr =
        doc.TransactionManager.StartTransaction();
      
      KinectPolyJig kj = new KinectPolyJig(doc, tr);

      kj.InitializeSpeech();

      if (!kj.StartSensor())
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );
        tr.Dispose();
        return;
      }

      PromptResult pr = ed.Drag(kj);

      if (pr.Status != PromptStatus.OK && !kj.Finished)
      {
        kj.ClearTransients();
        kj.StopSensor();
        tr.Dispose();
        return;
      }

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      kj.AddPolylines();
      tr.Commit();

      // Manually dispose to avoid scoping issues with
      // other variables

      tr.Dispose();

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }
  }
}
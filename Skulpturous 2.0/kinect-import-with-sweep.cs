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
  public class KinectSolidsJig : KinectPointCloudJig
  {
    // Draw our transient spline in red

    const short transPathColor = 1;

    // Our transient solids (cursor sphere & tube) are yellow

    const short transSolColor = 2;

    // Our final solids will be green

    const short finalSolColor = 3;

    // A transaction and database to add solids

    private Transaction _tr;
    private Document _doc;

    // A list of vertices to draw between
    // (we use this for the final polyline creation)
    
    private Point3dCollection _vertices;

    // The most recent vertex being captured/drawn

    private Point3d _curPt;
    private Entity _cursor;

    // Entities to create our solid

    private Entity _profile;   // A profile circle
    private Spline _path;      // A spline path
    private Solid3d _tube;     // The solid itself
    private bool _sweepBroken; // Can the sweep continue?

    // The sweep options for creating the solid

    private SweepOptions _sweepOpts;

    // The radius of the profile circle to create

    private double _profRad;

    // Flags to indicate Kinect gesture modes

    private bool _drawing;     // Drawing mode active

    public KinectSolidsJig(
      Document doc, Transaction tr, double profRad
    )
    {
      // Initialise the various members

      _doc = doc;
      _tr = tr;
      _vertices = new Point3dCollection();
      _cursor = null;
      _drawing = false;
      _profile = null;
      _sweepOpts = null;
      _path = null;
      _tube = null;
      _profRad = profRad;
      _sweepBroken = false;
    }

    public override void OnSkeletonFrameReady(
      object sender, SkeletonFrameReadyEventArgs e
    )
    {
      if (!Finished)
      {
        using (SkeletonFrame s = e.OpenSkeletonFrame())
        {
          if (s != null)
          {
            Skeleton[] skels = new Skeleton[s.SkeletonArrayLength];
            s.CopySkeletonDataTo(skels);

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
                    // ... check whether we're a certain distance
                    // away from the last one before adding it (this
                    // smooths off the jitters of adding every point)

                    Point3d lastVert =
                      _vertices[_vertices.Count - 1];
                    if (lastVert.DistanceTo(rightHand) > 0.2)
                    {
                      // Add the new vertex to our list

                      _vertices.Add(rightHand);
                    }
                  }
                  else
                  {
                    // Add the first vertex to our list

                    _vertices.Add(rightHand);
                  }
                }
                break;
              }
            }
          }
        }
      }
    }

    public void Cleanup()
    {
      if (_path != null)
      {
        _path.Dispose();
        _path = null;
      };
      if (_profile != null)
      {
        _profile.Dispose();
        _profile = null;
      };
      if (_tube != null)
      {
        _tube.Dispose();
        _tube = null;
      };

      _sweepOpts = null;
      _vertices.Clear();
    }

    protected override SamplerStatus SamplerData()
    {
      // If not finished, but stopped drawing, add the
      // geometry that was previously drawn to the database

      if (!_drawing && (_path != null || _tube != null))
      {
        AddSolidOrPath();
      }

      return base.SamplerData();
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!base.WorldDrawData(draw))
        return false;

      short origCol = draw.SubEntityTraits.Color;

      // If we're currently drawing...

      if (_drawing)
      {
        try
        {
          // Let's start by creating our spline path

          if ((_path == null && _vertices.Count > 1) ||
              (_path != null &&
                _vertices.Count > _path.NumFitPoints))
          {
            if (_path != null)
              _path.Dispose();

            _path = new Spline(_vertices, 0, 0.0);

            // And our sweep profile, if we don't have one

            if (_profile != null)
              _profile.Dispose();

            _profile =
              new Circle(
                _vertices[0],
                _vertices[1] - _vertices[0],
                _profRad
              );

            // And our sweep options, if we don't have one

            if (_sweepOpts == null)
            {
              SweepOptionsBuilder sob =
                new SweepOptionsBuilder();

              // Align the entity to sweep to the path

              sob.Align =
                SweepOptionsAlignOption.AlignSweepEntityToPath;

              // The base point is the start of the path

              sob.BasePoint = _path.StartPoint;

              // The profile will rotate to follow the path

              sob.Bank = true;
              _sweepOpts = sob.ToSweepOptions();
            }

            // Finally create a blank solid, if it's null

            if (_tube == null)
              _tube = new Solid3d();

            // And sweep our profile along our path

            _tube.CreateSweptSolid(_profile, _path, _sweepOpts);
          }
        }
        catch (Autodesk.AutoCAD.Runtime.Exception ex)
        {
          _sweepBroken = true;
          _tube.Dispose();
          _tube = null;
          _doc.Editor.WriteMessage(
            "\nException: {0}", ex.Message
          );
        }

        // Draw our path, if we have one

        if (_path != null)
        {
          draw.SubEntityTraits.Color = transPathColor;
          _path.WorldDraw(draw);
        }

        // And our solid

        if (_tube != null)
        {
          draw.SubEntityTraits.Color = transSolColor;
          _tube.WorldDraw(draw);
        }

        if (_vertices.Count > 0)
        {
          // Get the last point (at which our cursor should
          // be located, if it exists)

          Point3d lastPt = _vertices[_vertices.Count - 1];

          if (_cursor == null)
          {
            // Create a cursor sphere

            _cursor = new Solid3d();
            ((Solid3d)_cursor).CreateSphere(_profRad);
            _curPt = Point3d.Origin;
          }

          // Move it to the current point

          _cursor.TransformBy(
            Matrix3d.Displacement(lastPt - _curPt)
          );
          _curPt = lastPt;

          // Draw the cursor

          draw.SubEntityTraits.Color =
            (_sweepBroken ? transPathColor : transSolColor);

          _cursor.WorldDraw(draw);
        }
      }

      draw.SubEntityTraits.Color = origCol;

      return true;
    }

    public void AddSolidOrPath()
    {
      if (_tube != null || _path != null)
      {
        // We'll add the swept solid, if we have one, otherwise
        // we'll add the path

        Entity ent;

        if (_tube == null)
        {
          ent = _path;
          _path = null;
        }
        else
        {
          ent = _tube;
          _tube = null;
        }

        BlockTableRecord btr =
          (BlockTableRecord)_tr.GetObject(
            _doc.Database.CurrentSpaceId,
            OpenMode.ForWrite
          );

        ent.ColorIndex = finalSolColor;

        btr.AppendEntity(ent);
        _tr.AddNewlyCreatedDBObject(ent, true);

      }

      Cleanup();

      _vertices.Clear();

      _sweepBroken = false;
    }
  }

  public class KinectSolidsCommands
  {
    [CommandMethod("ADNPLUGINS", "KINEXT", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      PromptDoubleOptions pdo =
        new PromptDoubleOptions("\nEnter profile radius");
      pdo.AllowZero = false;
      pdo.AllowNegative = false;
      pdo.AllowNone = false;
      pdo.DefaultValue = 0.05;
      pdo.UseDefaultValue = true;
      PromptDoubleResult pdr = ed.GetDouble(pdo);

      if (pdr.Status != PromptStatus.OK)
        return;

      Transaction tr =
        doc.TransactionManager.StartTransaction();

      KinectSolidsJig kj = new KinectSolidsJig(doc, tr, pdr.Value);

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
        kj.StopSensor();
        kj.Cleanup();
        tr.Dispose();
        return;
      }

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      kj.AddSolidOrPath();
      tr.Commit();

      // Manually dispose to avoid scoping issues with
      // other variables

      tr.Dispose();

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }
  }
}
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using System.Collections.Generic;
using System;

#pragma warning disable 1591

namespace KinectSamples
{
  public class KinectCombinedJig : KinectPointCloudJig
  {
    // A list of lines representing our skeleton(s)

    private List<Line> _lines;

    public List<Line> Lines
    {
      get { return _lines; }
    }

    // Flags to make sure we don't end up both modifying
    // and accessing the _lines member at the same time

    private bool _drawing = false;
    private bool _capturing = false;

    public KinectCombinedJig()
    {
      // Initialise the various members

      _lines = new List<Line>();
    }

    public override void OnSkeletonFrameReady(
      object sender, SkeletonFrameReadyEventArgs e
    )
    {
      if (!_drawing)
      {
        _capturing = true;

        // Clear any previous lines

        ClearLines();

        // Get access to the SkeletonFrame

        using (SkeletonFrame s = e.OpenSkeletonFrame())
        {
          if (s != null)
          {
            // We'll colour the skeletons from yellow, onwards
            // (red is a bit dark)

            short col = 2;

            // Loop through each of the skeletons

            Skeleton[] skels = new Skeleton[s.SkeletonArrayLength];
            s.CopySkeletonDataTo(skels);
            for (int i = 0; i < skels.Length; i++)
            {
              Skeleton data = skels[i];

              // Add skeleton vectors for tracked/positioned
              // skeletons

              if (
                data.TrackingState ==
                  SkeletonTrackingState.Tracked ||
                data.TrackingState ==
                  SkeletonTrackingState.PositionOnly
              )
              {
                AddLinesForSkeleton(_lines, data, col++);
              }
            }
            _capturing = false;
          }
        }
      }
    }

    private void AddLinesForSkeleton(
      List<Line> lines, Skeleton sd, int idx
    )
    {
      // Hard-code lists of connections between joints

      int[][] links =
        new int[][]
        {
          // Head to left toe
          new int[] { 3, 2, 1, 0, 12, 13, 14, 15 },
          // Hips to right toe
          new int[] { 0, 16, 17, 18, 19 },
          // Left hand to right hand
          new int[] { 7, 6, 5, 4, 2, 8, 9, 10, 11 }
        };

      // Populate an array of joints

      Point3dCollection joints = new Point3dCollection();
      for (int i = 0; i < 20; i++)
      {
        joints.Add(
          PointFromVector(sd.Joints[(JointType)i].Position, false)
        );
      }

      // For each path of joints, create a sequence of lines

      foreach (int[] link in links)
      {
        for (int i = 0; i < link.Length - 1; i++)
        {
          // Line from this vertex to the next

          Line ln =
            new Line(joints[link[i]], joints[link[i + 1]]);

          // Set the color to distinguish between skeletons

          ln.ColorIndex = idx;

          // Make tracked skeletons bolder

          ln.LineWeight =
            (sd.TrackingState == SkeletonTrackingState.Tracked ?
              LineWeight.LineWeight050 :
              LineWeight.LineWeight000
            );

          lines.Add(ln);
        }
      }
    }

    private void ClearLines()
    {
      // Dispose each of the lines and clear the list

      foreach (Line ln in _lines)
      {
        ln.Dispose();
      }
      _lines.Clear();
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!_capturing)
      {
        _drawing = true;

        // Draw each of our lines

        short oidx = draw.SubEntityTraits.Color;

        foreach (Line ln in _lines)
        {
          // Set the colour and lineweight in the subentity
          // traits based on the original line

          if (ln != null)
          {
            draw.SubEntityTraits.Color = (short)ln.ColorIndex;
            draw.SubEntityTraits.LineWeight = ln.LineWeight;

            ln.WorldDraw(draw);
          }
        }

        draw.SubEntityTraits.Color = oidx;

        _drawing = false;
      }

      return base.WorldDrawData(draw);
    }
  }

  public class KinectCombinedCommands
  {
    [CommandMethod("ADNPLUGINS", "KINBOTH", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;


      KinectCombinedJig kj = new KinectCombinedJig();

      kj.InitializeSpeech();

      if (!kj.StartSensor())
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );
        return;
      }

      PromptResult pr = ed.Drag(kj);

      if (pr.Status != PromptStatus.OK)
      {
        kj.StopSensor();
        return;
      }

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      AddLines(doc, kj.Lines);

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }

    public void AddLines(Document doc, List<Line> lines)
    {
      if (lines.Count > 0)
      {
        Transaction tr =
          doc.TransactionManager.StartTransaction();
        using (tr)
        {
          BlockTableRecord btr =
            (BlockTableRecord)tr.GetObject(
              doc.Database.CurrentSpaceId,
              OpenMode.ForWrite
            );

          foreach (Line ln in lines)
          {
            ln.ColorIndex = 1;
            btr.AppendEntity(ln);
            tr.AddNewlyCreatedDBObject(ln, true);
          }
          tr.Commit();
        }
      }
    }
  }
}
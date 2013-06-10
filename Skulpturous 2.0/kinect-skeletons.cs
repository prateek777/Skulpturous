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

namespace KinectSkeletons
{
  public class KinectSkeletonJig : DrawJig, IDisposable
  {
    private KinectSensor _kinect = null;
    private List<Line> _lines;

    // Flags to make sure we don't end up both modifying
    // and accessing the _lines member at the same time

    private bool _drawing = false;
    private bool _capturing = false;

    // An offset value we use to move the mouse back
    // and forth by one screen unit

    private int _offset;

    public KinectSkeletonJig()
    {
      // Initialise members

      _offset = 1;
      _lines = new List<Line>();
      _kinect = KinectSensor.KinectSensors[0];

      _drawing = false;
      _capturing = false;

      // Attach the event handler

      _kinect.SkeletonFrameReady +=
        new EventHandler<SkeletonFrameReadyEventArgs>(
          OnSkeletonFrameReady
        );

      // Initialise the Kinect sensor

      _kinect.SkeletonStream.Enable();

      _kinect.Start();
    }

    public void Dispose()
    {
      // Uninitialise the Kinect sensor

      _kinect.Stop();

      // Detach the event handler

      _kinect.SkeletonFrameReady -=
        new EventHandler<SkeletonFrameReadyEventArgs>(
          OnSkeletonFrameReady
        );

      // Clear our line list

      ClearLines();
    }

    protected override SamplerStatus Sampler(JigPrompts prompts)
    {
      // We don't really need a point, but we do need some
      // user input event to allow us to loop, processing
      // for the Kinect input

      PromptPointResult ppr =
        prompts.AcquirePoint("\nClick to finish: ");
      if (ppr.Status == PromptStatus.OK)
      {
        // Let's move the mouse slightly to avoid having
        // to do it manually to keep the input coming

        System.Drawing.Point pt =
          System.Windows.Forms.Cursor.Position;
        System.Windows.Forms.Cursor.Position =
          new System.Drawing.Point(
            pt.X, pt.Y + _offset
          );
        _offset = -_offset;

        return SamplerStatus.OK;
      }
      return SamplerStatus.Cancel;
    }

    protected override bool WorldDraw(WorldDraw draw)
    {
      if (!_capturing)
      {
        _drawing = true;

        // Draw each of our lines

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

        _drawing = false;
      }

      return true;
    }

    void OnSkeletonFrameReady(
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

              // Add skeleton vectors for tracked/positioned skeletons

              if (
                data.TrackingState == SkeletonTrackingState.Tracked ||
                data.TrackingState == SkeletonTrackingState.PositionOnly
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

    // Get a Point3d from a Kinect Vector

    private static Point3d PointFromVector(SkeletonPoint p)
    {
      // Rather than just return a point, we're effectively
      // transforming it to the drawing space: flipping the
      // Y and Z axes

      return new Point3d(p.X, p.Z, p.Y);
    }

    private void AddLinesForSkeleton(
      List<Line> lines, Skeleton sk, int idx
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
        joints.Add(PointFromVector(sk.Joints[(JointType)i].Position));
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
            (sk.TrackingState == SkeletonTrackingState.Tracked ?
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
  }

  public class Commands
  {
    [CommandMethod("ADNPLUGINS", "KINSKEL", CommandFlags.Modal)]
    public void KinectSkeletons()
    {
      Editor ed =
        Application.DocumentManager.MdiActiveDocument.Editor;

      try
      {
        // Create and use our jig, disposing afterwards

        using (KinectSkeletonJig sj = new KinectSkeletonJig())
        {
          ed.Drag(sj);
        }
      }
      catch (System.Exception ex)
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor: " + ex.Message
        );
      }
    }
  }
}
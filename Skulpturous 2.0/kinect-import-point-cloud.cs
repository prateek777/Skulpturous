using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using Autodesk.AutoCAD.GraphicsInterface;
using Microsoft.Kinect;
using System.Collections.Generic;

#pragma warning disable 1591

namespace KinectSamples
{
  public class KinectPointCloudJig : KinectJig
  {
    // A list of points captured by the sensor
    // (for eventual export)

    private List<ColoredPoint3d> _vecs;

    public List<ColoredPoint3d> Vectors
    {
      get { return _vecs; }
    }

    // A list of points to be displayed
    // (we use this for the jig)

    private Point3dCollection _points;

    public KinectPointCloudJig()
    {
      _points = new Point3dCollection();
    }

    public void UpdatePointCloud()
    {
      _vecs = GeneratePointCloud(1, true);
    }

    protected override SamplerStatus SamplerData()
    {
      // Generate a point cloud

      try
      {
        // Use a user-defined sampling the points for the jig

        _vecs = GeneratePointCloud(Sampling);

        // Extract the points for display in the jig

        _points.Clear();

        foreach (ColoredPoint3d vec in _vecs)
        {
          _points.Add(
            new Point3d(vec.X, vec.Y, vec.Z)
          );
        }
      }
      catch {}

      ForceMessage();

      return SamplerStatus.OK;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      // This simply draws our points

      draw.Geometry.Polypoint(_points, null, null);

      return true;
    }
  }

  public class KinectPointCloudCommands
  {
    [CommandMethod("ADNPLUGINS", "KINECT", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      KinectPointCloudJig kj = new KinectPointCloudJig();

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

      if (pr.Status != PromptStatus.OK && !kj.Finished)
      {
        kj.StopSensor();
        return;
      }

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }
  }
}
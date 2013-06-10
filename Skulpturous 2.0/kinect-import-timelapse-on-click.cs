using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using System.Collections.Generic;
using System.Timers;

#pragma warning disable 1591

namespace KinectSamples
{
  public class KinectDelayJig : KinectJig
  {
    // A list of points captured by the sensor
    // (for eventual export)

    private List<ColoredPoint3d> _vecs;

    // Uber list of points for final export/import

    private List<ColoredPoint3d> _totalVecs;

    public List<ColoredPoint3d> Vectors
    {
      get { return _totalVecs; }
    }

    // A list of points to be displayed
    // (we use this for the jig)

    private Point3dCollection _points;

    // Transient points that have already been snapped

    private List<Point3dCollection> _snapped;

    // The number of snapshots to be taken

    private int _numShots;

    // Is it time to take a snapshot of the sensor data?

    private bool _snapshot;

    // Timer to tell us when to take a snapshot

    private Timer _timer;

    public KinectDelayJig(
      int numShots, double delay
    )
    {
      // Initialise the various members

      _numShots = numShots;
      
      _points = new Point3dCollection();
      _snapped = new List<Point3dCollection>();
      _totalVecs = new List<ColoredPoint3d>();
      _snapshot = false;
      _timer = new System.Timers.Timer(delay * 1000);
      _timer.Enabled = false;

      // Hook up the Elapsed event for the timer

      _timer.Elapsed +=
        delegate(object source, ElapsedEventArgs args)
        {
          // Flag that it's time to capture a snapshot
          
          _snapshot = true;
          
          // Turn off the timer to be re-enabled later

          ((Timer)source).Enabled = false;
        };
    }

    public void StartTimer()
    {
      if (_timer != null)
      {
        _timer.Enabled = true;
      }
    }

    protected override SamplerStatus Sampler(JigPrompts prompts)
    {
      // We don't really need a point, but we do need some
      // user input event to allow us to loop, processing
      // for the Kinect input

      PromptPointResult ppr =
        prompts.AcquirePoint("\nClick to capture: ");
      if (ppr.Status == PromptStatus.OK)
      {
        if (Finished)
        {
          CancelJig();
          return SamplerStatus.Cancel;
        }

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

          ForceMessage();
        }
        catch { }

        return SamplerStatus.OK;
      }
      return SamplerStatus.Cancel;
    }

    protected override bool WorldDraw(WorldDraw draw)
    {
      if (_snapshot)
      {
        if (_points.Count > 0)
        {
          List<ColoredPoint3d> vecList = GeneratePointCloud(1, true);

          // Add the core list to the total set

          _totalVecs.AddRange(vecList);

          // Make a copy of the latest set of jigged points

          Point3d[] tmp = new Point3d[_points.Count];
          _points.CopyTo(tmp, 0);

          // Add the copy to the list of snapshot previews

          _snapped.Add(new Point3dCollection(tmp));
        }
      }

      short origColor = draw.SubEntityTraits.Color;

      for (int i = 0; i < _snapped.Count; i++)
      {
        // Cycle through colour indeces for each snapshot

        draw.SubEntityTraits.Color = (short)(i + 1);
        
        // Draw the actual snapshot, one by one

        if (_snapped[i].Count > 0)
          draw.Geometry.Polypoint(_snapped[i], null, null);
      }

      // Set the colour back to the original

      draw.SubEntityTraits.Color = origColor;

      if (_snapshot)
      {
        // Reset the flag, timer and check whether finished

        _snapshot = false;
        Finished = (--_numShots == 0);
        _timer.Enabled = true;
      }
      else
      {
        // This simply draws our points

        if (_points.Count > 0)
          draw.Geometry.Polypoint(_points, null, null);
      }

      return true;
    }
  }

  public class KinectSnapshotCommands
  {
    // Static members for command settings

    private static int _numShots = 5;
    private static double _delay = 5.0;

    [CommandMethod("ADNPLUGINS", "KINSNAPS", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      // Get some user input for the number of snapshots...

      PromptIntegerOptions pio =
        new PromptIntegerOptions("\nNumber of captures");
      pio.AllowZero = false;
      pio.DefaultValue = _numShots;
      pio.UseDefaultValue = true;
      pio.UpperLimit = 20;
      pio.LowerLimit = 1;

      PromptIntegerResult pir = ed.GetInteger(pio);

      if (pir.Status != PromptStatus.OK)
        return;

      _numShots = pir.Value;

      // ... and the delay between them

      PromptDoubleOptions pdo =
        new PromptDoubleOptions("\nNumber of seconds delay");
      pdo.AllowZero = false;
      pdo.AllowNegative = false;
      pdo.AllowArbitraryInput = false;
      pdo.DefaultValue = _delay;
      pdo.UseDefaultValue = true;

      PromptDoubleResult pdr = ed.GetDouble(pdo);

      if (pdr.Status != PromptStatus.OK)
        return;

      _delay = pdr.Value;

      Transaction tr =
        doc.TransactionManager.StartTransaction();

      KinectDelayJig kj = new KinectDelayJig(_numShots, _delay);

      if (!kj.StartSensor())
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );
        return;
      }
      
      PromptResult pr = ed.Drag(kj);
      if (pr.Status == PromptStatus.OK)
      {
        kj.StartTimer();
        pr = ed.Drag(kj);
      }
      kj.StopSensor();

      if (pr.Status != PromptStatus.OK && !kj.Finished)
      {
        tr.Dispose();
        return;
      }

      tr.Commit();

      // Manually dispose to avoid scoping issues with
      // other variables

      tr.Dispose();

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }
  }
}
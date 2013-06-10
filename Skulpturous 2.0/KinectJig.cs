using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using System.Linq;
using System;
using System.IO;

#pragma warning disable 1591

namespace KinectSamples
{
  public class ColoredPoint3d
  {
    public double X, Y, Z;
    public int R, G, B;
  }

  public class WordFoundEventArgs : EventArgs
  {
    string _word = "";

    // Constructor

    public WordFoundEventArgs(string word)
    {
      _word = word;
    }

    // Property
    
    public string Word
    {
      get { return _word; }
    }
  }
  
  public abstract class KinectJig : DrawJig
  {
    // To stop the running jig by sending a cancel request

    [DllImport("acad.exe", CharSet = CharSet.Auto,
      CallingConvention = CallingConvention.Cdecl,
      EntryPoint = "?acedPostCommand@@YAHPEB_W@Z"
     )]  
    extern static private int acedPostCommand(string strExpr);

    // We need our Kinect sensor

    private KinectSensor _kinect = null;

    // And audio array

    private KinectAudioSource _audio = null;

    // Microsoft Speech recognition engine

    private SpeechRecognitionEngine _sre;

    // A word has been recognised

    private string _word = "";

    // With the data collected by the sensor

    private short[] _depthPixels = null;
    private byte[] _colorPixels = null;
    private Skeleton[] _skeletons = null;

    // An offset value we use to move the mouse back
    // and forth by one screen unit

    private int _offset;

    // Flag and property for when we want to exit

    private bool _finished;

    public bool Finished
    {
      get { return _finished; }
      set { _finished = value; }
    }

    // Selected color, for whatever usage makes sense
    // in child classes

    protected short _colorIndex;

    internal short ColorIndex
    {
      get { return _colorIndex; }
      set { _colorIndex = value; }
    }

    // Elevation setting to apply to Kinect camera

    private int _tilt;

    internal int Tilt
    {
      get { return _tilt; }
      set { _tilt = value; }
    }

    // Sampling value to reduce point set on capture

    internal static short Sampling
    {
      get
      {
        try
        {
          return (short)Application.GetSystemVariable("KINSAMP");
        }
        catch
        {
          return 50;
        }
      }
    }

    // Sampling value to reduce point set on capture

    internal static bool Speech
    {
      get
      {
        try
        {
          return
            (short)Application.GetSystemVariable("KINSPEECH") == 1;
        }
        catch
        {
          return false;
        }
      }
    }

    // Audio support requires words to check

    protected List<string> Words = new List<string>();

    public event EventHandler<WordFoundEventArgs> FoundWord;

    protected virtual void OnFoundWord(WordFoundEventArgs e)
    {
      if (FoundWord != null)
      {
        FoundWord(this, e);
      }

      _word = e.Word;

      switch (_word)
      {
        case "UP":
          Tilt = 1;
          break;
        case "DOWN":
          Tilt = -1;
          break;
        case "CANCEL":
          CancelJig();
          break;
        case "FINISH":
          Finished = true;
          break;
        default:
          break;
      }
    }

    // Extents to filter points

    private static Extents3d? _ext = null;

    public static Extents3d? Extents
    {
      get { return _ext; }
      set { _ext = value; }
    }

    public KinectJig()
    {
      // Initialise the various members

      _offset = 1;
      _colorIndex = 3;
      _tilt = 0;
      _finished = false;

      if (KinectSensor.KinectSensors.Count > 0)
      {
        _kinect = KinectSensor.KinectSensors[0];

        _kinect.AllFramesReady +=
          new EventHandler<AllFramesReadyEventArgs>(
            OnAllFramesReady
          );

        _kinect.SkeletonFrameReady +=
          new EventHandler<SkeletonFrameReadyEventArgs>(
            OnSkeletonFrameReady
          );
      }

      Words.Add("up");
      Words.Add("down");

      Words.Add("finish");
      Words.Add("cancel");
    }

    public virtual void OnSkeletonFrameReady(
      object sender, SkeletonFrameReadyEventArgs e
    )
    {
    }

    public virtual void OnAllFramesReady(
      object sender, AllFramesReadyEventArgs e
    )
    {
      using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
      {
        if (colorFrame != null)
        {
          Debug.Assert(
            colorFrame.Width == 640 &&
            colorFrame.Height == 480,
            "This app only uses 640x480."
          );

          if (_colorPixels == null ||
              _colorPixels.Length != colorFrame.PixelDataLength)
            _colorPixels = new byte[colorFrame.PixelDataLength];

          colorFrame.CopyPixelDataTo(_colorPixels);
        }
      }

      using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
      {
        if (depthFrame != null)
        {
          Debug.Assert(
            depthFrame.Width == 640 &&
            depthFrame.Height == 480,
            "This app only uses 640x480."
          );

          if (_depthPixels == null ||
              _depthPixels.Length != depthFrame.PixelDataLength)
            _depthPixels = new short[depthFrame.PixelDataLength];

          depthFrame.CopyPixelDataTo(_depthPixels);
        }
      }

      using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
      {
        if (skeletonFrame != null)
        {
          if (_skeletons == null) // allocate the first time
            _skeletons =
              new Skeleton[skeletonFrame.SkeletonArrayLength];
          skeletonFrame.CopySkeletonDataTo(_skeletons);
        }
      }
    }

    void OnSpeechHypothesized(
      object sender, SpeechHypothesizedEventArgs e
    )
    {
    }

    void OnSpeechRecognized(
      object sender, SpeechRecognizedEventArgs e
    )
    {
      // Ignore if we don't have a high degree of confidence

      if (e.Result.Confidence < 0.7)
        return;

      OnFoundWord(
        new WordFoundEventArgs(e.Result.Text.ToUpperInvariant())
      );
    }

    public bool StartSensor()
    {
      if (_kinect != null)
      {
        // We still need to enable skeletal tracking
        // in order to map to "real" space, even
        // if we're not actually getting skeleton data

        _kinect.ColorStream.Enable(
          ColorImageFormat.RgbResolution640x480Fps30
        );
        _kinect.DepthStream.Enable(
          DepthImageFormat.Resolution640x480Fps30
        );
        _kinect.SkeletonStream.Enable();

        _kinect.Start();

        _kinect.ElevationAngle = 0;

        // We need speech recognition started on a separate,
        // MTA thread

        if (Speech)
        {
          var t = new Thread(StartSpeech);
          t.Start();
        }

        return true;
      }
      return false;
    }

    private static RecognizerInfo GetKinectRecognizer()
    {
      Func<RecognizerInfo, bool> matchingFunc =
        r =>
        {
          string value;
          r.AdditionalInfo.TryGetValue("Kinect", out value);
          return
            "True".Equals(
              value,
              StringComparison.InvariantCultureIgnoreCase
            ) &&
            "en-US".Equals(
              r.Culture.Name,
              StringComparison.InvariantCultureIgnoreCase
            );
        };
      return
        SpeechRecognitionEngine.InstalledRecognizers().Where(
          matchingFunc
        ).FirstOrDefault();
    }

    public void InitializeSpeech()
    {
      Editor ed =
        Application.DocumentManager.MdiActiveDocument.Editor;

      if (!Speech || Words.Count == 0)
        return;

      // Create and setup our audio source

      _audio = _kinect.AudioSource;
      _audio.EchoCancellationMode =
        EchoCancellationMode.None;
      _audio.AutomaticGainControlEnabled = false;
      _audio.BeamAngleMode = BeamAngleMode.Automatic;

      RecognizerInfo ri = GetKinectRecognizer();
      if (ri == null)
      {
        ed.WriteMessage(
          "There was a problem initializing Speech Recognition. " +
          "Ensure you have the Microsoft Speech SDK installed " +
          "and configured."
        );
      }

      // Need to wait 4 seconds for device to be ready right after
      // initialization

      ed.WriteMessage("\n");

      int wait = 4;
      while (wait > 0)
      {
        ed.WriteMessage(
          "Kinect will be ready for speech recognition in {0} " +
          "second{1}.\n",
          wait--,
          wait == 0 ? "" : "s"
        );
        Thread.Sleep(1000);
      }
      ed.WriteMessage("Kinect ready for speech recognition.\n");

      try
      {
        _sre = new SpeechRecognitionEngine(ri.Id);
      }
      catch
      {
        ed.WriteMessage(
          "There was a problem initializing Speech Recognition. " +
          "Ensure you have the Microsoft Speech SDK installed " +
          "and configured."
        );
      }

      // Populate our word choices

      Choices words = new Choices();
      foreach (string word in Words)
      {
        words.Add(word);
      }

      // Create a GrammarBuilder from them

      GrammarBuilder gb = new GrammarBuilder();
      gb.Culture = ri.Culture;
      gb.Append(words);

      // Create the actual Grammar instance, and then load it
      // into the speech recognizer

      Grammar g = new Grammar(gb);
      _sre.LoadGrammar(g);

      // Attach our event handler for recognized commands
      // We won't worry about rejected or hypothesized callbacks

      _sre.SpeechHypothesized += OnSpeechHypothesized;
      _sre.SpeechRecognized += OnSpeechRecognized;
    }

    [MTAThread]
    private void StartSpeech()
    {
      if (_sre != null)
      {
        try
        {
          // Get the audio stream and pass it to the
          // speech recognition engine

          Stream kinectStream = _audio.Start();

          _sre.SetInputToAudioStream(
            kinectStream,
            new SpeechAudioFormatInfo(
              EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null
            )
          );
          _sre.RecognizeAsync(RecognizeMode.Multiple);
        }
        catch
        {
          Editor ed =
            Application.DocumentManager.MdiActiveDocument.Editor;
          ed.WriteMessage(
            "There was a problem initializing the KinectAudioSource." +
            " Ensure you have the Kinect SDK installed correctly."
          );
        }
      }
    }

    public void StopSensor()
    {
      if (_kinect != null)
      {
        if (_audio != null)
        {
          _sre.RecognizeAsyncStop();
          _audio.Stop();
          _audio = null;
        }
        
        _kinect.ElevationAngle = 0;

        _kinect.Stop();

        // Detach the event handlers

        _kinect.AllFramesReady +=
          new EventHandler<AllFramesReadyEventArgs>(
            OnAllFramesReady
          );

        _kinect.SkeletonFrameReady -=
          new EventHandler<SkeletonFrameReadyEventArgs>(
            OnSkeletonFrameReady
          );
        
        _kinect = null;
      }
    }

    protected virtual SamplerStatus SamplerData()
    {
      return SamplerStatus.Cancel;
    }

    protected virtual bool WorldDrawData(WorldDraw draw)
    {
      return false;
    }

    protected override SamplerStatus Sampler(JigPrompts prompts)
    {
      if (!String.IsNullOrEmpty(_word))
      {
        Document doc =
          Application.DocumentManager.MdiActiveDocument;
        if (doc != null)
        {
          doc.Editor.WriteMessage("\nWord recognised: {0}", _word);
          _word = "";
        }
      }

      // We don't really need a point, but we do need some
      // user input event to allow us to loop, processing
      // for the Kinect input

      PromptPointResult ppr =
        prompts.AcquirePoint("\nClick to capture: ");
      if (ppr.Status == PromptStatus.OK)
      {
        if (_finished)
        {
          CancelJig();
          return SamplerStatus.Cancel;
        }

        if (_tilt != 0)
        {
          int elev = _kinect.ElevationAngle + (_tilt * 5);
          if (
            elev <= _kinect.MaxElevationAngle &&
            elev >= _kinect.MinElevationAngle)
          {
            _kinect.ElevationAngle = elev;
            _tilt = 0;
          }
        }

        return SamplerData();
      }
      return SamplerStatus.Cancel;
    }

    protected override bool WorldDraw(WorldDraw draw)
    {
      return WorldDrawData(draw);
    }

    public void ForceMessage()
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
    }

    public List<ColoredPoint3d> GeneratePointCloud(
      int sampling, bool useColor = true
    )
    {      
      return GeneratePointCloud(
        _kinect, _depthPixels, _colorPixels, sampling, useColor
      );
    }

    // Generate a point cloud from depth and RGB data

    internal List<ColoredPoint3d> GeneratePointCloud(
      KinectSensor kinect, short[] depth, byte[] color,
      int sampling, bool withColor = false
    )
    {
      if (depth == null || color == null)
        return null;

      int depWidth = 640;

      // We will return a list of our ColoredPoint3d objects

      List<ColoredPoint3d> res = new List<ColoredPoint3d>();

      // Loop through the depth information - we process two
      // bytes at a time

      for (int i = 0; i < depth.Length; i += sampling)
      {
        // The x and y positions can be calculated using modulus
        // division from the array index

        int x = i % depWidth;
        int y = i / depWidth;

        SkeletonPoint p =
          kinect.MapDepthToSkeletonPoint(
            DepthImageFormat.Resolution640x480Fps30,
            x, y, depth[i]
          );

        // A zero value for Z means there is no usable depth for
        // that pixel

        if (p.Z > 0)
        {
          // Create a ColoredPoint3d to store our XYZ and RGB info
          // for a pixel

          ColoredPoint3d cv = new ColoredPoint3d();
          cv.X = p.X;
          cv.Y = p.Z;
          cv.Z = p.Y;

          // Only calculate the colour when it's needed (as it's
          // now more expensive, albeit more accurate)

          if (withColor)
          {
            // Get the colour indices for that particular depth
            // pixel

            ColorImagePoint cip =
              kinect.MapDepthToColorImagePoint(
                DepthImageFormat.Resolution640x480Fps30,
                x, y, depth[i],
                ColorImageFormat.RgbResolution640x480Fps30
              );

            // Extract the RGB data from the appropriate place
            // in the colour data

            int colIndex = 4 * (cip.X + (cip.Y * depWidth));
            if (colIndex <= color.GetUpperBound(0) - 2)
            {
                cv.B = (byte)(color[colIndex + 0]);
                cv.G = (byte)(color[colIndex + 1]);
                cv.R = (byte)(color[colIndex + 2]);
            }
          }
          else
          {
            // If we don't need colour information, just set each
            // pixel to white

            cv.B = 255;
            cv.G = 255;
            cv.R = 255;
          }

          // Add our pixel data to the list to return

          res.Add(cv);
        }
      }

      // Apply a bounding box filter, if one is defined

      if (_ext.HasValue)
      {
        // Use LINQ to get the points within the
        // bounding box

        var vecSet =
          from ColoredPoint3d vec in res
          where
            vec.X > _ext.Value.MinPoint.X &&
            vec.X < _ext.Value.MaxPoint.X &&
            vec.Y > _ext.Value.MinPoint.Y &&
            vec.Y < _ext.Value.MaxPoint.Y &&
            vec.Z > _ext.Value.MinPoint.Z &&
            vec.Z < _ext.Value.MaxPoint.Z
          select vec;

        // Convert our IEnumerable<> into a List<>

        res = vecSet.ToList<ColoredPoint3d>();
      }

      return res;
    }

    // Save the provided point cloud to a specific file

    internal static void ExportPointCloud(
      List<ColoredPoint3d> vecs, string filename
    )
    {
      if (vecs.Count > 0)
      {
        using (StreamWriter sw = new StreamWriter(filename))
        {
          // For each pixel, write a line to the text file:
          // X, Y, Z, R, G, B

          foreach (ColoredPoint3d pt in vecs)
          {
            sw.WriteLine(
              "{0}, {1}, {2}, {3}, {4}, {5}",
              pt.X, pt.Y, pt.Z, pt.R, pt.G, pt.B
            );
          }
        }
      }
    }

    // Translate from Skeleton Space to WCS

    internal static Point3d PointFromVector(
      SkeletonPoint p, bool flip = true
    )
    {
      // Rather than just return a point, we're effectively
      // transforming it to the drawing space: flipping the
      // Y and Z axes (which makes it consistent with the
      // point cloud, and makes sure Z is actually up - from
      // the Kinect's perspective Y is up), and reversing
      // the X axis (which is the result of choosing UseDepth
      // rather than UseDepthAndPlayerIndex)

      return new Point3d(flip ? -p.X : p.X, p.Z, p.Y);
    }

    // Cancel the running jig

    internal static void CancelJig()
    {
      acedPostCommand("CANCELCMD");
    }

    // Write the provided point cloud to file, then chain
    // the commands needed to import it into AutoCAD

    public void WriteAndImportPointCloud(
      Document doc, List<ColoredPoint3d> vecs
    )
    {
      Editor ed = doc.Editor;

      // We'll store most local files in the temp folder.
      // We get a temp filename, delete the file and
      // use the name for our folder

      string localPath = Path.GetTempFileName();
      File.Delete(localPath);
      Directory.CreateDirectory(localPath);
      localPath += "\\";

      // Paths for our temporary files

      string txtPath = localPath + "points.txt";
      string lasPath = localPath + "points.las";

      // Our PCG file will be stored under My Documents

      string outputPath =
        Environment.GetFolderPath(
          Environment.SpecialFolder.MyDocuments
        ) + "\\Kinect Point Clouds\\";

      if (!Directory.Exists(outputPath))
        Directory.CreateDirectory(outputPath);

      // We'll use the title as a base filename for the PCG,
      // but will use an incremented integer to get an unused
      // filename

      int cnt = 0;
      string pcgPath;
      do
      {
        pcgPath =
          outputPath + "Kinect" +
          (cnt == 0 ? "" : cnt.ToString()) + ".pcg";
        cnt++;
      }
      while (File.Exists(pcgPath));

      // The path to the txt2las tool will be the same as the
      // executing assembly (our DLL)

      string exePath =
        Path.GetDirectoryName(
          Assembly.GetExecutingAssembly().Location
        ) + "\\";

      if (!File.Exists(exePath + "txt2las.exe"))
      {
        ed.WriteMessage(
          "\nCould not find the txt2las tool: please make sure " +
          "it is in the same folder as the application DLL."
        );
        return;
      }

      // Export our point cloud from the jig

      ed.WriteMessage(
        "\nSaving TXT file of the captured points.\n"
      );

      ExportPointCloud(vecs, txtPath);

      // Use the txt2las utility to create a .LAS
      // file from our text file

      ed.WriteMessage(
        "\nCreating a LAS from the TXT file.\n"
      );

      ProcessStartInfo psi =
        new ProcessStartInfo(
          exePath + "txt2las",
          "-i \"" + txtPath +
          "\" -o \"" + lasPath +
          "\" -parse xyzRGB"
        );
      psi.CreateNoWindow = false;
      psi.WindowStyle = ProcessWindowStyle.Hidden;

      // Wait up to 20 seconds for the process to exit

      try
      {
        using (Process p = Process.Start(psi))
        {
          p.WaitForExit();
        }
      }
      catch
      { }

      // If there's a problem, we return

      if (!File.Exists(lasPath))
      {
        ed.WriteMessage(
          "\nError creating LAS file."
        );
        return;
      }

      File.Delete(txtPath);

      ed.WriteMessage(
        "Indexing the LAS and attaching the PCG.\n"
      );

      // Index the .LAS file, creating a .PCG

      string lasLisp = lasPath.Replace('\\', '/'),
              pcgLisp = pcgPath.Replace('\\', '/');

      doc.SendStringToExecute(
        "(command \"_.POINTCLOUDINDEX\" \"" +
          lasLisp + "\" \"" +
          pcgLisp + "\")(princ) ",
        false, false, false
      );

      // Attach the .PCG file

      doc.SendStringToExecute(
        "_.WAITFORFILE \"" +
        pcgLisp + "\" \"" +
        lasLisp + "\" " +
        "(command \"_.-POINTCLOUDATTACH\" \"" +
        pcgLisp +
        "\" \"0,0\" \"1\" \"0\")(princ) ",
        false, false, false
      );

      doc.SendStringToExecute(
        "_.-VISUALSTYLES _C _Realistic ",
        false, false, false
      );
    }
  }

  public class KinectCommands
  {
    // Set the clipping volume for the current point cloud

    [CommandMethod("ADNPLUGINS", "KINBOUNDS", CommandFlags.Modal)]
    public void SetBoundingBox()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      // Ask the user to select an entity

      PromptEntityOptions peo =
        new PromptEntityOptions(
          "\nSelect entity to define bounding box"
        );
      peo.AllowNone = true;
      peo.Keywords.Add("None");
      peo.Keywords.Default = "None";

      PromptEntityResult per = ed.GetEntity(peo);

      if (per.Status != PromptStatus.OK)
        return;

      // If "None" selected, clear the bounding box

      if (per.Status == PromptStatus.None ||
          per.StringResult == "None")
      {
        KinectJig.Extents = null;
        ed.WriteMessage("\nBounding box cleared.");
        return;
      }

      // Otherwise open the entity and gets its extents

      Transaction tr =
        doc.TransactionManager.StartTransaction();
      using (tr)
      {
        Entity ent =
          tr.GetObject(per.ObjectId, OpenMode.ForRead)
            as Entity;
        if (ent != null)
          KinectJig.Extents = ent.Bounds;

        ed.WriteMessage(
          "\nBounding box set to {0}", KinectJig.Extents
        );
        tr.Commit();
      }
    }

    // A command which waits for a particular PCG file to exist

    [CommandMethod(
      "ADNPLUGINS", "WAITFORFILE", CommandFlags.NoHistory
     )]
    public void WaitForFileToExist()
    {
      Document doc =
        Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;
      HostApplicationServices ha =
        HostApplicationServices.Current;

      PromptResult pr = ed.GetString("Enter path to PCG: ");
      if (pr.Status != PromptStatus.OK)
        return;
      string pcgPath = pr.StringResult.Replace('/', '\\');

      pr = ed.GetString("Enter path to LAS: ");
      if (pr.Status != PromptStatus.OK)
        return;
      string lasPath = pr.StringResult.Replace('/', '\\');

      ed.WriteMessage(
        "\nWaiting for PCG creation to complete...\n"
      );

      // Check the write time for the PCG file...
      // if it hasn't been written to for at least half a second,
      // then we try to use a file lock to see whether the file
      // is accessible or not

      const int ticks = 50;
      TimeSpan diff;
      bool cancelled = false;

      // First loop is to see when writing has stopped
      // (better than always throwing exceptions)

      while (true)
      {
        if (File.Exists(pcgPath))
        {
          DateTime dt = File.GetLastWriteTime(pcgPath);
          diff = DateTime.Now - dt;
          if (diff.Ticks > ticks)
            break;
        }
        System.Windows.Forms.Application.DoEvents();
        if (HostApplicationServices.Current.UserBreak())
        {
          cancelled = true;
          break;
        }
      }

      // Second loop will wait until file is finally accessible
      // (by calling a function that requests an exclusive lock)

      if (!cancelled)
      {
        int inacc = 0;
        while (true)
        {
          if (IsFileAccessible(pcgPath))
            break;
          else
            inacc++;
          System.Windows.Forms.Application.DoEvents();
          if (HostApplicationServices.Current.UserBreak())
          {
            cancelled = true;
            break;
          }
        }
        ed.WriteMessage("\nFile inaccessible {0} times.", inacc);

        try
        {
          CleanupTmpFiles(lasPath);
        }
        catch
        { }
      }
    }

    // Return whether a file is accessible

    internal bool IsFileAccessible(string filename)
    {
      // If the file can be opened for exclusive access it means
      // the file is accesible
      try
      {
        FileStream fs =
          File.Open(
            filename, FileMode.Open,
            FileAccess.Read, FileShare.None
          );
        using (fs)
        {
          return true;
        }
      }
      catch (IOException)
      {
        return false;
      }
    }

    // Remove any temporary files from the point cloud import

    internal void CleanupTmpFiles(string txtPath)
    {
      if (File.Exists(txtPath))
        File.Delete(txtPath);
      Directory.Delete(
        Path.GetDirectoryName(txtPath)
      );
    }
  }
}

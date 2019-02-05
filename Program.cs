using System;
using System.Security;
using System.Security.Cryptography;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading;
using System.Diagnostics;
using OSGeo.GDAL;
using OSGeo.OGR;
using OSGeo.OSR;
using Gdal = OSGeo.GDAL.Gdal;

namespace pixels2points
{
    class Program
    {
        static void Main(string[] args)
        {
            int row = 0;
            int col = 0;
            string inputdirectory = string.Empty;
            string outputfile = string.Empty;
            string argshelp = string.Empty;
            string maskshpfile = string.Empty;
            List<string> searchresults;
            List<string> masktilenames;
            List<string> maskresults;
            HandleFileInput fileops = new HandleFileInput();
            FindNoDataPixels findpix = new FindNoDataPixels();
            CreateConvexHullShp convexhull = new CreateConvexHullShp();
            GetSpatialReference getspatialref = new GetSpatialReference();
            GenerateMaskList maskinput = new GenerateMaskList();

            argshelp = String.Concat(argshelp, "Mandatory Arguments:\r\n");
            argshelp = String.Concat(argshelp, "-i\t\t\tinput directory (must be a valid, existing directory)\r\n");
            argshelp = String.Concat(argshelp, "-o\t\t\toutput file (.shp format)\r\n");
            argshelp = String.Concat(argshelp, "Optional Arguments:\r\n");
            argshelp = String.Concat(argshelp, "-m\t\t\tmask shapefile to limit which tiffs are processed");
            //make sure program has been provided the correct args
            if (args.Length < 4)
            {
                Console.WriteLine(argshelp);
                return;
            }
            if (!(args.Contains<String>("-i") && args.Contains<String>("-o")))
            {
                Console.WriteLine(argshelp);
                return;
            }
            for (var x = 0; x < args.Count(); x++)
            {
                switch (args[x].Trim())
                {
                    case "-i":
                        inputdirectory = args.Length == (x + 1) ? "" : args[++x];
                        break;
                    case "-o":
                        outputfile = args.Length == (x + 1) ? "" : args[++x];
                        break;
                    case "-m":
                        maskshpfile = args.Length == (x+1) ? "" : args[++x];
                        break;
                }
            }
            if (!String.IsNullOrEmpty(inputdirectory))
            {
                // get the file attributes for file or directory
                FileAttributes attr = new FileAttributes();
                try
                {
                    attr = File.GetAttributes(inputdirectory);
                }
                catch(FileNotFoundException)
                {
                    Console.WriteLine("    [-] Directory does not exist");
                    return;
                }
                if ((attr & FileAttributes.Directory) != FileAttributes.Directory)
                {
                    Console.WriteLine("    [-] Argument must be a directory");
                    return;
                }
            }
            else
            {
                Console.WriteLine(argshelp);
                return;
            }
            if (!String.IsNullOrEmpty(outputfile))
            {
                if (!(Path.GetExtension(outputfile) == ".shp"))
                {
                    Console.WriteLine("    [-] Argument must be a .shp file.");
                    return;
                }
                if (File.Exists(outputfile))
                {
                    Console.WriteLine("    [-] {0} already exists.", outputfile);
                    return;
                }
            }
            else
            {
                Console.WriteLine(argshelp);
                return;
            }
            if (!String.IsNullOrEmpty(maskshpfile))
            {
                if (!(Path.GetExtension(maskshpfile) == ".shp"))
                {
                    Console.WriteLine("    [-] Argument must be a .shp file.");
                    return;
                }
                if (File.Exists(outputfile))
                {
                    Console.WriteLine("    [-] {0} already exists.", outputfile);
                    return;
                }
            }

            //register gdal/ogr drivers
            GdalConfiguration.ConfigureGdal();
            GdalConfiguration.ConfigureOgr();

            //find all tif files in directory from args
            Console.WriteLine(">Searching directory for tiffs...        ");
            searchresults = fileops.GetTifFilePathsFromDirectory(inputdirectory);
            if (!searchresults.Any())
            {
                Console.WriteLine("    [-] Invalid directory or no tifs found. Exiting.");
                return;
            }
            else
            {
                Console.WriteLine("    [+] {0} Files found.", searchresults.Count);
            }
            if (!String.IsNullOrEmpty(maskshpfile))
            {
                Console.WriteLine(">Finding files matching shapefile features...");
                maskresults = new List<string>();
                masktilenames = maskinput.GetTileNameFromShapefile(maskshpfile);
                foreach (var filename in searchresults)
                {
                    foreach (var tilename in masktilenames)
                    {
                        if (tilename.Trim() == Path.GetFileNameWithoutExtension(filename))
                        {
                            maskresults.Add(filename);
                        }
                    }
                }
                searchresults = maskresults;
                if (!searchresults.Any())
                {
                    Console.WriteLine("    [-] Could not find any matches. Exiting.");
                    return;
                }
            }

            //find the coordinates of all pixels that are likely to be data voids
            Console.WriteLine(">Parsing tiffs for black pixels...    ");
            string outputdir = Path.GetDirectoryName(outputfile);
            string csvdirectory = fileops.CreateCSVDirectory(outputdir);
            if (String.IsNullOrEmpty(csvdirectory))
            {
                Console.WriteLine("    [-] Could not create output files. Exiting.");
                return;
            }
            row = Console.CursorTop - 1;
            col = Console.CursorLeft + 35;
            var spinner = new Spinner(col, row);
            spinner.Start();
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            int i = 0;
            foreach (var result in searchresults)
            {
                Console.SetCursorPosition((Console.CursorLeft), (Console.CursorTop));
                Console.Write("    [+] {0} out of {1} files processed", i, searchresults.Count);
                findpix.FindNoDataXYCoords(result, csvdirectory);
                i += 1;
            }
            spinner.Stop();
            stopwatch.Stop();
            TimeSpan ts = stopwatch.Elapsed;
            Console.WriteLine("    [+] Finished processing. Processing time: {0:hh\\:mm\\:ss}", ts);

            //see if any results were actually returned
            if (!ResultCoords.csvfiles.Any())
            {
                Console.WriteLine("    [-] No data voids found! Nothing left to do.");
                return;
            }

            //write output to shapefile
            //shp will contain multipoint features, one for each tiff
            Console.WriteLine(">Retrieving Projection Reference...");
            string spatialref = getspatialref.GetSpatialReferenceFromPath(searchresults.First());
            if (spatialref == null)
            {
                Console.WriteLine("    [-] Could not retrieve projection reference. Exiting.");
                return;
            }
            Console.WriteLine("    [+] Got projection reference.");
            Console.WriteLine(">Creating shapefile...");
            convexhull.CreateShapeFile(spatialref, outputfile);

            Console.WriteLine(">Cleaning Up...");
            try
            {
                Directory.Delete(csvdirectory, true);
            }
            catch (Exception)
            {
                Console.WriteLine("    [-] Could not delete temp data.");
            }
            Console.WriteLine("    [+] Done!");
        }
    }

    public class Spinner : IDisposable
    {
        private const string Sequence = @"/-\|";
        private int counter = 0;
        private readonly int left;
        private readonly int top;
        private readonly int delay;
        private bool active;
        private readonly Thread thread;

        public Spinner(int left, int top, int delay = 100)
        {
            this.left = left;
            this.top = top;
            this.delay = delay;
            thread = new Thread(Spin);
        }

        public void Start()
        {
            active = true;
            if (!thread.IsAlive)
                thread.Start();
        }

        public void Stop()
        {
            active = false;
            Draw(' ');
        }

        private void Spin()
        {
            while (active)
            {
                Turn();
                Thread.Sleep(delay);
            }
        }

        private void Draw(char c)
        {
            Console.SetCursorPosition(left, top);
            Console.Write("{0}{1}", c, Environment.NewLine);
        }

        private void Turn()
        {
            Draw(Sequence[++counter % Sequence.Length]);
        }

        public void Dispose()
        {
            Stop();
        }
    }

    public class HandleFileInput
    {
        private DirectoryInfo ValidateInputDir(string inputdir)
        {
            DirectoryInfo inputdirinfo = null;
            try
            {
                inputdirinfo = new DirectoryInfo(inputdir);
            }
            catch (SecurityException ex)
            {
                Console.WriteLine("    [-] Security Error. Please contact your system administrator for more information.");
                Console.WriteLine("    [-] Error message: {0}", ex.Message);
            }
            catch (ArgumentException)
            {
                Console.WriteLine("    [-] Path is incomplete or contains invalid characters. Please specify a valid path.");
            }
            catch (PathTooLongException)
            {
                Console.WriteLine("    [-] Path exceeds maximum length. Please specify a valid path.");
            }
            return inputdirinfo;
        }

        public string CreateCSVDirectory(string inputdir)
        {
            DirectoryInfo inputdirinfo = ValidateInputDir(inputdir);
            if (inputdirinfo == null)
            {
                Console.WriteLine("    [-] Invalid directory provided. Nothing left to do");
                return "";
            }
            using (RNGCryptoServiceProvider rng = new RNGCryptoServiceProvider())
            {
                byte[] randomnumber = new byte[8];
                rng.GetBytes(randomnumber);
                string randomstring = BitConverter.ToString(randomnumber);
                string directoryname = (Regex.Replace(randomstring, @"\-", "")).ToLower();
                string returnvalue = "";
                try
                {
                    DirectoryInfo subdir = inputdirinfo.CreateSubdirectory(directoryname);
                    returnvalue = subdir.ToString();
                }
                catch (Exception)
                {
                    Console.WriteLine("    [-] Could not create directory. Exiting.");
                }
                return returnvalue;
            }
        }

        public List<string> GetTifFilePathsFromDirectory(string inputdir)
        {
            DirectoryInfo inputdirinfo = ValidateInputDir(inputdir);
            List<string> tiffpaths = new List<string>();
            IEnumerable<System.IO.FileInfo> filelist;
            string pattern = @"tiff?";
            Regex rx = new Regex(pattern, RegexOptions.IgnoreCase);

            if (inputdirinfo == null)
            {
                Console.WriteLine("    [-] Invalid directory provided. Nothing left to do");
                return tiffpaths;
            }
            try
            {
                //LINQ query to get all tifs in directory
                filelist = inputdirinfo.EnumerateFiles();
                var querymatches = from f in filelist
                                   let matches = rx.Matches(f.Extension)
                                   where matches.Count > 0
                                   select f.FullName;
                foreach (var match in querymatches)
                {
                    tiffpaths.Add(match);
                }
                return tiffpaths;
            }
            catch (DirectoryNotFoundException)
            {
                Console.WriteLine("    [-] Directory not found. Please specify a valid path.");
            }
            catch (SecurityException ex)
            {
                Console.WriteLine("    [-] Security Error. Please contact your system administrator for more information.");
                Console.WriteLine("    [-] Error message: {0}", ex.Message);
            }
            return tiffpaths;
        }

        public FileStream CheckandCreateCSV(string filepath)
        {
            FileStream fs = null;
            string csvfields = "X,Y,Row,Column,Weight,Startposition,TileName\r\n";
            if (!File.Exists(filepath))
            {
                if (Path.GetExtension(filepath) != ".csv")
                {
                    Console.WriteLine("    [-] Must be a .csv file");
                    return fs;
                }
                else
                {
                    try
                    {
                        fs = File.Create(filepath);
                        byte[] datatowrite = new UTF8Encoding(true).GetBytes(csvfields);
                        fs.Write(datatowrite, 0, datatowrite.Length);
                        return fs;
                    }
                    catch (UnauthorizedAccessException)
                    {
                        Console.WriteLine("    [-] You do not have the required permission to create this file.");
                        fs.Dispose();
                    }
                    catch (ArgumentException)
                    {
                        Console.WriteLine("    [-] Path contains invalid characters. Please specify a valid path.");
                        fs.Dispose();
                    }
                    catch (PathTooLongException)
                    {
                        Console.WriteLine("    [-] Patch exceeds maximum length. Please specify a valid path.");
                        fs.Dispose();
                    }
                    catch (DirectoryNotFoundException)
                    {
                        Console.WriteLine("    [-] Could not find directory. Please specify a valid path.");
                        fs.Dispose();
                    }
                    catch (IOException)
                    {
                        Console.WriteLine("    [-] An I/O error occurred while creating the file.");
                        fs.Dispose();
                    }
                }
            }
            else
            {
                Console.WriteLine("File \"{0}\" already exists.", filepath);
                return fs;
            }
            return fs;
        }

        public static IEnumerable<string> ReadFromCSV(string filename)
        {
            string line;
            using (var reader = File.OpenText(filename))
            {
                while((line = reader.ReadLine()) != null)
                {
                    yield return line;
                }
            }
        }
    }

    static class ResultCoords
    {
        public static List<string> csvfiles = new List<string>();
    }
    
    public class GetSpatialReference
    {
        public string GetSpatialReferenceFromPath(string filepath)
        {
            string spatialref = null;
            Dataset ds = Gdal.Open(filepath, Access.GA_ReadOnly);
            if (ds == null)
            {
                Console.WriteLine("    [-] Could not create dataset from file.");
                return spatialref;
            }
            spatialref = ds.GetProjectionRef();
            ds.Dispose();
            return spatialref;
        }
    }

    public class FindNoDataPixels
    {
        private void ReduceXYList(List<string> xylist)
        {
            int pos = 0;
            for (int i = 0; i < xylist.Count; i += 2, pos++)
            {
                xylist[pos] = xylist[i];
            }
            xylist.RemoveRange(pos, xylist.Count - pos);
        }

        private int CheckExistingNodesDistance(List<string> PreviousRowXY, int runningsequence, double currx, double curry)
        {
            int sequencenumber = runningsequence;
            foreach(string point in PreviousRowXY)
            {
                double prevx = Convert.ToDouble(point.Split(',')[0]);
                double prevy = Convert.ToDouble(point.Split(',')[1]);
                int prevsequence = Convert.ToInt32(point.Split(',')[4]);
                //((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)) < d*d
                if (((currx - prevx) * (currx - prevx) + (curry - prevy) * (curry - prevy)) < 500 * 500)
                {
                    sequencenumber = prevsequence;
                    return sequencenumber;
                }
            }
            return sequencenumber;
        }

        private void WritePointData(string outputfile, List<string> points)
        {
            HandleFileInput fileops = new HandleFileInput();
            System.IO.FileStream csvfilestream;
            //crossing streams...
            //it's okay as long as caller remembers to wrap it in using() { }
            using (csvfilestream = fileops.CheckandCreateCSV(outputfile))
            {
                if (csvfilestream == null)
                {
                    Console.WriteLine("    [-] Could not create file for editing. Exiting");
                    return;
                }
                foreach (var point in points)
                {
                    byte[] datatowrite = new UTF8Encoding(true).GetBytes(point);
                    csvfilestream.Write(datatowrite, 0, datatowrite.Length);
                }
            }
            // in case the file stream is still open
            if (csvfilestream != null)
            {
                csvfilestream.Dispose();
            }
            return;
        }

        public void FindNoDataXYCoords(string filepath, string outputdir)
        {
            //returns coordinates of points below a certain value, but only if N adjacent
            //nodes also fall below that value. the goal is to only identify anomalous clusters
            //of near-0 points ('data voids')
            Dataset ds = Gdal.Open(filepath, Access.GA_ReadOnly);   //Read raster
            if (ds == null)
            {
                Console.WriteLine("    [-] Could not create raster dataset from file.");
                Environment.Exit(1);
            }
            if (ds.RasterCount < 3)
            {
                Console.WriteLine("    [-] Need at least a three band raster image.");
                Environment.Exit(1);
                ds.Dispose();
                Environment.Exit(1);
            }
            List<string> OutputCSV = new List<string>();
            List<string> PreviousRowXY = new List<string>();
            int runningsequence = 0;
            double[] gt = new double[6];
            ds.GetGeoTransform(gt); //Read geo transform info into array
            int Rows = ds.RasterYSize;
            int Cols = ds.RasterXSize;
            Band redband = ds.GetRasterBand(1);
            Band greenband = ds.GetRasterBand(2);
            Band blueband = ds.GetRasterBand(3);
            double startX = gt[0];  //Upper left lon
            double startY = gt[3];  //Upper left lat
            double interval = gt[1];    //Cell size
            double x, y;    //Current lon and lat
            string filename = Path.GetFileNameWithoutExtension(filepath);
            int adjacencycount = 0;
            int adjacencythreshold = 25;
            bool previousrow = false;
            bool isinvalid = false;
            int pointsoverflow = 0;

            for (int k = 0; k < Rows; k++)  //read one line
            {
                adjacencycount = 0;
                y = startY - k * interval;  //current lat
                int[][] buf = new int[3][];
                buf[0] = new int[Cols];
                buf[1] = new int[Cols];
                buf[2] = new int[Cols];
                int[] previouspixel = new int[3];
                //ReadRaster parameters are StartCol, StartRow, ColumnsToRead, RowsToRead, BufferToStoreInto, BufferColumns, BufferRows, 0, 0
                try
                {
                    redband.ReadRaster(0, k, Cols, 1, buf[0], Cols, 1, 0, 0);
                    greenband.ReadRaster(0, k, Cols, 1, buf[1], Cols, 1, 0, 0);
                    blueband.ReadRaster(0, k, Cols, 1, buf[2], Cols, 1, 0, 0);
                }
                catch
                {
                    if (isinvalid == false)
                    {
                        Console.WriteLine("Invalid compression result. Please rerun ImageChecker");
                        isinvalid = true;
                    }
                    continue;
                }
                List<List<double>> pixlists = new List<List<double>>();
                bool previous = false;
                bool hasblackpx = false;
                bool existingsequence = false;
                int firstinrow = 1;
                int sequencenumber = 0;
                int resultsindex = 0;
                //iterate each item in one line
                for (int r = 0; r < Cols; r++)
                {
                    bool isidentical = false;
                    bool isdarkpixel = false;

                    if (buf[0][r] == previouspixel[0] && buf[1][r] == previouspixel[1] && buf[2][r] == previouspixel[2])
                    {
                        isidentical = true;
                    }
                    if (buf[0][r] <= 15 && buf[1][r] <= 15 && buf[2][r] <= 15)
                    {
                        isdarkpixel = true;
                    }
                    //if we have reached this point then no additional coordinates will be found, so might as well avoid array accesses
                    if (r >= (Cols - adjacencythreshold) && adjacencycount < adjacencythreshold)
                    {
                        break;
                    }
                    if ((isdarkpixel || isidentical) && previous == true)
                    {
                        //only add pixels if they're directly adjacent
                        //this way, you avoid all the errant little shadows that aren't actual data voids
                        //needs reworking
                        x = startX + r * interval;  //current lon                             
                        List<double> potentialresult = new List<double>();
                        double ydistance = Convert.ToDouble(k);
                        double xdistance = Convert.ToDouble(r);
                        if (previousrow == true)
                        {
                            potentialresult.Add(x);
                            potentialresult.Add(y);
                            potentialresult.Add(ydistance);
                            potentialresult.Add(xdistance);
                            pixlists.Add(potentialresult);
                        }
                        ++adjacencycount;
                    }
                    else
                    {
                        existingsequence = false;
                        adjacencycount = 0;
                        previous = false;
                        if (isdarkpixel || isidentical)
                        {
                            previous = true;
                        }
                        pixlists.Clear();
                    }
                    if (adjacencycount == 0)
                    {
                        existingsequence = false;
                        pixlists.Clear();
                    }
                    if (adjacencycount == adjacencythreshold)
                    {
                        hasblackpx = true;
                        if (previousrow == true) //cuts down the # of false positives while preserving "actual" voids
                        {
                            List<double> firstresult = pixlists.First();
                            List<double> lastresult = pixlists.Last();
                            double firstx = firstresult[0];
                            double firsty = firstresult[1];
                            double firstrow = firstresult[2];
                            double firstcolumn = firstresult[3];
                            double lastx = lastresult[0];
                            double lasty = lastresult[1];
                            double lastrow = lastresult[2];
                            double lastcolumn = lastresult[3];
                            if (existingsequence == true) //save array space by only preserving first/last points in a contiguous sequence
                            {
                                //wish C# had something like pop_back()...
                                OutputCSV.RemoveAt(OutputCSV.Count - 1);
                            }
                            //if it has more probably a shoreline tile; calculating distance a lot of times can be too costly
                            while (PreviousRowXY.Count > 10000)
                            {
                                PreviousRowXY.RemoveAt(PreviousRowXY.Count - 2);
                                resultsindex -= 1;
                            }
                            //just in case, don't want to hit OOM
                            if (OutputCSV.Count > 500000)
                            {
                                Console.WriteLine("[-] Found a lot of valid BlackPx. Please consider using -m to mask out shoreline tiles");
                                string overflowfile = outputdir + "\\" + filename + "_" + pointsoverflow.ToString() + ".csv";
                                WritePointData(overflowfile, OutputCSV);
                                ResultCoords.csvfiles.Add(overflowfile);
                                pointsoverflow += 1;
                                OutputCSV.Clear();
                            }
                            if (runningsequence == Int32.MaxValue)
                            {
                                Console.WriteLine("[-] Found too many unique patterns. Please consider using -m to mask out shoreline tiles");
                                Console.WriteLine("    Reducing results size, may skew results");
                                ReduceXYList(OutputCSV);
                            }
                            if (existingsequence == false) //another optimization
                            {
                                runningsequence += 1;
                                sequencenumber = CheckExistingNodesDistance(PreviousRowXY, runningsequence, firstx, firsty);
                                string firstline = string.Format("{0},{1},{2},{3},{4},{5},{6}{7}", firstx, firsty, firstrow, firstcolumn, sequencenumber, firstinrow, filename, Environment.NewLine);
                                PreviousRowXY.Add(firstline);
                                OutputCSV.Add(firstline);
                                if (firstinrow == 0)
                                {
                                    resultsindex += 1;
                                }
                            }
                            if (existingsequence == false && firstinrow == 1)
                            {
                                firstinrow = 0;
                            }
                            sequencenumber = CheckExistingNodesDistance(PreviousRowXY, runningsequence, lastx, lasty);
                            string lastline = string.Format("{0},{1},{2},{3},{4},{5},{6}{7}", lastx, lasty, lastrow, lastcolumn, sequencenumber, firstinrow, filename, Environment.NewLine);
                            OutputCSV.Add(lastline);
                            PreviousRowXY.Add(lastline);
                            resultsindex += 1;
                        }
                        pixlists.Clear();
                        adjacencycount = 0;
                        existingsequence = true;
                    }
                    previouspixel[0] = buf[0][r];
                    previouspixel[1] = buf[1][r];
                    previouspixel[2] = buf[2][r];
                }
                if (hasblackpx == true)
                {
                    previousrow = true;
                }
                else
                {
                    previousrow = false;
                }
                PreviousRowXY.RemoveRange(0, (PreviousRowXY.Count - resultsindex));
            }
            if (OutputCSV.Any())
            {
                string outputfile = outputdir + "\\" + filename + ".csv";
                WritePointData(outputfile, OutputCSV);
                ResultCoords.csvfiles.Add(outputfile);
            }
            ds.Dispose();
        }
    }

    public class CreateConvexHullShp
    {
        private IEnumerable<List<string>> ReturnClusterCoords(string filepath)
        {
            HandleFileInput fileinput = new HandleFileInput(); 
            //more LINQ weirdness
            //group list elements into new lists by third comma-separated element in sublist, which represents the tile name
            //shouldn't be needed anymore now that csv's are output per-tile, will probably remove
            var groupedlist = from l in HandleFileInput.ReadFromCSV(filepath).Skip(1)
                              let x = l.Split(',')
                              group l by x[6] into g
                              select g.ToList();
            return groupedlist;
        }

        private void CreateFeature(Layer featurelayer, string layername, Geometry newgeom, int currentsequence)
        {
            //set "TileName" to last comma-separated element in sublist (the tile name...)
            Feature newfeature = new Feature(featurelayer.GetLayerDefn());
            newfeature.SetField("TileName", layername);
            newfeature.SetField("ClusterID", currentsequence);
            Geometry hullgeom = newgeom.ConvexHull();
            //ConvexHull() can return multiple types (ughh), left up to caller to handle this
            //easiest way to "convert" a linestring to polygon
            //Buffer(double distance, int quadsecs) where distance is represented in same
            //units as coordinate system (ie meters) and quadsecs = number of segments in
            //a 90 degree angle
            Geometry polygeom = hullgeom.Buffer(1.0, 30);
            newfeature.SetGeometry(polygeom);
            if (featurelayer.CreateFeature(newfeature) != Ogr.OGRERR_NONE)
            {
                Console.WriteLine("    [-] Failed to create feature in shapefile.");
                newfeature.Dispose();
                newgeom.Dispose();
                return;
            }
            newfeature.Dispose();
            newgeom.Dispose();
        }

        public void CreateShapeFile(string projref, string shapefilepath)
        {
            //creates a shapefile with a separate geometry feature for each tile in the results list
            string drivertype = "ESRI Shapefile";
            OSGeo.OGR.Driver shpdriver = Ogr.GetDriverByName(drivertype);
            if (shpdriver == null)
            {
                Console.WriteLine("    [-] Couldn't get shapefile driver.");
                return;
            }
            SpatialReference spatialref = new SpatialReference(projref);
            //a list of lists, each represent a separate tile name
            //create new datasource
            //DataSource shapefileds = shpdriver.CreateDataSource(Path.GetDirectoryName(shapefilepath), new string[] { });
            DataSource shapefileds = shpdriver.CreateDataSource(shapefilepath, new string[] { });
            if (shapefileds == null)
            {
                Console.WriteLine("    [-] Couldn't create shapefile datasource.");
                return;
            }
            //new field to add the tile name to
            FieldDefn newfield = new FieldDefn("TileName", FieldType.OFTString);
            FieldDefn clusterid = new FieldDefn("ClusterID", FieldType.OFTInteger);
            string fname = Path.GetFileName(shapefilepath);
            //create new layer to add features to
            Layer newlayer = shapefileds.CreateLayer(Path.ChangeExtension(fname, null), spatialref, wkbGeometryType.wkbPolygon, new string[] { });
            if (newlayer == null)
            {
                Console.WriteLine("    [-] Layer creation failed.");
                shapefileds.Dispose();
                newlayer.Dispose();
                return;
            }
            if (newlayer.CreateField(newfield, 1) != Ogr.OGRERR_NONE)
            {
                Console.WriteLine("    [-] Creating TileName field failed.");
                shapefileds.Dispose();
                newfield.Dispose();
                return;
            }
            if (newlayer.CreateField(clusterid, 1) != Ogr.OGRERR_NONE)
            {
                Console.WriteLine("    [-] Creating ClusterID field failed.");
                shapefileds.Dispose();
                newfield.Dispose();
                return;
            }
            foreach (string csvfile in ResultCoords.csvfiles)
            {
                IEnumerable<List<string>> coordsbytile = ReturnClusterCoords(csvfile);
                foreach (var tile in coordsbytile)
                {
                    //Sort list by sequence id
                    tile.Sort((x, y) => (Convert.ToInt32(x.Split(',')[4]).CompareTo(Convert.ToInt32(y.Split(',')[4]))));
                    //create new point geometry for every element in each list
                    Geometry clustergeom = new Geometry(wkbGeometryType.wkbMultiPoint);
                    clustergeom.AssignSpatialReference(spatialref);
                    string layername = (tile.First()).Split(',').Last();
                    int iter = 0;
                    int currentsequence = 0;
                    for (int i = 0; i < (tile.Count()); i++)
                    {
                        string point = tile[i];
                        string nextpoint;
                        if (!(i == tile.Count() - 1))
                        {
                            nextpoint = tile[i + 1];
                        }
                        else
                        {
                            nextpoint = point;
                        }
                        double x = Convert.ToDouble(point.Split(',')[0]);
                        double y = Convert.ToDouble(point.Split(',')[1]);
                        int currw = Convert.ToInt32(point.Split(',')[2]);
                        int nextrw = Convert.ToInt32(nextpoint.Split(',')[2]);
                        int currcol = Convert.ToInt32(point.Split(',')[3]);
                        int nextcol = Convert.ToInt32(nextpoint.Split(',')[3]);
                        currentsequence = Convert.ToInt32(point.Split(',')[4]);
                        int nextsequence = Convert.ToInt32(nextpoint.Split(',')[4]);
                        int ydistance = nextrw - currw;
                        int xdistance = nextcol - currcol;
                        bool samesequence = (currentsequence == nextsequence) ? true : false;
                        Geometry newpoint = new Geometry(wkbGeometryType.wkbPoint);
                        newpoint.SetPoint(0, x, y, 0);
                        clustergeom.AddGeometry(newpoint);
                        ++iter;
                        if (!samesequence && (ydistance > 300 | (xdistance > 300 | xdistance < -300)))
                        {
                            iter = 0;
                            CreateFeature(newlayer, layername, clustergeom, currentsequence);
                            clustergeom = null;
                            clustergeom = new Geometry(wkbGeometryType.wkbMultiPoint);
                        }
                    }
                    if (!clustergeom.IsEmpty() && iter > 0)
                    {
                        CreateFeature(newlayer, layername, clustergeom, currentsequence);
                    }
                }
            }
            shapefileds.Dispose();
            newfield.Dispose();
            newlayer.Dispose();
        }
    }

    public class GenerateMaskList
    {
        public List<string> GetTileNameFromShapefile(string shpfilepath)
        {
            string fieldname = "TileName";
            bool gotit = false;
            List<string> tilenames = new List<string>();

            DataSource ds1 = Ogr.Open(shpfilepath, 0);
            if (ds1 == null)
            {
                Console.WriteLine("    [-] Could not open {0}", shpfilepath);
                return tilenames;
            }
            Layer newlayer = ds1.GetLayerByIndex(0);
            if (newlayer == null)
            {
                Console.WriteLine("    [-] Could not fetch layer.");
                ds1.Dispose();
                return tilenames;
            }
            newlayer.ResetReading();
            FeatureDefn newfeaturedfn = newlayer.GetLayerDefn();
            for (int i = 0; i < newfeaturedfn.GetFieldCount(); i++)
            {
                //find TileName field in provided shapefile
                FieldDefn fielddefn = newfeaturedfn.GetFieldDefn(i);
                if (fielddefn.GetName() == fieldname)
                {
                    gotit = true;
                    break;
                }
            }
            if (gotit == false)
            {
                Console.WriteLine("    [-] Could not find tile name field. Please provide shapefile with the attribute field TileName");
                ds1.Dispose();
                return tilenames;
            }
            for (int i = 0; i < newlayer.GetFeatureCount(1); i++)
            {
                //return each string for TileName field in shapefile
                Feature newfeature = newlayer.GetFeature(i);
                string result = newfeature.GetFieldAsString(fieldname);
                tilenames.Add(result);
            }
            ds1.Dispose();
            return tilenames;
        }
    }
}

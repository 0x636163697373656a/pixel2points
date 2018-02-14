using System;
using System.Security;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
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
            List<string> searchresults;
            System.IO.FileStream csvfilestream;
            HandleFileInput fileops = new HandleFileInput();
            FindNoDataPixels findpix = new FindNoDataPixels();
            CreateConvexHullPolygons convexhull = new CreateConvexHullPolygons();
            GetSpatialReference getspatialref = new GetSpatialReference();
            bool IsParallel = false;

            argshelp = String.Concat(argshelp, "Mandatory Arguments (must come first):\r\n");
            argshelp = String.Concat(argshelp, "-i\t\t\tinput directory (must be a valid, existing directory)\r\n");
            argshelp = String.Concat(argshelp, "-o\t\t\toutput file (csv format)\r\n");
            if (args.Length < 4)
            {
                Console.WriteLine(argshelp);
                return;
            }
            if (!(args[0] == "-i" || args[0] == "-o"))
            {
                Console.WriteLine(argshelp);
                return;
            }
            for (var x = 0; x < args.Count(); x++)
            {
                switch (args[x].Trim())
                {
                    case "-i":
                        inputdirectory = args[++x];
                        break;
                    case "-o":
                        outputfile = args[++x];
                        break;
                    case "-para":
                        IsParallel = true;
                        break;
                }
            }
            if (!String.IsNullOrEmpty(outputfile))
            {
                if (!(Path.GetExtension(outputfile) == ".shp" | Path.GetExtension(outputfile) == ".csv"))
                {
                    Console.WriteLine("    [-] Argument must be a .shp or .csv file.");
                    return;
                }
                if (File.Exists(outputfile))
                {
                    Console.WriteLine("    [-] {0} already exists.", outputfile);
                    return;
                }
            }

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

            Console.WriteLine(">Registering GDAL Drivers...");
            GdalConfiguration.ConfigureGdal();

            Console.WriteLine(">Parsing tiffs for black pixels...    ");
            row = Console.CursorTop - 1;
            col = Console.CursorLeft + 35;
            var spinner = new Spinner(col, row);
            spinner.Start();
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            //let's see if it's worth the extra overhead
            if (IsParallel == true)
            {
                findpix.ParallelFindNoData(searchresults);
            }
            else
            {
                foreach (var result in searchresults)
                {
                    findpix.FindNoDataXYCoords(result, false);
                }
            }
            //for (int i = 0; i < searchresults.Count; i++)
            //{
                //Console.SetCursorPosition((Console.CursorLeft), (Console.CursorTop));
                //Console.WriteLine("    [+] {0} out of {1} files processed", i, searchresults.Count);
                //string filename = searchresults[i];
                //findpix.FindNoDataXYCoords(filename, i);
            //}
            spinner.Stop();
            stopwatch.Stop();
            TimeSpan ts = stopwatch.Elapsed;
            Console.WriteLine("    [+] Finished processing. Processing time: {0:hh\\:mm\\:ss}", ts);

            if (Path.GetExtension(outputfile) == ".csv")
            {
                Console.WriteLine(">Creating {0}...", outputfile);
                //crossing streams...
                csvfilestream = fileops.CheckandCreateCSV(outputfile);
                if (csvfilestream == null)
                {
                    Console.WriteLine("    [-] Could not create file for editing. Exiting");
                    return;
                }
                Console.WriteLine("    [+] Successfully created file");
                Console.WriteLine(">Writing results to {0}...", outputfile);
                if (IsParallel == true)
                {
                    foreach (var result in ResultCoords.concurrentresults)
                    {
                        byte[] datatowrite = new UTF8Encoding(true).GetBytes(result);
                        csvfilestream.Write(datatowrite, 0, datatowrite.Length);
                    }
                }
                else
                {
                    foreach (var result in ResultCoords.results)
                    {
                        byte[] datatowrite = new UTF8Encoding(true).GetBytes(result);
                        csvfilestream.Write(datatowrite, 0, datatowrite.Length);
                    }
                }
                Console.WriteLine("    [+] Done!");
                // in case the file stream is still open
                if (csvfilestream != null)
                {
                    csvfilestream.Dispose();
                }                
                //help out the garbage collector
                ResultCoords.results = null;
                ResultCoords.concurrentresults = null;
                GC.Collect();
                return;
            }

            if (Path.GetExtension(outputfile) == ".shp" )
            {
                Console.WriteLine(">Registering OGR Drivers...");
                GdalConfiguration.ConfigureOgr();
                Console.WriteLine(">Retrieving Projection Reference...");
                string spatialref = getspatialref.GetSpatialReferenceFromPath(searchresults.First());
                if (spatialref == null)
                {
                    Console.WriteLine("    [-] Could not retrieve projection reference. Exiting.");
                    return;
                }
                Console.WriteLine("    [+] Got projection reference.");
                List<string> coordslist = ResultCoords.concurrentresults.ToList();
                //help out the garbage collector
                ResultCoords.concurrentresults = null;
                GC.Collect();
                Console.WriteLine(">Creating shapefile...");
                if (IsParallel == true)
                {
                    convexhull.CreateShapeFile(spatialref, coordslist, outputfile);
                }
                else
                {
                    convexhull.CreateShapeFile(spatialref, ResultCoords.results, outputfile);
                }
                Console.WriteLine("    [+] Done!");
                coordslist = null;
                ResultCoords.results = null;
            }
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
                Console.WriteLine("    [-]Security Error. Please contact your system administrator for more information.");
                Console.WriteLine("    [-]Error message: {0}", ex.Message);
            }
            catch (ArgumentException)
            {
                Console.WriteLine("    [-]Path contains invalid characters. Please specify a valid path.");
            }
            catch (PathTooLongException)
            {
                Console.WriteLine("    [-]Path exceeds maximum length. Please specify a valid path.");
            }
            return inputdirinfo;
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
            string csvfields = "X,Y,TileName\r\n";
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
    }

    static class ResultCoords
    {
        public static List<string> results = new List<string>();
        public static ConcurrentBag<string> concurrentresults = new ConcurrentBag<string>();
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
            return spatialref;
        }
    }

    public class FindNoDataPixels
    {
        public void ParallelFindNoData(List<string> searchresults)
        {
            try
            {
                //trust the lambda. the lambda is your friend
                Parallel.ForEach(searchresults, 
                                (result) => 
                                {
                                    FindNoDataXYCoords(result, true);
                                });
            }
            catch (AggregateException e)
            {
                Console.WriteLine("Parallel foreach has thrown an exception: {0}", e);
            }
        }

        public void FindNoDataXYCoords(string filepath, bool para)
        {
            List<int> resolutionId = new List<int>() { 9 };

            Dataset ds = Gdal.Open(filepath, Access.GA_ReadOnly);   //Read raster
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

            for (int k = 0; k < Rows; k++)  //read one line
            {
                y = startY - k * interval;  //current lat
                int[] buf = new int[Cols];
                int[] buf2 = new int[Cols];
                int[] buf3 = new int[Cols];
                //ReadRaster parameters are StartCol, StartRow, ColumnsToRead, RowsToRead, BufferToStoreInto, BufferColumns, BufferRows, 0, 0
                redband.ReadRaster(0, k, Cols, 1, buf, Cols, 1, 0, 0);
                greenband.ReadRaster(0, k, Cols, 1, buf2, Cols, 1, 0, 0);
                blueband.ReadRaster(0, k, Cols, 1, buf3, Cols, 1, 0, 0);
                //iterate each item in one line
                for (int r = 0; r < Cols; r++)
                {
                    if (buf[r] < 10 && buf2[r] < 10 && buf3[r] < 10)
                    {
                        x = startX + r * interval;  //current lon                             
                        string filename = Path.GetFileNameWithoutExtension(filepath);
                        string csvline = string.Format("{0},{1},{2}{3}", x, y, filename, Environment.NewLine);
                        if (para == true)
                        {
                            ResultCoords.concurrentresults.Add(csvline);
                        }
                        else
                        {
                            ResultCoords.results.Add(csvline);
                        }
                    }
                }
            }
            ds.Dispose();
        }
    }

    public class CreateConvexHullPolygons
    {
        private IEnumerable<List<string>> ReturnClusterCoords(List<string> querylist)
        {
            //more LINQ weirdness
            var groupedlist = from l in querylist.Skip(1)
                              let x = l.Split(',')
                              group l by x[2] into g
                              select g.ToList();
            return groupedlist;
        }

        public void CreateShapeFile(string projref, List<string> xycoords, string shapefilepath)
        {
            string drivertype = "ESRI Shapefile";
            OSGeo.OGR.Driver shpdriver = Ogr.GetDriverByName(drivertype);
            if (shpdriver == null)
            {
                Console.WriteLine("    [-] Couldn't get shapefile driver.");
                return;
            }
            SpatialReference spatialref = new SpatialReference(projref);
            IEnumerable<List<string>> coordsbycluster = ReturnClusterCoords(xycoords);
            DataSource shapefileds = shpdriver.CreateDataSource(Path.GetDirectoryName(shapefilepath), new string[] { });
            if (shapefileds == null)
            {
                Console.WriteLine("    [-] Couldn't create shapefile datasource.");
            }
            FieldDefn newfield = new FieldDefn("TileName", FieldType.OFTString);
            string fname = Path.GetFileName(shapefilepath);
            Layer newlayer = shapefileds.CreateLayer(Path.ChangeExtension(fname, null), spatialref, wkbGeometryType.wkbMultiPoint, new string[] { });
            if (newlayer == null)
            {
                Console.WriteLine("    [-] Layer creation failed.");
                return;
            }
            if (newlayer.CreateField(newfield, 1) != Ogr.OGRERR_NONE)
            {
                Console.WriteLine("    [-] Creating TileName field failed.");
                return;
            }
            foreach (var cluster in coordsbycluster)
            {
                Feature newfeature = new Feature(newlayer.GetLayerDefn());
                Geometry clustergeom = new Geometry(wkbGeometryType.wkbMultiPoint);
                clustergeom.AssignSpatialReference(spatialref);
                foreach (var point in cluster)
                {
                    //wuhhh
                    Tuple<double, double> pointxy = new Tuple<double, double>(Convert.ToDouble(point.Split(',')[0]), Convert.ToDouble(point.Split(',')[1]));
                    double x = pointxy.Item1;
                    double y = pointxy.Item2;
                    Geometry newpoint = new Geometry(wkbGeometryType.wkbPoint);
                    newpoint.SetPoint(0, x, y, 0);
                    clustergeom.AddGeometry(newpoint);
                }
                string layername = (cluster.First()).Split(',').Last();
                newfeature.SetField("TileName", layername);
                newfeature.SetGeometry(clustergeom);
                if (newlayer.CreateFeature(newfeature) != Ogr.OGRERR_NONE)
                {
                    Console.WriteLine("    [-] Failed to create feature in shapefile.");
                }
                newfeature.Dispose();
            }
        }
    }
}

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
            string maskshpfile = string.Empty;
            List<string> searchresults;
            List<string> masktilenames;
            List<string> maskresults;
            System.IO.FileStream csvfilestream;
            HandleFileInput fileops = new HandleFileInput();
            FindNoDataPixels findpix = new FindNoDataPixels();
            CreateConvexHullShp convexhull = new CreateConvexHullShp();
            GetSpatialReference getspatialref = new GetSpatialReference();
            GenerateMaskList maskinput = new GenerateMaskList();
            bool IsParallel = false;

            argshelp = String.Concat(argshelp, "Mandatory Arguments (must come first):\r\n");
            argshelp = String.Concat(argshelp, "-i\t\t\tinput directory (must be a valid, existing directory)\r\n");
            argshelp = String.Concat(argshelp, "-o\t\t\toutput file (.csv or .shp format)\r\n");
            argshelp = String.Concat(argshelp, "Optional Arguments:\r\n");
            argshelp = String.Concat(argshelp, "-m\t\t\tmask shapefile to limit which tiffs are processed");
            //make sure program has been provided the correct args
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
                    case "-m":
                        maskshpfile = args[++x];
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
            if (!String.IsNullOrEmpty(maskshpfile))
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

            //register gdal/ogr drivers
            Console.WriteLine(">Registering GDAL Drivers...");
            GdalConfiguration.ConfigureGdal();
            Console.WriteLine(">Registering OGR Drivers...");
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
            row = Console.CursorTop - 1;
            col = Console.CursorLeft + 35;
            var spinner = new Spinner(col, row);
            spinner.Start();
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            //let's see if there's enough work being done to justify the added complexity
            //(this is just a request to the task scheduler, it doesn't guarantee parallelism)
            //most gdal objects are not thread-safe but with parallel foreach,
            //each *iteration* of the method will have exactly one thread, so should be no gdal objects shared between threads
            if (IsParallel == true)
            {
                findpix.ParallelFindNoData(searchresults);
            }
            else
            {
                int i = 0;
                foreach (var result in searchresults)
                {
                    Console.SetCursorPosition((Console.CursorLeft), (Console.CursorTop));
                    Console.Write("    [+] {0} out of {1} files processed", i, searchresults.Count);
                    findpix.FindNoDataXYCoords(result, false);
                    i += 1;
                }
            }
            spinner.Stop();
            stopwatch.Stop();
            TimeSpan ts = stopwatch.Elapsed;
            Console.WriteLine("    [+] Finished processing. Processing time: {0:hh\\:mm\\:ss}", ts);

            //see if any results were actually returned
            if (IsParallel == true)
            {
                if (ResultCoords.concurrentresults.IsEmpty == true)
                {
                    Console.WriteLine("    [-] No data voids found! Nothing left to do.");
                    return;
                }
            }
            else
            {
                if (!ResultCoords.results.Any())
                {
                    Console.WriteLine("    [-] No data voids found! Nothing left to do.");
                    return;
                }
            }

            //write output to csv if output file arg was a *.csv
            if (Path.GetExtension(outputfile) == ".csv")
            {
                Console.WriteLine(">Creating {0}...", outputfile);
                //crossing streams...
                //it's okay as long as caller remembers to wrap it in using() { }
                using (csvfilestream = fileops.CheckandCreateCSV(outputfile))
                {
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
                            ResultCoords.concurrentresults = null;
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
                }
                // in case the file stream is still open
                if (csvfilestream != null)
                {
                    csvfilestream.Dispose();
                }                
                return;
            }

            //write output to shapefile if output file arg was *.shp
            //shp will contain multipoint features, one for each tiff
            if (Path.GetExtension(outputfile) == ".shp" )
            {
                Console.WriteLine(">Retrieving Projection Reference...");
                string spatialref = getspatialref.GetSpatialReferenceFromPath(searchresults.First());
                if (spatialref == null)
                {
                    Console.WriteLine("    [-] Could not retrieve projection reference. Exiting.");
                    return;
                }
                Console.WriteLine("    [+] Got projection reference.");
                Console.WriteLine(">Creating shapefile...");
                if (IsParallel == true)
                {
                    List<string> coordslist = ResultCoords.concurrentresults.ToList();
                    ResultCoords.concurrentresults = null;
                    convexhull.CreateShapeFile(spatialref, coordslist, outputfile);
                }
                else
                {
                    convexhull.CreateShapeFile(spatialref, ResultCoords.results, outputfile);
                }
                Console.WriteLine("    [+] Done!");
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
            ds.Dispose();
            return spatialref;
        }
    }

    public class FindNoDataPixels
    {
        public void ParallelFindNoData(List<string> searchresults)
        {
            try
            {
                Parallel.ForEach(searchresults,
                                new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount >> 1},
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
            //returns coordinates of points below a certain value, but only if N adjacent
            //nodes also fall below that value. the goal is to only identify anomalous clusters
            //of near-0 points ('data voids')
            Dataset ds = Gdal.Open(filepath, Access.GA_ReadOnly);   //Read raster
            if (ds == null)
            {
                Console.WriteLine("    [-] Could not create raster dataset from file.");
                return;
            }
            if (ds.RasterCount < 3)
            {
                Console.WriteLine("    [-] Need at least a three band raster image.");
                ds.Dispose();
                return;
            }
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
            int adjacencythreshold = 20;

            for (int k = 0; k < Rows; k++)  //read one line
            {
                adjacencycount = 0;
                y = startY - k * interval;  //current lat
                int[][] buf = new int[3][];
                buf[0] = new int[Cols];
                buf[1] = new int[Cols];
                buf[2] = new int[Cols];
                //ReadRaster parameters are StartCol, StartRow, ColumnsToRead, RowsToRead, BufferToStoreInto, BufferColumns, BufferRows, 0, 0
                redband.ReadRaster(0, k, Cols, 1, buf[0], Cols, 1, 0, 0);
                greenband.ReadRaster(0, k, Cols, 1, buf[1], Cols, 1, 0, 0);
                blueband.ReadRaster(0, k, Cols, 1, buf[2], Cols, 1, 0, 0);
                List<List<double>> pixlists = new List<List<double>>();
                bool previous = false;
                //iterate each item in one line
                for (int r = 0; r < Cols; r ++)
                {
                    //if we have reached this point then no additional coordinates will be found, so might as well avoid array accesses
                    if (r >= (Cols - adjacencythreshold) && adjacencycount < adjacencythreshold)
                    {
                        break;
                    }
                    if (buf[0][r] <= 10 && buf[1][r] <= 10 && buf[2][r] <= 10 && previous == true)
                    {
                        //only add pixels if they're clustered together
                        //this way, you avoid all the errant little shadows that aren't actual data voids
                        //needs reworking
                        x = startX + r * interval;  //current lon                             
                        List<double> potentialresult = new List<double>();
                        potentialresult.Add(x);
                        potentialresult.Add(y);
                        pixlists.Add(potentialresult);
                        ++adjacencycount;
                    }
                    else
                    {
                        adjacencycount = 0;
                        previous = false;
                        if (buf[0][r] <= 10 && buf[1][r] <= 10 && buf[2][r] <= 10)
                        {
                            previous = true;
                        }
                    }
                    if (adjacencycount == adjacencythreshold)
                    {
                        for (int i = 0; i < adjacencythreshold; i++)
                        {
                            //the enumerator for List<T> will go through elements in the order in which they were created,
                            //so this will simply take the most recently added element and add it to results
                            int ndex = pixlists.Count() - 1;
                            List<double> actualresult = pixlists[ndex];
                            pixlists.Remove(pixlists[ndex]);
                            double thisx = actualresult[0];
                            double thisy = actualresult[1];
                            string csvline = string.Format("{0},{1},{2}{3}", thisx, thisy, filename, Environment.NewLine);
                            if (para == true)
                            {
                                ResultCoords.concurrentresults.Add(csvline);
                            }
                            else
                            {
                                ResultCoords.results.Add(csvline);
                            }
                        }
                        adjacencycount = 0;
                    }
                }
            }
            ds.Dispose();
        }
    }

    public class CreateConvexHullShp
    {
        private IEnumerable<List<string>> ReturnClusterCoords(List<string> querylist)
        {
            //more LINQ weirdness
            //group list elements into new lists by third comma-separated element in sublist, which represents the tile name
            var groupedlist = from l in querylist.Skip(1)
                              let x = l.Split(',')
                              group l by x[2] into g
                              select g.ToList();
            return groupedlist;
        }

        public void CreateShapeFile(string projref, List<string> xycoords, string shapefilepath)
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
            //lists everywhere (it's fine, it'll go out of scope and get GC'd soon...)
            IEnumerable<List<string>> coordsbycluster = ReturnClusterCoords(xycoords);
            //create new datasource
            DataSource shapefileds = shpdriver.CreateDataSource(Path.GetDirectoryName(shapefilepath), new string[] { });
            if (shapefileds == null)
            {
                Console.WriteLine("    [-] Couldn't create shapefile datasource.");
                return;
            }
            //new field to add the tile name to
            FieldDefn newfield = new FieldDefn("TileName", FieldType.OFTString);
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
            foreach (var cluster in coordsbycluster)
            {
                //create new point geometry for every element in each list
                Feature newfeature = new Feature(newlayer.GetLayerDefn());
                Geometry clustergeom = new Geometry(wkbGeometryType.wkbMultiPoint);
                clustergeom.AssignSpatialReference(spatialref);
                foreach (var point in cluster)
                {
                    double x = Convert.ToDouble(point.Split(',')[0]);
                    double y = Convert.ToDouble(point.Split(',')[1]);
                    Geometry newpoint = new Geometry(wkbGeometryType.wkbPoint);
                    newpoint.SetPoint(0, x, y, 0);
                    clustergeom.AddGeometry(newpoint);
                }
                string layername = (cluster.First()).Split(',').Last();
                //set "TileName" to last comma-separated element in sublist (the tile name...)
                newfeature.SetField("TileName", layername);
                Geometry hullgeom = clustergeom.ConvexHull();
                // ConvexHull() can return multiple types (ughh), left up to caller to handle this
                if (hullgeom.GetGeometryType() == wkbGeometryType.wkbLineString)
                {
                    //easiest way to "convert" a linestring to polygon
                    //Buffer(double distance, int quadsecs) where distance is represented in same
                    //units as coordinate system (ie meters) and quadsecs = number of segments in
                    //a 90 degree angle
                    Geometry polygeom = hullgeom.Buffer(5.0, 30);
                    newfeature.SetGeometry(polygeom);
                }
                else
                {
                    newfeature.SetGeometry(hullgeom);
                }
                if (newlayer.CreateFeature(newfeature) != Ogr.OGRERR_NONE)
                {
                    Console.WriteLine("    [-] Failed to create feature in shapefile.");
                    shapefileds.Dispose();
                    newfeature.Dispose();
                    clustergeom.Dispose();
                    hullgeom.Dispose();
                    return;
                }
                newfeature.Dispose();
                clustergeom.Dispose();
                hullgeom.Dispose();
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

The Essential GNSS Project

/data          The data folder:           Test data sets such as raw GPS data files, etc. 

The convention for storing data is:
/YYYYMMDD
or
/YYYYMMDD_YourComment

Why YYYYMMDD? Because it increases linearly with time.

e.g.'s
/20071120_UofC
/20071206

Within the data folder:
- Commit a folder containing the raw compressed files. 
- Do not commit the uncompressed RINEX obs files.
- Commit important processing options files.
- Commit RINEX navigation files (.YYn).
- Commit important comparative results.


Try to use the following convention with the data folder.
/YYYYMMDD/rawdata/ - compressed raw data files
/YYYYMMDD/         - all other files

e.g.
/20071120_UofC/rawdata/rawdata.zip   // All the raw data compressed into a zip file.
/20071120_UofC/README.TXT            // A readme file describing the test data, who collect it, when, where, etc.
/20071120_UofC/options.txt           // The post processing options.
/20071120_UofC/                      // Other important files and results/analysis.




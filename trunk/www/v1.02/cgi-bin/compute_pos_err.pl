#!/usr/bin/perl -w
use strict;
use warnings;
use CGI;

my $query = new CGI;
my $upload_dir = "/srv/www/htdocs/upload";
my $temp_dir = "/srv/www/htdocs/public_html/temp";

my $ref_lat = $query->param('ref_lat');
my $ref_lon = $query->param('ref_lon');
my $ref_hgt = $query->param('ref_hgt');
my $ref_lat_dms = $query->param('ref_lat_dms');
my $ref_lon_dms = $query->param('ref_lon_dms');
my $posfilename = $query->param('posfile');
my $timefield = $query->param('timefield');
my $latfield = $query->param('latfield');
my $lonfield = $query->param('lonfield');
my $hgtfield = $query->param('hgtfield');
my $ylimit = $query->param('ylimit');

$posfilename =~ s/.*[\/\\](.*)/$1/;
my $upload_filehandle = $query->upload("posfile");

open UPLOADFILE, ">$upload_dir/posdata.txt";
binmode UPLOADFILE;
while ( <$upload_filehandle> )
{
  print UPLOADFILE;
}
close UPLOADFILE;

if(0)
{
$ref_lat = 51.0916666667;
$ref_lon = -114.0000000000;
$ref_hgt = 1000;
$ref_lat_dms = " ";
$ref_lon_dms = " ";
$posfilename = "znav_lsq.txt";
$timefield = 0;
$latfield = 2;
$lonfield = 3;
$hgtfield = 4;
$ylimit = 5.0;
}



#my $outfilepath = ">$temp_dir/position_error.txt";
#my $bmpfilepath = ">$temp_dir/llherror.bmp";
#my $datafilepath = ">$upload_dir/posdata.txt";
my $outfilepath = "/srv/www/htdocs/public_html/temp/position_error.txt";
my $bmpfilepath = "/srv/www/htdocs/public_html/temp/llherror.bmp";
my $datafilepath = "/srv/www/htdocs/upload/posdata.txt";

my $stringcmd = sprintf("./computeposerr.bin %s %s %s %d %d %d %d 1.0 %.12f %.12f %.5f %.1f", $outfilepath, $bmpfilepath, $datafilepath, $timefield, $latfield, $lonfield, $hgtfield, $ref_lat, $ref_lon, $ref_hgt, $ylimit);

#my $stringcmd = sprintf("./computeposerr.bin /srv/www/htdocs/temp/position_error.txt /srv/www/htdocs/temp/llherror.bmp %s %d %d %d %d 1.0 %.12f %.12f %.5f %.0f", $posfilename, $timefield, $latfield, $lonfield, $hgtfield, $ref_lat, $ref_lon, $ref_hgt, $ylimit);

#print $stringcmd;
#printf "\n";
my $result = system $stringcmd;
#my $result = 0;

print "Content-type: text/html; charset=iso-8859-1\n\n";
#print "<html>\n";
#print "<body>\n";
#print $stringcmd;
#print "</body>\n";
#print "</html>\n\n";

 
if( $result != 0 )
{
  print "<html>\n";
  print "<body>\n";
  print "Unable to compute result. Check your data file. It must be a rectangular matrix. Check the field indices you selected. Check your reference coordinates.";
  print "<br>Position Data File: ";
  print $posfilename;
  print "\n<br>\n";
  print "time field index: ";
  print $timefield;
  print "\n<br>\n";
  print "latitude field index: ";
  print $latfield;
  print "\n<br>\n";
  print "longitude field index: ";
  print $lonfield;
  print "\n<br>\n";
  print "height field index: ";
  print $hgtfield;
  print "\n<br>\n<br>\n";  
  print "Reference latitude: ";
  print $ref_lat;
  print " degrees (";
  print $ref_lat_dms;
  print ")\n<br>\n";
  print "Reference longitude: ";
  print $ref_lon;
  print " degrees (";
  print $ref_lon_dms;
  print ")\n<br>\n";
  print "Reference height: ";
  print $ref_hgt;
  print " m\n<br>\n<br>\n";
  print "</body>\n";
  print "</html>\n\n";
}
else
{
  print "<html>\n";
  print "<head>\n";
  print "<title>Compute Position Error</title>\n";
  print "</head>\n";
  print "<body>\n";

  print "<br><h1>Computed Position Error</h1>\n";
  print "<br>Position Data File: ";
  print $posfilename;
  print "\n<br>\n";
  print "time field index: ";
  print $timefield;
  print "\n<br>\n";
  print "latitude field index: ";
  print $latfield;
  print "\n<br>\n";
  print "longitude field index: ";
  print $lonfield;
  print "\n<br>\n";
  print "height field index: ";
  print $hgtfield;
  print "\n<br>\n<br>\n";
  
  print "Reference latitude: ";
  print $ref_lat;
  print " degrees (";
  print $ref_lat_dms;
  print ")\n<br>\n";
  print "Reference longitude: ";
  print $ref_lon;
  print " degrees (";
  print $ref_lon_dms;
  print ")\n<br>\n";
  print "Reference height: ";
  print $ref_hgt;
  print " m\n<br>\n<br>\n";

  print "<a href=\"https://okeefesrv.geomatics.ucalgary.ca/temp/position_error.txt\">Download PositionError.txt</a><br>\n";
  print "File fields are: epoch(s) northing_error(m) easting_error(m) vertical_error(m) 2D_error(m) 3D_error(m)\n<br>\n<br>\n";

  print "<IMG src=\"https://okeefesrv.geomatics.ucalgary.ca/temp/llherror.bmp\" align=\"left\" border=\"0\">\n<br>\n<br>\n";
  print "<br>\n<br>\n";
  print "</body>\n";
  print "</html>\n\n";
}


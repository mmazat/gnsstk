#!/usr/bin/perl -w
use strict;
use warnings;
use CGI qw/escapeHTML param/;
use POSIX qw(ceil floor);

use constant PI => 4 * atan2(1, 1);
#print "Pi equals ", PI, "...\n";

my $lat1 = param('lat1');
my $lat1dms = param('lat1dms');
my $lon1 = param('lon1');
my $lon1dms = param('lon1dms');
my $hgt1 = param('hgt1');

my $lat2 = param('lat2');
my $lat2dms = param('lat2dms');
my $lon2 = param('lon2');
my $lon2dms = param('lon2dms');
my $hgt2 = param('hgt2');

my $x1 = param('x1');
my $y1 = param('y1');
my $z1 = param('z1');
my $x2 = param('x2');
my $y2 = param('y2');
my $z2 = param('z2');

# my $lat1 = 1;
# my $lat1dms = 2;
# my $lon1 = 3;
# my $lon1dms = 4;
# my $hgt1 = 5;
# 
# my $lat2 = 6;
# my $lat2dms = 7;
# my $lon2 = 8;
# my $lon2dms = 9;
# my $hgt2 = 10;
# 
# my $x1 = 11;
# my $y1 = 12;
# my $z1 = 13;
# my $x2 = 14;
# my $y2 = 15;
# my $z2 = 16;

my $az=0;
my $el=0;

my $stringcmd = sprintf("./geodesy.bin 0 4 /srv/www/htdocs/temp/azel.txt %.4f %.4f %.4f %.4f %.4f %.4f", $x1, $y1, $z1, $x2, $y2, $z2 );
#print $stringcmd;
#printf "\n";
system $stringcmd;

open(INFILE, "/srv/www/htdocs/temp/azel.txt") or die "Can't open data file: azel.txt $!";
my @datalines = <INFILE>;
close INFILE;

my @words = split /\s+/, $datalines[8];
$az = $words[2];
$az = $az * 180.0/PI;

@words = split /\s+/, $datalines[9];
$el = $words[2];
$el = $el * 180.0/PI;


print "Content-type: text/html; charset=iso-8859-1\n\n";

print <<_MULTILINE_A
<html>
<head><TITLE>Azimuth - Elevation</TITLE>
<script language="Javascript" src="https://okeefesrv.geomatics.ucalgary.ca/azel.js">
<!--
//-->
</script>
</head>
<body>
<h1>Compute Azimuth and Elevation</h1><br>
<hr>
<h2>Between from point 1 to point 2 (Earth Centered Earth Fixed, ECEF, WGS84)</h2><br>

<FORM name="azel" action="httpS://okeefesrv.geomatics.ucalgary.ca/cgi-bin/compute_azel.pl" method="POST">
<h3>Point 1</h3>
<table border="1" cellpadding="5" align="left">
  <tbody>
    <tr>
      <td>latitude (decimal degrees)</td>
      <td>longitude (decimal degrees)</td>
      <td>height (m)</td>
    </tr>
    <tr>
      <td><INPUT type="text" name="lat1" onkeypress="get_lat1_dms();" onkeyup="get_lat1_dms();" value="$lat1"></td>
      <td><INPUT type="text" name="lon1" onkeypress="get_lon1_dms();" onkeyup="get_lon1_dms();" value="$lon1"></td>
      <td><INPUT type="text" name="hgt1" value="$hgt1"></td>
    </tr>
    <tr>
      <td>latitude (dms)</td>
      <td>longitude (dms)</td>
      <td> </td>
    </tr>    
    <tr>
      <td><INPUT type="text" name="lat1dms" readonly value="$lat1dms"></td>
      <td><INPUT type="text" name="lon1dms" readonly value="$lon1dms"></td>
      <td> </td>
    </tr>
    <tr>
      <td> &nbsp </td>
      <td> <INPUT type="BUTTON" name="ComputeXYZ1" value="ComputeXYZ" onclick="llh2xyz_point1()"> </td>
      <td> &nbsp </td>
    </tr>
    <tr>
      <td> X (m) </td>
      <td> Y (m) </td>
      <td> Z (m) </td>
    </tr>
    <tr>
      <td> <INPUT type="text" name="x1" value="$x1"> </td>
      <td> <INPUT type="text" name="y1" value="$y1"> </td>
      <td> <INPUT type="text" name="z1" value="$z1"> </td>
    </tr>
  </tbody>
</table>

<br><br><br><br><br><br><br><br><br><br><br><br>

<h3>Point 2</h3>
<table border="1" cellpadding="5" align="left">
  <tbody>
    <tr>
      <td>latitude (decimal degrees)</td>
      <td>longitude (decimal degrees)</td>
      <td>height (m)</td>
    </tr>
    <tr>
      <td><INPUT type="text" name="lat2" onkeypress="get_lat2_dms();" onkeyup="get_lat2_dms();" value="$lat2"></td>
      <td><INPUT type="text" name="lon2" onkeypress="get_lon2_dms();" onkeyup="get_lon2_dms();" value="$lon2"></td>
      <td><INPUT type="text" name="hgt2" value="$hgt2"></td>
    </tr>
    <tr>
      <td>latitude (dms)</td>
      <td>longitude (dms)</td>
      <td> </td>
    </tr>    
    <tr>
      <td><INPUT type="text" name="lat2dms" readonly value="$lat2dms"></td>
      <td><INPUT type="text" name="lon2dms" readonly value="$lon2dms"></td>
      <td> </td>
    </tr>
    <tr>
      <td> &nbsp </td>
      <td> <INPUT type="BUTTON" name="ComputeXYZ2" value="ComputeXYZ" onclick="llh2xyz_point2()"> </td>
      <td> &nbsp </td>
    </tr>
    <tr>
      <td> X (m) </td>
      <td> Y (m) </td>
      <td> Z (m) </td>
    </tr>
    <tr>
      <td> <INPUT type="text" name="x2" value="$x2"> </td>
      <td> <INPUT type="text" name="y2" value="$y2"> </td>
      <td> <INPUT type="text" name="z2" value="$z2"> </td>
    </tr>
  </tbody>
</table>

<br><br><br><br><br><br><br><br><br><br><br><br><br><br>

<INPUT type="submit" name="Submit" value="Compute Results" onmouseover="llh2xyz_point1();llh2xyz_point2();"><br><br>

<hr>
The azimuth and elevation angles from <br>
point 1 at: <br>
latitude(decimal degrees) longitude(decimal degrees) height (m): <b><em> $lat1 &nbsp&nbsp&nbsp $lon1 &nbsp&nbsp&nbsp $hgt1 </b></em><br>
latitude(dmss) longitude(dms) height (m): <b><em> $lat1dms &nbsp&nbsp&nbsp $lon1dms &nbsp&nbsp&nbsp $hgt1 </b></em><br>
ECEF WGS84 X,Y,Z (m): <b><em> $x1 &nbsp&nbsp $y1 &nbsp&nbsp $z1 </b></em><br>
to <br>
point 2 at: <br>
latitude(decimal degrees) longitude(decimal degrees) height (m): <b><em> $lat2 &nbsp&nbsp&nbsp $lon2 &nbsp&nbsp&nbsp $hgt2 </b></em><br>
latitude(dmss) longitude(dms) height (m): <b><em> $lat2dms &nbsp&nbsp&nbsp $lon2dms &nbsp&nbsp&nbsp $hgt2 </b></em><br>
ECEF WGS84 X,Y,Z (m): <b><em> $x2 &nbsp&nbsp $y2 &nbsp&nbsp $z2 </b></em><br>
are: <br>
azimuth (degrees) = <b><em> $az </b></em><br>
elevation (degrees) = <b><em> $el </b></em><br>
<br><br>
<hr>
<br><br><br><br>

</FORM>
</body>
</html>
_MULTILINE_A
;
print "\n\n";

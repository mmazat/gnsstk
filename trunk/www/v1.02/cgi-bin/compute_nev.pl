#!/usr/bin/perl -w
use strict;
use warnings;
use CGI qw/escapeHTML param/;
use POSIX qw(ceil floor);

use constant PI => 4 * atan2(1, 1);
#print "Pi equals ", PI, "...\n";

my $ref_lat = param('ref_lat');
my $ref_lon = param('ref_lon');
my $ref_hgt = param('ref_hgt');
my $ref_lat_dms = param('ref_lat_dms');
my $ref_lon_dms = param('ref_lon_dms');

my $lat = param('user_lat');
my $lon = param('user_lon');
my $hgt = param('user_hgt');
my $lat_dms = param('user_lat_dms');
my $lon_dms = param('user_lon_dms');

#my $ref_lat = 51;
#my $ref_lon = -114;
#my $ref_hgt = 1000;
#my $ref_lat_dms = "N 51 00 00.000000";
#my $ref_lon_dms = "W 114 00 00.000000";
#my $lat = 51.1;
#my $lon = -114.1;
#my $hgt = 1100.0;
#my $lat_dms = "N 51 06 00.000000";
#my $lon_dms = "W 114 06 00.000000";

my $ref_lat_rad = $ref_lat * PI/180.0;
my $ref_lon_rad = $ref_lon * PI/180.0;

my $lat_rad = $lat * PI/180.0;
my $lon_rad = $lon * PI/180.0;

my $northing=0;
my $easting=0;
my $vertical=0;

my $stringcmd = sprintf("./geodesy.bin 0 3 /srv/www/htdocs/temp/nev.txt %.12f %.12f %.5f %.12f %.12f %.5f", $ref_lat_rad, $ref_lon_rad, $ref_hgt, $lat_rad, $lon_rad, $hgt );
#print $stringcmd;
#printf "\n";
system $stringcmd;

open(INFILE, "/srv/www/htdocs/temp/nev.txt") or die "Can't open data file: nev.txt $!";
my @datalines = <INFILE>;
close INFILE;

my @words = split /\s+/, $datalines[8];
$northing = $words[2];

@words = split /\s+/, $datalines[9];
$easting = $words[2];
  
@words = split /\s+/, $datalines[10];
$vertical = $words[2];

print "Content-type: text/html; charset=iso-8859-1\n\n";

print <<_MULTILINE_A
<html>
<head>
<title>Compute Local Geodetic Coordinates</title>
</head>
<body>

<script language = "JavaScript">
  function lat_dms(latitude)
  {    
    var lat = latitude;
    var sign = "N";
    if( lat < 0 )
    {
      lat = -lat;
      sign = "S";
    }
    var d = Math.floor(lat);
    var m = Math.floor((lat-d)*60.0);
    var s = (lat-d-m/60.0)*3600.0;
    var dmsv;
    if( d < 10 )
    {
      if( m < 10 )
      {
        dmsv = sign + " 0" + d.toString() + "\\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
      else
      {
        dmsv = sign + " 0" + d.toString() + "\\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
    }
    else
    {
      if( m < 10 )
      {
        dmsv = sign + " " + d.toString() + "\\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
      else
      {
        dmsv = sign + " " + d.toString() + "\\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
    }
    return dmsv;
    document.nev.ref_lat_dms.value = dmsv;
  }
  function get_ref_lat_dms()
  {
    var latitude = document.nev.ref_lat.value;
    document.nev.ref_lat_dms.value = lat_dms(latitude);
  }
  function get_user_lat_dms()
  {
    var latitude = document.nev.user_lat.value;
    document.nev.user_lat_dms.value = lat_dms(latitude);
  }
  function lon_dms(longitude)
  { 
    var lon = longitude;
    var sign = "E";
    if( lon < 0 )
    {
      lon = -lon;
      sign = "W";
    }
    var d = Math.floor(lon);
    var m = Math.floor((lon-d)*60.0);
    var s = (lon-d-m/60.0)*3600.0;
    var dmsv;
    if( d < 10 )
    {
      if( m < 10 )
      {
        dmsv = sign + " 0" + d.toString() + "\\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
      else
      {
        dmsv = sign + " 0" + d.toString() + "\\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
    }
    else
    {
      if( m < 10 )
      {
        dmsv = sign + " " + d.toString() + "\\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
      else
      {
        dmsv = sign + " " + d.toString() + "\\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\\"";
      }
    }
    return dmsv;
  }
  function get_ref_lon_dms()
  {
    var longitude = document.nev.ref_lon.value;
    document.nev.ref_lon_dms.value = lon_dms(longitude);
  }
  function get_user_lon_dms()
  {
    var longitude = document.nev.user_lon.value;
    document.nev.user_lon_dms.value = lon_dms(longitude);
  }
</script>


<h1>Compute Local Geodetic Coordinates from two Geodetic Coordinates </h1>

Computes Northing Easting and Vertical with respect to the first point (Assuming WGS84).
<br><br>
Reference Coordinates<br>
<FORM name="nev" action="https://okeefesrv.geomatics.ucalgary.ca/cgi-bin/compute_nev.pl" method="POST">
<TABLE width="800" border="1" cellpadding="5" align="left">
  <tbody>
  <tr>
    <TD><em>Reference latitude (&deg)</em></TD>
_MULTILINE_A
;
print "<TD><INPUT type=\"text\" name=\"ref_lat\" onkeypress=\"get_ref_lat_dms();\" onkeyup=\"get_ref_lat_dms();\" value=\"";
print $ref_lat;
print "\"></TD>\n";
print "<TD><INPUT type=\"text\" name=\"ref_lat_dms\" value=\"";
print $ref_lat_dms;
print "\"></TD>\n";
print "  </tr>\n";
print "  <tr>\n";
print "    <TD><em>Reference longitude (&deg)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"ref_lon\" onkeypress=\"get_ref_lon_dms();\" onkeyup=\"get_ref_lon_dms();\" value=\"";
print $ref_lon;
print "\"></TD>\n";
print "<TD><INPUT type=\"text\" name=\"ref_lon_dms\" value=\"";
print $ref_lon_dms;
print "\"></TD>\n";
print "</tr>\n";
print "  <tr>\n";
print "    <TD><em>Reference height (m)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"ref_hgt\" value=\"";
print $ref_hgt;
print "\"></TD>\n";
print <<_MULTILINE_B
    <TD>&nbsp</TD>
  </tr> 
  </tbody>
</TABLE>
<br><br><br><br><br><br>

User Coordinates
<TABLE width="800" border="1" cellpadding="5" align="left">
  <tbody>
  <tr>
    <TD><em>latitude (&deg)</em></TD>
_MULTILINE_B
;
print "<TD><INPUT type=\"text\" name=\"user_lat\" onkeypress=\"get_user_lat_dms();\" onkeyup=\"get_user_lat_dms();\" value=\"";
print $lat;
print "\"></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"user_lat_dms\" value=\"";
print $lat_dms;
print "\"></TD>\n";
print "  </tr>\n";
print "  <tr>\n";
print "    <TD><em>longitude (&deg)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"user_lon\" onkeypress=\"get_user_lon_dms();\" onkeyup=\"get_user_lon_dms();\" value=\"";
print $lon;
print "\"></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"user_lon_dms\" value=\"";
print $lon_dms;
print "\"></TD>\n";
print "  </tr>\n";
print "  <tr>\n";
print "    <TD><em>height (m)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"user_hgt\" value=\"";
print $hgt;
print "\"></TD>\n";
print <<_MULTILINE_C
<TD>&nbsp</TD>
  </tr> 
  </tbody>
</TABLE>
<br><br><br><br><br><br><br><br>

<INPUT type="submit" value="Compute" name="get_nev">
<br><br>

Northing, Easting, Vertical
<TABLE width="800" border="1" cellpadding="5" align="left">
  <tbody>
  <tr>
    <TD><em>Northing (m)</em></TD>
_MULTILINE_C
;
print "<TD><INPUT type=\"text\" name=\"northing\" value=\"";
print $northing;
print "\"></TD>\n";
print "  </tr>\n";
print "  <tr>    \n";
print "    <TD><em>Easting (m)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"easting\" value=\"";
print $easting;
print "\"></TD>    \n";
print "  </tr>\n";
print "  <tr>\n";
print "    <TD><em>Vertical (m)</em></TD>\n";
print "    <TD><INPUT type=\"text\" name=\"vertical\" value=\"";
print $vertical;
print "\"></TD>    \n";
print "  </tr> \n";
print "  </tbody>\n";
print "</TABLE>\n";
print "</FORM>\n";

print "<br><br><br><br><br><br>\n";
print "With reference point:<br>\n";
my $a_msg = sprintf("%.10f %.10f %.4f <br>\n", $ref_lat, $ref_lon, $ref_hgt );
print $a_msg;
print "and user point:<br>\n";
$a_msg = sprintf("%.10f %.10f %.4f <br>\n", $lat, $lon, $hgt );
print $a_msg;
print "the northing, easting and vertical offsets are: <br>\n";
$a_msg = sprintf("%.4f %.4f %.4f <br>\n", $northing, $easting, $vertical );
print $a_msg;
print "<br><br><br>";
#print $ref_lat_dms;
#print "<br>\n";
#print $ref_lon_dms;
#print "<br>\n";
#print $lat_dms;
#print "<br>\n";
#print $lon_dms;
#print "<br>\n";
print "</body>\n";
print "</html>\n\n";

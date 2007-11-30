#!/usr/bin/perl -w
use strict;
use warnings;
use CGI qw/escapeHTML param/;
use POSIX qw(ceil floor);

use constant PI => 4 * atan2(1, 1);
#print "Pi equals ", PI, "...\n";

my $ellipse_code = param('ellipse_code');
my $latitude = param('latitude');
my $longitude = param('longitude');
my $height = param('height');
my $userX  = param('userX');
my $userY  = param('userY');
my $userZ  = param('userZ');
my $llh2xyz = param('llh2xyz');

#my $ellipse_code = 0;
#my $latitude = 51;
#my $longitude = -114;
#my $height = 1000;
#my $userX = -1636163;
#my $userY = -3674882;
#my $userZ = 4934321;
#my $llh2xyz = "";

my $method = 0;
if( length($llh2xyz) < 1 )
{
  $method = 1;
}



my $latitude_rad = $latitude * PI/180.0;
my $longitude_rad = $longitude * PI/180.0;

my $stringcmd1 = sprintf("./geodesy.bin %d 0 /srv/www/htdocs/temp/ellipse.txt", $ellipse_code );
#print $stringcmd;
#printf "\n";
system $stringcmd1;

open(INFILE, "/srv/www/htdocs/temp/ellipse.txt") or die "Can't open data file: $!";
my @ellipsedatalines = <INFILE>;
close INFILE;

my @words = split /\s+/, $ellipsedatalines[2];
my $ellipse_a = $words[2];

@words = split /\s+/, $ellipsedatalines[3];
my $ellipse_b = $words[2];

@words = split /\s+/, $ellipsedatalines[4];
my $ellipse_f_inv = $words[2];

@words = split /\s+/, $ellipsedatalines[5];
my $ellipse_e2 = $words[2];

if( $method == 1 )
{
 my $stringcmd2 = sprintf("./geodesy.bin %d 2 /srv/www/htdocs/temp/coordinate_conversion.txt %.5f %.5f %.5f", $ellipse_code, $userX, $userY, $userZ );
  #print $stringcmd;
  #printf "\n";
  system $stringcmd2;
}
else
{
  my $stringcmd2 = sprintf("./geodesy.bin %d 1 /srv/www/htdocs/temp/coordinate_conversion.txt %.15f %.15f %.6f", $ellipse_code, $latitude_rad, $longitude_rad, $height );
  #print $stringcmd;
  #printf "\n";
  system $stringcmd2;
}

open(INFILE, "/srv/www/htdocs/temp/coordinate_conversion.txt") or die "Can't open data file: $!";
my @datalines = <INFILE>;
close INFILE;

if( $method == 1 )
{
  @words = split /\s+/, $datalines[2];
  $latitude_rad = $words[2];
  $latitude = $latitude_rad * 180.0/PI;

  @words = split /\s+/, $datalines[3];
  $longitude_rad = $words[2];
  $longitude = $longitude_rad * 180.0/PI;
  
  @words = split /\s+/, $datalines[4];
  $height = $words[2];
}
else
{
  @words = split /\s+/, $datalines[5];
  $userX = $words[2];

  @words = split /\s+/, $datalines[6];
  $userY = $words[2];
  
  @words = split /\s+/, $datalines[7];
  $userZ = $words[2];
}

my $direction;
my $lat = $latitude;
if( $lat < 0 )
{
  $lat = -$lat;
  $direction = "S";
}
else
{
  $direction = "N";
}
my $d = floor($lat);
my $m = floor($lat-$d)*60.0;
my $s = ($lat-$d-$m/60.0)*3600.0;
my $lat_dms = sprintf( "%s %2d&deg %02d' %09.6f&quot", $direction, $d, $m, $s );

my $lon = $longitude;
if( $lon < 0 )
{
  $lon = -$lon;
  $direction = "W";
}
else
{
  $direction = "E";
}
$d = floor($lon);
$m = floor($lon-$d)*60.0;
$s = ($lon-$d-$m/60.0)*3600.0;
my $lon_dms = sprintf( "%s %3d&deg %02d' %09.6f&quot", $direction, $d, $m, $s );

open(INFILE, "/srv/www/htdocs/temp/set_ellipse.js") or die "Can't open file: $!";
my @jslines = <INFILE>;
close INFILE;

print "Content-type: text/html; charset=iso-8859-1\n\n";
#print "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\">\n";
print "<html>\n";
print "<head>\n";
#print "<meta http-equiv=\"content-type\"\n";
#print "content=\"text/html; charset=ISO-8859-1\">\n";
print "<title>Coordinate Conversions</title>\n";
print "</head>\n";
print "<body>\n";

foreach(@jslines)
{
  print sprintf( "%s", $_ );
}

print "<h1>Compute Geodetic Coordinate Conversions</h1>\n";
print "Select Reference Ellipse<br>\n";
print "<form name=\"coordconv\">\n";
print "  <select id=\"ellipse_id\" name=\"ellipse_select\"\n";
print " onchange=\"set_ellipse();\">\n";

print "  <option";
if( $ellipse_code == 0 )
  {print " selected=\"yes\"";}
print ">World Geodetic System 1984</option>\n";

print "  <option";
if( $ellipse_code == 1 )
  {print " selected=\"yes\"";}
print ">Airy 1830</option>\n";

print "  <option";
if( $ellipse_code == 2 )
  {print " selected=\"yes\"";}
print ">Modified Airy</option>\n";

print "  <option";
if( $ellipse_code == 3 )
  {print " selected=\"yes\"";}
print ">Australian National</option>\n";

print "  <option";
if( $ellipse_code == 4 )
  {print " selected=\"yes\"";}
print ">Bessel 1841</option>\n";

print "  <option";
if( $ellipse_code == 5 )
  {print " selected=\"yes\"";}
print ">Clarke 1866</option>\n";

print "  <option";
if( $ellipse_code == 6 )
  {print " selected=\"yes\"";}
print ">Clarke 1880</option>\n";

print "  <option";
if( $ellipse_code == 7 )
  {print " selected=\"yes\"";}
print ">Everest(India 1830)</option>\n";

print "  <option";
if( $ellipse_code == 8 )
  {print " selected=\"yes\"";}
print ">Everest(Brunei and East Malaysia)</option>\n";

print "  <option";
if( $ellipse_code == 9 )
  {print " selected=\"yes\"";}
print ">Everest(West Malaysia and Singapore)</option>\n";

print "  <option";
if( $ellipse_code == 10 )
  {print " selected=\"yes\"";}
print ">Geodetic Reference System 1980</option>\n";

print "  <option";
if( $ellipse_code == 11 )
  {print " selected=\"yes\"";}
print ">Helmert 1906</option>\n";

print "  <option";
if( $ellipse_code == 12 )
  {print " selected=\"yes\"";}
print ">Hough 1960</option>\n";

print "  <option";
if( $ellipse_code == 13 )
  {print " selected=\"yes\"";}
print ">International 1924</option>\n";

print "  <option";
if( $ellipse_code == 14 )
  {print " selected=\"yes\"";}
print ">South American 1969</option>\n";

print "  <option";
if( $ellipse_code == 15 )
  {print " selected=\"yes\"";}
print ">World Geodetic System 1972</option>\n";

print "  </select>\n";
print "  <br><br>\n";
print "Ellipse Parameters <input value=\"";
print $ellipse_code;
print "\" name=\"ellipse_code\" type=\"hidden\">\n";
print "  <table style=\"width: 800px; height: 134px;\" border=\"1\" cellpadding=\"1\"\n";
print " cellspacing=\"2\">\n";
print "    <tbody>\n";
print "      <tr>\n";
print "        <td style=\"vertical-align: center;\"><em>a</em> <br>\n";
print "        </td>\n";
print "        <td>semi-major axis of the reference ellipse [m] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $ellipse_a;
print "\"";
print " name=\"ellipse_a\"></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>b</em> <br>\n";
print "        </td>\n";
print "        <td>semi-minor axis of the reference ellipse (b = a - a*f_inv)\n";
print "[m] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $ellipse_b;
print "\"";
print " name=\"ellipse_b\"></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>f_inv</em> <br>\n";
print "        </td>\n";
print "        <td>inverse of the flattening of the reference ellipse []<br>\n";
print "        </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $ellipse_f_inv;
print "\"";
print " name=\"ellipse_f_inv\"></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td style=\"vertical-align: center;\">e^2<br>\n";
print "        </td>\n";
print "        <td style=\"vertical-align: center;\">eccentricity of the reference\n";
print "ellipse (e^2 = (a*a-b*b)/(a*a)) []<br>\n";
print "        </td>\n";
print "        <td style=\"vertical-align: center;\"><input\n";
print " value=\"";
print $ellipse_e2;
print "\" name=\"ellipse_e2\"></td>\n";
print "      </tr>\n";
print "    </tbody>\n";
print "  </table>\n";
print "</form>\n";
print "<form name=\"coordconv_data\" ACTION=\"https://okeefesrv.geomatics.ucalgary.ca/cgi-bin/coordinate_conversion.pl\" METHOD=\"POST\">\n";
print "  <input value=\"";
print $ellipse_code;
print "\" name=\"ellipse_code\" type=\"hidden\"> <span\n";
print " style=\"font-style: italic;\"></span>\n";
print "Earth Centered Earth Fixed Coordinates<br>";
print "  <table style=\"width: 480px; height: 90px;\" border=\"1\" cellpadding=\"1\"\n";
print " cellspacing=\"2\">\n";
print "    <tbody>\n";
print "      <tr>\n";
print "        <td style=\"vertical-align: center;\"><em>X</em>&nbsp&nbsp&nbsp;</td>\n";
print "        <td>user X position [m] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $userX;
print "\" name=\"userX\"></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>Y</em>&nbsp&nbsp&nbsp;</td>\n";
print "        <td>user Y position [m] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $userY;
print "\" name=\"userY\"></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>Z</em>&nbsp&nbsp&nbsp;</td>\n";
print "        <td>user Z position [m] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $userZ;
print "\" name=\"userZ\"></td>\n";
print "      </tr>\n";
print "    </tbody>\n";
print "  </table>\n";
print "  <br>\n";
print "  <input value=\"Convert XYZ to LLH\" name=\"xyz2llh\" type=\"submit\"><br>\n";
print "  <br>\n";
print "  <input value=\"Convert LLH to XYZ\" name=\"llh2xyz\" type=\"submit\"><br>\n";
print "  <br>\n\n\n";
print "Geodetic Curvilinear Coordinates<br>";
print "<table style=\"width: 800px; height: 90px;\" border=\"1\" cellpadding=\"2\">\n";
print "    <tbody>\n";
print "      <tr>\n";
print "        <td style=\"vertical-align: center;\"><em>latitude</em></td>\n";
print "        <td>geodetic latitude [degrees] </td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $latitude;
print "\" name=\"latitude\"></td>\n";
print "        <td style=\"vertical-align: center;\"><em>latitude (dms)</em></td>\n";
print "        <td style=\"vertical-align: center;\"><em>";
print $lat_dms;
print "</em></td>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>longitude</em></td>\n";
print "        <td>geodetic longitude [degrees]</td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $longitude;
print "\" name=\"longitude\"></td>\n";
print "        <td style=\"vertical-align: center;\"><em>longitude (dms)</em></td>\n";
print "        <td style=\"vertical-align: center;\"><em>";
print $lon_dms;
print "</em> <br></td>\n";
print "      </tr>\n";
print "      <tr>\n";
print "        <td valign=\"top\"><em>height</em></td>\n";
print "        <td>geodetic height [m]</td>\n";
print "        <td style=\"vertical-align: center;\"><input value=\"";
print $height;
print "\" name=\"height\"></td>\n";
print "<td>&nbsp</td>\n";
print "<td>&nbsp</td>\n";
print "      </tr>\n";
print "    </tbody>\n";
print "  </table>\n";
print "  <br>\n";
print "  <br>\n";
print "  <br>\n";
print "  <span style=\"font-style: italic;\"></span></form>\n";
print "<br>";

my $out1 = sprintf( "%.4f %.4f %.4f", $userX, $userY, $userZ );
my $out2 = sprintf( "%.10f %.10f %.4f", $latitude, $longitude, $height );
 
if( $method == 1 )
{
  print $out1;
  print "<br>converted to:<br>";
  print $out2;
}
else
{
  print $out2;
  print "<br>converted to:<br>";
  print $out1;
}
print "<br><br>";
print "</body>\n";
print "</html>\n";
print "\n";


Example uses of the geodesy program, /bin/geodesy
___________________________________

>geodesy

USAGE
geodesy <ellipse> <function> <additional arguments for function ... >

ellipse:
0,   WGS84
1,   Airy 1830
2,   Modified Airy
3,   Australian National
4,   Bessel 1841
5,   Clarke 1866
6,   Clarke 1880
7,   Everest(India 1830)
8,   Everest(Brunei & E.Malaysia)
9,   Everest(W.Malaysia & Singapore)
10,  Geodetic Reference System 1980
11,  Helmert 1906
12,  Hough 1960
13,  International 1924
14,  South American 1969
15,  World Geodetic System 1972

function:
0, GEODESY_GetReferenceEllipseParameters
   requires no additions arguments
1, GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates
   latitude[deg], longitude[deg], height[m]
2, GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates
   userX[m], userY[m], userZ[m]
3, GEODESY_ComputeNorthingEastingVertical
   ref_lat[deg], ref_lon[deg] ref_hgt[m], lat[deg], lon[deg], hgt[m]
4  ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame
   fromX[m], fromY[m], fromZ[m], toX[m], toY[m], toZ[m]
5, GEODESY_ComputePositionDifference
   ref_lat[deg], ref_lon[deg] ref_hgt[m], lat[deg], lon[deg], hgt[m]


>geodesy 0 0

GEODESY_GetReferenceEllipseParameters
ellipse = World Geodetic System 1984
a       [m] =          6378137.000
b       [m] =    6356752.314245180
f_inv   []  =      298.25722356300
e2      []  = 0.006694379990141


>geodesy 0 1 51.0 -114.0 1000.0

GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates
ellipse   = World Geodetic System 1984
latitude  [deg] = 51.000000000000
longitude [deg] = -114.000000000000
height    [m]   = 1000.0000
x         [m]   = -1636163.2175
y         [m]   = -3674882.7546
z         [m]   = 4934321.7662


>geodesy 0 1 51.0 -114.0 10000.0

GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates
ellipse   = World Geodetic System 1984
latitude  [deg] = 51.000000000000
longitude [deg] = -114.000000000000
height    [m]   = 10000.0000
x         [m]   = -1638466.9265
y         [m]   = -3680056.9697
z         [m]   = 4941316.0798

>geodesy 0 4 -1636163.2175 -3674882.7546 4934321.7662 -1638466.9265 -3680056.9697 4941316.0798

GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame
ellipse         = World Geodetic System 1984
fromX     [m]   = -1636163.2175
fromY     [m]   = -3674882.7546
fromZ     [m]   = 4934321.7662
toX       [m]   = -1638466.9265
toY       [m]   = -3680056.9697
toZ       [m]   = 4941316.0798
azimuth   [deg] = 189.967850406907
elevation [deg] = 89.999999545927



>geodesy 0 3 51 -114 1000 51.1 -114 1000

GEODESY_ComputeNorthingEastingVertical
ellipse   = World Geodetic System 1984
reference_latitude  [deg] = 51.000000000000
reference_longitude [deg] = -114.000000000000
reference_height    [m]   = 1000.0000
latitude            [deg] = 51.100000000000
longitude           [deg] = -114.000000000000
height              [m]   = 1000.0000
northing            [m]   = 11126.6631
easting             [m]   = 0.0000
vertical            [m]   = -9.7099


>geodesy 0 5 51 -114 1000 51.1 -114 1000

GEODESY_ComputePositionDifference
ellipse   = World Geodetic System 1984
reference_latitude  [deg] = 51.000000000000
reference_longitude [deg] = -114.000000000000
reference_height    [m]   = 1000.0000
latitude            [deg] = 51.100000000000
longitude           [deg] = -114.000000000000
height              [m]   = 1000.0000
northing            [m]   = 11126.6631
easting             [m]   = 0.0000
vertical            [m]   = -9.7099



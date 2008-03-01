..\..\bin\plot2d.exe pvt.csv 1 12 1 13 1 14 -title "Solution Northing, Easting, Up from Reference" -ylabel "Value (m)" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -slabel 1 "North(m)" -slabel 2 "East(m)" -slabel 3 "Up(m)" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -out "Solution_NorthingEastingUp.bmp" -ylim -22 18 -sprecision 1 3 -sprecision 2 3 -sprecision 3 3

..\..\bin\plot2d.exe pvt.csv 13 12 -title "Plan View" -ylabel "Northing (m)" -xlabel "Easting (m)" -stats 0 -slabel 1 "data" -out "Solution_PlanView.bmp" -ylim -4 14 -xlim -18 -8

..\..\bin\plot2d.exe pvt.csv 1 15 1 16 1 17 -title "Estimated Solution Precision" -ylabel "Standard Deviation (m)"  -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -stats 0 -slabel 1 "lat stdev (m)" -slabel 2 "lon stdev (m)" -slabel 3 "hgt stdev (m)" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -out "Solution_Precision.bmp" -ylim 0 2

..\..\bin\plot2d.exe pvt.csv 1 10 -title "Clock Offset" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "clock offset (m)" -slabel 1 "clk offset" -out "Solution_ClockOffset.bmp"

..\..\bin\plot2d.exe pvt.csv 1 11 -title "Clock Drift" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "clock Drift (m/s)" -slabel 1 "clk drift" -out "Solution_ClockDrift.bmp"

..\..\bin\plot2d.exe pvt.csv 1 9 -title "Ground Speed" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "speed (km/hr)" -slabel 1 "speed" -out "Solution_GroundSpeed.bmp"

..\..\bin\plot2d.exe pvt.csv 1 6 1 7 1 8 -title "Velocity" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "Velocity (m/s)" -slabel 1 "V North" -slabel 2 "V East" -slabel 3 "V Up" -out "Solution_Velocity.bmp" -scolor 1 blue -scolor 2 limegreen -scolor 3 red

..\..\bin\plot2d.exe pvt.csv 1 23 1 24  -title "Number of Pseudorange" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "Number" -slabel 1 "Available" -slabel 2 "Used" -out "Solution_NrPsr.bmp" -scolor 1 blue -scolor 2 limegreen  -ylim -0.1 14 -ytick 0 1 14

..\..\bin\plot2d.exe pvt.csv 1 25 1 26 -title "Number of Doppler" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "Number" -slabel 1 "Available" -slabel 2 "Used" -out "Solution_NrDoppler.bmp" -scolor 1 blue -scolor 2 limegreen -ylim -0.1 14 -ytick 0 1 14

..\..\bin\plot2d.exe pvt.csv 1 27 1 28 -title "Number of Adr" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "Number" -slabel 1 "Available" -slabel 2 "Used" -out "Solution_NrAdr.bmp" -scolor 1 blue -scolor 2 limegreen -ylim -0.1 14 -ytick 0 1 14

..\..\bin\plot2d.exe pvt.csv 1 24 1 26 1 28 -title "Number Used in Solution" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "Number" -slabel 1 "Pseudorange" -slabel 2 "Doppler" -slabel 3 "ADR" -out "Solution_NrUsed.bmp" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -ylim -0.1 14 -ytick 0 1 14

..\..\bin\plot2d.exe pvt.csv 1 29 1 30 1 31 -title "Dilution of Precision" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "DOP" -slabel 1 "NDOP" -slabel 2 "EDOP" -slabel 3 "VDOP" -out "Solution_NDOP_EDOP_VDOP.bmp" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -ylim 0 5

..\..\bin\plot2d.exe pvt.csv 1 32 1 33 1 35 -title "Dilution of Precision" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "DOP" -slabel 1 "HDOP" -slabel 2 "PDOP" -slabel 3 "GDOP" -out "Solution_HDOP_PDOP_GDOP.bmp" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -ylim 0 5

..\..\bin\plot2d.exe pvt.csv 1 34 -title "Dilution of Precision" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "DOP" -slabel 1 "TDOP" -out "Solution_TDOP.bmp" -scolor 1 blue -ylim 0 5
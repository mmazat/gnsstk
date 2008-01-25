..\..\bin\computeposerr results.txt 0 2 3 4 1 51.079428117 -114.132818611 1116.585 1
del LatError.bmp
del LonError.bmp
del HgtError.bmp
del PositionError.mtx
del 2DError.bmp
del 3DError.bmp
del LatLonHgtError.bmp
..\..\bin\plot2d.exe PositionError.txt 1 2 1 3 1 4 -title "Solution Accuracy" -ylabel "Error (m)" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -stats 0 -slabel 1 "Error North(m)" -slabel 2 "Error East(m)" -slabel 3 "Error Up(m)" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -out "Solution_PositionError.bmp" -ylim -1 1

..\..\bin\plot2d.exe PositionError.txt 1 2 1 3 1 4 -title "Solution Accuracy" -ylabel "Error (m)" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -stats 1 -sprecision 1 3 -sprecision 2 3 -sprecision 3 3 -slabel 1 "North(m)" -slabel 2 "East(m)" -slabel 3 "Up(m)" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -out "Solution_PositionErrorWithStats.bmp" -ylim -1 1

..\..\bin\plot2d.exe results.txt 1 12 1 13 1 14 -title "Estimated Solution Precision" -ylabel "Standard Deviation (m)"  -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -stats 0 -slabel 1 "lat stdev (m)" -slabel 2 "lon stdev (m)" -slabel 3 "hgt stdev (m)" -scolor 1 blue -scolor 2 limegreen -scolor 3 red -out "Solution_Precision.bmp" -ylim 0 2

..\..\bin\plot2d.exe results.txt 1 6 -title "Clock Offset" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "clock offset (m)" -slabel 1 "clk offset" -out "Solution_ClockOffset.bmp"
..\..\bin\plot2d.exe results.txt 1 11 -title "Clock Drift" -gpslabel 1 14 -xlabel "GPS Time of Week (s) - UTC Time (hh:mm:ss)" -ylabel "clock Drift (m/s)" -slabel 1 "clk drift" -out "Solution_ClockDrift.bmp"


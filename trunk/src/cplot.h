/**
\file    cplot.h
\brief   C functions for 2D plotting directly to a compressed bitmap.

\author  Glenn D. MacGougan (GDM)
\date    2008-04-18
\since   2007-12-19

\b "LICENSE INFORMATION" \n
Copyright (c) 2007, refer to 'author' doxygen tags \n
All rights reserved. \n

Redistribution and use in source and binary forms, with or without
modification, are permitted provided the following conditions are met: \n

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer. \n
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution. \n
- The name(s) of the contributor(s) may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission. \n

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.
*/

#ifndef _CPLOT_H_
#define _CPLOT_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

/// \brief  These are the supported colors.
typedef enum 
{
  CPLOT_WHITE       = 0,
  CPLOT_BLACK       = 1,
  CPLOT_BLUE        = 2,
  CPLOT_GREEN       = 3,
  CPLOT_PURPLE      = 4,
  CPLOT_MAGENTA     = 5,
  CPLOT_DARKBLUE    = 6,
  CPLOT_INDIANRED   = 7,
  CPLOT_BABYBLUE    = 8,
  CPLOT_PAISLYBLUE  = 9,
  CPLOT_LIGHTPURPLE = 10,      
  CPLOT_DARKPURPLE  = 11,
  CPLOT_GREYPURPLE  = 12,
  CPLOT_BROWN       = 13,
  CPLOT_RED         = 14,
  CPLOT_PINK        = 15,
  CPLOT_YELLOW      = 16,
  CPLOT_ORANGE      = 17,
  CPLOT_CYAN        = 18,
  CPLOT_LIMEGREEN   = 19,
  CPLOT_GREY        = 20,
  CPLOT_LIGHTGREY   = 21
} CPLOT_enumColor;


/// \brief  A container struct used in CPLOT_structAxisOptions.
typedef struct
{
  BOOL doNotUseDefault;
  double val;
} CPLOT_structAxisSubOption;

/// \brief  A container struct for axis options.
typedef struct 
{
  char* label;
  CPLOT_structAxisSubOption lowerlimit;
  CPLOT_structAxisSubOption upperlimit;
  CPLOT_structAxisSubOption tickstart;
  CPLOT_structAxisSubOption ticksize;
  CPLOT_structAxisSubOption tickend;
  BOOL isGridOn;  

}CPLOT_structAxisOptions;


/// \brief  The user specified plot options.
typedef struct 
{
  int numberOfSeries;     //!< The total number of series to be plotted.  
  int PlotSize_Width_cm;  //!< The size of the plot portion of the figure [cm].
  int PlotSize_Height_cm; //!< The size of the plot portion of the figure [cm].
  char* title;            //!< The title string if any. NULL if none.
  
  /** \brief  The ylabel on the right hand side of the plot. NULL if none. Usually NULL.

    Data can be plotted with respect to y ticks on the right 
    hand side of the plot window. 
    For example, plot elevation angle (0-90, left side y ticks) 
    and plot azimuth angle (-180-180, right side y ticks). 

    \code
    rhs_ytick = lhs_ytick * scale_factor + offset
    e.g.
    Y_Label,              = "Elevation (deg)"
    Y_RightYLabel,        = "Azimuth (deg)"
    Y_RightYOffset,       = -180
    Y_RightYScaleFactor,  = 4
    \endcode
  */
  char*  y_label_right;            
  double y_label_right_scale_factor;  //!< Scale the lhs y ticks by this value for the rhs ticks.
  double y_label_right_bias;          //!< Offset the rhs y ticks by this value after scaling.
  CPLOT_enumColor RightYLabelColor;   //!< The color of the right y label.

  CPLOT_structAxisOptions x; //!< General axis options for X.
  CPLOT_structAxisOptions y; //!< General axis options for Y.

  /** 
  \brief  A boolean indicating if the special GPS x axis label is to be used.
          Enabling X_UseGPSLabel assumes that the X part of the 
          series is a GPS time of week, (0-604800 s), and will
          compute the corresponding UTC time and include it 
          below each xtick.
  */   
  BOOL useGPSLabel; 
  int UTCOffset;  //!< The GPS/UTC offset (always +ve).   

  BOOL plotStatistics; //!< A boolean indicating if statistics are to be included in the figure.
  
  BOOL plotLabelOnRight; //!< Place labels and statistics on the right of the plot. Otherwise below the plot.

  CPLOT_enumColor figureBackgroundColor; //!< The figure background color.

  double endOfWarmupEpoch;

  BOOL redrawAxes;
  
}CPLOT_structPlotOptions;







/// \brief  These are the dimensions of the plot part of the image in pixels.
typedef struct 
{
  int Width;  // pixels
  int Height; // pixels
} CPLOT_structImageSizeInPixels;

/// \brief  These are the coordinates in the image of parts of the axes.
typedef struct CPLOT_structAxes
{
  int StartX;           // pixels
  int StartY;           // pixels
  int FinishX;          // pixels 
  int FinishY;          // pixels
  int Width;            // pixels
  int Height;           // pixels
  int TickDashInPixels; // length of tick dash used
} CPLOT_structAxes;

/// \brief  A container for statistics information.
typedef struct
{
  double min;
  double max;
  double mean;
  double stdev;
  double rms;
  double range;
} CPLOT_structStats;

/// \brief  The container for series data and user options.
typedef struct 
{
  double *X; //!< A pointer to the x series array.
  double *Y; //!< A pointer to the y series array.
  int n;     //!< The number of items in X and Y arrays.

  CPLOT_structStats xStats; //!< Internally computed statistics.
  CPLOT_structStats yStats; //!< Internally computed statistics.

  BOOL connected;         //!< Are the data points connected.
  CPLOT_enumColor color;  //!< The color of the data points/line.
  char* label;            //!< The series label, NULL if none.
  char* units;            //!< The series units, NULL if none.
  int precision;          //!< The number of significant digits in the statistics.
  BOOL markOutlierData;   //!< Should the outlier data be marked.
  
}CPLOT_structSeries;


/// \brief  An RGB container.
typedef struct
{
  unsigned char Blue;
  unsigned char Green;
  unsigned char Red;
  unsigned char Reserved;
} CPLOT_structRGB;


/// \brief  Internal information container.
typedef struct
{
  double xtickstart;        // data units
  double xtickend;          // data units
  double xticksize;         // data units
  double ytickstart;        // data units
  double ytickend;          // data units
  double yticksize;         // data units
  double RangeX;            // MaxX - MinX // data units
  double RangeY;            // MaxY - MinY // data units
  double OnePercentRangeX;
  double OnePercentRangeY;
  double ScaleX;            // Scale X to pixels, mAxes.Width / RangeX
  double ScaleY;            // Scale Y to pixels, mAxes.Height / RangeY
  double MinX;              // data units
  double MaxX;              // data units
  double MinY;              // data units
  double MaxY;              // data units
} CPLOT_structInfoInDataUnits;

/// \brief  The 22 color default color table!
typedef struct 
{
  CPLOT_structRGB White; 
  CPLOT_structRGB Black;
  CPLOT_structRGB Blue;     
  CPLOT_structRGB Green;
  CPLOT_structRGB Purple;
  CPLOT_structRGB Magenta;
  CPLOT_structRGB DarkBlue;
  CPLOT_structRGB IndianRed;
  CPLOT_structRGB BabyBlue;
  CPLOT_structRGB PaislyBlue;
  CPLOT_structRGB LightPurple;      
  CPLOT_structRGB DarkPurple;
  CPLOT_structRGB GreyPurple;
  CPLOT_structRGB Brown;
  CPLOT_structRGB Red;
  CPLOT_structRGB Pink;            
  CPLOT_structRGB Yellow;
  CPLOT_structRGB Orange;
  CPLOT_structRGB Cyan;
  CPLOT_structRGB LimeGreen;
  CPLOT_structRGB Grey;
  CPLOT_structRGB LightGrey;            

}CPLOT_structColorTable;   


/// \brief  A data container for holding the bitmap bytes.
typedef struct
{  
  unsigned      nrows;   //!< The number of rows in the matrix.
  unsigned      ncols;   //!< The number of columns in the matrix.
  unsigned char **data;  //!< This is a pointer to an array of bytes row vectors.

} CPLOT_structByteMatrix;


/// \brief  The CPLOT C sytle object.
typedef struct
{
  BOOL mIsAxesDrawn;          //!< A boolean indicating if the axes are drawn.
  int mSeriesIndex;           //!< An index for the series.
  int mFootNoteIndex;         //!< An index for footnotes.
    
  int mLabelWidth;            //!< The width for a label [pixels].
  int mStatsValueHeight;      //!< The height allocation for the statisitics values.
  int mYLabelAllowance;       //!< The allowance for the ylabel [pixels].
  int mRightYLabelAllowance;  //!< The allowance for the right ylabel [pixels].
  int mTitleAllowance;        //!< The allowance for the title [pixels].
  int mXLabelAllowance;       //!< The allowance for the xlabel [pixels].
  
  CPLOT_structImageSizeInPixels mImage; //!< The image size [pixels].

  CPLOT_structAxes mAxes;

  CPLOT_structInfoInDataUnits mData;

  CPLOT_structPlotOptions  mOptions;    //!< The plotting options
  

  CPLOT_structColorTable mDefaultColorTable;
  BOOL mUseDefaultColorTable;
  unsigned char* mColorTable;
  
  CPLOT_structByteMatrix  mPlotData;

} CPLOT;


/// \brief  Initialize the general plotting options.
/// \return TRUE(1) if successful, FALSE(0) otherwise.
BOOL CPLOT_PlotOptionsInit( CPLOT_structPlotOptions *Opt );


/// \brief  Initialize the main CPLOT data container.
/// \return TRUE(1) if successful, FALSE(0) otherwise.
BOOL CPLOT_Init( CPLOT* P );

/// \brief  Set all the general plotting options.
/// \return TRUE(1) if successful, FALSE(0) otherwise.
BOOL CPLOT_SetPlotOptions( 
  CPLOT *P, 
  CPLOT_structPlotOptions *opt
  );

/// \brief  Plot a data series. Call this function repeatedly 
///         for each data series.
/// \return TRUE(1) if successful, FALSE(0) otherwise.
BOOL CPLOT_Plot( 
  CPLOT *P, 
  CPLOT_structSeries *Series 
  );

/// \brief  Once plotting is complete, save the bitmap to the
///         file specified.
/// \return TRUE(1) if successful, FALSE(0) otherwise.
BOOL CPLOT_SaveToFile( 
  CPLOT *P,
  const char *FileName 
  );


BOOL CPLOT_IsNAN( double value );
BOOL CPLOT_IsPostiveINF( double value );
BOOL CPLOT_IsNegativeINF( double value );

#ifdef __cplusplus
}
#endif


#endif // _CPLOT_H_






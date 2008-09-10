/*!
\file cmatrix.c
\brief 'c' functions for vector and matrix operations
*/
#include <stdio.h>  // for FILE*
#include <stdlib.h> // for calloc, malloc, free
#include <string.h> // for strlen, sprintf, strstr, strcmp, and others
#include <ctype.h>  // for isalpha
#include <math.h>
#include <float.h>
#include <errno.h>

#include "cmatrix.h"

#ifndef _MATRIX_NO_PLOTTING
#include "cplot.h"   // for CPLOT - plotting capabilities directly to an image file.
#endif

// deal with msvc empty projects
#ifndef WIN32
  #ifdef _WIN32
    #define WIN32
  #endif
#endif

#if defined _MSC_VER && _MSC_VER < 1400
#define _CRT_SECURE_NO_DEPRECATE
#endif

#ifndef _MSC_VER
#define _CRT_SECURE_NO_DEPRECATE
#endif

#include "kiss_fft.h" // Use kiss FFT, when INTEL_IPPS is disabled

//#define MTX_DEBUG
#ifdef MTX_DEBUG
#include <time.h>
#endif


#ifndef PI
#define PI (3.1415926535897932384626433832795) //!< better value
#endif

#ifndef TWOPI
#define TWOPI (6.283185307179586476925286766559) //!< 2.0*PI
#endif

#ifndef HALFPI
#define HALFPI (1.5707963267948966192313216916398) //!< PI/2.0
#endif


#define MTX_MAX_READFROMFILE_BUFFER (65536)
#define MATRIX_MIN_RLE_TOLERANCE (1.0e-16)
#define MTX_MAX_COMMENT_LENGTH (1024*6)

// binary version identifiers
#define MTX_ID_SIZE (8)
#define MTX_ID_COMPRESSED_01 ("MTX01\n") //!< identifier used to indicate a file stored using SaveCompressed (version 1)
#define MTX_ID_LEGACY_V01 ("Matrix\n") //!< legacy identifier used to indicate a file stored using Save with imprecise double RLE
#define MTX_ID_LEGACY_V02 ("MTXV02\n") //!< legacy identifier used to indicate a file stored using Save with imprecise double RLE

#define MTX_VERSION_NR_DEFAULT (101) //!< identifier used to indicate a file stored using basic Save
#define MTX_VERSION_NR_COMPRESSED_01 (102) //!< identifier used to indicate a file stored using SaveCompressed (version 1)
#define MTX_VERSION_NR_LEGACY_V01 (1) //!< legacy identifier used to indicate a file stored using Save with imprecise double RLE
#define MTX_VERSION_NR_LEGACY_V02 (2) //!< legacy identifier used to indicate a file stored using Save with imprecise double RLE

#define MTX_NK (8) //!< the number of byte columns used to represent a column of doubles when using MTX_ID_COMPRESSED_01

#define MTX_NAN     (sqrt(-1.0))
#define MTX_POS_INF (-log(0.0))
#define MTX_NEG_INF ( log(0.0))


/// \brief This static global variable indicates whether matrix
/// operations for single elements are treated as scalar
/// operations.
/// e.g. A = B*C, B is 1x1 C is 10x10. If this was disabled, this
/// operation would return FALSE as an error. When enabled, A, is
/// treated as a scalar and is multiplied into every element of C.
BOOL MTX_static_global_treat_1x1_as_scalar = TRUE;


typedef struct
{
  char id[8];
  unsigned headersize;
  unsigned isReal;
  unsigned nrows;
  unsigned ncols;
  unsigned filesize;
  unsigned crc;
  char *comment;
}_MTX_STRUCT_FileHeader;

typedef struct
{
  unsigned length[MTX_NK]; // length of compressed data column
  unsigned char isCompressed[MTX_NK]; // indicates which columns are compressed using RLE
  unsigned totalLength; // total length of MTX_NK data columns
}_MTX_STRUCT_CompressedColumnHeader;


/// struct specific for MTX_ReadFromFile and related functions (a simple linked list)
typedef struct _MTX_listItemCplx
{
  BOOL isReal; //!< A boolean to indicate if real data or complex data is attached to this item.
  double *rowptr; //!< The pointer to real data.
  stComplex *rowptr_cplx; //!< The pointer to complex data.
  struct _MTX_listItemCplx *next; //!< The pointer to the next item in the list.
}_MTX_STRUCT_ReadFromFileListElem;


/// static function for matrix memory allocation
static BOOL MTX_static_alloc( MTX *M, const unsigned nrows, const unsigned ncols, const BOOL setToZero, const BOOL isReal );

/// static function for converting a complex stored matrix to a real matrix (either all real component or all imaginary component)
static BOOL MTX_static_ConvertComplexTo( MTX *M, BOOL useReal );

static BOOL MTX_static_get_row_array_from_string( char *datastr, _MTX_STRUCT_ReadFromFileListElem *L, const unsigned ncols );

/// This function gets the next valid line of data. Whitespace lines are skipped.
static BOOL MTX_static_get_next_valid_data_line(
  FILE *in, //!< The input file pointer (input).
  char *linebuf, //!< A exisiting buffer to store the input line (input/output).
  unsigned *line_length, //!< The length of the line read (output).
  BOOL *atEOF //!< A boolean to indicate if EOF has been reached.
  );

/// This function gets the next valid line of data from a matrix string. Whitespace lines are skipped.
static BOOL MTX_static_get_next_valid_data_line_from_matrix_string(
  const char *strMatrix, //!< The matrix string pointer (input).
  const unsigned strLength, //!< The length of the matrix string (input).
  unsigned *index, //!< The starting/(next line) index into the matrix string pointer (input/output).
  char *linebuf, //!< A exisiting buffer to store the input line (input/output).
  unsigned *line_length, //!< The length of the line read (output).
  BOOL *atEndOfString //!< A boolean to indicate if the end of the strMatrix string has been reached.
  );


/// Extract a complex value from a string with a leading digit.
/// The string is either bi or a+bi.
static BOOL MTX_static_extract_cplx_from_string_with_leading_digit(
  char *datastr, //!< The entire input data string.
  const unsigned indexS, //!< The start index of the complex element.
  const unsigned indexE, //!< The inclusive end index of the complex element.
  double *re, //!< The extracted real component.
  double *im //!< The extracted imag component.
  );


static BOOL MTX_static_extract_real_into_cplx_from_string(
  char *datastr, //!< The entire input data string.
  const unsigned indexS, //!< The start index of the complex element.
  double *re, //!< The extracted real component.
  double *im //!< The extracted imag component.
  );

/// This static function looks for complex data in a line string.
static BOOL MTX_static_look_for_complex_data(
  char *linebuf, //!< A string containing a line of data (input).
  const unsigned line_length, //!< The length of the string (input).
  BOOL *hasComplex //!< A boolean indicating if there is any complex data (output).
  );



/// static function, rounds a value to an integer
static BOOL MTX_static_round_value_to_integer( double *value );

/// static function, rounds a value at the specified precision
static BOOL MTX_static_round_value( double *value, const unsigned precision );

////
// functions for quicksorting
static void MTX_static_quicksort( double *a, unsigned start, unsigned end ); //!< The normal quicksort function
static void MTX_static_swap_doubles( double *a, double *b ); //!< swap two doubles a and b
static int MTX_static_partition( double *a, unsigned start, unsigned end ); //!< partition the vector
static void MTX_static_quicksort_indexed( double *a, double *index, unsigned start, unsigned end ); //!< quicksort that also returns a sorted indexing vector
static void MTX_static_swap_doubles_indexed( double *a, double *b, double *index_a, double *index_b ); //!< swap the doubles and indexes
static int MTX_static_partition_indexed( double *a, double *index, unsigned start, unsigned end ); //!< partition the vectors


/// Compute the sqrt of a complex value.
static void MTX_static_quick_sqrt( const double *a_re, const double *a_im, double *re, double *im );


/// A static function to multiply a*b complex values
static void MTX_static_quick_complex_mult_ab(
  const double* a_re,
  const double* a_im,
  const double* b_re,
  const double* b_im,
  double *re,
  double *im );

/// A static function to multiply a*b*c complex values
static void MTX_static_quick_complex_mult_abc(
  const double* a_re,
  const double* a_im,
  const double* b_re,
  const double* b_im,
  const double* c_re,
  const double* c_im,
  double *re,
  double *im );


/// A static function to compute the complex result of a/b.
static void MTX_static_quick_complex_divide(
                                     const double* a_re, //!< The real part of a (input).
                                     const double* a_im, //!< The imag part of a (input).
                                     const double* b_re, //!< The real part of b (input).
                                     const double* b_im, //!< The imag part of b (input).
                                     double *re, //!< The real part of the result.
                                     double *im ); //!< The imag part of the result.



/// Calculate a CRC value to be used by CRC calculation functions.
static unsigned MTX_static_CRC32(unsigned ulCRC);

/// Updates the 32 bit CRC with a block of data.
/// This function can be called once (with uiCRC initialized to zero) to get the crc
/// for a single byte vector or multiple times to apply the crc calculation to multiple
/// bytes vectors.
static void MTX_static_updateCRC( unsigned char *pBytes, const unsigned nBytes, unsigned *uiCRC );

/// closes the file and frees memory used by MTX_SaveCompressed and MTX_Load
static void MTX_static_SaveAndLoadCleanUp( FILE *fid, unsigned char **bytes, unsigned char **compressed, const unsigned nk );

/// loads a legacy verison of a .mtx binary matrix file
static BOOL MTX_static_ReadCompressed_LegacyVersion( MTX* M, const char *path );


/// Performs factorization by gaussian elimination with scaled parital pivoting
/// /b Reference /n
/// [1] Chaney, Ward & David Kincaid, "Numerical Mathematics and Computing, 3rd Edition",
/// Cole Publishing Co., 1994, Belmont, CA, p.237)
static BOOL MTX_static_Factorize( BOOL *isFullRank, const unsigned n, unsigned* index, MTX *A );

/// Solve AX=b
/// factorized A is obtained from MTX_static_Factorize
static BOOL MTX_static_SolveByGaussianElimination(
  const MTX *b,
  MTX *X,
  const MTX *A, // factorized A
  unsigned *index );

/// Clean up dynamic memory used in MTX_Det.
static void MTX_static_Det_cleanup( unsigned *index, double *scale, MTX *U, MTX *magMtx );



/// Perform the FFT or IFFT of the columns in the src matrix and
/// store the result in the dst matrix. If the number of rows in the
/// src matrix is not a power of two, the DFT or IDFT is performed.
static BOOL MTX_static_fft(
                           const MTX *src, //!< The source matrix.
                           MTX *dst, //!< The result matrix (always complex).
                           BOOL isFwd //!< A boolean to indicate if this is a fwd transform or the inverse transform
                           );

/// Perform the FFT or IFFT of the columns in the src matrix inplace.
/// If the number of rows in the src matrix is not a power of two, the DFT or IDFT is performed.
static BOOL MTX_static_fft_inplace(
                                   MTX *src, //!< The source matrix.
                                   BOOL isFwd //!< A boolean to indicate if this is a fwd transform or the inverse transform
                                   );




/// \brief  Get a value from the uniform distribution [0,1].
/// \pre srand(seed) has been called.
static double MTX_static_get_rand_value();

/// \brief  Get a value from the standard normal gaussian distribution.
///
/// \pre srand(seed) has been called.
///
/// REFERENCE: \n
/// Scheinerman, E. R (2006). "C++ for Mathematicians: An Introduction for Students and Professionals."
/// Chapman and Hall/CRC, Taylor and Francis Group. pp 61-63.
static double MTX_static_get_randn_value();



BOOL MTX_Initialize_MTXEngine()
{
  return TRUE;
}

BOOL MTX_Enable1x1MatricesForTreatmentAsScalars( BOOL enable )
{
  MTX_static_global_treat_1x1_as_scalar = enable;
  return TRUE;
}

BOOL MTX_isNull( const MTX *M )
{
  if( !M )
    return TRUE;

  if( M->data == NULL && M->cplx == NULL )
    return TRUE;

  return FALSE;
}

BOOL MTX_isConformalForMultiplication( const MTX *A, const MTX *B )
{
  if( MTX_isNull( A ) )
  {
    return FALSE;
  }
  if( MTX_isNull( B ) )
  {
    return FALSE;
  }

  return( A->ncols == B->nrows );
}

BOOL MTX_isConformalForAddition( const MTX *A, const MTX *B )
{
  if( MTX_isNull( A ) )
  {
    return FALSE;
  }
  if( MTX_isNull( B ) )
  {
    return FALSE;
  }

  return( A->nrows == B->nrows && A->ncols == B->ncols );
}


BOOL MTX_isSquare( const MTX *A )
{
  if( MTX_isNull( A ) )
  {
    return FALSE;
  }

  return( A->nrows == A->ncols );
}

BOOL MTX_isSameSize( const MTX *A, const MTX *B )
{
  return MTX_isConformalForAddition( A, B );
}


BOOL MTX_Init( MTX *M )
{
  if( !M )
  {
    MTX_ERROR_MSG( "Cannot initialize NULL pointer." )
      return FALSE;
  }

  M->ncols = 0;
  M->nrows = 0;
  M->isReal = TRUE;
  M->cplx = NULL;
  M->data = NULL;
  M->comment = NULL;

  return TRUE;
}

BOOL MTX_SetComment( MTX *M, const char *comment )
{
  unsigned length;

  if( !M )
  {
    MTX_ERROR_MSG( "Cannot initialize NULL pointer." );
    return FALSE;
  }

  if( !comment )
  {
    MTX_ERROR_MSG( "if( !comment )" );
    return FALSE;
  }

  if( M->comment )
    free( M->comment );

  length = (unsigned int)strlen(comment);
  if( length == 0 )
  {
    MTX_ERROR_MSG( "strlen returned 0." );
    return FALSE;
  }

  M->comment = (char*)malloc( (length+1)*sizeof(char) ); // +1 for the null terminator
  if( !M->comment )
  {
    // memory allocation failure
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( strcpy_s( M->comment, length+1, comment ) != 0 )
  {
    MTX_ERROR_MSG( "strcpy_s returned 0." );
    free(M->comment);
    M->comment = NULL;
    return FALSE;
  }
#else
  strcpy( M->comment, comment );
#endif

  return TRUE;
}

BOOL MTX_Free( MTX *M )
{
  unsigned j = 0;

  if( !M )
  {
    MTX_ERROR_MSG( "Cannot free NULL pointer." );
    return FALSE;
  }

  if( M->isReal )
  {
    if( M->data == NULL )
    {
      if( M->comment )
        free( M->comment );

      M->comment = NULL;
      M->nrows = 0;
      M->ncols = 0;
      return TRUE;
    }
  }
  else
  {
    if( M->cplx == NULL )
    {
      if( M->comment )
        free( M->comment );

      M->comment = NULL;
      M->nrows = 0;
      M->ncols = 0;
      return TRUE;
    }
  }


  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      free( M->data[j] );
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      free( M->cplx[j] );
    }
  }

  // free the array of pointers
  if( M->isReal )
    free( M->data );
  else
    free( M->cplx );

  M->nrows = 0;
  M->ncols = 0;
  M->isReal = TRUE;
  M->cplx = NULL;
  M->data = NULL;

  if( M->comment )
    free( M->comment );
  M->comment = NULL;
  return TRUE;
}


BOOL MTX_Malloc( MTX *M, const unsigned nrows, const unsigned ncols, const BOOL isReal )
{
  return MTX_static_alloc( M, nrows, ncols, FALSE, isReal );
}

BOOL MTX_Calloc( MTX *M, const unsigned nrows, const unsigned ncols, const BOOL isReal )
{
  return MTX_static_alloc( M, nrows, ncols, TRUE, isReal );
}

BOOL MTX_static_alloc( MTX *M, const unsigned nrows, const unsigned ncols, const BOOL setToZero, const BOOL isReal )
{
  unsigned i = 0;
  unsigned j = 0;

  // invalid call
  if( nrows == 0 || ncols == 0 )
  {
    MTX_ERROR_MSG( "if( nrows == 0 || ncols == 0 )" );
    return FALSE;
  }
  if( !M )
  {
    MTX_ERROR_MSG( "Cannot set a NULL pointer." );
    return FALSE;
  }

  // Check if the matrix is already the right size and type.
  if( M->isReal == isReal )
  {
    if( M->nrows > 0 && M->ncols > 0 )
    {
      if( M->nrows == nrows && M->ncols == ncols )
      {
        // already the right size and type
        if( setToZero )
        {
          if( !MTX_Zero( M ) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }
          return TRUE;
        }
        else
        {
          return TRUE;
        }
      }
      else if( M->nrows == nrows && M->ncols < ncols )
      {
        if( setToZero )
        {
          if( !MTX_Zero( M ) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }
        }
        if( !MTX_AddZeroValuedColumns( M, ncols-M->ncols ) )
        {
          MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
          return FALSE;
        }
        return TRUE;
      }
      else if( M->nrows == nrows && M->ncols > ncols )
      {
        if( MTX_RemoveColumnsAfterIndex( M, ncols-1 ) == FALSE )
        {
          MTX_ERROR_MSG( "MTX_RemoveColumnsAfterIndex returned FALSE." );
          return FALSE;
        }
        if( setToZero )
        {
          if( !MTX_Zero( M ) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }
        }
        return TRUE;
      }
    }    
  }

  // The matrix must be built from scratch.
  MTX_Free( M );

  M->isReal = isReal;
  M->nrows = nrows;
  M->ncols = 0;

  // allocate the column array
  if( isReal )
  {
    M->data = (double**)malloc( ncols*sizeof(double*) );
    if( !M->data )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    M->cplx = (stComplex**)malloc( ncols*sizeof(stComplex*) );
    if( !M->cplx )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }


  // for each column allocate the rows
  if( isReal )
  {
    for( j = 0; j < ncols; j++ )
    {
      if( setToZero )
        M->data[j] = (double*)calloc( nrows, sizeof(double) );
      else
        M->data[j] = (double*)malloc( nrows*sizeof(double) );
      if( !M->data[j] )
      {
        // this is most likely to occur if allocating more memory than available
        MTX_ERROR_MSG( "malloc or calloc returned NULL." );
        MTX_Free( M );
        return FALSE;
      }
      M->ncols++;
    }
  }
  else
  {
    for( j = 0; j < ncols; j++ )
    {
      M->cplx[j] = (stComplex*)malloc( nrows*sizeof(stComplex) );
      if( !M->cplx[j] )
      {
        // this is most likely to occur if allocating more memory than available
        MTX_ERROR_MSG( "malloc returned NULL." );
        MTX_Free( M );
        return FALSE;
      }
      if( setToZero )
      {
        for( i = 0; i < nrows; i++ )
        {
          M->cplx[j][i].re = 0.0;
          M->cplx[j][i].im = 0.0;
        }
      }
      M->ncols++;
    }    
  }

  return TRUE;
}

// Set a scalar value in the matrix.
BOOL MTX_SetValue( MTX *M, const unsigned row, const unsigned col, const double value )
{
  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    M->data[col][row] = value;
  }
  else
  {
    M->cplx[col][row].re = value;
    M->cplx[col][row].im = 0.0;
  }

  return TRUE;
}

// Set a complex value in the matrix.
BOOL MTX_SetComplexValue( MTX *M, const unsigned row, const unsigned col, const double re, const double im )
{
  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal && im != 0.0 )
  {
    if( !MTX_ConvertRealToComplex( M ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  if( M->isReal )
  {
    // im == 0.0
    M->data[col][row] = re;
  }
  else
  {
    M->cplx[col][row].re = re;
    M->cplx[col][row].im = im;
  }

  return TRUE;
}


// Matrix M = Re + Im*i, where Re and Im are real matrices.
BOOL MTX_Complex( MTX *M, const MTX *Re, const MTX *Im )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( Re ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( Im ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_isSameSize( Re, Im ) )
  {
    MTX_ERROR_MSG( "MTX_isSameSize returned FALSE." );
    return FALSE;
  }

  if( !Re->isReal )
  {
    MTX_ERROR_MSG( "if( !Re->isReal )" );
    return FALSE;
  }
  if( !Im->isReal )
  {
    MTX_ERROR_MSG( "if( !Im->isReal )" );
    return FALSE;
  }

  if( M->isReal )
  {
    if( !MTX_Malloc( M, Re->nrows, Re->ncols, FALSE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_Resize( M, Re->nrows, Re->ncols, FALSE ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      M->cplx[j][i].re = Re->data[j][i];
      M->cplx[j][i].im = Im->data[j][i];
    }
  }
  return TRUE;
}


// Set the specified column in Matrix M to Re + Im*i, where Re and Im are real matrices.
// The dimensions of M must already be valid.
BOOL MTX_SetComplexColumn( MTX *M, const unsigned col, const MTX *Re, const MTX *Im )
{
  unsigned i = 0;
  unsigned j = 0;

  // check that M is complex
  if( M->isReal )
  {
    MTX_ERROR_MSG( "if( M->isReal )" );
    return FALSE;
  }

  if( MTX_isNull( Re ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( Im ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( Re->ncols != 1 )
  {
    MTX_ERROR_MSG( "if( Re->ncols != 1 )" );
    return FALSE;
  }
  if( !MTX_isSameSize( Re, Im ) )
  {
    MTX_ERROR_MSG( "MTX_isSameSize returned FALSE." );
    return FALSE;
  }

  if( !Re->isReal )
  {
    MTX_ERROR_MSG( "if( !Re->isReal )" );
    return FALSE;
  }
  if( !Im->isReal )
  {
    MTX_ERROR_MSG( "if( !Im->isReal )" );
    return FALSE;
  }

  // check that M has the right dimension
  if( M->nrows != Re->nrows )
  {
    MTX_ERROR_MSG( "if( M->nrows != Re->nrows )" );
    return FALSE;
  }
  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  for( i = 0; i < M->nrows; i++ )
  {
    M->cplx[col][i].re = Re->data[0][i];
    M->cplx[col][i].im = Im->data[0][i];
  }
  return TRUE;
}


BOOL MTX_ConvertRealToComplex( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !M->isReal )
    return TRUE; // already complex, nothing to do

  // allocate the complex column vector pointers
  M->cplx = (stComplex**)malloc( M->ncols*sizeof(stComplex*) );
  if( !M->cplx )
  {
    MTX_ERROR_MSG( "malloc retuned NULL." );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    M->cplx[j] = (stComplex*)malloc( sizeof(stComplex)*(M->nrows) );
    if( !(M->cplx[j]) )
    {
      // this is most likely to occur if allocating more memory than available
      for( k = 0; k < j; k++ ) // delete the complex column data already allocated
      {
        free( M->cplx[k] );
      }
      // delete the complex column pointer array
      free( M->cplx );
      M->cplx = NULL;
      MTX_ERROR_MSG( "malloc retuned NULL." );
      return FALSE; // note, the matrix M is still valid as a real matrix
    }
    // now copy the real data to the complex
    for( i = 0; i < M->nrows; i++ )
    {
      M->cplx[j][i].re = M->data[j][i];
      M->cplx[j][i].im = 0.0;
    }
  }

  // free the real data
  for( j = 0; j < M->ncols; j++ )
  {
    free( M->data[j] );
  }
  // free the array of real pointers
  free( M->data );
  M->data = NULL;
  M->isReal = FALSE;

  // successfully converted the matrix from real to complex
  return TRUE;
}


BOOL MTX_ConvertComplexToReal( MTX *M )
{
  return MTX_static_ConvertComplexTo( M, TRUE );
}

BOOL MTX_ConvertComplexToImag( MTX *M )
{
  return MTX_static_ConvertComplexTo( M, FALSE );
}

BOOL MTX_static_ConvertComplexTo( MTX *M, BOOL useReal )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // Deal with special case of already real values.
  if( useReal )
  {
    if( M->isReal )
      return TRUE; // already real, nothing to do
  }

  // Deal with special case of trying to get complex values from a real matrix.
  if( !useReal && M->isReal )
  {
    if( !MTX_Zero(M) )
    {
      MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
      return FALSE;
    }
    return TRUE;
  }

  // allocate the complex column vector pointers
  M->data = (double**)malloc( (M->ncols)*sizeof(double*) );
  if( !M->data )
  {
    MTX_ERROR_MSG( "if( !M->data )" );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    M->data[j] = (double*)malloc( sizeof(double)*(M->nrows) );
    if( !(M->data[j]) )
    {
      // this is most likely to occur if allocating more memory than available
      for( k = 0; k < j; k++ ) // delete the real column data already allocated
      {
        free( M->data[k] );
      }
      // delete the real column pointer array
      free( M->data );
      M->data = NULL;
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE; // note, the matrix M is still valid as a complex matrix
    }
    // now copy the complex real component of the data to the real data.
    for( i = 0; i < M->nrows; i++ )
    {
      if( useReal )
        M->data[j][i] = M->cplx[j][i].re;
      else
        M->data[j][i] = M->cplx[j][i].im;
    }
  }

  // free the complex data
  for( j = 0; j < M->ncols; j++ )
  {
    free( M->cplx[j] );
  }
  // free the array of real pointers
  free( M->cplx );
  M->cplx = NULL;
  M->isReal = TRUE;

  // successfully converted the matrix from complex to real
  return TRUE;
}


// Extract the real component of matrix M
BOOL MTX_Real( const MTX *M, MTX *Re )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
    return MTX_Copy( M, Re );

  if( !MTX_Malloc( Re, M->nrows, M->ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      Re->data[j][i] = M->cplx[j][i].re;
    }
  }
  return TRUE;
}


BOOL MTX_isReal( MTX *M, BOOL *isReal )
{
  if( MTX_isNull( M ) )
  {
    *isReal = TRUE;
    return TRUE; // A null matrix is real by default.
  }

  if( M->isReal )
  {
    *isReal = TRUE;
  }
  else
  {
    double maxabs;
    MTX imagM;

    MTX_Init(&imagM);

    if( !MTX_Imag(M,&imagM) )
    {
      MTX_ERROR_MSG( "MTX_Imag returned FALSE." );
      return FALSE;
    }

    if( !MTX_MaxAbs(&imagM,&maxabs) )
    {
      MTX_ERROR_MSG( "MTX_MaxAbs returned FALSE." );
      return FALSE;
    }

    if( maxabs == 0.0 )
    {
      if( !MTX_ConvertComplexToReal(M) )
      {
        MTX_ERROR_MSG( "MTX_ConvertComplexToReal returned FALSE." );
        return FALSE;
      }
      *isReal = TRUE;
    }
    else
    {
      *isReal = FALSE;
    }

    MTX_Free(&imagM);
  }

  return TRUE;
}

BOOL MTX_RealColumn( const MTX *M, const unsigned col, MTX *Re )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
    return MTX_CopyColumn( M, col, Re );

  if( !MTX_Malloc( Re, M->nrows, 1, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  for( i = 0; i < M->nrows; i++ )
  {
    Re->data[0][i] = M->cplx[col][i].re;
  }

  return TRUE;
}

BOOL MTX_Imag( const MTX *M, MTX *Im )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
    return MTX_Calloc( Im, M->nrows, M->ncols, TRUE ); // return a zero matrix

  if( !MTX_Malloc( Im, M->nrows, M->ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      Im->data[j][i] = M->cplx[j][i].im;
    }
  }
  return TRUE;
}

BOOL MTX_ImagColumn( const MTX *M, const unsigned col, MTX *Im )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
    return MTX_Calloc( Im, M->nrows, 1, TRUE ); // return a zero column

  if( !MTX_Malloc( Im, M->nrows, 1, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  for( i = 0; i < M->nrows; i++ )
  {
    Im->data[0][i] = M->cplx[col][i].im;
  }
  return TRUE;
}

BOOL MTX_Magnitude( const MTX *M, MTX *Magnitude )
{
  unsigned i = 0;
  unsigned j = 0;
  double re;
  double im;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    if( !MTX_Copy( M, Magnitude ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      return FALSE;
    }
    if( !MTX_Abs( Magnitude ) )
    {
      MTX_ERROR_MSG( "MTX_Abs returned FALSE." );
      return FALSE;
    }
    return TRUE;
  }

  if( !MTX_Malloc( Magnitude, M->nrows, M->ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      re = M->cplx[j][i].re;
      im = M->cplx[j][i].im;
      Magnitude->data[j][i] = sqrt( re*re + im*im );
    }
  }
  return TRUE;
}

BOOL MTX_Phase( const MTX *M, MTX *Phase )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Calloc( Phase, M->nrows, M->ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        if( M->data[j][i] < 0.0 )
        {
          Phase->data[j][i] = PI;
        }
        else
        {
          Phase->data[j][i] = 0.0;
        }
      }
    }
    return TRUE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      Phase->data[j][i] = atan2( M->cplx[j][i].im, M->cplx[j][i].re );
    }
  }
  return TRUE;
}


BOOL MTX_Conjugate( MTX *M )
{
  unsigned i=0;
  unsigned j=0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
    return TRUE;

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      M->cplx[j][i].im = -M->cplx[j][i].im;
    }
  }
  return TRUE;
}

BOOL MTX_RemoveColumn( MTX *M, const unsigned col )
{
  unsigned j = 0;
  unsigned k = 0;
  double **dptr = NULL;
  stComplex **cptr = NULL;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  // special case
  if( M->ncols == 1 )
  {
    return MTX_Free( M );
  }

  // allocate a new array of column vectors
  if( M->isReal )
  {
    dptr = (double**)malloc( (M->ncols-1)*sizeof(double*) );
    if( !dptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    cptr = (stComplex**)malloc( (M->ncols-1)*sizeof(stComplex*) );
    if( !cptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }

  // copy the previous array of pointers
  // except the one to remove
  k = 0;
  for( j = 0; j < M->ncols; j++ )
  {
    if( j != col )
    {
      if( M->isReal )
        dptr[k] = M->data[j];
      else
        cptr[k] = M->cplx[j];
      k++;
    }
    else
    {
      if( M->isReal )
        free( M->data[j] );
      else
        free( M->cplx[j] );
    }
  }

  // free the old column array, and copy the new
  if( M->isReal )
  {
    free( M->data );
    M->data = dptr;
  }
  else
  {
    free( M->cplx );
    M->cplx = cptr;
  }
  M->ncols--;

  return TRUE;
}

BOOL MTX_RemoveColumnsAfterIndex( MTX *dst, const unsigned col )
{
  unsigned ncols;
  unsigned j = 0;
  double **dptr = NULL;
  stComplex **cptr = NULL;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= dst->ncols )
  {
    MTX_ERROR_MSG( "if( col >= dst->ncols )" );
    return FALSE;
  }

  // special case
  if( dst->ncols == 1 )
  {
    return MTX_Free( dst );
  }

  ncols = col+1;

  // allocate a new array of column vectors
  if( dst->isReal )
  {
    dptr = (double**)malloc( ncols*sizeof(double*) );
    if( !dptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    cptr = (stComplex**)malloc( ncols*sizeof(stComplex*) );
    if( !cptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    if( j < ncols )
    {
      if( dst->isReal )
        dptr[j] = dst->data[j];
      else
        cptr[j] = dst->cplx[j];
    }
    else
    {
      if( dst->isReal )
        free( dst->data[j] );
      else
        free( dst->cplx[j] );
    }
  }

  // free the old column array, and copy the new
  if( dst->isReal )
  {
    free( dst->data );
    dst->data = dptr;
  }
  else
  {
    free( dst->cplx );
    dst->cplx = cptr;
  }
  dst->ncols = ncols;

  return TRUE;
}


BOOL MTX_InsertColumn( MTX *dst, const MTX *src, const unsigned dst_col, const unsigned src_col )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned m = 0;
  double **dptr = NULL;
  stComplex **cptr = NULL;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( dst->nrows != src->nrows )
  {
    MTX_ERROR_MSG( "if( dst->nrows != src->nrows )" );
    return FALSE;
  }

  // note the missing '=' here, a column can be inserted at the end (i.e. AddColumn )
  if( dst_col > dst->ncols )
  {
    MTX_ERROR_MSG( "if( dst_col > dst->ncols )" );
    return FALSE;
  }

  if( src_col >= src->ncols )
  {
    MTX_ERROR_MSG( "if( src_col >= src->ncols )" );
    return FALSE;
  }

  if( !src->isReal && dst->isReal )
  {
    // convert the destination matrix to complex
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  // allocate a new array of column vectors
  if( dst->isReal )
  {
    dptr = (double**)malloc( (dst->ncols+1)*sizeof(double*) );
    if( !dptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    cptr = (stComplex**)malloc( (dst->ncols+1)*sizeof(stComplex*) );
    if( !cptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }

  // copy the previous array of pointers
  // and add the new column
  k = 0;
  for( j = 0; j <= dst->ncols; j++ )
  {
    if( j == dst_col )
    {
      // allocate a new column vector
      if( dst->isReal )
        dptr[k] = (double*)malloc( dst->nrows*sizeof(double) );
      else
        cptr[k] = (stComplex*)malloc( dst->nrows*sizeof(stComplex) );
      if( dst->isReal )
      {
        if( !dptr[k] )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < k; m++ )
          {
            free( dptr[m] );
          }

          MTX_ERROR_MSG( "malloc returned NULL." );
          free( dptr );
          return FALSE;
        }
      }
      else
      {
        if( !cptr[k] )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < k; m++ )
          {
            free( cptr[m] );
          }
          MTX_ERROR_MSG( "malloc returned NULL." );
          free( cptr );
          return FALSE;
        }
      }

      // copy the src column vector
      for( i = 0; i < dst->nrows; i++ )
      {
        if( dst->isReal )
        {
          dptr[k][i] = src->data[src_col][i];
        }
        else
        {
          if( src->isReal )
          {
            cptr[k][i].re = src->data[src_col][i];
            cptr[k][i].im = 0;
          }
          else
          {
            cptr[k][i] = src->cplx[src_col][i];
          }
        }
      }
      // copy the data that was at the insertion index
      // unless this is the after the last column
      if( j != dst->ncols )
      {
        k++;
        if( dst->isReal )
          dptr[k] = dst->data[j];
        else
          cptr[k] = dst->cplx[j];
      }
    }
    else
    {
      if( j != dst->ncols )
      {
        if( dst->isReal )
          dptr[k] = dst->data[j];
        else
          cptr[k] = dst->cplx[j];
      }
    }
    k++;
  }

  // free the old column array, and copy the new
  if( dst->isReal )
  {
    free( dst->data );
    dst->data = dptr;
  }
  else
  {
    free( dst->cplx );
    dst->cplx = cptr;
  }
  dst->ncols++;

  return TRUE;
}

BOOL MTX_AddColumn( MTX *dst, const MTX *src, const unsigned src_col )
{
  return MTX_InsertColumn( dst, src, dst->ncols, src_col );
}

BOOL MTX_Concatonate( MTX *dst, const MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned ncols;
  unsigned m = 0;
  double **dptr = NULL;
  stComplex **cptr = NULL;

  if( dst == NULL )
  {
    MTX_ERROR_MSG( "dst is a NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( dst->nrows == 0 && dst->ncols == 0 )
  {
    return MTX_Copy( src, dst );
  }
  else if( dst->nrows != src->nrows )
  {
    MTX_ERROR_MSG( "if( dst->nrows != src->nrows )" );
    return FALSE;
  }

  ncols = dst->ncols+src->ncols;

  if( dst->isReal && !src->isReal )
  {
    // Convert dst to complex
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  // allocate a new array of column vectors
  if( dst->isReal )
  {
    dptr = (double**)malloc( ncols*sizeof(double*) );
    if( !dptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    cptr = (stComplex**)malloc( ncols*sizeof(stComplex*) );
    if( !cptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }

  for( j = 0; j < ncols; j++ )
  {
    if( j < dst->ncols )
    {
      // keep the original data
      if( dst->isReal )
        dptr[j] = dst->data[j];
      else
        cptr[j] = dst->cplx[j];
    }
    else
    {
      // copy the new data
      if( dst->isReal )
        dptr[j] = (double*)malloc( dst->nrows*sizeof(double) );
      else
        cptr[j] = (stComplex*)malloc( dst->nrows*sizeof(stComplex) );
      if( dst->isReal )
      {
        if( !(dptr[j]) )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < j; m++ )
          {
            free( dptr[m] );
          }

          MTX_ERROR_MSG( "malloc returned NULL." );
          free( dptr );
          return FALSE;
        }
      }
      else
      {
        if( !(cptr[j]) )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < j; m++ )
          {
            free( cptr[m] );
          }
          free( cptr );
          MTX_ERROR_MSG( "malloc returned NULL." );
          return FALSE;
        }
      }
      // copy the src column vector
      for( i = 0; i < dst->nrows; i++ )
      {
        if( dst->isReal )
        {
          dptr[j][i] = src->data[j-dst->ncols][i];
        }
        else
        {
          if( src->isReal )
          {
            cptr[j][i].re = src->data[j-dst->ncols][i];
            cptr[j][i].im = 0;
          }
          else
          {
            cptr[j][i] = src->cplx[j-dst->ncols][i];
          }
        }
      }
    }
  }

  // free the old column array, and copy the new
  if( dst->isReal )
  {
    free( dst->data );
    dst->data = dptr;
  }
  else
  {
    free( dst->cplx );
    dst->cplx = cptr;
  }
  dst->ncols = ncols;

  return TRUE;
}



// A becomes A|0|0|0|.. etc
BOOL MTX_AddZeroValuedColumns( MTX *dst, const unsigned nr_new_cols )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned ncols;
  unsigned m = 0;
  double **dptr = NULL;
  stComplex **cptr = NULL;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  ncols = dst->ncols + nr_new_cols;

  // allocate a new array of column vectors
  if( dst->isReal )
  {
    dptr = (double**)malloc( ncols*sizeof(double*) );
    if( !dptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    cptr = (stComplex**)malloc( ncols*sizeof(stComplex*) );
    if( !cptr )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }

  for( j = 0; j < ncols; j++ )
  {
    if( j < dst->ncols )
    {
      // keep the original data
      if( dst->isReal )
        dptr[j] = dst->data[j];
      else
        cptr[j] = dst->cplx[j];
    }
    else
    {
      // copy the new data
      if( dst->isReal )
      {
        dptr[j] = (double*)calloc( dst->nrows, sizeof(double) );
      }
      else
      {
        cptr[j] = (stComplex*)calloc( dst->nrows, sizeof(stComplex) );
      }
      if( dst->isReal )
      {
        if( !(dptr[j]) )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < j; m++ )
          {
            free( dptr[m] );
          }
          free( dptr );

          MTX_ERROR_MSG( "calloc returned NULL." );
          return FALSE;
        }
      }
      else
      {
        if( !(cptr[j]) )
        {
          // this is most likely to occur if allocating more memory than available
          for( m = 0; m < j; m++ )
          {
            free( cptr[m] );
          }
          free( cptr );

          MTX_ERROR_MSG( "calloc returned NULL." );
          return FALSE;
        }
      }

    }
  }

  // free the old column array, and copy the new
  if( dst->isReal )
  {
    free( dst->data );
    dst->data = dptr;
  }
  else
  {
    free( dst->cplx );
    dst->cplx = cptr;
  }
  dst->ncols = ncols;

  return TRUE;
}

BOOL MTX_Redim( MTX *dst, const unsigned nrows, const unsigned ncols )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned nc;
  unsigned nr;
  MTX copy;
  double **dptr = NULL;
  stComplex **cptr = NULL;
  const BOOL isReal = dst->isReal;

  MTX_Init( &copy );

  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  if( nrows == 0 || ncols == 0 )
  {
    MTX_ERROR_MSG( "if( nrows == 0 || ncols == 0 )" );
    return FALSE;
  }

  // special case - calling Redim with a null matrix
  if( dst->ncols == 0 && dst->nrows == 0 )
    return MTX_Calloc( dst, nrows, ncols, dst->isReal );


  // check same size
  if( dst->nrows == nrows && dst->ncols == ncols )
    return TRUE;

  // special cases, adding or removing columns
  if( dst->nrows == nrows )
  {
    if( ncols < dst->ncols )
    {
      if( MTX_RemoveColumnsAfterIndex( dst, ncols-1 ) == FALSE )
      {
        MTX_ERROR_MSG( "MTX_RemoveColumnsAfterIndex returned FALSE." );
        return FALSE;
      }

      return TRUE;
    }
    else
    {
      // Add the extra columns
      if( !MTX_AddZeroValuedColumns( dst, ncols-dst->ncols ) )
      {
        MTX_ERROR_MSG( "MTX_AddZeroValuedColumns returned FALSE." );
        return FALSE;
      }
      return TRUE;
    }
  }

  // make a copy of the previous data
  if( !MTX_Malloc( &copy, dst->nrows, dst->ncols, isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }
  if( !MTX_Copy( dst, &copy ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &copy );
    return FALSE;
  }

  // must reallocate the matrix
  MTX_Free( dst );

  if( !MTX_Calloc( dst, nrows, ncols, isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    MTX_Free( &copy );
    return FALSE;
  }

  // Copy the previous data.
  if( dst->ncols < copy.ncols )
    nc = dst->ncols;
  else
    nc = copy.ncols;

  if( dst->nrows < copy.nrows )
    nr = dst->nrows;
  else
    nr = copy.nrows;

  if( isReal )
  {
    for( j = 0; j < nc; j++ )
    {
      memcpy( dst->data[j], copy.data[j], sizeof(double)*nr );
    }
  }
  else
  {
    for( j = 0; j < nc; j++ )
    {
      memcpy( dst->cplx[j], copy.cplx[j], sizeof(stComplex)*nr );
    }
  }
  MTX_Free( &copy );
  return TRUE;
}

BOOL MTX_Resize( MTX *dst, const unsigned nrows, const unsigned ncols, const BOOL isReal )
{
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  if( nrows == 0 || ncols == 0 )
  {
    MTX_ERROR_MSG( "if( nrows == 0 || ncols == 0 )" );
    return FALSE;
  }

  // MTX_Calloc is smart. It only re-allocates memory if it needs to
  // and always sets the data to zero.
  if( !MTX_Calloc( dst, nrows, ncols, isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    return FALSE;
  }

  return TRUE;
}

BOOL MTX_Copy( const MTX *src, MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  // both must be real or both must be complex
  if( dst->isReal != src->isReal )
  {
    if( !MTX_Resize( dst, src->nrows, src->ncols, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( !MTX_isSameSize( src, dst ) )
  {
    if( !MTX_Resize( dst, src->nrows, src->ncols, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  

  if( src->isReal )
  {  
    for( j = 0; j < dst->ncols; j++ )
    {
      memcpy( dst->data[j], src->data[j], sizeof(double)*(dst->nrows) );
    }
  }
  else
  {
    for( j = 0; j < dst->ncols; j++ )
    {
      memcpy( dst->cplx[j], src->cplx[j], sizeof(stComplex)*(dst->nrows) );
    }
  }
  
  return TRUE;
}

BOOL MTX_CopyIntoColumnWiseVector( const MTX *src, MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  // both must be real or both must be complex
  if( dst->isReal != src->isReal )
  {
    if( !MTX_Resize( dst, src->nrows*src->ncols, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( dst->nrows != src->nrows*src->ncols )
  {
    if( !MTX_Resize( dst, src->nrows*src->ncols, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      memcpy( &(dst->data[0][j*src->nrows]), src->data[j], sizeof(double)*src->nrows );
    }
  }
  else
  {
    for( j = 0; j < src->ncols; j++ )
    {
      memcpy( &(dst->cplx[0][j*src->nrows]), src->cplx[j], sizeof(stComplex)*src->nrows );
    }
  }

  return TRUE;
}


BOOL MTX_SetFromStaticMatrix( MTX *dst, const double mat[], const unsigned nrows, const unsigned ncols )
{
  unsigned i = 0;
  unsigned j = 0;

  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  if( !mat )
  {
    MTX_ERROR_MSG( "mat is a NULL pointer." );
    return FALSE;
  }

  if( dst->nrows != nrows || dst->ncols != ncols )
  {
    if( !MTX_Resize( dst, nrows, ncols, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < ncols; j++ )
    for( i = 0; i < nrows; i++ )
      dst->data[j][i] = mat[i*ncols + j];

  return TRUE;
}

BOOL MTX_CopyColumn( const MTX *src, const unsigned col, MTX *dst )
{
  unsigned i = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }
  if( col >= src->ncols )
  {
    MTX_ERROR_MSG( "if( col >= src->ncols )" );
    return FALSE;
  }

  if( src->isReal != dst->isReal )
  {
    if( !MTX_Malloc( dst, src->nrows, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  else if( dst->nrows != src->nrows || dst->ncols != 1 )
  {
    if( !MTX_Malloc( dst, src->nrows, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( src->isReal )
  {
    memcpy( dst->data[0], src->data[col], sizeof(double)*(dst->nrows) );
  }
  else
  {
    memcpy( dst->cplx[0], src->cplx[col], sizeof(stComplex)*(dst->nrows) );
  }

  return TRUE;
}

BOOL MTX_CopyRow( const MTX *src, const unsigned row, MTX *dst )
{
  unsigned i = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }
  if( row >= src->nrows )
  {
    MTX_ERROR_MSG( "if( row >= src->nrows )" );
    return FALSE;
  }

  if( dst->nrows != 1 || dst->ncols != src->ncols )
  {
    if( !MTX_Resize( dst, 1, src->ncols, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  for( i = 0; i < dst->ncols; i++ )
  {
    if( src->isReal )
      dst->data[i][0] = src->data[i][row];
    else
      dst->cplx[i][0] = src->cplx[i][row];
  }

  return TRUE;
}

BOOL MTX_CopyRowIntoAColumnMatrix( const MTX *src, const unsigned row, MTX *dst )
{
  unsigned i = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }
  if( row >= src->nrows )
  {
    MTX_ERROR_MSG( "if( row >= src->nrows )" );
    return FALSE;
  }

  if( dst->nrows != src->ncols || dst->ncols != 1 )
  {
    if( !MTX_Resize( dst, src->ncols, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  for( i = 0; i < dst->nrows; i++ )
  {
    if( src->isReal )
      dst->data[0][i] = src->data[i][row];
    else
      dst->cplx[0][i] = src->cplx[i][row];
  }

  return TRUE;
}

BOOL MTX_InsertSubMatrix( MTX *dst, const MTX *src, const unsigned dst_row, const unsigned dst_col )
{
  unsigned i = 0;
  unsigned j = 0;

  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // check that the submatrix doesn't exceed the bounds of the matrix
  if( dst_row + src->nrows > dst->nrows )
  {
    MTX_ERROR_MSG( "if( dst_row + src->nrows > dst->nrows )" );
    return FALSE;
  }
  if( dst_col + src->ncols > dst->ncols )
  {
    MTX_ERROR_MSG( "if( dst_col + src->ncols > dst->ncols )" );
    return FALSE;
  }

  if( !src->isReal && dst->isReal )
  {
    // convert the matrix to complex if the src matrix is complex
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  // insert the submatrix
  if( dst->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      memcpy( &(dst->data[dst_col+j][dst_row]), src->data[j], sizeof(double)*src->nrows );
    }
  }
  else
  {
    if( src->isReal )
    {
      for( j = 0; j < src->ncols; j++ )
      {
        for( i = 0; i < src->nrows; i++ )
        {
          dst->cplx[dst_col+j][dst_row+i].re = src->data[j][i];
          dst->cplx[dst_col+j][dst_row+i].im = 0.0;
        }
      }
    }
    else
    {
      for( j = 0; j < src->ncols; j++ )
      {
        memcpy( &(dst->cplx[dst_col+j][dst_row]), src->cplx[j], sizeof(stComplex)*src->nrows );
      }
    }
  }
  return TRUE;
}

BOOL MTX_ExtractSubMatrix( 
  const MTX* src,          //!< The source matrix.                        
  MTX* dst,                //!< The destination matrix to contain the submatrix.
  const unsigned from_row, //!< The zero-based index for the from row.
  const unsigned from_col, //!< The zero-based index for the from column.
  const unsigned to_row,   //!< The zero-based index for the to row.
  const unsigned to_col    //!< The zero-based index for the to column.
  )
{
  unsigned i;
  unsigned j;
  unsigned k;
  unsigned m;

  if( src == NULL )
  {
    MTX_ERROR_MSG( "NULL source matrix" );
    return FALSE;
  }
  if( dst == NULL )
  {
    MTX_ERROR_MSG( "NULL destination matrix" );
    return FALSE;
  }
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL source matrix" );
    return FALSE;
  }

  if( to_row - from_row < 0 )
  {
    MTX_ERROR_MSG( "The destination matrix has invalid dimension. to_row - from_row < 0" );
    return FALSE;
  }
  if( to_col - from_col < 0 )
  {
    MTX_ERROR_MSG( "The destination matrix has invalid dimension. to_col - from_col < 0" );
    return FALSE;
  }

  if( from_row >= src->nrows )
  {
    MTX_ERROR_MSG( "from_row > number of source rows" );
    return FALSE;
  }
  if( from_col >= src->ncols )
  {
    MTX_ERROR_MSG( "from_col > number of source columns" );
    return FALSE;
  }
  if( to_row >= src->nrows )
  {
    MTX_ERROR_MSG( "to_row > number of source rows" );
    return FALSE;
  }
  if( to_col >= src->ncols )
  {
    MTX_ERROR_MSG( "to_col > number of source columns" );
    return FALSE;
  }

  if( !MTX_Malloc( dst, to_row-from_row+1, to_col-from_col+1, src->isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  m = 0;
  for( j = from_col; j <= to_col; j++ )
  {
    k = 0;
    for( i = from_row; i <= to_row; i++ )
    {
      if( src->isReal )
      {
        dst->data[m][k] = src->data[j][i];
      }
      else
      {
        dst->cplx[m][k].re = src->cplx[j][i].re;
        dst->cplx[m][k].im = src->cplx[j][i].im;
      }
      k++;
    }
    m++;
  }

  return TRUE;
}

BOOL MTX_Zero( MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    if( dst->isReal )
    {
      memset( dst->data[j], 0, sizeof(double)*dst->nrows );
    }
    else
    {
      memset( dst->cplx[j], 0, sizeof(stComplex)*dst->nrows );
    }
  }
  return TRUE;
}

BOOL MTX_ZeroColumn( MTX *dst, const unsigned col )
{
  unsigned i = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= dst->ncols )
  {
    MTX_ERROR_MSG( "if( col >= dst->ncols )" );
    return FALSE;
  }

  if( dst->isReal )
  {
    memset( dst->data[col], 0, sizeof(double)*dst->nrows );
  }
  else
  {
    memset( dst->cplx[col], 0, sizeof(stComplex)*dst->nrows );
  }

  return TRUE;
}

BOOL MTX_ZeroRow( MTX *dst, const unsigned row )
{
  return MTX_FillRow( dst, row, 0.0 );
}

BOOL MTX_Fill( MTX *dst, const double value )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // use memcpy after the first column is set for efficiency and speed.
  if( dst->isReal )
  {
    j = 0;
    for( i = 0; i < dst->nrows; i++ )
    {
      dst->data[j][i] = value;          
    }
    for( j = 1; j < dst->ncols; j++ )
    {
      memcpy( dst->data[j], dst->data[j-1], sizeof(double)*dst->nrows );
    }
  }
  else
  {
    j = 0;
    for( i = 0; i < dst->nrows; i++ )
    {
      dst->cplx[j][i].re = value;
      dst->cplx[j][i].im = 0.0;
    }
    for( j = 1; j < dst->ncols; j++ )
    {
      memcpy( dst->cplx[j], dst->cplx[j-1], sizeof(stComplex)*dst->nrows );
    }
  }

  return TRUE;
}

BOOL MTX_FillComplex( MTX *dst, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( dst->isReal )
  {
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    if( j == 0 )
    {
      for( i = 0; i < dst->nrows; i++ )
      {
        dst->cplx[j][i].re = re;
        dst->cplx[j][i].im = im;
      }
    }
    else
    {
      memcpy( dst->cplx[j], dst->cplx[j-1], sizeof(stComplex)*dst->nrows );
    }
  }
  return TRUE;
}

BOOL MTX_FillColumn( MTX *dst, const unsigned col, const double value )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= dst->ncols )
  {
    MTX_ERROR_MSG( "if( col >= dst->ncols )" );
    return FALSE;
  }

  if( dst->isReal )
  {
    for( i = 0; i < dst->nrows; i++ )
    {
      dst->data[col][i] = value;
    }
  }
  else
  {
    for( i = 0; i < dst->nrows; i++ )
    {
      dst->cplx[col][i].re = value;
      dst->cplx[col][i].im = 0;
    }
  }

  return TRUE;
}

BOOL MTX_FillColumnComplex( MTX *dst, const unsigned col, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= dst->ncols )
  {
    MTX_ERROR_MSG( "if( col >= dst->ncols )" );
    return FALSE;
  }

  if( dst->isReal )
  {
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( i = 0; i < dst->nrows; i++ )
  {
    dst->cplx[col][i].re = re;
    dst->cplx[col][i].im = im;
  }

  return TRUE;
}

BOOL MTX_FillRow( MTX *dst, const unsigned row, const double value )
{
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= dst->nrows )
  {
    MTX_ERROR_MSG( "if( row >= dst->nrows )" );
    return FALSE;
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    if( dst->isReal )
    {
      dst->data[j][row] = value;
    }
    else
    {
      dst->cplx[j][row].re = value;
      dst->cplx[j][row].im = value;
    }
  }

  return TRUE;
}

BOOL MTX_FillRowComplex( MTX *dst, const unsigned row, const double re, const double im )
{
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= dst->nrows )
  {
    MTX_ERROR_MSG( "if( row >= dst->nrows )" );
    return FALSE;
  }

  if( dst->isReal )
  {
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    dst->cplx[j][row].re = re;
    dst->cplx[j][row].im = im;
  }

  return TRUE;
}


// set the matrix to an identity
BOOL MTX_Identity( MTX *dst )
{
  unsigned j = 0;

  if( MTX_isNull( dst ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_isSquare( dst ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  if( !dst->isReal )
  {
    if( !MTX_Calloc( dst, dst->nrows, dst->ncols, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_Zero( dst ) )
    {
      MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < dst->ncols; j++ )
  {
    dst->data[j][j] = 1.0;
  }
  return TRUE;
}


// Transpose the matrix src into the matrix dst.
BOOL MTX_Transpose( const MTX *src, MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;

  if( !dst )
  {
    MTX_ERROR_MSG( "dst is a NULL pointer." );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // special case inplace transpose
  if( dst == src )
  {
    return MTX_TransposeInplace( dst );
  }

  // complex/real mixed cases
  if( !src->isReal && dst->isReal )
  {
    MTX_Free( dst );

    if( !MTX_Malloc( dst, src->ncols, src->nrows, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }
  else if( src->isReal && !dst->isReal )
  {
    MTX_Free( dst );

    if( !MTX_Malloc( dst, src->ncols, src->nrows, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }


  // resize if needed
  if( dst->nrows != src->ncols || dst->ncols != src->nrows )
  {
    if( !MTX_Resize( dst, src->ncols, src->nrows, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( dst->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        dst->data[i][j] = src->data[j][i];        
      }
    }
  }
  else
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        dst->cplx[i][j].re = src->cplx[j][i].re;
        dst->cplx[i][j].im = src->cplx[j][i].im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_TransposeInplace( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  double tmp = 0.0;
  stComplex cplxval;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // special case, square matrix
  if( MTX_isSquare( M ) )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        if( i == j )
          break; // only need to go halfway

        if( M->isReal )
        {
          tmp = M->data[j][i];
          M->data[j][i] = M->data[i][j];
          M->data[i][j] = tmp;
        }
        else
        {
          cplxval.re = M->cplx[j][i].re;
          cplxval.im = M->cplx[j][i].im;
          M->cplx[j][i].re = M->cplx[i][j].re;
          M->cplx[j][i].im = M->cplx[i][j].im;
          M->cplx[i][j].re = cplxval.re;
          M->cplx[i][j].im = cplxval.im;
        }
      }
    }
  }
  else
  {
    MTX copy;
    MTX_Init( &copy );

    if( !MTX_Malloc( &copy, M->nrows, M->ncols, M->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
    if( !MTX_Copy( M, &copy ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      return FALSE;
    }

    // resize the matrix from nxm to mxn
    if( !MTX_Resize( M, copy.ncols, copy.nrows, copy.isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }

    for( j = 0; j < copy.ncols; j++ )
    {
      for( i = 0; i < copy.nrows; i++ )
      {
        if( M->isReal )
        {
          M->data[i][j] = copy.data[j][i];
        }
        else
        {
          M->cplx[i][j].re = copy.cplx[j][i].re;
          M->cplx[i][j].im = copy.cplx[j][i].im;
        }
      }
    }

    MTX_Free( &copy );
  }
  return TRUE;
}

// round the matrix elements to the specified presision
// e.g. precision = 0 1.8 -> 2
// e.g. precision = 1, 1.45 -> 1.5
// e.g. precision = 2 1.456 -> 1.46
// e.g. precision = 3, 1.4566 -> 1.457
BOOL MTX_Round( MTX *M, const unsigned precision )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( precision > 32 )
    return TRUE;

  if( precision == 0 )
  {
    if( M->isReal )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        for( i = 0; i < M->nrows; i++ )
        {
          MTX_static_round_value_to_integer( &(M->data[j][i]) );
        }
      }
    }
    else
    {
      for( j = 0; j < M->ncols; j++ )
      {
        for( i = 0; i < M->nrows; i++ )
        {
          MTX_static_round_value_to_integer( &(M->cplx[j][i].re) );
          MTX_static_round_value_to_integer( &(M->cplx[j][i].im) );
        }
      }
    }
  }
  else
  {
    if( M->isReal )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        for( i = 0; i < M->nrows; i++ )
        {
          MTX_static_round_value( &(M->data[j][i]), precision );
        }
      }
    }
    else
    {
      for( j = 0; j < M->ncols; j++ )
      {
        for( i = 0; i < M->nrows; i++ )
        {
          MTX_static_round_value( &(M->cplx[j][i].re), precision );
          MTX_static_round_value( &(M->cplx[j][i].im), precision );
        }
      }
    }
  }
  return TRUE;
}

BOOL MTX_static_round_value_to_integer( double *value )
{
  if( *value < 0 )
    *value = ceil( *value - 0.5 );
  else
    *value = floor( *value + 0.5 );
  return TRUE;
}

BOOL MTX_static_round_value( double *value, const unsigned precision )
{
  double pow10;

  pow10 = pow( 10.0, (double)(precision) );

  *value *= pow10;
  if( *value < 0 )
    *value = ceil( *value - 0.5 );
  else
    *value = floor( *value + 0.5 );
  *value /= pow10;

  return TRUE;
}

BOOL MTX_Floor( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {      
        M->data[j][i] = floor(M->data[j][i]);
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {                  
        M->cplx[j][i].re = floor(M->cplx[j][i].re);
        M->cplx[j][i].im = floor(M->cplx[j][i].im);
      }
    }
  }

  return TRUE;
}

BOOL MTX_Ceil( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = ceil(M->data[j][i]);
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re = ceil(M->cplx[j][i].re);
        M->cplx[j][i].im = ceil(M->cplx[j][i].im);
      }
    }
  }

  return TRUE;
}

BOOL MTX_Fix( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        if( M->data[j][i] < 0 )
          M->data[j][i] = ceil(M->data[j][i]);
        else
          M->data[j][i] = floor(M->data[j][i]);
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        if( M->cplx[j][i].re < 0 )
          M->cplx[j][i].re = ceil(M->cplx[j][i].re);
        else
          M->cplx[j][i].re = floor(M->cplx[j][i].re);

        if( M->cplx[j][i].im < 0 )
          M->cplx[j][i].im = ceil(M->cplx[j][i].im);
        else
          M->cplx[j][i].im = floor(M->cplx[j][i].im);
      }
    }
  }
  return TRUE;
}


BOOL MTX_OneMinus( const MTX* src, MTX *dst )
{
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( dst == NULL )
  {
    MTX_ERROR_MSG( "if( dst == NULL )" );
    return FALSE;
  }

  if( !(dst->isReal == src->isReal &&
    dst->nrows == src->nrows &&
    dst->ncols == src->ncols ) )
  {
    if( !MTX_Malloc( dst, src->nrows, src->ncols, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  if( !MTX_Fill( dst, 1.0 ) )
  {
    MTX_ERROR_MSG( "MTX_Fill returned FALSE." );
    return FALSE;
  }
  if( !MTX_Subtract_Inplace( dst, src ) )
  {
    MTX_ERROR_MSG( "MTX_Subtract_Inplace returned FALSE." );
    return FALSE;
  }
  return TRUE;
}


BOOL MTX_DetermineFileDelimiter(
                                const char *path, //!< path to the input file
                                char *delimiter, //!< delimiter, 'b' is binary
                                BOOL *hasComment, //!< BOOL to indicate if a comment line is present
                                char **comment //!< pointer to a string to store the comment line, *comment memory must be freed later.
                                )
{
  unsigned i = 0;
  unsigned line_length = 0;
  char line[MTX_MAX_READFROMFILE_BUFFER];
  char *linebuf;
  FILE *in;
  BOOL atEOF = FALSE;

  *hasComment = FALSE;

  // open the input file
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( line, MTX_MAX_READFROMFILE_BUFFER, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( line );
#else
    if( sprintf( line, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( line );
#endif
    return FALSE;
  }

  // get the first line of data (or the comment line)
  if( !MTX_static_get_next_valid_data_line( in, line, &line_length, &atEOF ) )
  {
    MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
    return FALSE;
  }
  if( atEOF )
  {
    MTX_ERROR_MSG( "Unexpected end of file." );
    return FALSE;
  }

  // check is this is a binary compressed matrix
  if( strcmp( line, MTX_ID_COMPRESSED_01 ) == 0 )
  {
    *delimiter = 'b'; // binary
    fclose( in );
    return TRUE;
  }

  // check is this is a legacy binary compressed matrix
  if( strcmp( line, MTX_ID_LEGACY_V01 ) == 0 )
  {
    *delimiter = 'b'; // binary
    fclose( in );
    return TRUE;
  }

  // check is this is a legacy binary compressed matrix
  if( strcmp( line, MTX_ID_LEGACY_V02 ) == 0 )
  {
    *delimiter = 'b'; // binary
    fclose( in );
    return TRUE;
  }

  // check the first line with isalpha (the first line might be the comment line
  linebuf = (char *)line;
  for( i = 0; i < line_length; i++ )
  {
    if( isalpha( line[i] ) )
    {
      if( line[i] == 'e' || line[i] == 'E' || line[i] == '-' || line[i] == '+' || line[i] == '.' || line[i] == 'i' || line[i] == 'j' )
        continue;

      // first line is likely a comment line, store this for decoding later
      *hasComment = TRUE;

      // allocate the comment line
      (*comment) = (char*)malloc( (line_length+1)*sizeof(char) );
      if( !(*comment) )
      {
        // memory allocation failure
        fclose(in);
        MTX_ERROR_MSG( "malloc returned NULL." );
        return FALSE;
      }
#ifndef _CRT_SECURE_NO_DEPRECATE
      if( strcpy_s( *comment, line_length+1, line ) != 0 )
      {
        MTX_ERROR_MSG( "strcpy_s returned failure." );
        return FALSE;
      }
#else
      strcpy( *comment, line );

#endif

      if( !MTX_static_get_next_valid_data_line( in, line, &line_length, &atEOF ) )
      {
        MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
        return FALSE;
      }
      if( atEOF )
      {
        MTX_ERROR_MSG( "Unexpected end of file." );
        return FALSE;
      }

      linebuf = (char *)line;
      break;
    }
  }
  fclose( in );

  // default is whitespace
  *delimiter = 'w';

  if( strstr( linebuf, "," ) ) { *delimiter = ','; return TRUE; }
  if( strstr( linebuf, ":" ) ) { *delimiter = ':'; return TRUE; }
  if( strstr( linebuf, ";" ) ) { *delimiter = ';'; return TRUE; }
  if( strstr( linebuf, "|" ) ) { *delimiter = '|'; return TRUE; }
  if( strstr( linebuf, "`" ) ) { *delimiter = '`'; return TRUE; }
  if( strstr( linebuf, "~" ) ) { *delimiter = '~'; return TRUE; }
  if( strstr( linebuf, "!" ) ) { *delimiter = '!'; return TRUE; }
  if( strstr( linebuf, "@" ) ) { *delimiter = '@'; return TRUE; }
  if( strstr( linebuf, "#" ) ) { *delimiter = '#'; return TRUE; }
  if( strstr( linebuf, "$" ) ) { *delimiter = '$'; return TRUE; }
  if( strstr( linebuf, "%%" ) ) { *delimiter = '%'; return TRUE; }
  if( strstr( linebuf, "^" ) ) { *delimiter = '^'; return TRUE; }
  if( strstr( linebuf, "&" ) ) { *delimiter = '&'; return TRUE; }
  if( strstr( linebuf, "*" ) ) { *delimiter = '*'; return TRUE; }
  if( strstr( linebuf, "(" ) ) { *delimiter = '('; return TRUE; }
  if( strstr( linebuf, ")" ) ) { *delimiter = ')'; return TRUE; }
  if( strstr( linebuf, "_" ) ) { *delimiter = '_'; return TRUE; }
  if( strstr( linebuf, "=" ) ) { *delimiter = '='; return TRUE; }
  if( strstr( linebuf, "{" ) ) { *delimiter = '{'; return TRUE; }
  if( strstr( linebuf, "}" ) ) { *delimiter = '}'; return TRUE; }
  if( strstr( linebuf, "[" ) ) { *delimiter = '['; return TRUE; }
  if( strstr( linebuf, "]" ) ) { *delimiter = ']'; return TRUE; }
  if( strstr( linebuf, "\\" ) ) { *delimiter = '\\'; return TRUE; }
  if( strstr( linebuf, "\'" ) ) { *delimiter = '\''; return TRUE; }
  if( strstr( linebuf, "<" ) ) { *delimiter = '<'; return TRUE; }
  if( strstr( linebuf, "<" ) ) { *delimiter = '<'; return TRUE; }
  if( strstr( linebuf, ">" ) ) { *delimiter = '>'; return TRUE; }
  if( strstr( linebuf, "?" ) ) { *delimiter = '?'; return TRUE; }
  if( strstr( linebuf, "/" ) ) { *delimiter = '/'; return TRUE; }

  return TRUE;
}

// determine the size of the file indicated
BOOL MTX_DetermineFileSize( const char *path, unsigned *size )
{
  FILE* in;
  unsigned fstart;
  unsigned fend;
  char msg[512];

  *size = 0;

  // check path
  if( !path )
  {
    MTX_ERROR_MSG( "path is NULL." );
    return FALSE;
  }

  // open the file and check path exists
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( msg, 512, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( msg );
#else
    if( sprintf( msg, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( msg );
#endif
    return FALSE;
  }

  fstart = ftell( in );
  fseek( in, 0, SEEK_END );
  fend = ftell( in );

  *size = fend - fstart;
  fclose(in);

  return TRUE;
}


BOOL MTX_DetermineNumberOfColumnsInDataString( const char *datastr, unsigned *ncols )
{
  unsigned i = 0;
  unsigned line_length;
  char c;
  BOOL wasLastCharData = FALSE;
  double dtmp;
  int rv;

  if( !datastr )
  {
    MTX_ERROR_MSG( "datastr is NULL." );
    return FALSE;
  }

  line_length = (unsigned int)strlen(datastr);
  if( line_length == 0 )
  {
    MTX_ERROR_MSG( "if( line_length == 0 )" );
    return FALSE;
  }

  // intialize ncols
  *ncols = 0;

  // advance over whitespace to the first data element
  for( i = 0; i < line_length; i++ )
  {
    if( isspace(datastr[i]) )
      continue;
    else
      break;
  }
  if( i == line_length )
  {
    // no data in the string
    MTX_ERROR_MSG( "if( i == line_length ) - no data in the string." );
    return FALSE;
  }

  // determine the number of columns in the matrix
  for( i; i < line_length; i++ )
  {
    c = datastr[i];
    if( isdigit(c) || c == '.' || c == '-' || c == '+' )
    {
      if( !wasLastCharData )
      {
        // try to read in the data to be sure the element is valid
#ifndef _CRT_SECURE_NO_DEPRECATE
        rv = sscanf_s( &(datastr[i]), "%lf", &dtmp );
#else
        rv = sscanf( &(datastr[i]), "%lf", &dtmp );
#endif
        if( rv == 0 || rv == EOF )
        {
          // invalid element, file or data string is corrupt
          (*ncols) = 0;
          MTX_ERROR_MSG( "Invalid element found." );
          return FALSE;
        }
        (*ncols)++;
      }
      wasLastCharData = TRUE;
    }
    else if( c == 'e' || c == 'E' )
    {
      if( !wasLastCharData )
      {
        // the exponent should not be the leading character in
        // a data element
        (*ncols) = 0;
        MTX_ERROR_MSG( "Exponent cannot be a leading character." );
        return FALSE;
      }
      continue;
    }
    else
    {
      wasLastCharData = FALSE;
    }
  }

  return TRUE;
}




BOOL MTX_DetermineNumberOfColumnsInDataStringCplx( const char *datastr, const char delimiter, unsigned *ncols )
{
  unsigned i = 0;
  unsigned line_length;
  char c;

  BOOL isElementOnlyReal;
  BOOL isElementOnlyComplex;
  BOOL isComplexMix;

  if( !datastr )
  {
    MTX_ERROR_MSG( "datastr is NULL." );
    return FALSE;
  }

  line_length = (unsigned int)strlen(datastr);
  if( line_length == 0 )
  {
    MTX_ERROR_MSG( "if( line_length == 0 )" );
    return FALSE;
  }

  // intialize ncols
  *ncols = 0;

  // advance over whitespace to the first data element
  for( i = 0; i < line_length; i++ )
  {
    if( isspace(datastr[i]) )
      continue;
    else
      break;
  }
  if( i == line_length )
  {
    // no data in the string
    MTX_ERROR_MSG( "if( i == line_length ) - No data in the string." );
    return FALSE;
  }

  isElementOnlyComplex = FALSE;
  isElementOnlyReal = FALSE;
  isComplexMix = FALSE;

  for( i; i < line_length; i++ )
  {
    c = datastr[i];

    // looking for a digit
    if( isdigit(c) )
    {
      i++;
      // actually found a value
      // now search for an imag component
      for( i; i < line_length; i++ )
      {
        c = datastr[i];
        if( c == 'i' || c == 'j' )
        {
          // found one of bi, -bi, +bi, a+bi or a-bi element
          isComplexMix = TRUE;
          break;
        }
        else if( delimiter == 'w' )
        {
          if( isspace(c) )
          {
            // we have reached the next delimiter and no imag component was found
            isElementOnlyReal = TRUE;
            break;
          }
        }
        else if( c == delimiter )
        {
          // we have reached the next delimiter and no imag component was found
          isElementOnlyReal = TRUE;
          break;
        }
        // continue searching
      }

      (*ncols)++;
      isComplexMix = FALSE;
      isElementOnlyReal = FALSE;
    }
    else
    {
      if( c == 'i' || c == 'j' )
      {
        isElementOnlyComplex = TRUE;
        (*ncols)++;
        isElementOnlyComplex = FALSE;
      }
    }
  }

  return TRUE;
}



BOOL MTX_static_extract_cplx_from_string_with_leading_digit(
  char *datastr, //!< The entire input data string.
  const unsigned indexS, //!< The start index of the complex element.
  const unsigned indexE, //!< The inclusive end index of the complex element.
  double *re, //!< The extracted real component.
  double *im //!< The extracted imag component.
  )
{
  unsigned i = 0;
  int rv;
  unsigned len = indexE - indexS + 1;
  char str[128];

  for( i = 0; i < len && i < 128; i++ )
  {
    str[i] = datastr[indexS+i];
    if( i != 0 )
    {
      if( (str[i-1] == '+' || str[i-1] == '-') && (str[i] == 'i' || str[i] == 'j') )
      {
        str[i] = '1';
      }
    }
  }
  str[i] = '\0';

  // try to read two value
#ifndef _CRT_SECURE_NO_DEPRECATE
  rv = sscanf_s( str, "%lf%lf", re, im );
#else
  rv = sscanf( str, "%lf%lf", re, im );
#endif
  if( rv == 2 )
  {
    return TRUE;
  }
  else if( rv == 1 )
  {
    *im = *re;
    *re = 0;
    return TRUE;
  }
  else
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    MTX_ERROR_MSG( "sscanf_s returned failure." );
#else
    MTX_ERROR_MSG( "sscanf returned failure." );
#endif
    return FALSE;
  }
}


BOOL MTX_static_extract_real_into_cplx_from_string(
  char *datastr, //!< The entire input data string.
  const unsigned indexS, //!< The start index of the complex element.
  double *re, //!< The extracted real component.
  double *im //!< The extracted imag component.
  )
{
  int rv;

  // try to read two value
#ifndef _CRT_SECURE_NO_DEPRECATE
  rv = sscanf_s( &(datastr[indexS]), "%lf", re );
#else
  rv = sscanf( &(datastr[indexS]), "%lf", re );
#endif
  if( rv == 1 )
  {
    *im = 0;
    return TRUE;
  }
  else
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    MTX_ERROR_MSG( "sscanf_s returned failure." );
#else
    MTX_ERROR_MSG( "sscanf returned failure." );
#endif
    return FALSE;
  }
}




BOOL MTX_static_get_row_array_from_string_cplx( char *datastr, const char delimiter, _MTX_STRUCT_ReadFromFileListElem *L, const unsigned ncols )
{
  unsigned i = 0;
  int j = 0;
  unsigned line_length;
  char c;

  unsigned indexS; // index of the start of an element
  unsigned indexE; // index of the end of an element

  BOOL isElementOnlyReal;
  BOOL isElementOnlyComplex;
  BOOL isComplexMix;

  unsigned n = 0;

  if( !datastr )
  {
    MTX_ERROR_MSG( "datastr is NULL." );
    return FALSE;
  }

  line_length = (unsigned int)strlen(datastr);
  if( line_length == 0 )
  {
    MTX_ERROR_MSG( "if( line_length == 0 )" );
    return FALSE;
  }

  // advance over whitespace to the first data element
  for( i = 0; i < line_length; i++ )
  {
    if( isspace(datastr[i]) )
      continue;
    else
      break;
  }
  if( i == line_length )
  {
    // no data in the string
    MTX_ERROR_MSG( "if( i == line_length ) - No data in the string." );
    return FALSE;
  }

  isElementOnlyComplex = FALSE;
  isElementOnlyReal = FALSE;
  isComplexMix = FALSE;

  for( i; i < line_length; i++ )
  {
    c = datastr[i];

    // looking for a digit
    if( isdigit(c) || ((c == '+' || c == '-') && isdigit(datastr[i+1]) ) )
    {
      indexS = i;
      i++;
      // actually found a value
      // now search for an imag component
      for( i; i <= line_length; i++ ) // notice the <= (this algorithm is made easier by allowing this)
      {
        c = datastr[i];
        if( c == 'i' || c == 'j' )
        {
          // found one of bi, -bi, +bi, a+i, a-i, a+bi or a-bi element (or with 'j')
          indexE = i;
          isComplexMix = TRUE;

          // don't allow invalid indexing
          if( n < ncols )
          {
            if( !MTX_static_extract_cplx_from_string_with_leading_digit(
              datastr,
              indexS,
              indexE,
              &(L->rowptr_cplx[n].re),
              &(L->rowptr_cplx[n].im) ) )
            {
              MTX_ERROR_MSG( "MTX_static_extract_cplx_from_string_with_leading_digit returned FALSE." );
              return FALSE;
            }
          }

          break;
        }
        else if( delimiter == 'w' )
        {
          if( isspace(c) || c == 0 )
          {
            // we have reached the next delimiter and no imag component was found
            isElementOnlyReal = TRUE;

            // don't allow invalid indexing
            if( n < ncols )
            {
              if( !MTX_static_extract_real_into_cplx_from_string(
                datastr,
                indexS,
                &(L->rowptr_cplx[n].re),
                &(L->rowptr_cplx[n].im) ) )
              {
                MTX_ERROR_MSG( "MTX_static_extract_real_into_cplx_from_string returned FALSE." );
                return FALSE;
              }
            }
            break;
          }
        }
        else if( c == delimiter || c == 0 )
        {
          // we have reached the next delimiter and no imag component was found
          isElementOnlyReal = TRUE;

          // don't allow invalid indexing
          if( n < ncols )
          {

            if( !MTX_static_extract_real_into_cplx_from_string(
              datastr,
              indexS,
              &(L->rowptr_cplx[n].re),
              &(L->rowptr_cplx[n].im) ) )
            {
              MTX_ERROR_MSG( "MTX_static_extract_real_into_cplx_from_string returned FALSE." );
              return FALSE;
            }
          }
          break;
        }
        // continue searching
      }

      n++;
      if( n > ncols )
        break; // no sense in continuing
      isComplexMix = FALSE;
      isElementOnlyReal = FALSE;
    }
    else
    {
      if( c == 'i' || c == 'j' )
      {
        isElementOnlyComplex = TRUE;

        // don't allow invalid indexing
        if( n < ncols )
        {
          // figure if it is signed
          L->rowptr_cplx[n].re = 0;
          L->rowptr_cplx[n].im = 1.0;
          j = i-1;
          while( j >= 0 )
          {
            c = datastr[j];
            if( c == '-' )
            {
              L->rowptr_cplx[n].im = -1.0;
              break;
            }
            else if( c == '+' )
            {
              break;
            }
            else if( c == 'i' || c == 'j' )
            {
              break;
            }
            else if( isdigit(c) )
            {
              break;
            }
            j--;
          }
        }
        n++;
        if( n > ncols )
          break; // no sense in continuing
        isElementOnlyComplex = FALSE;
      }
    }
  }

  if( n != ncols )
  {
    MTX_ERROR_MSG( "if( n != ncols )" );
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}


BOOL MTX_static_get_row_array_from_string( char *datastr, _MTX_STRUCT_ReadFromFileListElem *L, const unsigned ncols )
{
  unsigned i = 0;
  unsigned n; // number of columns read successfully
  unsigned line_length;
  char c;
  BOOL wasLastCharData = FALSE;
  int rv;

  if( !datastr )
  {
    MTX_ERROR_MSG( "datastr is NULL." );
    return FALSE;
  }

  if( !L )
  {
    MTX_ERROR_MSG( "L is NULL." );
    return FALSE;
  }

  if( !L->rowptr )
  {
    MTX_ERROR_MSG( "L->rowptr is NULL." );
    return FALSE;
  }

  line_length = (unsigned int)strlen(datastr);
  if( line_length == 0 )
  {
    MTX_ERROR_MSG( "if( line_length == 0 )" );
    return FALSE;
  }

  // advance over whitespace to the first data element
  for( i = 0; i < line_length; i++ )
  {
    if( isspace(datastr[i]) )
      continue;
    else
      break;
  }
  if( i == line_length )
  {
    // no data in the string
    MTX_ERROR_MSG( "if( i == line_length ) - No data in the string." );
    return FALSE;
  }

  // read in one row of data
  n = 0;
  for( i; i < line_length; i++ )
  {
    c = datastr[i];
    if( isdigit(c) || c == '.' || c == '-' || c == '+' )
    {
      if( !wasLastCharData )
      {
        // try to read in the data to be sure the element is valid
#ifndef _CRT_SECURE_NO_DEPRECATE
        rv = sscanf_s( &(datastr[i]), "%lf", &(L->rowptr[n]) );
#else
        rv = sscanf( &(datastr[i]), "%lf", &(L->rowptr[n]) );
#endif
        if( rv == 0 || rv == EOF )
        {
          // invalid element, file is corrupt
          MTX_ERROR_MSG( "Invalid element found." );
          return FALSE;
        }
        n++;
        if( n == ncols )
          break;
      }
      wasLastCharData = TRUE;
    }
    else if( c == 'e' || c == 'E' )
    {
      if( !wasLastCharData )
      {
        // the exponent should not be the leading character in
        // a data element
        MTX_ERROR_MSG( "Invalid element found. Exponent cannot be leading character." );
        return FALSE;
      }
      continue;
    }
    else
    {
      wasLastCharData = FALSE;
    }
  }

  if( n != ncols )
  {
    // the number of columns does not match
    MTX_ERROR_MSG( "if( n != ncols ) - The number of columns do not match." );
    return FALSE;
  }

  return TRUE;
}


BOOL MTX_static_get_next_valid_data_line(
  FILE *in, //!< The input file pointer (input).
  char *linebuf, //!< A exisiting buffer to store the input line (input/output).
  unsigned *line_length, //!< The length of the line read (output).
  BOOL *atEOF //!< A boolean to indicate if EOF has been reached.
  )
{
  unsigned i = 0;
  i = 0;
  *line_length = 0;
  while( i == *line_length )
  {
    if( fgets( linebuf, MTX_MAX_READFROMFILE_BUFFER, in ) == NULL )
    {
      *atEOF = TRUE;
      if( feof(in) )
      {
        // reached the end of the file properly
        fclose(in);
        return TRUE;
      }
      else if( ferror(in) != 0 )
      {
        // error in reading the datafile
        fclose(in);
        MTX_ERROR_MSG( "Error reading in the data." );
        return FALSE;
      }
      else
      {
        // error in reading the datafile
        fclose(in);
        MTX_ERROR_MSG( "Error reading in the data." );
        return FALSE;
      }
    }

    *line_length = (unsigned int)strlen(linebuf);
    for( i = 0; i < *line_length; i++ )
    {
      if( !isspace(linebuf[i]) )
        break;
    }
  }

  // provide some buffer room, in case of overreading the linebuffer.
  if( *line_length < MTX_MAX_READFROMFILE_BUFFER-2 )
  {
    linebuf[*line_length] = '\0';
    linebuf[(*line_length)+1] = '\0';
  }

  return TRUE;
}



BOOL MTX_static_get_next_valid_data_line_from_matrix_string(
  const char *strMatrix, //!< The matrix string pointer (input).
  const unsigned strLength, //!< The length of the matrix string (input).
  unsigned *index, //!< The starting/(next line) index into the matrix string pointer (input/output).
  char *linebuf, //!< A exisiting buffer to store the input line (input/output).
  unsigned *line_length, //!< The length of the line read (output).
  BOOL *atEndOfString //!< A boolean to indicate if the end of the strMatrix string has been reached.
  )
{
  unsigned i = 0;
  unsigned j = 0;
  *line_length = 0;
  *atEndOfString = FALSE;

  // sanity checks
  if( strMatrix == NULL )
  {
    MTX_ERROR_MSG( "strMatrix is NULL." );
    return FALSE;
  }
  if( index == NULL )
  {
    MTX_ERROR_MSG( "index is NULL." );
    return FALSE;
  }
  if( *index == strLength )
  {
    *atEndOfString = TRUE;
    return TRUE;
  }
  if( *index > strLength )
  {
    MTX_ERROR_MSG( "if( *index > strLength )" );
    return FALSE;
  }

  while( i == *line_length )
  {
    *line_length = 0;
    for( j = *index; j < strLength; j++ )
    {
      linebuf[j-(*index)] = strMatrix[j];
      *line_length = *line_length + 1;
      if( strMatrix[j] == '\n' )
        break;
    }
    linebuf[*line_length] = '\0';

    if( *line_length == 0 )
    {
      *atEndOfString = TRUE;
      *index = strLength;
      break;
    }

    for( i = 0; i < *line_length; i++ )
    {
      if( !isspace(linebuf[i]) )
        break;
    }
    *index += *line_length;
  }

  // provide some buffer room, in case of overreading the linebuffer.
  if( *line_length < MTX_MAX_READFROMFILE_BUFFER-2 )
  {
    linebuf[*line_length] = '\0';
    linebuf[(*line_length)+1] = '\0';
  }

  return TRUE;
}

BOOL MTX_static_look_for_complex_data(
                                      char *linebuf, //!< A string containing a line of data (input).
                                      const unsigned line_length, //!< The length of the string (input).
                                      BOOL *hasComplex //!< A boolean indicating if there is any complex data (output).
                                      )
{
  unsigned i;

  *hasComplex = FALSE;
  if( linebuf == NULL )
  {
    MTX_ERROR_MSG( "linebuf is NULL." );
    return FALSE;
  }

  for( i = 0; i < line_length; i++ )
  {
    if( linebuf[i] == 'i' || linebuf[i] == 'j' )
    {
      *hasComplex = TRUE;
      break;
    }
  }
  return TRUE;
}





// Reads in the matrix M->data from the specified file using the indicated *delimiter
// ReadFromFile is 'read smart' (it determines the size of the input matrix on its own)
// The number of columns are first determined then all M->data is read into linked lists
// untill end of file is reached. Data is then stored in the matrix.
BOOL MTX_ReadFromFileRealOnly( MTX *M, const char *path )
{
  unsigned i = 0;
  FILE *in = NULL;
  char delimiter = 0;
  char linebuf[MTX_MAX_READFROMFILE_BUFFER];
  unsigned ncols = 0;
  unsigned nrows = 0;
  unsigned fsize = 0;
  unsigned line_length = 0;
  BOOL hasCommentLine = FALSE;
  BOOL errorInReadingDataFile = FALSE;
  BOOL errorInTranspose = FALSE;
  BOOL atEOF = FALSE;
  MTX RowMatrix;

  // a linked list of row arrays
  _MTX_STRUCT_ReadFromFileListElem *L = NULL;
  _MTX_STRUCT_ReadFromFileListElem *nL = NULL;
  _MTX_STRUCT_ReadFromFileListElem head;
  head.next = NULL;
  head.rowptr = NULL;

  if( M == NULL )
  {
    MTX_ERROR_MSG( "M is NULL." );
    return FALSE;
  }

  // check path
  if( !path )
  {
    MTX_ERROR_MSG( "path is NULL." );
    return FALSE;
  }

  // check path exists
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( linebuf, MTX_MAX_READFROMFILE_BUFFER, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#else
    if( sprintf( linebuf, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#endif
    return FALSE;
  }
  fclose(in);

  MTX_Init( &RowMatrix );

  // determine the file delimiter
  if( MTX_DetermineFileDelimiter( path, &delimiter, &hasCommentLine, &(M->comment) ) == FALSE )
  {
    MTX_ERROR_MSG( "MTX_DetermineFileDelimiter returned FALSE." );
    return FALSE;
  }

  // check if this is a binary compressed matrix
  if( delimiter == 'b' )
  {
    if( MTX_ReadCompressed( M, path ) )
    {
      return TRUE;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ReadCompressed returned FALSE." );
      return FALSE;
    }
  }

  // determine the size of the file
  if( MTX_DetermineFileSize( path, &fsize ) == FALSE )
  {
    MTX_ERROR_MSG( "MTX_DetermineFileSize returned FALSE." );
    return FALSE;
  }

  // open the input file for full input operations
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( linebuf, MTX_MAX_READFROMFILE_BUFFER, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#else
    if( sprintf( linebuf, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#endif
    return FALSE;
  }

  // advance over whitespace lines at the beginning of the file
  if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
  {
    MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
    return FALSE;
  }
  if( atEOF )
  {
    MTX_ERROR_MSG( "Unexpected end of file." );
    return FALSE;
  }

  if( hasCommentLine )
  {
    if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
    {
      MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
      return FALSE;
    }
    if( atEOF )
    {
      MTX_ERROR_MSG( "Unexpected end of file." );
      return FALSE;
    }
  }

  // determine the number of columns in the first line of data
  if( MTX_DetermineNumberOfColumnsInDataString( linebuf, &ncols ) == FALSE )
  {
    MTX_ERROR_MSG( "MTX_DetermineNumberOfColumnsInDataString returned FALSE." );
    return FALSE;
  }


  // super fast rowwise input routine
  // a rowwise matrix is constructed using a linked list approach
  // line by line input
  nrows = 0;
  head.rowptr = (double*)malloc( ncols*sizeof(double) );
  // get the row data from the string and store it in the list item row array
  if( MTX_static_get_row_array_from_string( linebuf, &head, ncols ) == FALSE )
  {
    // must free head's rowarray
    free( head.rowptr );
    MTX_ERROR_MSG( "MTX_static_get_row_array_from_string returned FALSE." );
    return FALSE;
  }
  nrows++;
  nL = &head;

  while(1)
  {
    // get the next string of data
    if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
    {
      errorInReadingDataFile = TRUE;
      break;
    }
    if( atEOF )
    {
      break;
    }

    // the 'current' list itme
    L = nL;

    // allocate the next list item
    nL = (_MTX_STRUCT_ReadFromFileListElem*)malloc( sizeof( _MTX_STRUCT_ReadFromFileListElem ) );
    if( !nL )
    {
      // memory allocate failure
      // must free the linked list
      errorInReadingDataFile = TRUE;
      // must free head's rowarray
      free( head.rowptr );
      MTX_ERROR_MSG( "if( !nL )" );
      return FALSE;
    }
    // intialize the row
    nL->rowptr = NULL;
    nL->next = NULL;
    // allocate the row
    nL->rowptr = (double*)malloc( ncols*sizeof(double) );
    if( !nL->rowptr )
    {
      // memory allocate failure
      // must free the linked list
      errorInReadingDataFile = TRUE;
      break;
    }

    // store the pointer to the next list item
    // in the 'current'
    L->next = nL;

    // get the row data from the string and store it in the list item row array
    if( MTX_static_get_row_array_from_string( linebuf, nL, ncols ) == FALSE )
    {
      // must free the linked list
      errorInReadingDataFile = TRUE;
      break;
    }
    nrows++;
  }

  if( errorInReadingDataFile )
  {
    // free the list
    L = head.next;
    while( L!=NULL )
    {
      nL = L->next;
      free( L->rowptr );
      free(L);
      L = nL;
    }
    free( head.rowptr );

    MTX_ERROR_MSG( "if( errorInReadingDataFile )" );
    return FALSE;
  }

  // copy the list into a MTX object

  // allocate the row array of pointers
  RowMatrix.data = (double**)malloc( nrows*sizeof(double*) );
  if( !RowMatrix.data )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

  // looks weird but this is rowwise input
  RowMatrix.ncols = nrows;
  RowMatrix.nrows = ncols;

  L = &head;
  for( i = 0; i < nrows; i++ )
  {
    if( L == NULL )
    {
      // this should never happen
      free(RowMatrix.data);
      RowMatrix.data = NULL;
      MTX_ERROR_MSG( "if( L == NULL )" );
      return FALSE;
    }
    RowMatrix.data[i] = L->rowptr; // only copying a pointer!
    L = L->next;
  }

  // copy the data by means of transpose
  // this places the data in the correct MTX storage format
  // only one copy operation
  if( !MTX_Transpose( &RowMatrix, M ) )
  {
    errorInTranspose = TRUE;
  }

  // free the list data and the list items
  L = head.next;
  while( L!=NULL )
  {
    nL = L->next;
    free( L->rowptr );
    free(L);
    L = nL;
  }
  // free the head data
  free( head.rowptr );

  // free the RowMatrix data ptr
  free( RowMatrix.data );

  if( errorInTranspose )
  {
    MTX_ERROR_MSG( "if( errorInTranspose )" );
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

// Reads in the matrix (real or complex) from the specified file using the indicated *delimiter
// ReadFromFile is 'read smart' (it determines the size of the input matrix on its own)
// The number of columns are first determined then all data is read into linked lists
// untill end of file is reached. Data is then stored in the matrix. The input is treated as
// complex.
BOOL MTX_ReadFromFile( MTX *M, const char *path )
{
  unsigned i;
  FILE *in = NULL; // The input file pointer.
  char delimiter = 0; // The file delimiter ('b' is a binary file, 'w' is whitespace delimited).
  char linebuf[MTX_MAX_READFROMFILE_BUFFER]; // A fairly large line buffer.
  unsigned ncols = 0; // The number of columns determined in the matrix.
  unsigned nrows = 0; // The number of rows determined in the matrix.
  unsigned fsize = 0; // The size of the input file [bytes].
  unsigned line_length = 0; // The length of the linebuf buffer.
  BOOL hasCommentLine = FALSE; // A boolean to indicate if the file has a comment line for the first line.
  BOOL errorInReadingDataFile = FALSE; // A boolean to indicate an error in reading the data file.
  BOOL errorInTranspose = FALSE; // A boolean to indicate an error in the transpose operation at the end.
  BOOL atEOF = FALSE; // A boolean to indicate when EOF is reached.
  BOOL isReal = TRUE; // A boolean to indicate if the matrix is real or complex.
  BOOL complexDetected = FALSE; // A boolean to indicate that complex data has been detected.
  BOOL needToConvertRealToComplex = FALSE; // A boolean to indicate if the matrix was real and needs to be converted to complex.
  MTX RowMatrix; // A rowwise storage version of the matrix. The transpose of this matrix is the final result.
  char *commentLine = NULL; // The comment line if one is found.

  // A linked list of row arrays (of either real or complex data).
  _MTX_STRUCT_ReadFromFileListElem *L = NULL; // The current list item.
  _MTX_STRUCT_ReadFromFileListElem *nL = NULL; // The next list item.
  _MTX_STRUCT_ReadFromFileListElem head; // The head of the list.
  head.next = NULL;
  head.rowptr = NULL;

  // check path
  if( !path )
  {
    MTX_ERROR_MSG( "path is NULL." );
    return FALSE;
  }

  // check path exists
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( linebuf, MTX_MAX_READFROMFILE_BUFFER, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#else
    if( sprintf( linebuf, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#endif
    return FALSE;
  }
  fclose(in);

  // initialize the single row container
  MTX_Init( &RowMatrix );

  // determine the file delimiter
  if( MTX_DetermineFileDelimiter( path, &delimiter, &hasCommentLine, &commentLine ) == FALSE )
  {
    MTX_ERROR_MSG( "MTX_DetermineFileDelimiter returned FALSE." );
    return FALSE;
  }

  // check if this is a binary compressed matrix
  if( delimiter == 'b' )
  {
    // fill in later
    if( MTX_ReadCompressed( M, path ) )
    {
      return TRUE;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ReadCompressed returned FALSE." );
      return FALSE;
    }
  }

  // determine the size of the file
  if( MTX_DetermineFileSize( path, &fsize ) == FALSE )
  {
    MTX_ERROR_MSG( "MTX_DetermineFileSize returned FALSE." );
    return FALSE;
  }

  // open the input file for full input operations
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &in, path, "r" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned failure." );
    return FALSE;
  }
#else
  in = fopen( path, "r" );
#endif
  if( !in )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( linebuf, MTX_MAX_READFROMFILE_BUFFER, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#else
    if( sprintf( linebuf, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( linebuf );
#endif
    return FALSE;
  }

  // advance over whitespace lines at the beginning of the file
  if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
  {
    MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
    return FALSE;
  }
  if( atEOF )
  {
    MTX_ERROR_MSG( "Unexpected end of file." );
    return FALSE;
  }

  if( hasCommentLine )
  {
    if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
    {
      MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line returned FALSE." );
      return FALSE;
    }
    if( atEOF )
    {
      MTX_ERROR_MSG( "Unexpected end of file." );
      return FALSE;
    }
  }

  // check the first line for complex data
  if( !MTX_static_look_for_complex_data( linebuf, line_length, &complexDetected ) )
  {
    MTX_ERROR_MSG( "MTX_static_look_for_complex_data returned FALSE." );
    return FALSE;
  }
  if( complexDetected )
  {
    needToConvertRealToComplex = FALSE; // no need to convert, the entire matrix will be read in as complex.
    isReal = FALSE;
    // no need to convert anything though
  }

  // determine the number of columns in the matrix
  if( isReal )
  {
    if( !MTX_DetermineNumberOfColumnsInDataString( linebuf, &ncols ) )
    {
      MTX_ERROR_MSG( "MTX_DetermineNumberOfColumnsInDataString returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_DetermineNumberOfColumnsInDataStringCplx( linebuf, delimiter, &ncols ) )
    {
      MTX_ERROR_MSG( "MTX_DetermineNumberOfColumnsInDataStringCplx returned FALSE." );
      return FALSE;
    }
  }


  // super fast rowwise input routine
  // a rowwise matrix is constructed using a linked list approach
  // line by line input.
  // The matrix can even be entirely real for all but the last line.
  // The matrix will treat it's input as real until complex data is
  // detected. The ensuing data will all be read as complex. Once
  // all data is read in, any initial real row arrays are converted
  // into complex data. In this was all real matrices are very efficiently
  // read and so are complex matrices. The file is only read once.
  nrows = 0;
  if( isReal )
  {
    head.isReal = TRUE;
    head.rowptr = (double*)malloc( ncols*sizeof(double) );

    // get the row data from the string and store it in the list item row array
    if( !MTX_static_get_row_array_from_string( linebuf, &head, ncols ) )
    {
      // must free head's rowarray
      free( head.rowptr );
      MTX_ERROR_MSG( "MTX_static_get_row_array_from_string returned false." );
      return FALSE;
    }
  }
  else
  {
    head.isReal = FALSE;
    head.rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );

    // get the row data from the string and store it in the list item row array
    if( MTX_static_get_row_array_from_string_cplx( linebuf, delimiter, &head, ncols ) == FALSE )
    {
      // must free head's rowarray
      free( head.rowptr_cplx );
      MTX_ERROR_MSG( "MTX_static_get_row_array_from_string_cplx returned false." );
      return FALSE;
    }
  }
  nrows++;
  nL = &head;

  while(1)
  {
    // get the next string of data
    if( !MTX_static_get_next_valid_data_line( in, linebuf, &line_length, &atEOF ) )
    {
      errorInReadingDataFile = TRUE;
      break;
    }
    if( atEOF )
    {
      break;
    }

    // If the matrix is currently real, check this line for complex data.
    if( isReal )
    {
      if( !MTX_static_look_for_complex_data( linebuf, line_length, &complexDetected ) )
      {
        MTX_ERROR_MSG( "MTX_static_look_for_complex_data returned FALSE." );
        return FALSE;
      }

      if( complexDetected )
      {
        isReal = FALSE;
        needToConvertRealToComplex = TRUE; // there is mixed real rows and complex rows, so conversion is needed.
      }
    }

    // the 'current' list itme
    L = nL;

    // allocate the next list item
    nL = (_MTX_STRUCT_ReadFromFileListElem*)malloc( sizeof( _MTX_STRUCT_ReadFromFileListElem ) );
    if( !nL )
    {
      // memory allocate failure
      // must free the linked list
      errorInReadingDataFile = TRUE;
      MTX_ERROR_MSG( "if( !nL )" );
      return FALSE;
    }

    // intialize the row
    nL->isReal = isReal;
    nL->rowptr = NULL;
    nL->rowptr_cplx = NULL;
    nL->next = NULL;

    // allocate the row
    if( isReal )
    {
      nL->rowptr = (double*)malloc( ncols*sizeof(double) );
      if( !nL->rowptr )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingDataFile = TRUE;
        break;
      }
    }
    else
    {
      nL->rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );
      if( !nL->rowptr_cplx )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingDataFile = TRUE;
        break;
      }
    }

    // store the pointer to the next list item
    // in the 'current'
    L->next = nL;

    if( isReal )
    {
      // get the row data from the string and store it in the list item row array
      if( MTX_static_get_row_array_from_string( linebuf, nL, ncols ) == FALSE )
      {
        // must free the linked list
        errorInReadingDataFile = TRUE;
        break;
      }
    }
    else
    {
      // get the row data from the string and store it in the list item row array
      if( MTX_static_get_row_array_from_string_cplx( linebuf, delimiter, nL, ncols ) == FALSE )
      {
        // must free the linked list
        errorInReadingDataFile = TRUE;
        break;
      }
    }
    nrows++;
  }

  if( errorInReadingDataFile )
  {
    // free the list
    L = head.next;
    while( L!=NULL )
    {
      nL = L->next;
      if( L->isReal )
      {
        free( L->rowptr );
      }
      else
      {
        free( L->rowptr_cplx );
      }
      free(L);
      L = nL;
    }
    if( head.isReal )
    {
      free( head.rowptr );
    }
    else
    {
      free( head.rowptr_cplx );
    }

    MTX_ERROR_MSG( "if( errorInReadingDataFile )" );
    return FALSE;
  }


  // If there are mixed real and complex rows, the real rows must be converted to complex.
  if( needToConvertRealToComplex )
  {
    L = &head;
    // go through the linked list until the data changes from real to complex

    while( L->isReal )
    {
      L->rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );
      if( !L->rowptr_cplx )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingDataFile = TRUE;
        break;
      }

      // copy the data from the real row vector to the complex row vector
      for( i = 0; i < ncols; i++ )
      {
        L->rowptr_cplx[i].re = L->rowptr[i];
        L->rowptr_cplx[i].im = 0.0;
      }

      // free the real vector
      if( L->isReal )
      {
        free( L->rowptr );
      }
      L->rowptr = NULL;

      // move to the next item
      L = L->next;

      // break at the end of the list, this shouldn't ever happen though.
      if( L == NULL )
        break;
    }
  }



  /////
  // copy the list into a MTX object

  // first allocate the row array of pointers
  if( isReal )
  {
    RowMatrix.isReal = TRUE;
    RowMatrix.data = (double**)malloc( nrows*sizeof(double*) );
    if( !RowMatrix.data )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    RowMatrix.isReal = FALSE;
    RowMatrix.cplx = (stComplex**)malloc( nrows*sizeof(stComplex*) );
    if( !RowMatrix.cplx )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  // looks weird but this is rowwise input
  RowMatrix.ncols = nrows;
  RowMatrix.nrows = ncols;

  L = &head;
  for( i = 0; i < nrows; i++ )
  {
    if( L == NULL )
    {
      // this should never happen
      MTX_ERROR_MSG( "if( L == NULL ) - This should never happen." );
      return FALSE;
    }
    if( isReal )
    {
      RowMatrix.data[i] = L->rowptr; // only copying a pointer!
    }
    else
    {
      RowMatrix.cplx[i] = L->rowptr_cplx; // only copying a pointer!
    }
    L = L->next;
  }

  // copy the data by means of transpose
  // this places the data in the correct MTX storage format
  // only one copy operation
  if( !MTX_Transpose( &RowMatrix, M ) )
  {
    errorInTranspose = TRUE;
  }

  if( commentLine != NULL )
  {
    // Set the comment line.
    M->comment = commentLine;
  }

  // free the list data and the list items
  L = head.next;
  while( L!=NULL )
  {
    nL = L->next;
    if( isReal )
    {
      free( L->rowptr );
    }
    else
    {
      free( L->rowptr_cplx );
    }
    free(L);
    L = nL;
  }
  // free the head data
  if( isReal )
  {
    free( head.rowptr );
  }
  else
  {
    free( head.rowptr_cplx );
  }

  // free the RowMatrix data ptr
  if( isReal )
  {
    free( RowMatrix.data );
  }
  else
  {
    free( RowMatrix.cplx );
  }

  if( errorInTranspose )
  {
    MTX_ERROR_MSG( "if( errorInTranspose )" );
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

// Reads in the matrix (real or complex) from the specified file using the indicated *delimiter
// ReadFromFile is 'read smart' (it determines the size of the input matrix on its own)
// The number of columns are first determined then all data is read into linked lists
// untill end of file is reached. Data is then stored in the matrix. The input is treated as
// complex.
BOOL MTX_SetFromMatrixString( MTX *M, const char *strMatrix )
{
  unsigned i;
  const char delimiter = 'w'; // The file delimiter ('w' is whitespace delimited). Comma delimters are replace with whitespace.
  char strMatrixCopy[MTX_MAX_READFROMFILE_BUFFER]; // A fairly large buffer for copying the strMatrix.
  char linebuf[MTX_MAX_READFROMFILE_BUFFER]; // A fairly large line buffer.
  unsigned ncols = 0; // The number of columns determined in the matrix.
  unsigned nrows = 0; // The number of rows determined in the matrix.
  unsigned line_length = 0; // The length of the linebuf buffer.
  unsigned strMatrixLength = 0; // The length of the input string matrix.
  unsigned strMatrixIndex = 0; // An index into the copy of the strMatrix string.
  BOOL errorInReadingData = FALSE; // A boolean to indicate an error in reading the data file.
  BOOL errorInTranspose = FALSE; // A boolean to indicate an error in the transpose operation at the end.
  BOOL atEndOfString = FALSE; // A boolean to indicate when end of string is reached.
  BOOL isReal = TRUE; // A boolean to indicate if the matrix is real or complex.
  BOOL complexDetected = FALSE; // A boolean to indicate that complex data has been detected.
  BOOL needToConvertRealToComplex = FALSE; // A boolean to indicate if the matrix was real and needs to be converted to complex.
  MTX RowMatrix; // A rowwise storage version of the matrix. The transpose of this matrix is the final result.

  // A linked list of row arrays (of either real or complex data).
  _MTX_STRUCT_ReadFromFileListElem *L = NULL; // The current list item.
  _MTX_STRUCT_ReadFromFileListElem *nL = NULL; // The next list item.
  _MTX_STRUCT_ReadFromFileListElem head; // The head of the list.
  head.next = NULL;
  head.rowptr = NULL;

  // sanity check
  if( strMatrix == NULL )
  {
    MTX_ERROR_MSG( "strMatrix is NULL." );
    return FALSE;
  }

  strMatrixLength = (unsigned int)strlen( strMatrix );
  if( strMatrixLength == 0 )
  {
    MTX_ERROR_MSG( "if( strMatrixLength == 0 )" );
    return FALSE;
  }

  // overrun check
  if( strMatrixLength+1 > MTX_MAX_READFROMFILE_BUFFER ) // plus one for the null terminator
  {
    MTX_ERROR_MSG( "if( strMatrixLength+1 > MTX_MAX_READFROMFILE_BUFFER )" );
    return FALSE;
  }

  // initialize the single row container
  MTX_Init( &RowMatrix );

  // operate on a copy of strMatrix
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( strncpy_s( strMatrixCopy, MTX_MAX_READFROMFILE_BUFFER, strMatrix, strMatrixLength ) != 0 )
  {
    MTX_ERROR_MSG( "strncpy_s returned failure." );
    return FALSE;
  }
#else
  strncpy( strMatrixCopy, strMatrix, strMatrixLength );
#endif
  strMatrixCopy[strMatrixLength] = '\0';

  // replace ;'s with the newline character and ',' with ' '
  // replace '[' and ']' with ' '
  for( i = 0; i < strMatrixLength; i++ )
  {
    if( strMatrixCopy[i] == ',' )
    {
      strMatrixCopy[i] = ' ';
    }
    else if( strMatrixCopy[i] == ';' )
    {
      strMatrixCopy[i] = '\n';
    }
    else if( strMatrixCopy[i] == '[' )
    {
      strMatrixCopy[i] = ' ';
    }
    else if( strMatrixCopy[i] == ']' )
    {
      strMatrixCopy[i] = ' ';
    }
  }

  // advance over whitespace lines at the beginning of the matrix in the string
  if( !MTX_static_get_next_valid_data_line_from_matrix_string(
    strMatrixCopy,
    strMatrixLength,
    &strMatrixIndex,
    linebuf,
    &line_length,
    &atEndOfString ) )
  {
    MTX_ERROR_MSG( "MTX_static_get_next_valid_data_line_from_matrix_string returned FALSE." );
    return FALSE;
  }
  if( atEndOfString )
  {
    MTX_ERROR_MSG( "Unexpected end of string." );
    return FALSE;
  }

  // check the first line for complex data
  if( !MTX_static_look_for_complex_data( linebuf, line_length, &complexDetected ) )
  {
    MTX_ERROR_MSG( "MTX_static_look_for_complex_data returned FALSE." );
    return FALSE;
  }
  if( complexDetected )
  {
    needToConvertRealToComplex = FALSE; // no need to convert, the entire matrix will be read in as complex.
    isReal = FALSE;
    // no need to convert anything though
  }

  // determine the number of columns in the matrix
  if( isReal )
  {
    if( !MTX_DetermineNumberOfColumnsInDataString( linebuf, &ncols ) )
    {
      MTX_ERROR_MSG( "MTX_DetermineNumberOfColumnsInDataString returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_DetermineNumberOfColumnsInDataStringCplx( linebuf, delimiter, &ncols ) )
    {
      MTX_ERROR_MSG( "MTX_DetermineNumberOfColumnsInDataStringCplx returned FALSE." );
      return FALSE;
    }
  }

  // super fast rowwise input routine
  // a rowwise matrix is constructed using a linked list approach
  // line by line input.
  // The matrix can even be entirely real for all but the last line.
  // The matrix will treat it's input as real until complex data is
  // detected. The ensuing data will all be read as complex. Once
  // all data is read in, any initial real row arrays are converted
  // into complex data. In this was all real matrices are very efficiently
  // read and so are complex matrices. The file is only read once.
  nrows = 0;
  if( isReal )
  {
    head.isReal = TRUE;
    head.rowptr = (double*)malloc( ncols*sizeof(double) );

    // get the row data from the string and store it in the list item row array
    if( !MTX_static_get_row_array_from_string( linebuf, &head, ncols ) )
    {
      // must free head's rowarray
      free( head.rowptr );
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  else
  {
    head.isReal = FALSE;
    head.rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );

    // get the row data from the string and store it in the list item row array
    if( MTX_static_get_row_array_from_string_cplx( linebuf, delimiter, &head, ncols ) == FALSE )
    {
      // must free head's rowarray
      free( head.rowptr_cplx );
      MTX_ERROR_MSG( "malloc returned NULL." );
      return FALSE;
    }
  }
  nrows++;
  nL = &head;

  while(1)
  {
    // get the next string of data
    if( !MTX_static_get_next_valid_data_line_from_matrix_string(
      strMatrixCopy,
      strMatrixLength,
      &strMatrixIndex,
      linebuf,
      &line_length,
      &atEndOfString ) )
    {
      errorInReadingData = TRUE;
      break;
    }
    if( atEndOfString )
    {
      break;
    }

    // If the matrix is currently real, check this line for complex data.
    if( isReal )
    {
      if( !MTX_static_look_for_complex_data( linebuf, line_length, &complexDetected ) )
      {
        MTX_ERROR_MSG( "MTX_static_look_for_complex_data returned FALSE." );
        return FALSE;
      }

      if( complexDetected )
      {
        isReal = FALSE;
        needToConvertRealToComplex = TRUE; // there is mixed real rows and complex rows, so conversion is needed.
      }
    }

    // the 'current' list itme
    L = nL;

    // allocate the next list item
    nL = (_MTX_STRUCT_ReadFromFileListElem*)malloc( sizeof( _MTX_STRUCT_ReadFromFileListElem ) );
    if( !nL )
    {
      // memory allocate failure
      // must free the linked list
      errorInReadingData = TRUE;
      break;
    }

    // intialize the row
    nL->isReal = isReal;
    nL->rowptr = NULL;
    nL->rowptr_cplx = NULL;
    nL->next = NULL;

    // allocate the row
    if( isReal )
    {
      nL->rowptr = (double*)malloc( ncols*sizeof(double) );
      if( !nL->rowptr )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingData = TRUE;
        break;
      }
    }
    else
    {
      nL->rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );
      if( !nL->rowptr_cplx )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingData = TRUE;
        break;
      }
    }

    // store the pointer to the next list item
    // in the 'current'
    L->next = nL;

    if( isReal )
    {
      // get the row data from the string and store it in the list item row array
      if( MTX_static_get_row_array_from_string( linebuf, nL, ncols ) == FALSE )
      {
        // must free the linked list
        errorInReadingData = TRUE;
        break;
      }
    }
    else
    {
      // get the row data from the string and store it in the list item row array
      if( MTX_static_get_row_array_from_string_cplx( linebuf, delimiter, nL, ncols ) == FALSE )
      {
        // must free the linked list
        errorInReadingData = TRUE;
        break;
      }
    }
    nrows++;
  }

  if( errorInReadingData )
  {
    // free the list
    L = head.next;
    while( L!=NULL )
    {
      nL = L->next;
      if( L->isReal )
      {
        free( L->rowptr );
      }
      else
      {
        free( L->rowptr_cplx );
      }
      free(L);
      L = nL;
    }
    if( head.isReal )
    {
      free( head.rowptr );
    }
    else
    {
      free( head.rowptr_cplx );
    }
    MTX_ERROR_MSG( "if( errorInReadingData )" );
    return FALSE;
  }


  // If there are mixed real and complex rows, the real rows must be converted to complex.
  if( needToConvertRealToComplex )
  {
    L = &head;
    // go through the linked list until the data changes from real to complex

    while( L->isReal )
    {
      L->rowptr_cplx = (stComplex*)malloc( ncols*sizeof(stComplex) );
      if( !L->rowptr_cplx )
      {
        // memory allocate failure
        // must free the linked list
        errorInReadingData = TRUE;
        break;
      }

      // copy the data from the real row vector to the complex row vector
      for( i = 0; i < ncols; i++ )
      {
        L->rowptr_cplx[i].re = L->rowptr[i];
        L->rowptr_cplx[i].im = 0.0;
      }

      // free the real vector
      if( L->isReal )
      {
        free( L->rowptr );
      }
      L->rowptr = NULL;

      // move to the next item
      L = L->next;

      // break at the end of the list, this shouldn't ever happen though.
      if( L == NULL )
        break;
    }
  }



  /////
  // copy the list into a MTX object

  // first allocate the row array of pointers
  if( isReal )
  {
    RowMatrix.isReal = TRUE;
    RowMatrix.data = (double**)malloc( nrows*sizeof(double*) );
    if( !RowMatrix.data )
    {
      MTX_ERROR_MSG( "malloc returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    RowMatrix.isReal = FALSE;
    RowMatrix.cplx = (stComplex**)malloc( nrows*sizeof(stComplex*) );
    if( !RowMatrix.cplx )
    {
      MTX_ERROR_MSG( "malloc returned FALSE." );
      return FALSE;
    }
  }
  // looks weird but this is rowwise input
  RowMatrix.ncols = nrows;
  RowMatrix.nrows = ncols;

  L = &head;
  for( i = 0; i < nrows; i++ )
  {
    if( L == NULL )
    {
      // this should never happen
      MTX_ERROR_MSG( "if( L == NULL ) - this should never happen." );
      return FALSE;
    }
    if( isReal )
    {
      RowMatrix.data[i] = L->rowptr; // only copying a pointer!
    }
    else
    {
      RowMatrix.cplx[i] = L->rowptr_cplx; // only copying a pointer!
    }
    L = L->next;
  }

  // copy the data by means of transpose
  // this places the data in the correct MTX storage format
  // only one copy operation
  // If M previously held other data, it is dealt with accordingly.
  if( !MTX_Transpose( &RowMatrix, M ) )
  {
    errorInTranspose = TRUE;
  }

  // free the list data and the list items
  L = head.next;
  while( L!=NULL )
  {
    nL = L->next;
    if( isReal )
    {
      free( L->rowptr );
    }
    else
    {
      free( L->rowptr_cplx );
    }
    free(L);
    L = nL;
  }
  // free the head data
  if( isReal )
  {
    free( head.rowptr );
  }
  else
  {
    free( head.rowptr_cplx );
  }

  // free the RowMatrix data ptr
  if( isReal )
  {
    free( RowMatrix.data );
  }
  else
  {
    free( RowMatrix.cplx );
  }

  if( errorInTranspose )
  {
    MTX_ERROR_MSG( "if( errorInTranspose )" );
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}





BOOL MTX_ValueToString(
                       const double value, //!< The double value to output.
                       const unsigned width, //!< The width of the field.
                       const unsigned precision, //!< The precision, %g style.
                       const BOOL isReal, //!< The the value the real part or the imaginary part.
                       const BOOL alignLeft, //!< Align the output left (for real data only).
                       char *ValueBuffer, //!< The output buffer.
                       const unsigned ValueBufferSize //!< The size of the output buffer.
                       )
{
  char format[16];
  char valbuf[512];
  char* strptr;

  if( width == 0 )
  {
    MTX_ERROR_MSG( "if( width == 0 )" );
    return FALSE;
  }

  valbuf[0] = '\0';

#ifndef _CRT_SECURE_NO_DEPRECATE
  // special case, only output rounded integer data
  if( precision == 0 )
  {
    if( isReal )
    {
      if( value < 0 )
      {
        if( sprintf_s( valbuf, 512, "%d", (int)(-floor( -value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
      }
      else
      {
        if( sprintf_s( valbuf, 512, " %d", (int)(floor( value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
      }
    }
    else
    {
      if( value < 0 )
      {
        if( sprintf_s( valbuf, 512, "%+di", (int)(-floor( -value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
      }
      else
      {
        if( sprintf_s( valbuf, 512, "%+di", (int)(floor( value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
      }
    }
  }
  else
  {
    if( isReal )
    {
      if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 ) // using the 'blank' flag
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      if( sprintf_s( valbuf, 512, format, value ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
    }
    else
    {
      if( sprintf_s( format, 16, "%%+.%dgi", precision ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      if( sprintf_s( valbuf, 512, format, value ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
    }
  }

  if( alignLeft )
  {
    if( sprintf_s( format, 16, "%%-%ds", width ) < 0 ) // left align flag
    {
      MTX_ERROR_MSG( "sprintf_s returned failure." );
      return FALSE;
    }
  }
  else
  {
    if( sprintf_s( format, 16, "%%%ds", width ) < 0 )
    {
      MTX_ERROR_MSG( "sprintf_s returned failure." );
      return FALSE;
    }
  }

  if( sprintf_s( ValueBuffer, ValueBufferSize, format, valbuf ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    return FALSE;
  }

#else
  // special case, only output rounded integer data
  if( precision == 0 )
  {
    if( isReal )
    {
      if( value < 0 )
      {
        if( sprintf( valbuf, "%d", (int)(-floor( -value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
      }
      else
      {
        if( sprintf( valbuf, " %d", (int)(floor( value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
      }
    }
    else
    {
      if( value < 0 )
      {
        if( sprintf( valbuf, "%+di", (int)(-floor( -value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
      }
      else
      {
        if( sprintf( valbuf, "%+di", (int)(floor( value + 0.5 )) ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
      }
    }
  }
  else
  {
    if( isReal )
    {
      if( sprintf( format, "%% .%dg", precision ) < 0 ) // using the 'blank' flag
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      if( sprintf( valbuf, format, value ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
    }
    else
    {
      if( sprintf( format, "%%+.%dgi", precision ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      if( sprintf( valbuf, format, value ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
    }
  }

  if( alignLeft )
  {
    if( sprintf( format, "%%-%ds", width ) < 0 ) // left align flag
    {
      MTX_ERROR_MSG( "sprintf returned failure." );
      return FALSE;
    }
  }
  else
  {
    if( sprintf( format, "%%%ds", width ) < 0 )
    {
      MTX_ERROR_MSG( "sprintf returned failure." );
      return FALSE;
    }
  }

  if( sprintf( ValueBuffer, format, valbuf ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }

#endif

  // deal with "-0 " or "-0" output values.
  strptr = strstr( ValueBuffer, "-0" );
  if( strptr != NULL )
  {
    if( strptr[2] == '\0' || strptr[2] == ' ' )
      strptr[0] = ' '; // get rid of the negative value.
  }

  return TRUE;
}


BOOL MTX_Print( const MTX *M, const char *path, const unsigned width, const unsigned precision, const BOOL append )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  char ValueBuffer[512];
  FILE* out;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  if( width > 511 )
  {
    MTX_ERROR_MSG( "if( width > 511 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  if( append )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "at+" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s returned failure." );
      return FALSE;
    }
#else
    out = fopen( path, "at+" );
#endif
  }
  else
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "w" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s returned failure." );
      return FALSE;
    }
#else
    out = fopen( path, "w" );
#endif
  }
  if( !out )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( ValueBuffer, 512, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#else
    if( sprintf( ValueBuffer, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#endif
    return FALSE;
  }

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        MTX_ValueToString( M->data[j][i], width, precision, TRUE, TRUE, ValueBuffer, 512 );
        fprintf( out, ValueBuffer );
      }
      fprintf( out, "\n" );
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          MTX_ValueToString( M->cplx[j][i].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
          for( k = 0; k < width; k++ )
            fprintf( out, " " );
        }
        else
        {
          // output both
          MTX_ValueToString( M->cplx[j][i].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
          MTX_ValueToString( M->cplx[j][i].im, width, precision, FALSE, TRUE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
        }
      }
      fprintf( out, "\n" );
    }    
  }
  fclose(out);

  return TRUE;
}

BOOL MTX_Print_ToBuffer( const MTX *M, char *buffer, const unsigned maxlength, const unsigned width, const unsigned precision )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned scount = 0;
  unsigned dcount = 0;
  BOOL endOfBuffer = FALSE;
  char ValueBuffer[512];

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( buffer == NULL )
  {
    MTX_ERROR_MSG( "buffer is NULL." );
    return FALSE;
  }

  if( maxlength == 0 )
  {
    MTX_ERROR_MSG( "if( maxlength == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  if( width > 511 )
  {
    MTX_ERROR_MSG( "if( width > 511 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  for( i = 0; i < M->nrows; i++ )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      if( M->isReal )
      {
        MTX_ValueToString( M->data[j][i], width, precision, TRUE, TRUE, ValueBuffer, 512 );
        if( scount + width >= maxlength )
        {
          endOfBuffer = TRUE;
          break;
        }
#ifndef _CRT_SECURE_NO_DEPRECATE
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
#else
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
#endif
      }
      else
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          MTX_ValueToString( M->cplx[j][i].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
          if( scount + width >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;

          for( k = 0; k < width; k++ )
          {
            dcount = sprintf_s( buffer+scount, maxlength-scount, " ", ValueBuffer );
            if( dcount < 0 )
            {
              MTX_ERROR_MSG( "sprintf_s returned failure." );
              return FALSE;
            }
            scount += dcount;
          }
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;

          for( k = 0; k < width; k++ )
          {
            dcount = sprintf( buffer+scount, " ", ValueBuffer );
            if( dcount < 0 )
            {
              MTX_ERROR_MSG( "sprintf returned failure." );
              return FALSE;
            }
            scount += dcount;
          }
#endif
        }
        else
        {
          // output both components
          MTX_ValueToString( M->cplx[j][i].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
          if( scount + width >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }

#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength - scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
#endif

          MTX_ValueToString( M->cplx[j][i].im, width, precision, FALSE, TRUE, ValueBuffer, 512 );
          if( scount + width >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }

#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength - scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
#endif
        }
      }
    }
    if( endOfBuffer )
      break;
    if( scount + 2 >= maxlength )
      break;

#ifndef _CRT_SECURE_NO_DEPRECATE
    dcount = sprintf_s( buffer+scount, maxlength - scount, "\n" );
    if( dcount < 0 )
    {
      MTX_ERROR_MSG( "sprintf returned failure." );
      return FALSE;
    }
    scount += dcount;
#else
    dcount = sprintf( buffer+scount, "\n" );
    if( dcount < 0 )
    {
      MTX_ERROR_MSG( "sprintf returned failure." );
      return FALSE;
    }
    scount += dcount;
#endif
  }

  return TRUE;
}





BOOL MTX_PrintAutoWidth( const MTX *M, const char *path, const unsigned precision, const BOOL append )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned n = 0;
  unsigned maxwidth = 0;
  unsigned maxwidth_im = 0;
  unsigned length = 0;
  unsigned *maxColumnWidth;
  char format[16];
  char ValueBuffer[512];
  FILE* out;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  if( append )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "at+" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s failed to open the file." );
      return FALSE;
    }
#else
    out = fopen( path, "at+" );
#endif
  }
  else
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "w" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s failed to open the file." );
      return FALSE;
    }
#else
    out = fopen( path, "w" );
#endif
  }
  if( !out )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( ValueBuffer, 512, "Unable to open path.", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#else
    if( sprintf( ValueBuffer, "Unable to open path.", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#endif
    return FALSE;
  }

  if( M->isReal )
    n = M->ncols;
  else
    n = M->ncols*2;

  maxColumnWidth = (unsigned*)malloc( sizeof(unsigned)*n );
  if( !maxColumnWidth )
  {
    fclose(out);
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;
      }
      maxColumnWidth[j] = maxwidth+1;
    }
  }
  else
  {
    k = 0;
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      maxwidth_im = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif

        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;

#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth_im )
          maxwidth_im = length;
      }
      maxColumnWidth[k] = maxwidth+1;
      k++;
      maxColumnWidth[k] = maxwidth_im+1;
      k++;
    }

  }

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        MTX_ValueToString( M->data[j][i], maxColumnWidth[j], precision, TRUE, TRUE, ValueBuffer, 512 );
        fprintf( out, ValueBuffer );
      }
      fprintf( out, "\n" );
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
          for( k = 0; k < maxColumnWidth[j*2+1]; k++ )
            fprintf( out, " " );
        }
        else
        {
          // output both
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
          MTX_ValueToString( M->cplx[j][i].im, maxColumnWidth[j*2+1], precision, FALSE, TRUE, ValueBuffer, 512 );
          fprintf( out, ValueBuffer );
        }
      }
      fprintf( out, "\n" );
    }
  }

  fclose(out);

  free(maxColumnWidth);

  return TRUE;
}


BOOL MTX_PrintStdoutAutoWidth( const MTX *M, const unsigned precision )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned n = 0;
  unsigned maxwidth = 0;
  unsigned maxwidth_im = 0;
  unsigned length = 0;
  unsigned *maxColumnWidth;
  char format[16];
  char ValueBuffer[512];

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  if( M->isReal )
    n = M->ncols;
  else
    n = M->ncols*2;

  maxColumnWidth = (unsigned*)malloc( sizeof(unsigned)*n );
  if( !maxColumnWidth )
  {
    MTX_ERROR_MSG( "if( !maxColumnWidth )" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;
      }
      maxColumnWidth[j] = maxwidth+1;
    }
  }
  else
  {
    k = 0;
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      maxwidth_im = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif

        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;

#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth_im )
          maxwidth_im = length;
      }
      maxColumnWidth[k] = maxwidth+1;
      k++;
      maxColumnWidth[k] = maxwidth_im+1;
      k++;
    }

  }

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        MTX_ValueToString( M->data[j][i], maxColumnWidth[j], precision, TRUE, TRUE, ValueBuffer, 512 );
        printf( ValueBuffer );
      }
      printf( "\n" );
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          printf( ValueBuffer );
          for( k = 0; k < maxColumnWidth[j*2+1]; k++ )
            printf( " " );
        }
        else
        {
          // output both
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          printf( ValueBuffer );
          MTX_ValueToString( M->cplx[j][i].im, maxColumnWidth[j*2+1], precision, FALSE, TRUE, ValueBuffer, 512 );
          printf( ValueBuffer );
        }
      }
      printf( "\n" );
    }
  }

  free(maxColumnWidth);

  return TRUE;
}



BOOL MTX_PrintAutoWidth_ToBuffer( const MTX *M, char *buffer, const unsigned maxlength, const unsigned precision )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned n = 0;
  unsigned maxwidth = 0;
  unsigned maxwidth_im = 0;
  unsigned length = 0;
  unsigned scount = 0; // count into buffer
  unsigned dcount = 0;
  BOOL endOfBuffer = FALSE;
  unsigned *maxColumnWidth;
  char format[16];
  char ValueBuffer[512];

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( buffer == NULL )
  {
    MTX_ERROR_MSG( "buffer is a NULL pointer." );
    return FALSE;
  }

  if( maxlength == 0 )
  {
    MTX_ERROR_MSG( "if( maxlength == 0 )" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  if( M->isReal )
    n = M->ncols;
  else
    n = M->ncols*2;

  maxColumnWidth = (unsigned*)malloc( sizeof(unsigned)*n );
  if( !maxColumnWidth )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;
      }
      maxColumnWidth[j] = maxwidth+1;
    }
  }
  else
  {
    k = 0;
    for( j = 0; j < M->ncols; j++ )
    {
      // determine the maximum width needed for the given precision
      maxwidth = 0;
      maxwidth_im = 0;
      for( i = 0; i < M->nrows; i++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%% .%dg", precision ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth )
          maxwidth = length;

#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        length = (unsigned int)strlen( ValueBuffer );
        if( length > maxwidth_im )
          maxwidth_im = length;
      }
      maxColumnWidth[k] = maxwidth+1;
      k++;
      maxColumnWidth[k] = maxwidth_im+1;
      k++;
    }
  }

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        MTX_ValueToString( M->data[j][i], maxColumnWidth[j], precision, TRUE, TRUE, ValueBuffer, 512 );
        if( scount+maxColumnWidth[j] >= maxlength )
        {
          endOfBuffer = TRUE;
          break;
        }
#ifndef _CRT_SECURE_NO_DEPRECATE
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
#else
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
#endif
      }
      if( endOfBuffer )
        break;
      if( scount+2 >= maxlength )
        break;
#ifndef _CRT_SECURE_NO_DEPRECATE
      dcount = sprintf_s( buffer+scount, maxlength-scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;
#else
      dcount = sprintf( buffer+scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;
#endif
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          if( scount+maxColumnWidth[j] >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;

          for( k = 0; k < maxColumnWidth[j*2+1]; k++ )
          {
            dcount = sprintf_s( buffer+scount, maxlength-scount, " ", ValueBuffer );
            if( dcount < 0 )
            {
              MTX_ERROR_MSG( "sprintf_s returned failure." );
              return FALSE;
            }
            scount += dcount;
          }
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;

          for( k = 0; k < maxColumnWidth[j*2+1]; k++ )
          {
            dcount = sprintf( buffer+scount, " ", ValueBuffer );
            if( dcount < 0 )
            {
              MTX_ERROR_MSG( "sprintf returned failure." );
              return FALSE;
            }
            scount += dcount;
          }
#endif
        }
        else
        {
          // output both
          MTX_ValueToString( M->cplx[j][i].re, maxColumnWidth[j*2], precision, TRUE, FALSE, ValueBuffer, 512 );
          if( scount+maxColumnWidth[j] >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
#endif

          MTX_ValueToString( M->cplx[j][i].im, maxColumnWidth[j*2+1], precision, FALSE, TRUE, ValueBuffer, 512 );
          if( scount+maxColumnWidth[j] >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
#ifndef _CRT_SECURE_NO_DEPRECATE
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
#else
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
#endif
        }
      }
      if( endOfBuffer )
        break;
      if( scount+2 >= maxlength )
        break;

#ifndef _CRT_SECURE_NO_DEPRECATE
      dcount = sprintf_s( buffer+scount, maxlength-scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;
#else
      dcount = sprintf( buffer+scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;
#endif
    }
  }

  free(maxColumnWidth);

  return TRUE;
}

BOOL MTX_PrintDelimited( const MTX *M, const char *path, const unsigned precision, const char delimiter, const BOOL append )
{
  unsigned i = 0;
  unsigned j = 0;
  char format[16];
  char ValueBuffer[512];
  FILE* out;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

  if( append )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "at+" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s failed to open the file." );
      return FALSE;
    }
#else
    out = fopen( path, "at+" );
#endif
  }
  else
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &out, path, "w" ) != 0 )
    {
      MTX_ERROR_MSG( "fopen_s failed to open the file." );
      return FALSE;
    }
#else
    out = fopen( path, "w" );
#endif
  }
  if( !out )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( ValueBuffer, 512, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#else
    if( sprintf( ValueBuffer, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( ValueBuffer );
#endif
    return FALSE;
  }

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%%.%dg%c", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%%.%dg%c", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        fprintf( out, ValueBuffer );
      }
#ifndef _CRT_SECURE_NO_DEPRECATE
      if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
#else
      if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
#endif
      fprintf( out, ValueBuffer );
      fprintf( out, "\n" );
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
#ifndef _CRT_SECURE_NO_DEPRECATE
          if( sprintf_s( format, 16, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
#else
          if( sprintf( format, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
#endif
          fprintf( out, ValueBuffer );
        }
        else
        {
          // output both
#ifndef _CRT_SECURE_NO_DEPRECATE
          if( sprintf_s( format, 16, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
#else
          if( sprintf( format, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
#endif
          fprintf( out, ValueBuffer );

#ifndef _CRT_SECURE_NO_DEPRECATE
          if( sprintf_s( format, 16, "%%+.%dgi%c", precision, delimiter ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
#else
          if( sprintf( format, "%%+.%dgi%c", precision, delimiter ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
#endif
          fprintf( out, ValueBuffer );
        }
      }
      if( M->cplx[j][i].im == 0 )
      {
        // output only the real component
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        fprintf( out, ValueBuffer );
      }
      else
      {
        // output both
#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        fprintf( out, ValueBuffer );

#ifndef _CRT_SECURE_NO_DEPRECATE
        if( sprintf_s( format, 16, "%%+.%dgi", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
#else
        if( sprintf( format, "%%+.%dgi", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
#endif
        fprintf( out, ValueBuffer );
      }
      fprintf( out, "\n" );
    }
  }

  fclose(out);

  return TRUE;
}

BOOL MTX_PrintDelimited_ToBuffer( const MTX *M, char *buffer, const unsigned maxlength, const unsigned precision, const char delimiter )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned scount = 0;
  unsigned dcount = 0;
  BOOL endOfBuffer = FALSE;
  char format[16];
  char ValueBuffer[512];

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( buffer == NULL )
  {
    MTX_ERROR_MSG( "if( buffer == NULL )" );
    return FALSE;
  }

  if( maxlength == 0 )
  {
    MTX_ERROR_MSG( "if( maxlength == 0 )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

  ValueBuffer[0] = '\0';

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
        if( sprintf_s( format, 16, "%%.%dg%c", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
        {
          endOfBuffer = TRUE;
          break;
        }
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
      if( endOfBuffer )
        break;
      if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      if( sprintf_s( ValueBuffer, 512, format, M->data[j][i] ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      if( scount + strlen(ValueBuffer) >= maxlength )
        break;
      dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;

      if( scount + 2 >= maxlength )
        break;
      dcount = sprintf_s( buffer+scount, maxlength-scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          if( sprintf_s( format, 16, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
        else
        {
          // output both
          if( sprintf_s( format, 16, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;

          if( sprintf_s( format, 16, "%%+.%dgi%c", precision, delimiter ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
      }
      if( endOfBuffer )
        break;

      if( M->cplx[j][i].im == 0 )
      {
        // output only the real component
        if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
      else
      {
        // output both
        if( sprintf_s( format, 16, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;

        if( sprintf_s( format, 16, "%%+.%dgi", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( sprintf_s( ValueBuffer, 512, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf_s( buffer+scount, maxlength-scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
      }

      if( scount + 2 >= maxlength )
        break;

      dcount = sprintf_s( buffer+scount, maxlength-scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
  }
#else
  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
        if( sprintf( format, "%%.%dg%c", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
        {
          endOfBuffer = TRUE;
          break;
        }
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
      if( endOfBuffer )
        break;
      if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      if( sprintf( ValueBuffer, format, M->data[j][i] ) < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      if( scount + strlen(ValueBuffer) >= maxlength )
        break;
      dcount = sprintf( buffer+scount, "%s", ValueBuffer );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;

      if( scount + 2 >= maxlength )
        break;
      dcount = sprintf( buffer+scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      for( j = 0; j < M->ncols-1; j++ )
      {
        if( M->cplx[j][i].im == 0 )
        {
          // output only the real component
          if( sprintf( format, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
        else
        {
          // output both
          if( sprintf( format, "%%.%dg", precision ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;

          if( sprintf( format, "%%+.%dgi%c", precision, delimiter ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          if( scount + strlen(ValueBuffer) >= maxlength )
          {
            endOfBuffer = TRUE;
            break;
          }
          dcount = sprintf( buffer+scount, "%s", ValueBuffer );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
      }
      if( endOfBuffer )
        break;

      if( M->cplx[j][i].im == 0 )
      {
        // output only the real component
        if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
      else
      {
        // output both
        if( sprintf( format, "%%.%dg", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].re ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;

        if( sprintf( format, "%%+.%dgi", precision, delimiter ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( sprintf( ValueBuffer, format, M->cplx[j][i].im ) < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        if( scount + strlen(ValueBuffer) >= maxlength )
          break;
        dcount = sprintf( buffer+scount, "%s", ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
      }

      if( scount + 2 >= maxlength )
        break;

      dcount = sprintf( buffer+scount, "\n" );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
  }
#endif

  return TRUE;
}

BOOL MTX_PrintRowToString( const MTX *M, const unsigned row, char *buffer, const unsigned maxlength, const int width, const int precision )
{
  unsigned j = 0;
  int k = 0;
  unsigned len = 0;
  unsigned scount = 0;
  unsigned dcount = 0;

  char ValueBuffer[512];
  ValueBuffer[0] = '\0';

  if( !buffer )
  {
    MTX_ERROR_MSG( "buffer is a NULL pointer." );
    return FALSE;
  }

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->ncols == 0 || M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->ncols == 0 || M->nrows == 0 )" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( precision > 200 )
  {
    MTX_ERROR_MSG( "if( precision > 200 )" );
    return FALSE;
  }

#ifndef _CRT_SECURE_NO_DEPRECATE
  for( j = 0; j < M->ncols; j++ )
  {
    if( M->isReal )
    {
      MTX_ValueToString( M->data[j][row], width, precision, TRUE, TRUE, ValueBuffer, 512 );
      len = (unsigned int)strlen( ValueBuffer );
      if( len + scount >= maxlength )
      {
        MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
        return FALSE;
      }
      dcount = sprintf_s( buffer+scount, maxlength-scount, ValueBuffer );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf_s returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
    else
    {
      if( M->cplx[j][row].im == 0 )
      {
        // output only the real component
        MTX_ValueToString( M->cplx[j][row].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf_s( buffer+scount, maxlength-scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
        for( k = 0; k < width; k++ )
        {
          dcount = sprintf_s( buffer+scount, maxlength-scount, " " );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf_s returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
      }
      else
      {
        // output both
        MTX_ValueToString( M->cplx[j][row].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf_s( buffer+scount, maxlength-scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;

        MTX_ValueToString( M->cplx[j][row].im, width, precision, FALSE, TRUE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf_s( buffer+scount, maxlength-scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf_s returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
    }
  }
  if( sprintf_s( buffer+scount, maxlength-scount, "\n" ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    return FALSE;
  }
#else
  for( j = 0; j < M->ncols; j++ )
  {
    if( M->isReal )
    {
      MTX_ValueToString( M->data[j][row], width, precision, TRUE, TRUE, ValueBuffer, 512 );
      len = (unsigned int)strlen( ValueBuffer );
      if( len + scount >= maxlength )
      {
        MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
        return FALSE;
      }
      dcount = sprintf( buffer+scount, ValueBuffer );
      if( dcount < 0 )
      {
        MTX_ERROR_MSG( "sprintf returned failure." );
        return FALSE;
      }
      scount += dcount;
    }
    else
    {
      if( M->cplx[j][row].im == 0 )
      {
        // output only the real component
        MTX_ValueToString( M->cplx[j][row].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf( buffer+scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
        for( k = 0; k < width; k++ )
        {
          dcount = sprintf( buffer+scount, " " );
          if( dcount < 0 )
          {
            MTX_ERROR_MSG( "sprintf returned failure." );
            return FALSE;
          }
          scount += dcount;
        }
      }
      else
      {
        // output both
        MTX_ValueToString( M->cplx[j][row].re, width, precision, TRUE, FALSE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf( buffer+scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;

        MTX_ValueToString( M->cplx[j][row].im, width, precision, FALSE, TRUE, ValueBuffer, 512 );
        len = (unsigned int)strlen( ValueBuffer );
        if( len + scount >= maxlength )
        {
          MTX_ERROR_MSG( "if( len + scount >= maxlength )" );
          return FALSE;
        }
        dcount = sprintf( buffer+scount, ValueBuffer );
        if( dcount < 0 )
        {
          MTX_ERROR_MSG( "sprintf returned failure." );
          return FALSE;
        }
        scount += dcount;
      }
    }
  }
  if( sprintf( buffer+scount, "\n" ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }
#endif
  return TRUE;
}




//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
// Math Operations
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

BOOL MTX_Add_Scalar( MTX *M, const double scalar )
{
  unsigned i = 0;
  unsigned j = 0;

  if( scalar == 0.0 )
    return TRUE;

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] += scalar;
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re += scalar;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Add_ScalarComplex( MTX *M, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;

  // special cases
  if( re == 0.0 && im == 0.0 )
  {
    return TRUE;
  }
  if( im == 0.0 )
  {
    return MTX_Add_Scalar( M, re );
  }

  if( M->isReal )
  {
    if( !MTX_ConvertRealToComplex( M ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  if( re == 0.0 )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].im += im;
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re += re;
        M->cplx[j][i].im += im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Subtract_Scalar( MTX *M, const double scalar )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( scalar == 0.0 )
    return TRUE;

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] -= scalar;
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {      
        M->cplx[j][i].re -= scalar;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Subtract_ScalarComplex( MTX *M, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;

  // special cases
  if( re == 0.0 && im == 0.0 )
  {
    return TRUE;
  }
  if( im == 0.0 )
  {
    return MTX_Subtract_Scalar( M, re );
  }

  if( M->isReal )
  {
    if( !MTX_ConvertRealToComplex( M ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  if( re == 0.0 )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].im -= im;        
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re -= re;
        M->cplx[j][i].im -= im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Multiply_Scalar( MTX *M, const double scalar )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( scalar == 0.0 )
  {
    return MTX_Zero( M );
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] *= scalar;      
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re *= scalar;
        M->cplx[j][i].im *= scalar;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Multiply_ScalarComplex( MTX *M, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;
  double tre = 0;
  double tim = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // special cases
  if( re == 0.0 && im == 0.0 )
  {
    return TRUE;
  }
  if( im == 0.0 )
  {
    return MTX_Multiply_Scalar( M, re );
  }

  if( M->isReal )
  {
    if( !MTX_ConvertRealToComplex( M ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      tre = M->cplx[j][i].re * re - M->cplx[j][i].im * im;
      tim = M->cplx[j][i].re * im + M->cplx[j][i].im * re;
      M->cplx[j][i].re = tre;
      M->cplx[j][i].im = tim;
    }
  }
  return TRUE;
}

BOOL MTX_Divide_Scalar( MTX *M, const double scalar )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      { 
        M->data[j][i] /= scalar;
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      { 
        M->cplx[j][i].re /= scalar;
        M->cplx[j][i].im /= scalar;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Divide_ScalarComplex( MTX *M, const double re, const double im )
{
  unsigned i = 0;
  unsigned j = 0;
  double mag = 0;
  double tre = 0;
  double tim = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // special case
  if( im == 0.0 )
  {
    return MTX_Divide_Scalar( M, re );
  }

  if( M->isReal )
  {
    if( !MTX_ConvertRealToComplex( M ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < M->ncols; j++ )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      mag = re*re + im*im;

      tre = (M->cplx[j][i].re * re + M->cplx[j][i].im * im )/mag;
      tim = (M->cplx[j][i].im * re - M->cplx[j][i].re * im )/mag;

      M->cplx[j][i].re = tre;
      M->cplx[j][i].im = tim;
    }
  }
  return TRUE;
}

BOOL MTX_Negate( MTX *M )
{
  unsigned i = 0;
  unsigned j;  
  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = -M->data[j][i];
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->cplx[j][i].re = -M->cplx[j][i].re;
        M->cplx[j][i].im = -M->cplx[j][i].im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Abs( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !M->isReal )
  {
    // special case for optimization, both the data and complex
    // pointers are used. i.e. the matrix is real and complex
    // at the same time
    M->data = (double**)calloc( (M->ncols), sizeof(double*) );
    if( !M->data )
    {
      MTX_ERROR_MSG( "calloc returned NULL." );
      MTX_Free( M );
      return FALSE;
    }
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
      for( i = 0; i < M->nrows; i++ )
        M->data[j][i] = fabs(M->data[j][i]);
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      // allocate the real data
      M->data[j] = (double*)malloc( sizeof(double)*(M->nrows) );
      if( !M->data[j] )
      {
        for( k = 0; k < j; k++ )
        {
          if( M->data[j] )
            free( M->data[j] );
        }
        free( M->data );
        M->data = NULL;
        MTX_Free( M );
        MTX_ERROR_MSG( "malloc returned NULL." );
        return FALSE;
      }

      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = sqrt( M->cplx[j][i].re*M->cplx[j][i].re + M->cplx[j][i].im*M->cplx[j][i].im );
      }

      // free the complex data
      free( M->cplx[j] );
    }
  }

  if( !M->isReal )
  {
    // free the complex matrix pointer
    free( M->cplx );
    M->cplx = NULL;
    M->isReal = TRUE;
  }

  return TRUE;
}


BOOL MTX_acos( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  double maxabs = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( M->isReal )
  {
    // check the input range of the data, -1<=x<=1
    if( !MTX_MaxAbs( M, &maxabs ) )
    {
      MTX_ERROR_MSG( "MTX_MaxAbs returned FALSE." );
      return FALSE;
    }

    if( maxabs > 1.0 )
    {
      if( !MTX_ConvertRealToComplex( M ) )
      {
        MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
        return FALSE;
      }
    }
  }

  if( M->isReal )
  {
    // The data is real and well bounded.
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = acos(M->data[j][i]);
      }
    }
  }
  else
  {
    // refer http://idlastro.gsfc.nasa.gov/idl_html_help/ACOS.html
    double Xp2;
    double Xm2;
    double Y2;
    double A;
    double B;

    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        Xp2 = M->cplx[j][i].re + 1.0;
        Xp2*= Xp2;

        Xm2 = M->cplx[j][i].re - 1.0;
        Xm2*= Xm2;

        Y2 = M->cplx[j][i].im;
        Y2 *= Y2;

        //A = 0.5 * sqrt((X + 1.0)*(X + 1.0) + Y*Y) + 0.5 * sqrt((X - 1.0)*(X - 1.0) + Y*Y)
        //B = 0.5 * sqrt((X + 1.0)*(X + 1.0) + Y*Y) - 0.5 * sqrt((X - 1.0)*(X - 1.0) + Y*Y)

        A = 0.5 * ( sqrt(Xp2 + Y2) + sqrt(Xm2 + Y2) );
        B = 0.5 * ( sqrt(Xp2 + Y2) - sqrt(Xm2 + Y2) );

        if( M->cplx[j][i].im >= 0 )
        {
          M->cplx[j][i].re = acos(B);
          M->cplx[j][i].im = -log( A + sqrt( A*A-1.0 ) );
        }
        else
        {
          M->cplx[j][i].re = acos(B);
          M->cplx[j][i].im = log( A + sqrt( A*A-1.0 ) );
        }
      }
    }
  }
  return TRUE;
}

BOOL MTX_angle( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  MTX copyM;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  MTX_Init(&copyM);

  if( !MTX_Copy(M,&copyM) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  if( !MTX_Phase(&copyM,M) )
  {
    MTX_ERROR_MSG( "MTX_Phase returned FALSE." );
    return FALSE;
  }

  MTX_Free(&copyM);

  return TRUE;
}

BOOL MTX_asin( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( M->isReal )
  {
    double maxabs;
    // check the input range of the data, -1<=x<=1
    if( !MTX_MaxAbs( M, &maxabs ) )
    {
      MTX_ERROR_MSG( "MTX_MaxAbs returned FALSE." );
      return FALSE;
    }

    if( maxabs > 1.0 )
    {
      if( !MTX_ConvertRealToComplex( M ) )
      {
        MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
        return FALSE;
      }
    }
  }

  if( M->isReal )
  {
    // The data is real and well bounded.
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = asin(M->data[j][i]);
      }
    }
  }
  else
  {
    // refer http://idlastro.gsfc.nasa.gov/idl_html_help/ASIN.html
    double Xp2;
    double Xm2;
    double Y2;
    double A;
    double B;

    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        Xp2 = M->cplx[j][i].re + 1.0;
        Xp2*= Xp2;

        Xm2 = M->cplx[j][i].re - 1.0;
        Xm2*= Xm2;

        Y2 = M->cplx[j][i].im;
        Y2 *= Y2;

        //A = 0.5 * sqrt((X + 1.0)*(X + 1.0) + Y*Y) + 0.5 * sqrt((X - 1.0)*(X - 1.0) + Y*Y)
        //B = 0.5 * sqrt((X + 1.0)*(X + 1.0) + Y*Y) - 0.5 * sqrt((X - 1.0)*(X - 1.0) + Y*Y)

        A = 0.5 * ( sqrt(Xp2 + Y2) + sqrt(Xm2 + Y2) );
        B = 0.5 * ( sqrt(Xp2 + Y2) - sqrt(Xm2 + Y2) );

        if( M->cplx[j][i].im >= 0 )
        {
          M->cplx[j][i].re = asin(B);
          M->cplx[j][i].im = log( A + sqrt( A*A-1.0 ) );
        }
        else
        {
          M->cplx[j][i].re = asin(B);
          M->cplx[j][i].im = -log( A + sqrt( A*A-1.0 ) );
        }
      }
    }
  }
  return TRUE;
}

BOOL MTX_Sqr( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0;
  double im = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] *= M->data[j][i];
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {      
        re = M->cplx[j][i].re*M->cplx[j][i].re - M->cplx[j][i].im*M->cplx[j][i].im;
        im = 2.0 * M->cplx[j][i].re*M->cplx[j][i].im;
        M->cplx[j][i].re = re;
        M->cplx[j][i].im = im;
      }
    }
  }
  return TRUE;
}

void MTX_static_quick_sqrt( const double *a_re, const double *a_im, double *re, double *im )
{
  double mag;
  if( *a_im == 0.0 )
  {
    if( *a_re < 0 )
    {
      *re = 0.0;
      *im = sqrt( -(*a_re) );
    }
    else
    {
      *re = sqrt( *a_re );
      *im = 0.0;
    }
  }
  else
  {
    mag = sqrt( (*a_re)*(*a_re) + (*a_im)*(*a_im) );

    *re = sqrt( (mag + (*a_re))/2.0 );
    if( *a_im < 0 )
      *im = -1.0*sqrt( (mag - (*a_re))/2.0 );
    else
      *im = sqrt( (mag - (*a_re))/2.0 );
  }
}

BOOL MTX_Sqrt( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0;
  double im = 0;
  BOOL convert = FALSE;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    // check every element in a real matrix for negative values
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        if( M->data[j][i] < 0 )
        {
          convert = TRUE;
          break;
        }
      }
      if( convert )
        break;
    }

    if( convert )
    {
      if( !MTX_ConvertRealToComplex( M ) )
      {
        MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
        return FALSE;
      }
    }
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      { 
        M->data[j][i] = sqrt(M->data[j][i]);
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      { 
        MTX_static_quick_sqrt( &(M->cplx[j][i].re), &(M->cplx[j][i].im), &re, &im );
        M->cplx[j][i].re = re;
        M->cplx[j][i].im = im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Exp( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = exp(M->data[j][i]);
      }
    }
  }
  else
  {
    // exp(M) = exp(real)*(cos(imag)+i*sin(imag)).
    MTX Re;
    MTX Im;
    double real_part;
    double imag_part;

    MTX_Init( &Re );
    MTX_Init( &Im );

    if( !MTX_Real( M, &Re ) )
    {
      MTX_ERROR_MSG( "MTX_Real returned FALSE." );
      MTX_Free( &Re );
      MTX_Free( &Im );
      return FALSE;
    }

    if( !MTX_Imag( M, &Im ) )
    {
      MTX_ERROR_MSG( "MTX_Imag returned FALSE." );
      MTX_Free( &Re );
      MTX_Free( &Im );      
      return FALSE;
    }

    if( !MTX_Exp(&Re) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free( &Re );
      MTX_Free( &Im );      
      return FALSE;
    }

    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        real_part = Re.data[j][i];
        imag_part = Im.data[j][i];
        M->cplx[j][i].re = real_part * cos( imag_part );
        M->cplx[j][i].im = real_part * sin( imag_part );
      }
    }
    MTX_Free( &Re );
    MTX_Free( &Im );      
  }
  return TRUE;
}

BOOL MTX_Eye( MTX *M, const unsigned nrows, const unsigned ncols )
{
  unsigned i = 0;
  unsigned j = 0;

  if( !MTX_Calloc( M, nrows, ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    return FALSE;
  }

  if( nrows < ncols )
  {
    for( i = 0; i < nrows; i++ )
      M->data[i][i] = 1.0;
  }
  else
  {
    for( j = 0; j < ncols; j++ )
      M->data[j][j] = 1.0;
  }

  return TRUE;
}

BOOL MTX_Ln( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  double re;
  double im;  
  MTX lnmag;
  MTX phase;
  MTX_Init( &lnmag );
  MTX_Init( &phase );

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Min( M, &re, &im ) )
  {
    MTX_ERROR_MSG( "MTX_Min returned FALSE." );
    return FALSE;
  }

  if( M->isReal && re >= 0 )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = log(M->data[j][i]);
      }
    }
    return TRUE;
  }
  if( !MTX_Magnitude( M, &lnmag ) )
  {
    MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
    MTX_Free(&lnmag);
    return FALSE;
  }
  if( !MTX_Phase( M, &phase ) )
  {
    MTX_ERROR_MSG( "MTX_Phase returned FALSE." );
    MTX_Free(&lnmag);
    MTX_Free(&phase);
    return FALSE;
  }

  if( !MTX_Ln( &lnmag ) )
  {
    MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
    MTX_Free(&lnmag);
    MTX_Free(&phase);
    return FALSE;
  }

  if( !MTX_Complex( M, &lnmag, &phase ) )
  {
    MTX_ERROR_MSG( "MTX_Complex returned FALSE." );
    MTX_Free(&lnmag);
    MTX_Free(&phase);
    return FALSE;
  }

  MTX_Free(&lnmag);
  MTX_Free(&phase);
  return TRUE;
}


BOOL MTX_Pow( const MTX *src, MTX *dst, const double power_re, const double power_im )
{
  unsigned i = 0;
  unsigned j = 0;
  BOOL success = TRUE;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !dst )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( power_im == 0.0 )
  {
    // if real, assume destination will be real initially.
    if( !MTX_Malloc( dst, src->nrows, src->ncols, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_Malloc( dst, src->nrows, src->ncols, FALSE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }

  // deal with real-real case first
  if( src->isReal && power_im == 0 && power_re >= 1.0 )
  {
    for( j = 0; j < src->ncols; j++ )
      for( i = 0; i < src->nrows; i++ )
        dst->data[j][i] = pow(src->data[j][i],power_re);
  }
  else
  {
    // x^y, can be expressed as e^(y*ln(x))
    MTX yLnX;
    stComplex cplxval;

    MTX_Init( &yLnX );
    cplxval.re = power_re;
    cplxval.im = power_im;

    if( !MTX_Copy( src, &yLnX ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      return FALSE;
    }

    if( !MTX_Ln( &yLnX ) )
    {
      MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
      return FALSE;
    }

    if( cplxval.im == 0 )
    {
      if( !MTX_Multiply_Scalar( &yLnX, cplxval.re ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Multiply_ScalarComplex( &yLnX, cplxval.re, cplxval.im ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
        return FALSE;
      }
    }

    if( !MTX_Copy( &yLnX, dst ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      return FALSE;
    }

    if( !MTX_Exp( dst ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      return FALSE;
    }

    MTX_Free( &yLnX );
  }

  return TRUE;
}


BOOL MTX_PowInplace( MTX *src, const double power_re, const double power_im )
{
  MTX copy;
  MTX_Init( &copy );
  
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Copy( src, &copy ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &copy );
    return FALSE;
  }

  if( !MTX_Pow( src, &copy, power_re, power_im ) )
  {
    MTX_ERROR_MSG( "MTX_Pow returned FALSE." );
    MTX_Free( &copy );
    return FALSE;
  }

  if( !MTX_Copy( &copy, src ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &copy );
    return FALSE;
  }

  MTX_Free( &copy );
  return TRUE;
}



BOOL MTX_atan( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        M->data[j][i] = atan( M->data[j][i] );
      }
    }
  }
  else
  {
    // complex arctan!
    // arctan( z ) = 1/2 * i * ( ln( 1-iz ) - ln(1+iz) ), where z is complex
    MTX LnOneMinusZ;
    MTX LnOnePlusZ;
    stComplex halfi;

    halfi.re = 0;
    halfi.im = 0.5;

    MTX_Init( &LnOneMinusZ );
    MTX_Init( &LnOnePlusZ );

    // copy M to LnOnePlusZ
    if( !MTX_Copy( M, &LnOnePlusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      return FALSE;
    }
    // make LnOnePlusZ = M*i
    if( !MTX_Multiply_ScalarComplex( &LnOnePlusZ, 0.0, 1.0 ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }
    // make LnOneMinusZ = M*i
    if( !MTX_Copy( &LnOnePlusZ, &LnOneMinusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }

    // compute 1-iz
    for( j = 0; j < LnOneMinusZ.ncols; j++ )
    {
      for( i = 0; i < LnOneMinusZ.nrows; i++ )
      {
        LnOneMinusZ.cplx[j][i].re = 1.0 - LnOneMinusZ.cplx[j][i].re;
        LnOneMinusZ.cplx[j][i].im = -LnOneMinusZ.cplx[j][i].im;
      }
    }

    if( !MTX_Increment( &LnOnePlusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
      MTX_Free( &LnOneMinusZ );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }

    if( !MTX_Ln( &LnOneMinusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
      MTX_Free( &LnOneMinusZ );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }
    if( !MTX_Ln( &LnOnePlusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
      MTX_Free( &LnOneMinusZ );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }

    if( !MTX_Subtract( M, &LnOneMinusZ, &LnOnePlusZ ) )
    {
      MTX_ERROR_MSG( "MTX_Subtract returned FALSE." );
      MTX_Free( &LnOneMinusZ );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }

    if( !MTX_Multiply_ScalarComplex( M, halfi.re, halfi.im ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
      MTX_Free( &LnOneMinusZ );
      MTX_Free( &LnOnePlusZ );
      return FALSE;
    }

    MTX_Free( &LnOneMinusZ );
    MTX_Free( &LnOnePlusZ );
  }
  return TRUE;
}

BOOL MTX_Increment( MTX *M )
{
  return MTX_Add_Scalar( M, 1.0 );
}

BOOL MTX_Decrement( MTX *M )
{
  return MTX_Subtract_Scalar( M, 1.0 );
}

BOOL MTX_Add_Inplace( MTX *A, const MTX* B )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0.0;
  double im = 0.0;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( A->nrows == 1 && A->ncols == 1 )
    {
      if( A->isReal )
      {
        re = A->data[0][0];
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_Scalar( A, re );
      }
      else
      {
        re = A->cplx[0][0].re;
        im = A->cplx[0][0].im;
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_ScalarComplex( A, re, im );
      }
    }
    else if( B->nrows == 1 && B->ncols == 1 )
    {
      if( B->isReal )
      {
        return MTX_Add_Scalar( A, B->data[0][0] );
      }
      else
      {
        return MTX_Add_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
  }

  if( !MTX_isConformalForAddition( A, B ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( A->isReal && !B->isReal )
  {
    if( !MTX_ConvertRealToComplex( A ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }
  
  if( A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {    
      for( i = 0; i < A->nrows; i++ )
      {
        A->data[j][i] += B->data[j][i];
      }
    }
  }
  else if( !A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {  
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re += B->cplx[j][i].re;
        A->cplx[j][i].im += B->cplx[j][i].im;
      }
    }
  }
  else
  {
    for( j = 0; j < A->ncols; j++ )
    {  
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re += B->data[j][i];
      }
    }
  }
  return TRUE;
}

BOOL MTX_Subtract_Inplace( MTX *A, const MTX* B )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0.0;
  double im = 0.0;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( A->nrows == 1 && A->ncols == 1 )
    {
      // Set tmp = A; A=-B; A+=tmp;
      if( A->isReal )
      {
        re = A->data[0][0];
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        if( !MTX_Negate( A ) )
        {
          MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
          return FALSE;
        }
        return MTX_Add_Scalar( A, re );
      }
      else
      {
        re = A->cplx[0][0].re;
        im = A->cplx[0][0].im;
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        if( !MTX_Negate( A ) )
        {
          MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
          return FALSE;
        }
        return MTX_Add_ScalarComplex( A, re, im );
      }
    }
    else if( B->nrows == 1 && B->ncols == 1 )
    {
      if( B->isReal )
      {
        return MTX_Subtract_Scalar( A, B->data[0][0] );
      }
      else
      {
        return MTX_Subtract_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
  }

  if( !MTX_isConformalForAddition( A, B ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( A->isReal && !B->isReal )
  {
    if( !MTX_ConvertRealToComplex( A ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  if( A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        A->data[j][i] -= B->data[j][i];
      }
    }
  }
  else if( !A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )      
      {
        A->cplx[j][i].re -= B->cplx[j][i].re;
        A->cplx[j][i].im -= B->cplx[j][i].im;
      }
    }
  }
  else
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )      
      {
        A->cplx[j][i].re -= B->data[j][i];
      }
    }
  }
  return TRUE;
}

// multiply A = A*B, inplace
BOOL MTX_PostMultiply_Inplace( MTX *A, const MTX* B )
{
  MTX M;
  MTX_Init( &M );

  if( !MTX_Multiply( &M, A, B ) )
  {
    MTX_ERROR_MSG( "MTX_Multiply returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  if( !MTX_Copy( &M, A ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  MTX_Free( &M );
  return TRUE;
}

// multiply A = A*transpose(B), inplace
BOOL MTX_PostMultiplyTranspose_Inplace( MTX *A, const MTX* B )
{
  MTX M;
  MTX_Init( &M );

  if( !MTX_MultiplyTranspose( &M, A, B ) )
  {
    MTX_ERROR_MSG( "MTX_MultiplyTranspose returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  if( !MTX_Copy( &M, A ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  MTX_Free( &M );
  return TRUE;
}



// multiply A = B*A, inplace
BOOL MTX_PreMultiply_Inplace( MTX *A, const MTX* B )
{
  MTX M;
  MTX_Init( &M );

  if( !MTX_Multiply( &M, B, A ) )
  {
    MTX_ERROR_MSG( "MTX_Multiply returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  if( !MTX_Copy( &M, A ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  MTX_Free( &M );
  return TRUE;
}

BOOL MTX_TransposePreMultiply_Inplace( MTX *A, const MTX *B )
{
  MTX M;
  MTX_Init( &M );

  if( !MTX_TransposeMultiply( &M, B, A ) )
  {
    MTX_ERROR_MSG( "MTX_TransposeMultiply returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  if( !MTX_Copy( &M, A ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }
  MTX_Free( &M );
  return TRUE;
}

BOOL MTX_DotMultiply_Inplace( MTX *A, const MTX* B )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0;
  double im = 0;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( A->nrows == 1 && A->ncols == 1 )
    {
      if( A->isReal )
      {
        re = A->data[0][0];
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, re );
      }
      else
      {
        re = A->cplx[0][0].re;
        im = A->cplx[0][0].im;
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, re, im );
      }
    }
    else if( B->nrows == 1 && B->ncols == 1 )
    {
      if( B->isReal )
      {
        return MTX_Multiply_Scalar( A, B->data[0][0] );
      }
      else
      {
        return MTX_Multiply_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
  }

  if( !MTX_isConformalForAddition( A, B ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( A->isReal && !B->isReal )
  {
    if( !MTX_ConvertRealToComplex( A ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }
  
  if( A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )    
      for( i = 0; i < A->nrows; i++ )
        A->data[j][i] *= B->data[j][i];
  }
  else if( !A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )    
    {
      for( i = 0; i < A->nrows; i++ )
      {
        re = A->cplx[j][i].re * B->cplx[j][i].re - A->cplx[j][i].im * B->cplx[j][i].im;
        im = A->cplx[j][i].re * B->cplx[j][i].im + A->cplx[j][i].im * B->cplx[j][i].re;
        A->cplx[j][i].re = re;
        A->cplx[j][i].im = im;
      }
    }
  }
  else // !A->isReal && B->isReal
  {
    for( j = 0; j < A->ncols; j++ )    
    {
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re *= B->data[j][i];
        A->cplx[j][i].im *= B->data[j][i];
      }
    }
  }

  return TRUE;
}

BOOL MTX_DotDivide_Inplace( MTX *A, const MTX* B )
{
  unsigned i = 0;
  unsigned j = 0;
  double re = 0;
  double im = 0;
  double mag = 0;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( A->nrows == 1 && A->ncols == 1 && B->nrows == 1 && B->ncols == 1 )
  {
    if( !A->isReal && !B->isReal )
    {
      re = A->cplx[0][0].re;
      im = A->cplx[0][0].im;
      MTX_static_quick_complex_divide( &re, &im, &B->cplx[0][0].re, &B->cplx[0][0].im, &A->cplx[0][0].re, &A->cplx[0][0].im );
      return TRUE;
    }
    else if( A->isReal && B->isReal )
    {
      A->data[0][0] /= B->data[0][0];
      return TRUE;
    }
    else
    {
      if( A->isReal )
      {
        // B is complex
        re = A->data[0][0];
        mag = B->cplx[0][0].re*B->cplx[0][0].re + B->cplx[0][0].im*B->cplx[0][0].im;
        if( !MTX_Malloc(A,1,1,FALSE) )
        {
          MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
          return FALSE;
        }

        A->cplx[0][0].re = re*B->cplx[0][0].re / mag;
        A->cplx[0][0].im = -re*B->cplx[0][0].im / mag;
      }
      else
      {
        re = B->data[0][0];
        // B is real
        A->cplx[0][0].re /= re;
        A->cplx[0][0].im /= re;
      }
    }
    return TRUE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( A->nrows == 1 && A->ncols == 1 )
    {
      // Make A the same dimensions as B and filled with A's scalar value.
      // Then compute A./B.
      if( A->isReal )
      {
        // make A the same dimensions as B and filled with A's scalar value
        re = A->data[0][0];
        if( !MTX_Malloc( A, B->nrows, B->ncols, B->isReal ) )
        {
          MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
          return FALSE;
        }
        if( !MTX_Fill( A, re ) )
        {
          MTX_ERROR_MSG( "MTX_Fill returned FALSE." );
          return FALSE;
        }
        return MTX_DotDivide_Inplace( A, B );
      }
      else
      {
        re = A->cplx[0][0].re;
        im = A->cplx[0][0].im;
        if( !MTX_Malloc( A, B->nrows, B->ncols, FALSE ) )
        {
          MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
          return FALSE;
        }
        if( !MTX_FillComplex( A, re, im ) )
        {
          MTX_ERROR_MSG( "MTX_FillComplex returned FALSE." );
          return FALSE;
        }
        return MTX_DotDivide_Inplace( A, B );
      }
    }
    else if( B->nrows == 1 && B->ncols == 1 )
    {
      if( B->isReal )
      {
        return MTX_Divide_Scalar( A, B->data[0][0] );
      }
      else
      {
        return MTX_Divide_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
  }

  if( !MTX_isConformalForAddition( A, B ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( A->isReal && !B->isReal )
  {
    if( !MTX_ConvertRealToComplex( A ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  if( A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        A->data[j][i] /= B->data[j][i];
      }
    }
  }
  else if( !A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        mag = B->cplx[j][i].re*B->cplx[j][i].re + B->cplx[j][i].im*B->cplx[j][i].im;

        re = (A->cplx[j][i].re * B->cplx[j][i].re + A->cplx[j][i].im * B->cplx[j][i].im ) / mag;
        im = (A->cplx[j][i].im * B->cplx[j][i].re - A->cplx[j][i].re * B->cplx[j][i].im ) / mag;

        A->cplx[j][i].re = re;
        A->cplx[j][i].im = im;
      }
    }
  }
  else // !A->isReal && B->isReal
  {
    for( j = 0; j < A->ncols; j++ )
    {    
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re /= B->data[j][i];
        A->cplx[j][i].im /= B->data[j][i];
      }
    }
  }
  return TRUE;
}

BOOL MTX_Add( MTX *A, const MTX* B, const MTX* C )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( C ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( B->nrows == 1 && B->ncols == 1 )
    {
      if( B->isReal )
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_Scalar( A, B->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
    else if( C->nrows == 1 && C->ncols == 1 )
    {
      if( C->isReal )
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_Scalar( A, C->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Add_ScalarComplex( A, C->cplx[0][0].re, C->cplx[0][0].im );
      }
    }
  }


  if( !MTX_isConformalForAddition( B, C ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( !B->isReal || !C->isReal )
  {
    // A will be complex
    if( !A->isReal )
    {
      if( !MTX_Resize( A, B->nrows, B->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Malloc( A, B->nrows, B->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
  }
  else // A will be real
  {
    if( !A->isReal )
    {
      if( !MTX_Malloc( A, B->nrows, B->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Resize( A, B->nrows, B->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
  }
  
  if( B->isReal && C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )    
      for( i = 0; i < A->nrows; i++ )
        A->data[j][i] = B->data[j][i] + C->data[j][i];
  }
  else if( !B->isReal && !C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->cplx[j][i].re + C->cplx[j][i].re;
        A->cplx[j][i].im = B->cplx[j][i].im + C->cplx[j][i].im;
      }
    }
  }
  else if( !B->isReal && C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {    
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->cplx[j][i].re + C->data[j][i];
        A->cplx[j][i].im = B->cplx[j][i].im;
      }
    }
  }
  else // ( B->isReal && !C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {    
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->data[j][i] + C->cplx[j][i].re;
        A->cplx[j][i].im = C->cplx[j][i].im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Subtract( MTX *A, const MTX* B, const MTX* C )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( C ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( B->nrows == 1 && B->ncols == 1 )
    {
      // Set A = -C then add B as a scalar.
      if( B->isReal )
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        if( !MTX_Negate( A ) )
        {
          MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
          return FALSE;
        }
        return MTX_Add_Scalar( A, B->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        if( !MTX_Negate( A ) )
        {
          MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
          return FALSE;
        }
        return MTX_Add_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
    else if( C->nrows == 1 && C->ncols == 1 )
    {
      // Set A = B, then subtract C as a scalar.
      if( C->isReal )
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Subtract_Scalar( A, C->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Subtract_ScalarComplex( A, C->cplx[0][0].re, C->cplx[0][0].im );
      }
    }
  }

  if( !MTX_isConformalForAddition( B, C ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  if( !B->isReal || !C->isReal )
  {
    // A will be complex
    if( !A->isReal )
    {
      // and is currently complex, so resize
      if( !MTX_Resize( A, B->nrows, B->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Malloc( A, B->nrows, B->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    // A will be real
    if( !A->isReal )
    {
      if( !MTX_Malloc( A, B->nrows, B->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      // and is currently real, so resize
      if( !MTX_Resize( A, B->nrows, B->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
  }
  
  if( B->isReal && C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )        
      for( i = 0; i < A->nrows; i++ )
        A->data[j][i] = B->data[j][i] - C->data[j][i];
  }
  else if( !B->isReal && !C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {        
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->cplx[j][i].re - C->cplx[j][i].re;
        A->cplx[j][i].im = B->cplx[j][i].im - C->cplx[j][i].im;
      }
    }
  }
  else if( !B->isReal && C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {        
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->cplx[j][i].re - C->data[j][i];
        A->cplx[j][i].im = B->cplx[j][i].im;
      }
    }
  }
  else // ( B->isReal && !C->isReal )
  {
    for( j = 0; j < B->ncols; j++ )
    {    
      for( i = 0; i < A->nrows; i++ )
      {
        A->cplx[j][i].re = B->data[j][i] - C->cplx[j][i].re;
        A->cplx[j][i].im = -1.0*C->cplx[j][i].im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Multiply( MTX *A, const MTX* B, const MTX* C )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( C ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( B->nrows == 1 && B->ncols == 1 )
    {
      // Set A = C then multiply B as a scalar.
      if( B->isReal )
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, B->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
    else if( C->nrows == 1 && C->ncols == 1 )
    {
      // Set A = B, then multiply C as a scalar.
      if( C->isReal )
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, C->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, C->cplx[0][0].re, C->cplx[0][0].im );
      }
    }
  }


  if( !MTX_isConformalForMultiplication( B, C ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForMultiplication returned FALSE." );
    return FALSE;
  }

  if( !B->isReal || !C->isReal )
  {
    // A will be complex
    if( !A->isReal )
    {
      // and is currently complex, so resize
      if( !MTX_Resize( A, B->nrows, C->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Malloc( A, B->nrows, C->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    // A will be real
    if( !A->isReal )
    {
      if( !MTX_Malloc( A, B->nrows, C->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      // and is currently real, so resize
      if( !MTX_Resize( A, B->nrows, C->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
  }



  if( B->isReal && C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->data[j][i] = B->data[k][i] * C->data[j][k];
        for( k = 1; k < B->ncols; k++ )
        {
          A->data[j][i] += B->data[k][i] * C->data[j][k];
        }
      }
    }
  }
  else if( !B->isReal && !C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[k][i].re * C->cplx[j][k].re - B->cplx[k][i].im * C->cplx[j][k].im;
        A->cplx[j][i].im = B->cplx[k][i].re * C->cplx[j][k].im + B->cplx[k][i].im * C->cplx[j][k].re;
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->cplx[k][i].re * C->cplx[j][k].re - B->cplx[k][i].im * C->cplx[j][k].im;
          A->cplx[j][i].im += B->cplx[k][i].re * C->cplx[j][k].im + B->cplx[k][i].im * C->cplx[j][k].re;
        }
      }
    }
  }
  else if( !B->isReal && C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[k][i].re * C->data[j][k];
        A->cplx[j][i].im = B->cplx[k][i].im * C->data[j][k];
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->cplx[k][i].re * C->data[j][k];
          A->cplx[j][i].im += B->cplx[k][i].im * C->data[j][k];
        }
      }
    }
  }
  else if( B->isReal && !C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->data[k][i] * C->cplx[j][k].re;
        A->cplx[j][i].im = B->data[k][i] * C->cplx[j][k].im;
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->data[k][i] * C->cplx[j][k].re;
          A->cplx[j][i].im += B->data[k][i] * C->cplx[j][k].im;
        }
      }
    }
  }

  return TRUE;
}

BOOL MTX_TransposeMultiply( MTX *A, const MTX* B, const MTX* C ) // A = trans(B) x C
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  MTX vec;
  MTX_Init( &vec );

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( C ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( B->nrows == 1 && B->ncols == 1 )
    {
      // Set A = C then multiply B as a scalar.
      if( B->isReal )
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, B->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
    else if( C->nrows == 1 && C->ncols == 1 )
    {
      // Set A = B, then multiply C as a scalar.
      if( C->isReal )
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, C->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, C->cplx[0][0].re, C->cplx[0][0].im );
      }
    }
  }

  // Check conformal for multiplication
  if( B->nrows != C->nrows ) 
  {
    MTX_ERROR_MSG( "Not conformal for multiplication." );
    return FALSE;
  }

  if( !B->isReal || !C->isReal )
  {
    // A will be complex
    if( !A->isReal )
    {
      // and is currently complex, so resize
      if( !MTX_Resize( A, B->ncols, C->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Malloc( A, B->ncols, C->ncols, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    // A will be real
    if( !A->isReal )
    {
      if( !MTX_Malloc( A, B->ncols, C->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      // and is currently real, so resize
      if( !MTX_Resize( A, B->ncols, C->ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
  }



  if( B->isReal && C->isReal )
  {
    for( i = 0; i < B->ncols; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->data[j][i] = B->data[i][k] * C->data[j][k];
        for( k = 1; k < B->nrows; k++ )
        {
          A->data[j][i] += B->data[i][k] * C->data[j][k];
        }
      }
    }
  }
  else if( !B->isReal && !C->isReal )
  {
    for( i = 0; i < B->ncols; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[i][k].re * C->cplx[j][k].re - B->cplx[i][k].im * C->cplx[j][k].im;
        A->cplx[j][i].im = B->cplx[i][k].re * C->cplx[j][k].im + B->cplx[i][k].im * C->cplx[j][k].re;
        for( k = 1; k < B->nrows; k++ )
        {
          A->cplx[j][i].re += B->cplx[i][k].re * C->cplx[j][k].re - B->cplx[i][k].im * C->cplx[j][k].im;
          A->cplx[j][i].im += B->cplx[i][k].re * C->cplx[j][k].im + B->cplx[i][k].im * C->cplx[j][k].re;
        }
      }
    }
  }
  else if( !B->isReal && C->isReal )
  {
    for( i = 0; i < B->ncols; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[i][k].re * C->data[j][k];
        A->cplx[j][i].im = B->cplx[i][k].im * C->data[j][k];
        for( k = 1; k < B->nrows; k++ )
        {
          A->cplx[j][i].re += B->cplx[i][k].re * C->data[j][k];
          A->cplx[j][i].im += B->cplx[i][k].im * C->data[j][k];
        }
      }
    }
  }
  else if( B->isReal && !C->isReal )
  {
    for( i = 0; i < B->ncols; i++ )
    {
      for( j = 0; j < C->ncols; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->data[i][k] * C->cplx[j][k].re;
        A->cplx[j][i].im = B->data[i][k] * C->cplx[j][k].im;
        for( k = 1; k < B->nrows; k++ )
        {
          A->cplx[j][i].re += B->data[i][k] * C->cplx[j][k].re;
          A->cplx[j][i].im += B->data[i][k] * C->cplx[j][k].im;
        }
      }
    }
  }

  return TRUE;
}


BOOL MTX_MultiplyTranspose( MTX *A, const MTX* B, const MTX* C ) // A = B*transpose(C)
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( C ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( B->nrows == 1 && B->ncols == 1 )
    {
      // Set A = C then multiply B as a scalar.
      if( B->isReal )
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, B->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( C, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, B->cplx[0][0].re, B->cplx[0][0].im );
      }
    }
    else if( C->nrows == 1 && C->ncols == 1 )
    {
      // Set A = B, then multiply C as a scalar.
      if( C->isReal )
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_Scalar( A, C->data[0][0] );
      }
      else
      {
        if( !MTX_Copy( B, A ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        return MTX_Multiply_ScalarComplex( A, C->cplx[0][0].re, C->cplx[0][0].im );
      }
    }
  }

  // check conformal for multiplication
  if( B->ncols != C->ncols )
  {
    MTX_ERROR_MSG( "Not conformal for multiplication." );
    return FALSE;
  }



  if( !B->isReal || !C->isReal )
  {
    // A will be complex
    if( !A->isReal )
    {
      // and is currently complex, so resize
      if( !MTX_Resize( A, B->nrows, C->nrows, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Malloc( A, B->nrows, C->nrows, FALSE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    // A will be real
    if( !A->isReal )
    {
      if( !MTX_Malloc( A, B->nrows, C->nrows, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      // and is currently real, so resize
      if( !MTX_Resize( A, B->nrows, C->nrows, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
        return FALSE;
      }
    }
  }

  if( B->isReal && C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->nrows; j++ )
      {
        k = 0;
        A->data[j][i] = B->data[k][i] * C->data[k][j];
        for( k = 1; k < B->ncols; k++ )
        {
          A->data[j][i] += B->data[k][i] * C->data[k][j];
        }
      }
    }
  }
  else if( !B->isReal && !C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->nrows; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[k][i].re * C->cplx[k][j].re - B->cplx[k][i].im * C->cplx[k][j].im;
        A->cplx[j][i].im = B->cplx[k][i].re * C->cplx[k][j].im + B->cplx[k][i].im * C->cplx[k][j].re;
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->cplx[k][i].re * C->cplx[k][j].re - B->cplx[k][i].im * C->cplx[k][j].im;
          A->cplx[j][i].im += B->cplx[k][i].re * C->cplx[k][j].im + B->cplx[k][i].im * C->cplx[k][j].re;
        }
      }
    }
  }
  else if( !B->isReal && C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->nrows; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->cplx[k][i].re * C->data[k][j];
        A->cplx[j][i].im = B->cplx[k][i].im * C->data[k][j];
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->cplx[k][i].re * C->data[k][j];
          A->cplx[j][i].im += B->cplx[k][i].im * C->data[k][j];
        }
      }
    }
  }
  else if( B->isReal && !C->isReal )
  {
    for( i = 0; i < B->nrows; i++ )
    {
      for( j = 0; j < C->nrows; j++ )
      {
        k = 0;
        A->cplx[j][i].re = B->data[k][i] * C->cplx[k][j].re;
        A->cplx[j][i].im = B->data[k][i] * C->cplx[k][j].im;
        for( k = 1; k < B->ncols; k++ )
        {
          A->cplx[j][i].re += B->data[k][i] * C->cplx[k][j].re;
          A->cplx[j][i].im += B->data[k][i] * C->cplx[k][j].im;
        }
      }
    }
  }
  return TRUE;
}



BOOL MTX_IsEqual( const MTX *A, const MTX *B, const double tolerance, BOOL *isEqual )
{
  unsigned i = 0;
  unsigned j = 0;
  double fabsval;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( MTX_static_global_treat_1x1_as_scalar )
  {
    if( A->nrows == 1 && A->ncols == 1 )
    {
      if( A->isReal )
      {
        if( B->isReal )
        {
          for( j = 0; j < B->ncols; j++ )
          {
            for( i = 0; i < B->nrows; i++ )
            {
              fabsval = fabs( A->data[0][0] - B->data[j][i] );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }
            }
          }
          *isEqual = TRUE;
          return TRUE;
        }
        else
        {
          // Check if there are any complex values larger than the tolerance.
          for( j = 0; j < B->ncols; j++ )
          {
            for( i = 0; i < B->nrows; i++ )
            {
              fabsval = fabs( A->data[0][0] - B->cplx[j][i].re );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }

              fabsval = fabs( B->cplx[j][i].im );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }
            }
          }
          *isEqual = TRUE;
          return TRUE;
        }
      }
      else
      {
        if( B->isReal )
        {
          fabsval = fabs( A->cplx[0][0].im );
          if( fabsval > tolerance )
          {
            *isEqual = FALSE;
            return TRUE;
          }

          for( j = 0; j < B->ncols; j++ )
          {
            for( i = 0; i < B->nrows; i++ )
            {
              fabsval = fabs( A->cplx[0][0].re - B->data[j][i] );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }
            }
          }
          *isEqual = TRUE;
          return TRUE;
        }
        else
        {
          // Check if there are any complex values larger than the tolerance.
          for( j = 0; j < B->ncols; j++ )
          {
            for( i = 0; i < B->nrows; i++ )
            {
              fabsval = fabs( A->cplx[0][0].re - B->cplx[j][i].re );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }

              fabsval = fabs( A->cplx[0][0].im - B->cplx[j][i].im );
              if( fabsval > tolerance )
              {
                *isEqual = FALSE;
                return TRUE;
              }
            }
          }
          *isEqual = TRUE;
          return TRUE;
        }
      } // if( A->isReal ) else
    }
    else if( B->nrows == 1 && B->ncols == 1 )
    {
      // Cheat a little by calling this function again but reorder the arguments.
      return MTX_IsEqual( B, A, tolerance, isEqual );
    }
  }

  if( !MTX_isConformalForAddition( A, B ) )
  {
    MTX_ERROR_MSG( "MTX_isConformalForAddition returned FALSE." );
    return FALSE;
  }

  *isEqual = TRUE;

  if( A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        fabsval = fabs( A->data[j][i] - B->data[j][i] );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
      }
    }
  }
  else if( !A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        fabsval = fabs( A->cplx[j][i].re - B->cplx[j][i].re );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
        fabsval = fabs( A->cplx[j][i].im - B->cplx[j][i].im );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
      }
    }
  }
  else if( !A->isReal && B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        fabsval = fabs( A->cplx[j][i].re - B->data[j][i] );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
        fabsval = fabs( A->cplx[j][i].im );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
      }
    }
  }
  else if( A->isReal && !B->isReal )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      for( i = 0; i < A->nrows; i++ )
      {
        fabsval = fabs( A->data[j][i] - B->cplx[j][i].re );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
        fabsval = fabs( B->cplx[j][i].im );
        if( fabsval > tolerance )
        {
          *isEqual = FALSE;
          return TRUE;
        }
      }
    }
  }

  return TRUE;
}

BOOL MTX_ColumnDiff( const MTX *M, MTX *Diff, const unsigned col )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->nrows == 1 )
  {
    MTX_ERROR_MSG( "if( M->nrows == 1 )" );
    return FALSE;
  }

  if( M->isReal != Diff->isReal )
  {
    if( !MTX_Resize( Diff, M->nrows-1, 1, M->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  else if( Diff->nrows != M->nrows-1 || Diff->ncols != 1 )
  {
    if( !MTX_Resize( Diff, M->nrows-1, 1, M->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  
  if( M->isReal )
  {
    for( i = 0; i < M->nrows-1; i++ )
    {
      Diff->data[0][i] = M->data[col][i+1] - M->data[col][i];
    }
  }
  else
  {
    for( i = 0; i < M->nrows-1; i++ )
    {
      Diff->cplx[0][i].re = M->cplx[col][i+1].re - M->cplx[col][i].re;
      Diff->cplx[0][i].im = M->cplx[col][i+1].im - M->cplx[col][i].im;
    }
  }

  return TRUE;
}

BOOL MTX_Diff( const MTX *M, MTX *Diff )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->nrows == 1 )
  {
    MTX_ERROR_MSG( "if( M->nrows == 1 )" );
    return FALSE;
  }

  if( M->isReal != Diff->isReal )
  {
    if( !MTX_Resize( Diff, M->nrows-1, M->ncols, M->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  else if( Diff->nrows != M->nrows-1 || Diff->ncols != M->ncols )
  {
    if( !MTX_Resize( Diff, M->nrows-1, M->ncols, M->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }


  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows-1; i++ )
      {
        Diff->data[j][i] = M->data[j][i+1] - M->data[j][i];
      }
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows-1; i++ )
      {
        Diff->cplx[j][i].re = M->cplx[j][i+1].re - M->cplx[j][i].re;
        Diff->cplx[j][i].im = M->cplx[j][i+1].im - M->cplx[j][i].im;
      }
    }
  }

  return TRUE;
}





//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
// Statistics Operations
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//


BOOL MTX_MaxColIndex( const MTX *M, const unsigned col, double *re, double *im, unsigned *row )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    *im = 0;
    *re = M->data[col][0];
    *row = 0;
    for( i = 1; i < M->nrows; i++ )
    {
      if( M->data[col][i] > *re )
      {
        *re = M->data[col][i];
        *row = i;
      }
    }
  }
  else
  {
    MTX C; // the column to search
    MTX mag; // the magnitude of that column
    MTX_Init( &C );
    MTX_Init( &mag );

    if( !MTX_CopyColumn( M, col, &C ) )
    {
      MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
      return FALSE;
    }
    if( !MTX_Magnitude( &C, &mag ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &C );
      return FALSE;
    }

    *re = 0;
    *im = 0;
    *re = mag.data[0][0];
    *row = 0;
    for( i = 1; i < mag.nrows; i++ )
    {
      if( mag.data[0][i] > *re )
      {
        *re = mag.data[0][i];
        *row = i;
      }
    }

    *re = C.cplx[0][*row].re;
    *im = C.cplx[0][*row].im;

    MTX_Free( &C );
    MTX_Free( &mag );
  }

  return TRUE;
}

BOOL MTX_MaxRowIndex( const MTX *M, const unsigned row, double *re, double *im, unsigned *col )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( M->isReal )
  {
    *im = 0;
    *re = M->data[0][row];
    *col = 0;
    for( i = 1; i < M->ncols; i++ )
    {
      if( M->data[i][row] > *re )
      {
        *re = M->data[i][row];
        *col = i;
      }
    }
  }
  else
  {
    MTX C; // A column matrix (nx1) that is 1xn row of M (transposed).
    MTX mag; // The magnitude of that column.
    MTX_Init( &C );
    MTX_Init( &mag );

    if( !MTX_CopyRowIntoAColumnMatrix( M, row, &C ) )
    {
      MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
      return FALSE;
    }
    if( !MTX_Magnitude( &C, &mag ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &C );
      return FALSE;
    }

    *re = 0;
    *im = 0;
    *re = mag.data[0][0];
    *col = 0;
    for( i = 1; i < mag.nrows; i++ )
    {
      if( mag.data[0][i] > *re )
      {
        *re = mag.data[0][i];
        *col = i;
      }
    }

    *re = M->cplx[*col][row].re;
    *im = M->cplx[*col][row].im;

    MTX_Free( &C );
    MTX_Free( &mag );
  }

  return TRUE;
}

BOOL MTX_MinColIndex( const MTX *M, const unsigned col, double *re, double *im, unsigned *row )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    *im = 0;
    *re = M->data[col][0];
    *row = 0;
    for( i = 1; i < M->nrows; i++ )
    {
      if( M->data[col][i] < *re )
      {
        *re = M->data[col][i];
        *row = i;
      }
    }
  }
  else
  {
    MTX C; // the column to search
    MTX mag; // the magnitude of that column
    MTX_Init( &C );
    MTX_Init( &mag );

    if( !MTX_CopyColumn( M, col, &C ) )
    {
      MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
      return FALSE;
    }
    if( !MTX_Magnitude( &C, &mag ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &C );
      return FALSE;
    }

    *re = 0;
    *im = 0;
    *re = mag.data[0][0];
    *row = 0;
    for( i = 1; i < mag.nrows; i++ )
    {
      if( mag.data[0][i] < *re )
      {
        *re = mag.data[0][i];
        *row = i;
      }
    }

    *re = C.cplx[0][*row].re;
    *im = C.cplx[0][*row].im;

    MTX_Free( &C );
    MTX_Free( &mag );
  }

  return TRUE;
}

BOOL MTX_MinRowIndex( const MTX *M, const unsigned row, double *re, double *im, unsigned *col )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( M->isReal )
  {
    *im = 0;
    *re = M->data[0][row];
    *col = 0;
    for( i = 1; i < M->ncols; i++ )
    {
      if( M->data[i][row] < *re )
      {
        *re = M->data[i][row];
        *col = i;
      }
    }
  }
  else
  {
    MTX C; // A column matrix (nx1) that is 1xn row of M (transposed).
    MTX mag; // The magnitude of that column.
    MTX_Init( &C );
    MTX_Init( &mag );

    if( !MTX_CopyRowIntoAColumnMatrix( M, row, &C ) )
    {
      MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
      return FALSE;
    }
    if( !MTX_Magnitude( &C, &mag ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &C );
      return FALSE;
    }

    *re = 0;
    *im = 0;
    *re = mag.data[0][0];
    *col = 0;
    for( i = 1; i < mag.nrows; i++ )
    {
      if( mag.data[0][i] < *re )
      {
        *re = mag.data[0][i];
        *col = i;
      }
    }

    *re = M->cplx[*col][row].re;
    *im = M->cplx[*col][row].im;

    MTX_Free( &C );
    MTX_Free( &mag );
  }

  return TRUE;
}

BOOL MTX_MaxAbsColIndex( const MTX *M, const unsigned col, double *value, unsigned *row )
{
  double re;
  double im;
  MTX copyCol; // A copy of the column to search.

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  // init the copy
  MTX_Init( &copyCol );

  // make a copy
  if( !MTX_CopyColumn( M, col, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
    return FALSE;
  }

  // take the abs
  if( !MTX_Abs( &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_Abs returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // find the maximum
  if( !MTX_MaxColIndex( &copyCol, 0, &re, &im, row ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColIndex returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  *value = re;

  MTX_Free( &copyCol );
  return TRUE;
}

BOOL MTX_MaxAbsRowIndex( const MTX *M, const unsigned row, double *value, unsigned *col )
{
  double re;
  double im;
  MTX copyCol; // A copy of the row to search transposed into a column.

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  // init the copy
  MTX_Init( &copyCol );

  // make a copy or the row into a column matrix for faster searching.
  if( !MTX_CopyRowIntoAColumnMatrix( M, row, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // take the abs
  if( !MTX_Abs( &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_Abs returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // find the maximum
  if( !MTX_MaxColIndex( &copyCol, 0, &re, &im, col ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColIndex returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  *value = re;

  MTX_Free( &copyCol );
  return TRUE;
}

BOOL MTX_MinAbsColIndex( const MTX *M, const unsigned col, double *value, unsigned *row )
{
  double re;
  double im;
  MTX copyCol; // A copy of the column to search.

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  // init the copy
  MTX_Init( &copyCol );

  // make a copy
  if( !MTX_CopyColumn( M, col, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // take the abs
  if( !MTX_Abs( &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_Abs returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // find the maximum
  if( !MTX_MinColIndex( &copyCol, 0, &re, &im, row ) )
  {
    MTX_ERROR_MSG( "MTX_MinColIndex returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  *value = re;

  MTX_Free( &copyCol );
  return TRUE;
}

BOOL MTX_MinAbsRowIndex( const MTX *M, const unsigned row, double *value, unsigned *col )
{
  double re;
  double im;
  MTX copyCol; // A copy of the row to search transposed into a column.

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  // init the copy
  MTX_Init( &copyCol );

  // make a copy or the row into a column matrix for faster searching.
  if( !MTX_CopyRowIntoAColumnMatrix( M, row, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // take the abs
  if( !MTX_Abs( &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_Abs returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  // find the maximum
  if( !MTX_MinColIndex( &copyCol, 0, &re, &im, col ) )
  {
    MTX_ERROR_MSG( "MTX_MinColIndex returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  *value = re;

  MTX_Free( &copyCol );
  return TRUE;
}

BOOL MTX_MaxColumn( const MTX *M, const unsigned col, double *re, double *im )
{
  unsigned row;
  return MTX_MaxColIndex( M, col, re, im, &row );
}

BOOL MTX_MaxRow( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned col;
  return MTX_MaxRowIndex( M, row, re, im, &col );
}

BOOL MTX_MinColumn( const MTX *M, const unsigned col, double *re, double *im )
{
  unsigned row;
  return MTX_MinColIndex( M, col, re, im, &row );
}

BOOL MTX_MinRow( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned col;
  return MTX_MinRowIndex( M, row, re, im, &col );
}

BOOL MTX_MaxAbsColumn( const MTX *M, const unsigned col, double *value )
{
  unsigned row;
  return MTX_MaxAbsColIndex( M, col, value, &row );
}

BOOL MTX_MaxAbsRow( const MTX *M, const unsigned row, double *value )
{
  unsigned col;
  return MTX_MaxAbsRowIndex( M, row, value, &col );
}

BOOL MTX_MinAbsColumn( const MTX *M, const unsigned col, double *value )
{
  unsigned row;
  return MTX_MinAbsColIndex( M, col, value, &row );
}

BOOL MTX_MinAbsRow( const MTX *M, const unsigned row, double *value )
{
  unsigned col;
  return MTX_MinAbsRowIndex( M, row, value, &col );
}

BOOL MTX_MaxAbsIndex( const MTX *M, double* value, unsigned *row, unsigned *col )
{
  unsigned j = 0;
  unsigned trow = 0;
  double tval = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *col = 0;
  if( !MTX_MaxAbsColIndex( M, 0, value, row ) )
  {
    MTX_ERROR_MSG( "MTX_MaxAbsColIndex returned FALSE." );
    return FALSE;
  }

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MaxAbsColIndex( M, j, &tval, &trow ) )
    {
      MTX_ERROR_MSG( "MTX_MaxAbsColIndex returned FALSE." );
      return FALSE;
    }

    if( tval > *value )
    {
      *value = tval;
      *row = trow;
      *col = j;
    }
  }
  return TRUE;
}


BOOL MTX_MaxIndex( const MTX *M, double *re, double *im, unsigned *row, unsigned *col )
{
  unsigned j = 0;
  unsigned trow = 0;
  double tre = 0.0;
  double tim = 0.0;
  double tmp = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *col = 0;
  if( !MTX_MaxColIndex( M, 0, re, im, row ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColIndex returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
    tmp = *re;
  else
    tmp = (*re)*(*re) + (*im)*(*im);

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MaxColIndex( M, j, &tre, &tim, &trow ) )
    {
      MTX_ERROR_MSG( "MTX_MaxColIndex returned FALSE." );
      return FALSE;
    }

    if( M->isReal )
    {
      if( tre > tmp )
      {
        *re = tre;
        tmp = *re;
        *row = trow;
        *col = j;
      }
    }
    else
    {
      if( (tre*tre+tim*tim) > tmp )
      {
        *re = tre;
        *im = tim;
        tmp = (*re)*(*re) + (*im)*(*im);
        *row = trow;
        *col = j;
      }
    }
  }
  return TRUE;
}


BOOL MTX_MaxAbs( const MTX *M, double* value )
{
  unsigned row;
  unsigned col;
  return MTX_MaxAbsIndex( M, value, &row, &col );
}


BOOL MTX_Max( const MTX *M, double *re, double *im )
{
  unsigned j = 0;
  double tre = 0.0;
  double tim = 0.0;
  double tmp = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_MaxColumn( M, 0, re, im ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
    return FALSE;
  }
  if( M->isReal )
    tmp = *re;
  else
    tmp = (*re)*(*re) + (*im)*(*im);

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MaxColumn( M, j, &tre, &tim ) )
    {
      MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
      return FALSE;
    }

    if( M->isReal )
    {
      if( tre > tmp )
      {
        *re = tre;
        tmp = *re;
      }
    }
    else
    {
      if( (tre*tre+tim*tim) > tmp )
      {
        *re = tre;
        *im = tim;
        tmp = (*re)*(*re) + (*im)*(*im);
      }
    }
  }
  return TRUE;
}


BOOL MTX_MinAbsIndex( const MTX *M, double* value, unsigned *row, unsigned *col )
{
  unsigned j = 0;
  unsigned trow = 0;
  double tval = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *col = 0;
  if( !MTX_MinAbsColIndex( M, 0, value, row ) )
  {
    MTX_ERROR_MSG( "MTX_MinAbsColIndex returned FALSE." );
    return FALSE;
  }

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MinAbsColIndex( M, j, &tval, &trow ) )
    {
      MTX_ERROR_MSG( "MTX_MinAbsColIndex returned FALSE." );
      return FALSE;
    }

    if( tval < *value )
    {
      *value = tval;
      *row = trow;
      *col = j;
    }
  }
  return TRUE;
}

BOOL MTX_MinAbs( const MTX *M, double* value )
{
  unsigned row;
  unsigned col;
  return MTX_MinAbsIndex( M, value, &row, &col );
}

BOOL MTX_MinIndex( const MTX *M, double *re, double *im, unsigned *row, unsigned *col )
{
  unsigned j = 0;
  unsigned trow = 0;
  double tre = 0.0;
  double tim = 0.0;
  double tmp = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *col = 0;
  if( !MTX_MinColIndex( M, 0, re, im, row ) )
  {
    MTX_ERROR_MSG( "MTX_MinColIndex returned FALSE." );
    return FALSE;
  }
  if( M->isReal )
    tmp = *re;
  else
    tmp = (*re)*(*re) + (*im)*(*im);

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MinColIndex( M, j, &tre, &tim, &trow ) )
    {
      MTX_ERROR_MSG( "MTX_MinColIndex returned FALSE." );
      return FALSE;
    }

    if( M->isReal )
    {
      if( tre < tmp )
      {
        *re = tre;
        tmp = *re;
        *row = trow;
        *col = j;
      }
    }
    else
    {
      if( (tre*tre+tim*tim) < tmp )
      {
        *re = tre;
        *im = tim;
        tmp = (*re)*(*re) + (*im)*(*im);
        *row = trow;
        *col = j;
      }
    }
  }
  return TRUE;
}

BOOL MTX_Min( const MTX *M, double *re, double *im )
{
  unsigned j = 0;
  double tre = 0.0;
  double tim = 0.0;
  double tmp = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_MinColumn( M, 0, re, im ) )
  {
    MTX_ERROR_MSG( "MTX_MinColumn returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
    tmp = *re;
  else
    tmp = (*re)*(*re) + (*im)*(*im);

  for( j = 1; j < M->ncols; j++ )
  {
    if( !MTX_MinColumn( M, j, &tre, &tim ) )
    {
      MTX_ERROR_MSG( "MTX_MinColumn returned FALSE." );
      return FALSE;
    }

    if( M->isReal )
    {
      if( tre < tmp )
      {
        *re = tre;
        tmp = *re;
      }
    }
    else
    {
      if( (tre*tre+tim*tim) < tmp )
      {
        *re = tre;
        *im = tim;
        tmp = (*re)*(*re) + (*im)*(*im);
      }
    }
  }
  return TRUE;
}

BOOL MTX_ColumnRange( const MTX *M, const unsigned col, double *re, double *im )
{
  double re_maxval = 0;
  double im_maxval = 0;
  double re_minval = 0;
  double im_minval = 0;
  int minIndx = 0;
  int maxIndx = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_MinColumn( M, col, &re_minval, &im_minval ) )
  {
    MTX_ERROR_MSG( "MTX_MinColumn returned FALSE." );
    return FALSE;
  }
  if( !MTX_MaxColumn( M, col, &re_maxval, &im_maxval ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
    return FALSE;
  }

  *re = re_maxval - re_minval;
  *im = im_maxval - im_minval;
  return TRUE;
}

BOOL MTX_RowRange( const MTX *M, const unsigned row, double *re, double *im )
{
  double re_maxval = 0;
  double im_maxval = 0;
  double re_minval = 0;
  double im_minval = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( !MTX_MinRow( M, row, &re_minval, &im_minval ) )
  {
    MTX_ERROR_MSG( "MTX_MinRow returned FALSE." );
    return FALSE;
  }
  if( !MTX_MaxRow( M, row, &re_maxval, &im_maxval ) )
  {
    MTX_ERROR_MSG( "MTX_MaxRow returned FALSE." );
    return FALSE;
  }

  *re = re_maxval - re_minval;
  *im = im_maxval - im_minval;
  return TRUE;
}


BOOL MTX_Range( const MTX *M, double *re, double *im )
{
  double re_maxval = 0;
  double im_maxval = 0;
  double re_minval = 0;
  double im_minval = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Min( M, &re_minval, &im_minval ) )
  {
    MTX_ERROR_MSG( "MTX_Min returned FALSE." );
    return FALSE;
  }
  if( !MTX_Max( M, &re_maxval, &im_maxval ) )
  {
    MTX_ERROR_MSG( "MTX_Max returned FALSE." );
    return FALSE;
  }

  *re = re_maxval - re_minval;
  *im = im_maxval - im_minval;
  return TRUE;
}


BOOL MTX_ColumnSum( const MTX *M, const unsigned col, double *re, double *im )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *re += M->data[col][i];
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *re += M->cplx[col][i].re;
      *im += M->cplx[col][i].im;
    }
  }
  return TRUE;
}


BOOL MTX_ColumnSumAbs( const MTX *M, const unsigned col, double *value )
{
  unsigned i = 0;
  MTX copyCol;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  MTX_Init(&copyCol);

  if( !MTX_CopyColumn( M, col, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  *value = 0;

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *value += fabs(M->data[col][i]);
    }
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *value += sqrt( M->cplx[col][i].re*M->cplx[col][i].re + M->cplx[col][i].im*M->cplx[col][i].im );
    }
  }
  return TRUE;
}



BOOL MTX_RowSum( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      *re += M->data[j][row];
    }
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      *re += M->cplx[j][row].re;
      *im += M->cplx[j][row].im;
    }
  }
  return TRUE;
}

BOOL MTX_Sum( const MTX *M, double *re, double *im )
{
  unsigned j = 0;
  double sumre = 0.0;
  double sumim = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  for( j = 0; j < M->ncols; j++ )
  {
    if( !MTX_ColumnSum( M, j, &sumre, &sumim ) )
    {
      MTX_ERROR_MSG( "MTX_ColumnSum returned FALSE." );
      return FALSE;
    }

    *re += sumre;
    *im += sumim;
  }
  return TRUE;
}


BOOL MTX_ColumnMean( const MTX *M, const unsigned col, double *re, double *im )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *re += M->data[col][i];
    }
    *re /= (double)(M->nrows);
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *re += M->cplx[col][i].re;
      *im += M->cplx[col][i].im;
    }
    *re /= (double)(M->nrows);
    *im /= (double)(M->nrows);
  }
  return TRUE;
}

BOOL MTX_RowMean( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      *re += M->data[j][row];
    }
    *re /= (double)(M->ncols);
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      *re += M->cplx[j][row].re;
      *im += M->cplx[j][row].im;
    }
    *re /= (double)(M->ncols);
    *im /= (double)(M->ncols);
  }

  return TRUE;
}


BOOL MTX_Mean( const MTX *M, double *re, double *im )
{
  double sumre = 0.0;
  double sumim = 0.0;
  double n = (double)(M->nrows*M->ncols);

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( !MTX_Sum( M, &sumre, &sumim ) )
  {
    MTX_ERROR_MSG( "MTX_Sum returned FALSE." );
    return FALSE;
  }

  *re = sumre/n;
  *im = sumim/n;

  return TRUE;
}

BOOL MTX_ColumnStdev( const MTX *M, const unsigned col, double *value )
{
  unsigned i = 0;
  double n = 0;
  double sumx2 = 0;
  double sumx_re = 0;
  double sumx_im = 0;
  double var = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  // special case
  if( M->nrows == 1 )
  {
    *value = 0.0;
    return TRUE;
  }

  if( M->isReal)
  {
    n = M->nrows;
    for( i = 0; i < M->nrows; i++ )
    {
      sumx_re += M->data[col][i];
      sumx2 += M->data[col][i]*M->data[col][i];
    }
    if( MTX_IsPostiveINF( sumx2 ) )
    {
      *value = MTX_POS_INF;
    }
    else
    {
      var = (n*sumx2 - sumx_re*sumx_re) / (n*(n-1.0));
      *value = sqrt(var);
    }
  }
  else
  {
    n = M->nrows;
    for( i = 0; i < M->nrows; i++ )
    {
      sumx_re += M->cplx[col][i].re;
      sumx_im += M->cplx[col][i].im;
      sumx2 += M->cplx[col][i].re*M->cplx[col][i].re + M->cplx[col][i].im*M->cplx[col][i].im;
    }
    if( MTX_IsPostiveINF( sumx2 ) )
    {
      *value = MTX_POS_INF;
    }
    else
    {
      sumx_re = sqrt( sumx_re*sumx_re + sumx_im*sumx_im );
      var = (n*sumx2 - sumx_re*sumx_re) / (n*(n-1.0));
      *value = sqrt(var);
    }
  }
  return TRUE;
}

BOOL MTX_RowStdev( const MTX *M, const unsigned row, double *value )
{
  MTX copyCol; // A column copy = row vector of M, i.e. row tranposed
  MTX_Init( &copyCol );
  if( !MTX_CopyRowIntoAColumnMatrix( M, row, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  if( !MTX_ColumnStdev( &copyCol, 0, value ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }
  MTX_Free( &copyCol );
  return TRUE;
}

BOOL MTX_Stdev( const MTX *M, double *value )
{
  unsigned i = 0;
  unsigned j = 0;
  double n = M->nrows*M->ncols;
  double sumx2 = 0;
  double sumx_re = 0;
  double sumx_im = 0;
  double var = 0;

  *value = 0.0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // special case
  if( M->nrows == 1 && M->ncols == 1 )
    return TRUE;



  if( M->isReal)
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        sumx_re += M->data[j][i];
        sumx2 += M->data[j][i]*M->data[j][i];
      }
    }
    var = (n*sumx2 - sumx_re*sumx_re) / (n*(n-1.0));
    *value = sqrt(var);
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        sumx_re += M->cplx[j][i].re;
        sumx_im += M->cplx[j][i].im;
        sumx2 += M->cplx[j][i].re*M->cplx[j][i].re + M->cplx[j][i].im*M->cplx[j][i].im;
      }
    }
    sumx_re = sqrt( sumx_re*sumx_re + sumx_im*sumx_im );
    var = (n*sumx2 - sumx_re*sumx_re) / (n*(n-1.0));
    *value = sqrt(var);
  }


  return TRUE;
}

BOOL MTX_ColumnVar( const MTX *M, const unsigned col, double *value )
{
  if( !MTX_ColumnStdev( M, col, value ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
    return FALSE;
  }
  // square the result
  *value *= *value;
  return TRUE;
}

BOOL MTX_RowVar( const MTX *M, const unsigned row, double *value )
{
  if( !MTX_RowStdev( M, row, value ) )
  {
    MTX_ERROR_MSG( "MTX_RowStdev returned FALSE." );
    return FALSE;
  }
  // square the result
  *value *= *value;
  return TRUE;
}

BOOL MTX_Var( const MTX *M, double *value )
{
  if( !MTX_Stdev( M, value ) )
  {
    MTX_ERROR_MSG( "MTX_Stdev returned FALSE." );
    return FALSE;
  }
  // square the result
  *value *= *value;
  return TRUE;
}

BOOL MTX_ColumnNorm( const MTX *M, const unsigned col, double *value )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  *value = 0;
  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *value += M->data[col][i] * M->data[col][i];
    }
    *value = sqrt(*value);
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *value += M->cplx[col][i].re * M->cplx[col][i].re + M->cplx[col][i].im * M->cplx[col][i].im;
    }
    *value = sqrt(*value);
  }
  return TRUE;
}

BOOL MTX_RowNorm( const MTX *M, const unsigned row, double *value )
{
  MTX copyCol; // A column copy = row vector of M, i.e. row tranposed
  MTX_Init( &copyCol );
  if( !MTX_CopyRowIntoAColumnMatrix( M, row, &copyCol ) )
  {
    MTX_ERROR_MSG( "MTX_CopyRowIntoAColumnMatrix returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }

  if( !MTX_ColumnNorm( &copyCol, 0, value ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnNorm returned FALSE." );
    MTX_Free( &copyCol );
    return FALSE;
  }
  MTX_Free( &copyCol );
  return TRUE;
}


BOOL MTX_Norm( const MTX *M, double *value )
{
  unsigned i = 0;
  unsigned j = 0;

  *value = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  *value = 0;
  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        *value += M->data[j][i] * M->data[j][i];
      }
    }
    *value = sqrt(*value);
  }
  else
  {
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        *value += M->cplx[j][i].re * M->cplx[j][i].re + M->cplx[j][i].im * M->cplx[j][i].im;
      }
    }
    *value = sqrt(*value);
  }


  return TRUE;
}

BOOL MTX_ColumnRMS( const MTX *M, const unsigned col, double *value )
{
  if( !MTX_ColumnNorm( M, col, value ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnNorm returned FALSE." );
    return FALSE;
  }

  // redundant but better to be sure
  if( M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->nrows == 0 )" );
    return FALSE;
  }

  *value /= sqrt( (double)(M->nrows) );
  return TRUE;
}

BOOL MTX_RowRMS( const MTX *M, const unsigned row, double *value )
{
  if( !MTX_RowNorm( M, row, value ) )
  {
    MTX_ERROR_MSG( "MTX_RowNorm returned FALSE." );
    return FALSE;
  }

  // redundant but better to be sure
  if( M->nrows == 0 )
  {
    MTX_ERROR_MSG( "if( M->nrows == 0 )" );
    return FALSE;
  }

  *value /= sqrt( M->ncols );
  return TRUE;
}

BOOL MTX_RMS( const MTX *M, double *value )
{
  const int n = M->nrows*M->ncols;
  const double nd = n;

  if( !MTX_Norm( M, value ) )
  {
    MTX_ERROR_MSG( "MTX_Norm returned FALSE." );
    return FALSE;
  }

  // redundant but better to be sure
  if( n == 0 )
  {
    MTX_ERROR_MSG( "if( n == 0 )" );
    return FALSE;
  }

  *value /= sqrt( nd );
  return TRUE;
}

BOOL MTX_ColumnSkewness( const MTX *M, const unsigned col, double *re, double* im )
{
  unsigned i = 0;

  double dtmp = 0.0;

  double sum = 0.0,
    meanval_re = 0.0,
    meanval_im = 0.0,
    stdev = 0.0;

  const double n = (double)M->nrows;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->nrows < 3 )
  {
    MTX_ERROR_MSG( "if( M->nrows < 3 )" );
    return FALSE;
  }

  if( !MTX_ColumnMean( M, col, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnMean returned FALSE." );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    if( !MTX_ColumnStdev( M, col, &stdev ) )
    {
      MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
      return FALSE;
    }

    for( i = 0; i < M->nrows; i++ )
    {
      dtmp = M->data[col][i] - meanval_re;
      sum += dtmp*dtmp*dtmp;
    }
    *re = n*sum / ((n-1.0)*(n-2.0)*stdev*stdev*stdev);
  }
  else
  {
    // REFERENCE: http://en.wikipedia.org/wiki/Skewness, sample skewness
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum3;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum3.re = 0;
    cplxSum3.im = 0;

    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^3 = (a^3-3ab^2) + (3a^2b - b^3)i
    for( i = 0; i < M->nrows; i++ )
    {
      a = M->cplx[col][i].re - meanval_re;
      b = M->cplx[col][i].im - meanval_im;
      a2 = a*a;
      b2 = b*b;

      cplxSum2.re += a2-b2;
      cplxSum2.im += 2*a*b;

      cplxSum3.re += a2*a - 3.0*a*b2;
      cplxSum3.im += 3.0*a2*b - b2*b;
    }

    // compute (cplxSum2)^(3/2)
    a = cplxSum2.re;
    b = cplxSum2.im;
    a2 = a*a;
    b2 = b*b;
    cplxSum2.re = a2*a - 3.0*a*b2;
    cplxSum2.im = 3.0*a2*b - b2*b;
    a = cplxSum2.re;
    b = cplxSum2.im;

    dtmp = sqrt( a*a + b*b ); // magnitude
    cplxSum2.re = sqrt( (dtmp + a)/2.0 );
    if( b < 0 )
      cplxSum2.im = -1.0*sqrt( (dtmp - a)/2.0 );
    else
      cplxSum2.im = sqrt( (dtmp - a)/2.0 );


    // compute cplxSum3/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "Divide by zero not allowed." );
      return FALSE;
    }

    a = (cplxSum3.re * cplxSum2.re + cplxSum3.im * cplxSum2.im )/dtmp;
    b = (cplxSum3.im * cplxSum2.re - cplxSum3.re * cplxSum2.im )/dtmp;

    cplxSum3.re = a;
    cplxSum3.im = b;

    dtmp = sqrt(n); // n*sqrt(n-1.0)/(n-2.0); use the commented term to remove bias

    *re = dtmp*cplxSum3.re;
    *im = dtmp*cplxSum3.im;
  }
  return TRUE;
}

BOOL MTX_RowSkewness( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned i = 0;

  double dtmp = 0.0;

  double sum = 0.0,
    meanval_re = 0.0,
    meanval_im = 0.0,
    stdev = 0.0;

  const double n = (double)M->ncols;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( M->ncols < 3 )
  {
    MTX_ERROR_MSG( "if( M->ncols < 3 )" );
    return FALSE;
  }

  if( !MTX_RowMean( M, row, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_RowMean returned FALSE." );
    return FALSE;
  }

  if( !MTX_RowStdev( M, row, &stdev ) )
  {
    MTX_ERROR_MSG( "MTX_RowStdev returned FALSE." );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    for( i = 0; i < M->ncols; i++ )
    {
      dtmp = M->data[i][row] - meanval_re;
      sum += dtmp*dtmp*dtmp;
    }
    *re = n*sum / ((n-1.0)*(n-2.0)*stdev*stdev*stdev);
  }
  else
  {
    // REFERENCE: http://en.wikipedia.org/wiki/Skewness, sample skewness
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum3;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum3.re = 0;
    cplxSum3.im = 0;

    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^3 = (a^3-3ab^2) + (3a^2b - b^3)i
    for( i = 0; i < M->ncols; i++ )
    {
      a = M->cplx[i][row].re - meanval_re;
      b = M->cplx[i][row].im - meanval_im;
      a2 = a*a;
      b2 = b*b;

      cplxSum2.re += a2-b2;
      cplxSum2.im += 2*a*b;

      cplxSum3.re += a2*a - 3.0*a*b2;
      cplxSum3.im += 3.0*a2*b - b2*b;
    }

    // compute (cplxSum2)^(3/2)
    a = cplxSum2.re;
    b = cplxSum2.im;
    a2 = a*a;
    b2 = b*b;
    cplxSum2.re = a2*a - 3.0*a*b2;
    cplxSum2.im = 3.0*a2*b - b2*b;
    a = cplxSum2.re;
    b = cplxSum2.im;

    dtmp = sqrt( a*a + b*b ); // magnitude
    cplxSum2.re = sqrt( (dtmp + a)/2.0 );
    if( b < 0 )
      cplxSum2.im = -1.0*sqrt( (dtmp - a)/2.0 );
    else
      cplxSum2.im = sqrt( (dtmp - a)/2.0 );


    // compute cplxSum3/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "Divide by zero not allowed." );
      return FALSE;
    }

    a = (cplxSum3.re * cplxSum2.re + cplxSum3.im * cplxSum2.im )/dtmp;
    b = (cplxSum3.im * cplxSum2.re - cplxSum3.re * cplxSum2.im )/dtmp;

    cplxSum3.re = a;
    cplxSum3.im = b;

    dtmp = sqrt(n); // n*sqrt(n-1.0)/(n-2.0);

    *re = dtmp*cplxSum3.re;
    *im = dtmp*cplxSum3.im;
  }
  return TRUE;
}


BOOL MTX_Skewness( const MTX *M, double *re, double* im )
{
  unsigned j = 0;
  unsigned i = 0;
  double dtmp = 0.0;
  double sum = 0.0;
  double meanval_re = 0.0;
  double meanval_im = 0.0;
  double stdev = 0.0;

  const double n = (double)(M->nrows*M->ncols);

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( n < 3 )
  {
    MTX_ERROR_MSG( "if( n < 3 )" );
    return FALSE;
  }



  if( !MTX_Mean( M, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_Mean returned FALSE." );
    return FALSE;
  }

  *re = 0;
  *im = 0;

  if( M->isReal )
  {
    if( !MTX_Stdev( M, &stdev ) )
    {
      MTX_ERROR_MSG( "MTX_Stdev returned FALSE." );
      return FALSE;
    }

    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        dtmp = M->data[j][i] - meanval_re;
        sum += dtmp*dtmp*dtmp;
      }
    }
    *re = n*sum / ((n-1.0)*(n-2.0)*stdev*stdev*stdev);
  }
  else
  {
    // REFERENCE: http://en.wikipedia.org/wiki/Skewness, sample skewness
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum3;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum3.re = 0;
    cplxSum3.im = 0;

    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^3 = (a^3-3ab^2) + (3a^2b - b^3)i
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        a = M->cplx[j][i].re - meanval_re;
        b = M->cplx[j][i].im - meanval_im;
        a2 = a*a;
        b2 = b*b;

        cplxSum2.re += a2-b2;
        cplxSum2.im += 2*a*b;

        cplxSum3.re += a2*a - 3.0*a*b2;
        cplxSum3.im += 3.0*a2*b - b2*b;
      }
    }

    // compute (cplxSum2)^(3/2)
    a = cplxSum2.re;
    b = cplxSum2.im;
    a2 = a*a;
    b2 = b*b;
    cplxSum2.re = a2*a - 3.0*a*b2;
    cplxSum2.im = 3.0*a2*b - b2*b;
    a = cplxSum2.re;
    b = cplxSum2.im;

    dtmp = sqrt( a*a + b*b ); // magnitude
    cplxSum2.re = sqrt( (dtmp + a)/2.0 );
    if( b < 0 )
      cplxSum2.im = -1.0*sqrt( (dtmp - a)/2.0 );
    else
      cplxSum2.im = sqrt( (dtmp - a)/2.0 );


    // compute cplxSum3/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "Divide by zero not allowed." );
      return FALSE;
    }

    a = (cplxSum3.re * cplxSum2.re + cplxSum3.im * cplxSum2.im )/dtmp;
    b = (cplxSum3.im * cplxSum2.re - cplxSum3.re * cplxSum2.im )/dtmp;

    cplxSum3.re = a;
    cplxSum3.im = b;

    dtmp = sqrt(n); // n*sqrt(n-1.0)/(n-2.0); use the commented term to remove bias

    *re = dtmp*cplxSum3.re;
    *im = dtmp*cplxSum3.im;
  }


  return TRUE;
}

BOOL MTX_ColumnKurtosis( const MTX *M, const unsigned col, double *re, double *im )
{
  unsigned i = 0;

  double dtmp = 0.0;

  double sum = 0.0,
    meanval_re = 0.0,
    meanval_im = 0.0,
    stdev = 0.0;

  const double n = (double)M->nrows;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->nrows < 4 )
  {
    MTX_ERROR_MSG( "if( M->nrows < 4 )" );
    return FALSE;
  }

  if( !MTX_ColumnMean( M, col, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_ColumnMean returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
  {
    if( !MTX_ColumnStdev( M, col, &stdev ) )
    {
      MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
      return FALSE;
    }

    for( i = 0; i < M->nrows; i++ )
    {
      dtmp = M->data[col][i] - meanval_re;
      sum += dtmp*dtmp*dtmp*dtmp;
    }
    dtmp = 1.0 / (stdev*stdev*stdev*stdev);
    dtmp *= n*(n+1) / ((n-1.0)*(n-2.0)*(n-3.0));
    dtmp *= sum;
    dtmp -= 3.0*(n-1.0)*(n-1.0) / ((n-2.0)*(n-3.0));
    *re = dtmp;
  }
  else
  {
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum4;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum4.re = 0;
    cplxSum4.im = 0;

    // g_2 = \frac{m_4}{m_{2}^2} -3 = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^4 = (a^4 - 6*a^2*b^2 + b^4) + (4*a^3*b - 4*a*b^3)i
    for( i = 0; i < M->nrows; i++ )
    {
      a = M->cplx[col][i].re - meanval_re;
      b = M->cplx[col][i].im - meanval_im;
      a2 = a*a;
      b2 = b*b;

      cplxSum2.re += a2-b2;
      cplxSum2.im += 2.0*a*b;

      cplxSum4.re += a2*a2 - 6.0*a2*b2 + b2*b2;
      cplxSum4.im += 4.0*a2*a*b - 4.0*a*b2*b;
    }

    // compute (cplxSum2)^(2)
    a = cplxSum2.re * cplxSum2.re - cplxSum2.im * cplxSum2.im;
    b = 2.0*cplxSum2.re * cplxSum2.im;
    cplxSum2.re = a;
    cplxSum2.im = b;

    // compute cplxSum4 = cplxSum4/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "Divide by zero not allowed." );
      return FALSE;
    }

    a = (cplxSum4.re * cplxSum2.re + cplxSum4.im * cplxSum2.im )/dtmp;
    b = (cplxSum4.im * cplxSum2.re - cplxSum4.re * cplxSum2.im )/dtmp;

    cplxSum4.re = a;
    cplxSum4.im = b;

    *re = n*cplxSum4.re;
    *im = n*cplxSum4.im;
  }

  return TRUE;
}

BOOL MTX_RowKurtosis( const MTX *M, const unsigned row, double *re, double *im )
{
  unsigned j = 0;

  double dtmp = 0.0;

  double sum = 0.0,
    meanval_re = 0.0,
    meanval_im = 0.0,
    stdev = 0.0;

  const double n = (double)M->ncols;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( M->ncols < 4 )
  {
    MTX_ERROR_MSG( "if( M->ncols < 4 )" );
    return FALSE;
  }

  if( !MTX_RowMean( M, row, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_RowMean returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
  {
    if( !MTX_RowStdev( M, row, &stdev ) )
    {
      MTX_ERROR_MSG( "MTX_RowStdev returned FALSE." );
      return FALSE;
    }

    for( j = 0; j < M->ncols; j++ )
    {
      dtmp = M->data[j][row] - meanval_re;
      sum += dtmp*dtmp*dtmp*dtmp;
    }
    dtmp = 1.0 / (stdev*stdev*stdev*stdev);
    dtmp *= n*(n+1) / ((n-1.0)*(n-2.0)*(n-3.0));
    dtmp *= sum;
    dtmp -= 3.0*(n-1.0)*(n-1.0) / ((n-2.0)*(n-3.0));
    *re = dtmp;
  }
  else
  {
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum4;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum4.re = 0;
    cplxSum4.im = 0;

    // g_2 = \frac{m_4}{m_{2}^2} -3 = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^4 = (a^4 - 6*a^2*b^2 + b^4) + (4*a^3*b - 4*a*b^3)i
    for( j = 0; j < M->ncols; j++ )
    {
      a = M->cplx[j][row].re - meanval_re;
      b = M->cplx[j][row].im - meanval_im;
      a2 = a*a;
      b2 = b*b;

      cplxSum2.re += a2-b2;
      cplxSum2.im += 2.0*a*b;

      cplxSum4.re += a2*a2 - 6.0*a2*b2 + b2*b2;
      cplxSum4.im += 4.0*a2*a*b - 4.0*a*b2*b;
    }

    // compute (cplxSum2)^(2)
    a = cplxSum2.re * cplxSum2.re - cplxSum2.im * cplxSum2.im;
    b = 2.0*cplxSum2.re * cplxSum2.im;
    cplxSum2.re = a;
    cplxSum2.im = b;

    // compute cplxSum4 = cplxSum4/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "if( dtmp == 0.0 )" );
      return FALSE;
    }

    a = (cplxSum4.re * cplxSum2.re + cplxSum4.im * cplxSum2.im )/dtmp;
    b = (cplxSum4.im * cplxSum2.re - cplxSum4.re * cplxSum2.im )/dtmp;

    cplxSum4.re = a;
    cplxSum4.im = b;

    *re = n*cplxSum4.re;
    *im = n*cplxSum4.im;
  }

  return TRUE;
}

BOOL MTX_Kurtosis( const MTX *M, double *re, double *im )
{
  unsigned j = 0;
  unsigned i = 0;
  double dtmp = 0.0;
  double sum = 0.0;
  double meanval_re = 0.0;
  double meanval_im = 0.0;
  double stdev = 0.0;

  const double n = (double)(M->nrows*M->ncols);

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( n < 4 )
  {
    MTX_ERROR_MSG( "if( n < 4 )" );
    return FALSE;
  }


  if( !MTX_Mean( M, &meanval_re, &meanval_im ) )
  {
    MTX_ERROR_MSG( "MTX_Mean returned FALSE." );
    return FALSE;
  }

  if( M->isReal )
  {
    if( !MTX_Stdev( M, &stdev ) )
    {
      MTX_ERROR_MSG( "MTX_Stdev returned FALSE." );
      return FALSE;
    }

    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        dtmp = M->data[j][i] - meanval_re;
        sum += dtmp*dtmp*dtmp*dtmp;
      }
    }
    dtmp = 1.0 / (stdev*stdev*stdev*stdev);
    dtmp *= n*(n+1) / ((n-1.0)*(n-2.0)*(n-3.0));
    dtmp *= sum;
    dtmp -= 3.0*(n-1.0)*(n-1.0) / ((n-2.0)*(n-3.0));
    *re = dtmp;
  }
  else
  {
    double a;
    double b;
    double a2;
    double b2;
    stComplex cplxSum2;
    stComplex cplxSum4;
    cplxSum2.re = 0;
    cplxSum2.im = 0;
    cplxSum4.re = 0;
    cplxSum4.im = 0;

    // g_2 = \frac{m_4}{m_{2}^2} -3 = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    // (a+bi)^2 = (a^2-b^2) + 2abi
    // (a+bi)^4 = (a^4 - 6*a^2*b^2 + b^4) + (4*a^3*b - 4*a*b^3)i
    for( j = 0; j < M->ncols; j++ )
    {
      for( i = 0; i < M->nrows; i++ )
      {
        a = M->cplx[j][i].re - meanval_re;
        b = M->cplx[j][i].im - meanval_im;
        a2 = a*a;
        b2 = b*b;

        cplxSum2.re += a2-b2;
        cplxSum2.im += 2.0*a*b;

        cplxSum4.re += a2*a2 - 6.0*a2*b2 + b2*b2;
        cplxSum4.im += 4.0*a2*a*b - 4.0*a*b2*b;
      }
    }

    // compute (cplxSum2)^(2)
    a = cplxSum2.re * cplxSum2.re - cplxSum2.im * cplxSum2.im;
    b = 2.0*cplxSum2.re * cplxSum2.im;
    cplxSum2.re = a;
    cplxSum2.im = b;

    // compute cplxSum4 = cplxSum4/cplxSum2

    dtmp = cplxSum2.re*cplxSum2.re + cplxSum2.im*cplxSum2.im;

    if( dtmp == 0.0 )
    {
      MTX_ERROR_MSG( "Divide by zero not allowed." );
      return FALSE;
    }

    a = (cplxSum4.re * cplxSum2.re + cplxSum4.im * cplxSum2.im )/dtmp;
    b = (cplxSum4.im * cplxSum2.re - cplxSum4.re * cplxSum2.im )/dtmp;

    cplxSum4.re = a;
    cplxSum4.im = b;

    *re = n*cplxSum4.re;
    *im = n*cplxSum4.im;
  }


  return TRUE;
}

BOOL MTX_Trace( const MTX *M, double *re, double *im )
{
  unsigned i = 0;

  if( !MTX_isSquare(M) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  *re = 0.0;
  *im = 0.0;

  if( M->isReal )
  {
    for( i = 0; i < M->nrows; i++ )
      *re += M->data[i][i];
  }
  else
  {
    for( i = 0; i < M->nrows; i++ )
    {
      *re += M->cplx[i][i].re;
      *im += M->cplx[i][i].im;
    }
  }

  return TRUE;
}

BOOL MTX_Diagonal( const MTX *M, MTX *D )
{
  unsigned i = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Resize( D, M->nrows, 1, M->isReal ) )    
  {
    MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
    return FALSE;
  }

  for( i = 0; i < M->nrows; i++ )
  {
    if( i < M->ncols )
    {
      if( M->isReal )
      {
        D->data[0][i] = M->data[i][i];
      }
      else
      {
        D->cplx[0][i].re = M->cplx[i][i].re;
        D->cplx[0][i].im = M->cplx[i][i].im;
      }
    }
  }
  return TRUE;
}

BOOL MTX_FlipColumn( MTX *M, const unsigned col )
{
  unsigned i = 0;
  double re = 0;
  double im = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( i = 0; i < (M->nrows/2); i++ )
    {
      re = M->data[col][i];
      M->data[col][i] = M->data[col][M->nrows - i - 1];
      M->data[col][M->nrows - i - 1] = re;
    }
  }
  else
  {
    for( i = 0; i < (M->nrows/2); i++ )
    {
      re = M->cplx[col][i].re;
      im = M->cplx[col][i].im;
      M->cplx[col][i].re = M->cplx[col][M->nrows - i - 1].re;
      M->cplx[col][i].im = M->cplx[col][M->nrows - i - 1].im;
      M->cplx[col][M->nrows - i - 1].re = re;
      M->cplx[col][M->nrows - i - 1].im = im;
    }
  }
  return TRUE;
}

BOOL MTX_FlipRow( MTX *M, const unsigned row )
{
  unsigned i = 0;
  double re = 0;
  double im = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( row >= M->nrows )
  {
    MTX_ERROR_MSG( "if( row >= M->nrows )" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( i = 0; i < (M->ncols/2); i++ )
    {
      re = M->data[i][row];
      M->data[i][row] = M->data[M->ncols - i - 1][row];
      M->data[M->ncols - i - 1][row] = re;
    }
  }
  else
  {
    for( i = 0; i < (M->ncols/2); i++ )
    {
      re = M->cplx[i][row].re;
      im = M->cplx[i][row].im;
      M->cplx[i][row].re = M->cplx[M->ncols - i - 1][row].re;
      M->cplx[i][row].im = M->cplx[M->ncols - i - 1][row].im;
      M->cplx[M->ncols - i - 1][row].re = re;
      M->cplx[M->ncols - i - 1][row].im = im;
    }
  }
  return TRUE;
}

BOOL MTX_SortAscending( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  int k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      MTX_static_quicksort( M->data[j], 0, M->nrows-1 );
    }
  }
  else
  {
    MTX indexvec;
    MTX_Init( &indexvec );

    for( j = 0; j < M->ncols; j++ )
    {
      if( !MTX_SortColumnIndexed( M, j, &indexvec ) )
      {
        MTX_ERROR_MSG( "MTX_SortColumnIndexed returned FALSE." );
        MTX_Free( &indexvec );
        return FALSE;
      }
    }

    MTX_Free( &indexvec );
  }

  return TRUE;
}

BOOL MTX_SortDescending( MTX *M )
{
  unsigned j = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( M->isReal )
  {
    for( j = 0; j < M->ncols; j++ )
    {
      MTX_static_quicksort( M->data[j], 0, M->nrows-1 );
      if( !MTX_FlipColumn( M, j ) )
      {
        MTX_ERROR_MSG( "MTX_FlipColumn returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    if( !MTX_SortAscending( M ) )
    {
      MTX_ERROR_MSG( "MTX_SortAscending returned FALSE." );
      return FALSE;
    }

    for( j = 0; j < M->ncols; j++ )
    {
      if( !MTX_FlipColumn( M, j ) )
      {
        MTX_ERROR_MSG( "MTX_FlipColumn returned FALSE." );
        return FALSE;
      }
    }
  }

  return TRUE;
}

BOOL MTX_SortColumnAscending( MTX *M, const unsigned col )
{
  unsigned i = 0;
  int k = 0;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    MTX_static_quicksort( M->data[col], 0, M->nrows-1 );
  }
  else
  {
    MTX indexvec;
    MTX_Init( &indexvec );

    if( !MTX_SortColumnIndexed( M, col, &indexvec ) )
    {
      MTX_ERROR_MSG( "MTX_SortColumnIndexed returned FALSE." );
      MTX_Free( &indexvec );
      return FALSE;
    }

    MTX_Free( &indexvec );
  }

  return TRUE;
}

BOOL MTX_SortColumnDescending( MTX *M, const unsigned col )
{
  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( M->isReal )
  {
    MTX_static_quicksort( M->data[col], 0, M->nrows-1 );
    if( !MTX_FlipColumn( M, col ) )
    {
      MTX_ERROR_MSG( "MTX_FlipColumn returned FALSE." );
      return FALSE;
    }
  }
  else
  {
    if( !MTX_SortColumnAscending( M, col ) )
    {
      MTX_ERROR_MSG( "MTX_SortColumnAscending returned FALSE." );
      return FALSE;
    }

    if( !MTX_FlipColumn( M, col ) )
    {
      MTX_ERROR_MSG( "MTX_FlipColumn returned FALSE." );
      return FALSE;
    }
  }

  return TRUE;
}

BOOL MTX_SortColumnIndexed( MTX *M, const unsigned col, MTX *index )
{
  unsigned i = 0;
  int k;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( index->ncols != 1 || index->nrows != M->nrows || !(index->isReal) )
  {
    if( !MTX_Resize( index, M->nrows, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  for( i = 0; i < M->nrows; i++ )
    index->data[0][i] = i;

  if( M->isReal )
  {
    MTX_static_quicksort_indexed( M->data[col], index->data[0], 0, M->nrows-1 );
  }
  else
  {
    double re;
    double im;
    MTX colM;
    MTX mag;

    MTX_Init( &colM );
    MTX_Init( &mag );

    // make a copy of the column data
    if( !MTX_CopyColumn( M, col, &colM ) )
    {
      MTX_ERROR_MSG( "MTX_CopyColumn returned FALSE." );
      MTX_Free( &colM );
      return FALSE;
    }

    // get the magnitude
    if( !MTX_Magnitude( &colM, &mag ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &colM );
      MTX_Free( &mag );
      return FALSE;
    }

    // sort the magnitude of column j, and get the indexing vector
    MTX_static_quicksort_indexed( mag.data[0], index->data[0], 0, M->nrows-1 );

    // make a copy of the indexing vector (because colM is used and some values are set to -1)
    if( !MTX_Copy( index, &colM ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &colM );
      MTX_Free( &mag );
      return FALSE;
    }

    // sort the original data in the right order
    for( i = 0; i < M->nrows; i++ )
    {
      k = (int)(colM.data[0][i]);
      if( k >= 0 )
      {
        colM.data[0][k] = -1.0; // don't allow the reverse indexing
        if( i == k )
          continue;

        re = M->cplx[col][i].re;
        im = M->cplx[col][i].im;
        M->cplx[col][i].re = M->cplx[col][k].re;
        M->cplx[col][i].im = M->cplx[col][k].im;
        M->cplx[col][k].re = re;
        M->cplx[col][k].im = im;
      }
    }

    MTX_Free( &colM );
    MTX_Free( &mag );
  }

  return TRUE;
}

BOOL MTX_SortByColumn( MTX *M, const unsigned col )
{
  unsigned i = 0;
  unsigned j = 0;
  int k = 0;
  MTX indexvec;
  MTX vec;

  MTX_Init( &indexvec );
  MTX_Init( &vec );

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= M->ncols )
  {
    MTX_ERROR_MSG( "if( col >= M->ncols )" );
    return FALSE;
  }

  if( !MTX_SortColumnIndexed( M, col, &indexvec ) )
  {
    MTX_ERROR_MSG( "MTX_SortColumnIndexed returned FALSE." );
    MTX_Free( &indexvec );
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    if( j == col )
      continue; // already sorted

    if( !MTX_CopyColumn( M, j, &vec ) )
    {
      MTX_ERROR_MSG("MTX_Copy returned FALSE." );
      return FALSE;
    }

    for( i = 0; i < M->nrows; i++ )
    {
      k = (int)(indexvec.data[0][i]);
      if( k >= 0 )
      {
        if( i == k )
          continue;

        if( M->isReal )
        {
          M->data[j][i] = vec.data[0][k];          
        }
        else
        {
          M->cplx[j][i].re = vec.cplx[0][k].re;
          M->cplx[j][i].im = vec.cplx[0][k].im;
        }
      }
    }
  }

  MTX_Free( &indexvec );
  MTX_Free( &vec );
  return TRUE;
}

///////////////////////////////////////////////////////////////////////////
// The following the functions are used in a recursive quicksort
// algorith for double arrays
//
//
// The normal quicksort function
void MTX_static_quicksort( double *a, unsigned start, unsigned end )
{
  int split;
  if( start < end )
  {
    split = MTX_static_partition(a, start, end);
    MTX_static_quicksort(a, start, split);
    MTX_static_quicksort(a, split + 1, end);
  }
}

// swap two doubles a and b
void MTX_static_swap_doubles( double *a, double *b )
{
  double temp = *a;
  *a = *b;
  *b = temp;
}

// partition the vector
int MTX_static_partition( double *a, unsigned start, unsigned end )
{
  int right = end + 1;
  int left = start - 1;
  double pivot = a[start];

  while( right > left )
  {
    do{ left++; } while(a[left] < pivot);
    do{ right--; } while(a[right] > pivot);
    MTX_static_swap_doubles( &(a[left]), (&a[right]) );
  }
  MTX_static_swap_doubles( &(a[left]), &(a[right]) );
  return right;
}

// quicksort that also returns a sorted indexing vector
void MTX_static_quicksort_indexed( double *a, double *index, unsigned start, unsigned end )
{
  int split;
  if( start < end )
  {
    split = MTX_static_partition_indexed( a, index, start, end );
    MTX_static_quicksort_indexed( a, index, start, split );
    MTX_static_quicksort_indexed( a, index, split + 1, end );
  }
}


// swap the doubles
void MTX_static_swap_doubles_indexed( double *a, double *b, double *index_a, double *index_b )
{
  double temp = *a;
  double temp_ind = *index_a;
  *a = *b;
  *index_a = *index_b;
  *b = temp;
  *index_b = temp_ind;
}

// partition the vectors
int MTX_static_partition_indexed( double *a, double *index, unsigned start, unsigned end )
{
  int right = end + 1;
  int left = start - 1;
  double pivot = a[start];
  while( right > left )
  {
    do{ left++; } while(a[left] < pivot);
    do{ right--; } while(a[right] > pivot);
    MTX_static_swap_doubles_indexed( &(a[left]), &(a[right]), &(index[left]), &(index[right]) );
  }
  MTX_static_swap_doubles_indexed( &(a[left]), &(a[right]), &(index[left]), &(index[right]) );
  return right;
}
//
//
///////////////////////////////////////////////////////////////////////////





// Compression works as follows:
// take a column of doubles
// take each 8 bytes of a double
// and RLE encode with the next 8 bytes of the next double
// [ xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx ] a double
// [ byte0_0 byte1_0 byte2_0 byte3_0 byte4_0 byte5_0 byte6_0 byte7_0 ] as bytes
// [ .. .. .. .. .. .. .. .. ]
// [ .. .. .. .. .. .. .. .. ]
// [ byte0_n byte1_n byte2_n byte3_n byte4_n byte5_n byte6_n byte7_n ] as bytes
//
// RLE encode byte0_0:n, byte1_0:n, etc
// in this way one column becomes 8 columns
//
// this process takes advantage of the similarities in the data of each column
// and the RLE methods available with INTEL_IPPS
//
// Note: The INTEL_IPPS RLE encoding method did not prove significantly faster in testing.
//
// aaabbbbaaaaabbabbbbbb
// compressed becomes
// aa1bb2aa3bb0abb4
//
// Columns where compression does not provide benefit are written without compression
BOOL MTX_SaveCompressed( const MTX *M, const char *path )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned ncols = 0; // number of columns if real, number of columns * 2 if complex.

  MTX MtxCol; // The current column for complex matrices only. Either real part or imag part.

#ifdef MTX_DEBUG
  time_t t0, t1; /* time_t is defined on <time.h> and <sys/types.h> as long */
  clock_t c0, c1; /* clock_t is defined on <time.h> and <sys/types.h> as int */
  unsigned total_length = 0;
#endif

  const unsigned nk = MTX_NK; // number of byte columns per double column


  unsigned char prevByte = 0;
  unsigned char currByte = 0;
  unsigned n = 0; // number of times the byte repeats (256 is upper bound!!)
  unsigned p = 0; // counter
  unsigned ncompressed = 0;
  unsigned nbytes = 0;

  unsigned char* bytes[MTX_NK];
  unsigned char* compressed[MTX_NK];
  unsigned char curr[MTX_NK];

  char msg[512];

  FILE* fid = NULL; // the output file pointer

  unsigned commentLength = 0; // the length of the matrix comment if any
  long filemark = 0; // a file position

  double col_stdev = 0; // The variance of a column.
  size_t count = 0;

  _MTX_STRUCT_FileHeader fileHeader;
  _MTX_STRUCT_CompressedColumnHeader columnHeader;

  // initialize MtxCol
  MTX_Init( &MtxCol );

  memset( &fileHeader, 0, sizeof(fileHeader) );
  memset( &columnHeader, 0, sizeof(columnHeader) );


  // initialize the file header information
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( strcpy_s( fileHeader.id, 8, MTX_ID_COMPRESSED_01 ) != 0 )
  {
    MTX_ERROR_MSG( "strcpy_s returned an error condition." );
    return FALSE;
  }
#else
  strcpy( fileHeader.id, MTX_ID_COMPRESSED_01 );
#endif

  if( M->comment != NULL )
  {
    fileHeader.comment = M->comment;
    commentLength = (unsigned int)strlen( fileHeader.comment );
  }
  fileHeader.headersize = MTX_ID_SIZE + 6*sizeof(unsigned int) + commentLength;
  fileHeader.isReal = M->isReal;
  fileHeader.nrows = M->nrows;
  fileHeader.ncols = M->ncols;
  fileHeader.filesize = 0;
  fileHeader.crc = 0;

  // open the output binary file
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, path, "wb" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned an error condition." );
    return FALSE;
  }
#else
  fid = fopen( path, "wb" );
#endif
  if( fid == NULL )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( msg, 512, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( msg );
#else
    if( sprintf( msg, "Unable to open %s", path ) > 0 )
      MTX_ERROR_MSG( msg );
#endif
    return FALSE;
  }

  // allocate memory for the RLE byte vectors
  // at most twice as long is possible with RLE
  for( k = 0; k < nk; k++ )
  {
    bytes[k] = (unsigned char*)malloc( M->nrows*sizeof(unsigned char) );
    if( bytes[k] == NULL )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      fclose(fid);
      return FALSE;
    }
    compressed[k] = (unsigned char*)malloc( M->nrows*sizeof(unsigned char)*2 );
    if( compressed[k] == NULL )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      fclose(fid);
      return FALSE;
    }
  }

  // write the matrix file identifier
  // in this case the compressed binary matrix version 01
  count = fwrite( fileHeader.id, sizeof(char), MTX_ID_SIZE, fid );
  if( count != MTX_ID_SIZE )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  // write the header size
  count = fwrite( &fileHeader.headersize, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  // write is the matrix is real or complex
  count = fwrite( &fileHeader.isReal, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  // write the size of the matrix
  count = fwrite( &M->nrows, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  count = fwrite( &M->ncols, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }


  // store the file position where the filesize and crc are written
  // after writing the data, we'll come back to this position to
  // rewrite this information correctly
  filemark = ftell( fid );
  count = fwrite( &fileHeader.filesize, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  count = fwrite( &fileHeader.crc, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  // write the matrix comment if any
  if( commentLength )
  {
    count = fwrite( &fileHeader.comment, sizeof(char), commentLength, fid );
    if( count != commentLength )
    {
      MTX_ERROR_MSG( "fwrite returned an error condition." );
      MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
      return FALSE;
    }
  }


#ifdef MTX_DEBUG
  t0 = time(NULL);
  c0 = clock();
#endif

  if( M->isReal )
    ncols = M->ncols;
  else
    ncols = M->ncols*2;

  for( j = 0; j < ncols; j++ )
  {
    // complex matrices are treated as re,im,re,im, etc so the matrix is stored as M->ncols*2 columns
    if( !M->isReal )
    {
      if( j == 0 || j%2 == 0 )
      {
        if( !MTX_RealColumn( M, j/2, &MtxCol ) )
        {
          MTX_ERROR_MSG( "MTX_RealColumn returned FALSE." );
          MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
          return FALSE;
        }
      }
      else
      {
        if( !MTX_ImagColumn( M, j/2, &MtxCol ) )
        {
          MTX_ERROR_MSG( "MTX_ImagColumn returned FALSE." );
          MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
          return FALSE;
        }
      }
    }

    // initialize lengths
    for( k = 0; k < nk; k++ )
    {
      columnHeader.length[k] = 0;
      columnHeader.isCompressed[k] = 0;
    }
    columnHeader.totalLength = 0;


    col_stdev = 1.0; // A non zero default value.
    if( M->isReal )
    {
      if( !MTX_ColumnStdev( M, j, &col_stdev ) )
      {
        MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_ColumnStdev( &MtxCol, 0, &col_stdev ) )
      {
        MTX_ERROR_MSG( "MTX_ColumnStdev returned FALSE." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
    }

    if( col_stdev == 0.0 )
    {
      // output a single 0 for the first length indicating that a single double value represents this column
      count = fwrite( &(columnHeader.length[0]), sizeof(unsigned), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fwrite returned an error condition." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      MTX_static_updateCRC( (unsigned char*)(columnHeader.length), 4, &(fileHeader.crc) );

      // output the double
      if( M->isReal )
      {
        count = fwrite( &M->data[j][0], sizeof(double), 1, fid );
        if( count != 1 )
        {
          MTX_ERROR_MSG( "fwrite returned an error condition." );
          MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
          return FALSE;
        }
      }
      else
      {
        count = fwrite( &MtxCol.data[0][0], sizeof(double), 1, fid );
        if( count != 1 )
        {
          MTX_ERROR_MSG( "fwrite returned an error condition." );
          MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
          return FALSE;
        }
      }

      // copy the double's bytes to update the crc
      if( M->isReal )
      {
        memcpy( curr, &(M->data[j][0]), sizeof(double) );
        MTX_static_updateCRC( curr, 8, &(fileHeader.crc) );
      }
      else
      {
        memcpy( curr, &(MtxCol.data[0][0]), sizeof(double) );
        MTX_static_updateCRC( curr, 8, &(fileHeader.crc) );
      }

      continue;
    }

    // form the bytes vector for RLE input
    for( i = 0; i < M->nrows; i++ )
    {
      if( M->isReal )
        memcpy( curr, &(M->data[j][i]), sizeof(double) );
      else
        memcpy( curr, &(MtxCol.data[0][i]), sizeof(double) );

      for( k = 0; k < nk; k++ )
      {
        bytes[k][i] = curr[k];
      }
    }


    // quick test code
    /*
    k = 0;
    bytes[k][0] = 'a';
    bytes[k][1] = 'b';
    bytes[k][2] = 'b';
    bytes[k][3] = 'a';
    bytes[k][4] = 'b';
    bytes[k][5] = 'a';

    bytes[k][6] = 'a';
    bytes[k][7] = 'a';
    bytes[k][8] = 'b';
    bytes[k][9] = 'b';
    bytes[k][10] = 'b';
    bytes[k][11] = 'b';

    bytes[k][12] = 'a';
    bytes[k][13] = 'a';
    bytes[k][14] = 'a';
    bytes[k][15] = 'a';
    bytes[k][16] = 'a';
    bytes[k][17] = 'b';

    bytes[k][18] = 'b';
    bytes[k][19] = 'a';
    bytes[k][20] = 'b';
    bytes[k][21] = 'b';
    bytes[k][22] = 'a';
    */

    for( k = 0; k < nk; k++ )
    {
      // abbabaaabbbbaaaaabbabba
      // compressed becomes
      // abb0abaa1bb2aa3bb0abb0a
      i = 0;
      n = 0;
      p = 0;
      prevByte = bytes[k][i];
      i++;
      compressed[k][p] = prevByte;
      p++;

      while( i < M->nrows )
      {
        currByte = bytes[k][i];
        i++;

        if( currByte == prevByte )
        {
          n++;
          if( n == 256 )
          {
            compressed[k][p] = prevByte;
            p++;
            compressed[k][p] = n-1;
            p++;
            n = 0;

            prevByte = bytes[k][i];
            i++;
            compressed[k][p] = prevByte;
            p++;
            if( i == M->nrows )
              break;
          }
        }
        else
        {
          if( n > 0 )
          {
            compressed[k][p] = prevByte;
            p++;
            compressed[k][p] = n-1;
            p++;
            n = 0;
          }
          compressed[k][p] = currByte;
          p++;
          prevByte = currByte;
        }
      }
      if( n > 0 )
      {
        compressed[k][p] = prevByte;
        p++;
        compressed[k][p] = n-1;
        p++;
        n = 0;
      }

      columnHeader.length[k] = p;
      columnHeader.totalLength += columnHeader.length[k];
    }

    /*
    for( k = 0; k < nk; k++ )
    {
    for( n = 0; n < p; n++ )
    {
    if( compressed[k][p] != compressed_intel[k][p] )
    {
    count = 99;
    }
    }
    }
    */

#ifdef MTX_DEBUG
    total_length += columnHeader.totalLength;
#endif

    // test the compressed data lengths to determine if the compressed version should be output
    for( k = 0; k < nk; k++ )
    {
      if( columnHeader.length[k] >= M->nrows )
      {
        columnHeader.length[k] = M->nrows;
        columnHeader.isCompressed[k] = 0;
      }
      else
      {
        columnHeader.isCompressed[k] = 1;
      }
    }

    // output the data lengths
    for( k = 0; k < nk; k++ )
    {
      count = fwrite( &(columnHeader.length[k]), sizeof(unsigned), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fwrite returned an error condition." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      MTX_static_updateCRC( (unsigned char*)(&(columnHeader.length[k])), 4, &(fileHeader.crc) );
    }

    // output the compression indication
    for( k = 0; k < nk; k++ )
    {
      count = fwrite( &(columnHeader.isCompressed[k]), sizeof(unsigned char), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fwrite returned an error condition." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      MTX_static_updateCRC( &(columnHeader.isCompressed[k]), 1, &(fileHeader.crc) );
    }

    // output the data or compressed data
    for( k = 0; k < nk; k++ )
    {
      if( columnHeader.isCompressed[k] )
      {
        count = fwrite( compressed[k], sizeof(unsigned char), columnHeader.length[k], fid );
      }
      else
      {
        count = fwrite( bytes[k], sizeof(unsigned char), columnHeader.length[k], fid );
      }
      if( count != columnHeader.length[k] )
      {
        MTX_ERROR_MSG( "fwrite returned an error condition." );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      if( columnHeader.isCompressed[k] )
      {
        MTX_static_updateCRC( compressed[k], columnHeader.length[k], &(fileHeader.crc) );
      }
      else
      {
        MTX_static_updateCRC( bytes[k], columnHeader.length[k], &(fileHeader.crc) );
      }
    }
  }

  // determine the filesize
  fileHeader.filesize = ftell(fid);
  fseek( fid, 0, SEEK_SET );
  fileHeader.filesize -= ftell(fid);

  // write the filesize and the crc back to the header position in the file
  fseek( fid, filemark, SEEK_SET );
  count = fwrite( &fileHeader.filesize, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }
  count = fwrite( &fileHeader.crc, sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fwrite returned an error condition." );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }

#ifdef MTX_DEBUG
  t1 = time(NULL);
  c1 = clock();
  printf("\n%s compressed to: %.1lf (%%) of the original size.\n", path, 100.0*total_length/(M->nrows*M->ncols*8) );

  printf("Compression (GDM's) RLE: %.2f (s)\n", (float) (c1 - c0)/CLOCKS_PER_SEC);

#endif

  // cleanup
  MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );

  return TRUE;
}


unsigned MTX_static_CRC32(unsigned ulCRC)
{
#define CRC32_POLYNOMIAL 0xEDB88320L
  int k;
  for( k = 8 ; k > 0; k-- )
  {
    if( ulCRC & 1 )
      ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
    else
      ulCRC >>= 1;
  }
  return ulCRC;
}


void MTX_static_updateCRC( unsigned char *pBytes, const unsigned nBytes, unsigned *uiCRC )
{
  unsigned tmp1;
  unsigned tmp2;
  unsigned char *CRCData = pBytes;
  unsigned byteCount = nBytes;

  while( byteCount-- != 0 )
  {
    tmp1 = ( (*uiCRC) >> 8 ) & 0x00FFFFFFL;
    tmp2 = MTX_static_CRC32( ((unsigned) (*uiCRC) ^ *CRCData++ ) & 0xff );
    *uiCRC = tmp1 ^ tmp2;
  }
}

void MTX_static_SaveAndLoadCleanUp( FILE *fid, unsigned char **bytes, unsigned char **compressed, const unsigned nk )
{
  unsigned k;
  if( fid != NULL )
    fclose(fid);

  // cleanup
  for( k = 0; k < nk; k++ )
  {
    if( bytes[k] != NULL )
      free( bytes[k] );
    if( compressed[k] != NULL )
      free( compressed[k] );
  }
}


BOOL MTX_GetCompressedFileAttributes(
                                     const char *path,
                                     unsigned* nrows,
                                     unsigned* ncols,
                                     BOOL* isReal
                                     )
{
  FILE* fid = NULL;
  char msg[512];

  int version = 0; // matrix file version nr (0 is invalid)
  size_t count = 0;

  _MTX_STRUCT_FileHeader fileHeader; // file header information

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, path, "rb" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned an error condition." );
    return FALSE;
  }
#else
  fid = fopen( path, "rb" );
#endif
  if( !fid )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( msg, 512, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#else
    if( sprintf( msg, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#endif
    return FALSE;
  }

  // read the file identifiers
  count = fread( fileHeader.id, sizeof(char), MTX_ID_SIZE, fid );
  if( count != MTX_ID_SIZE )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  if( strcmp( fileHeader.id, MTX_ID_COMPRESSED_01 ) == 0 ){ version = MTX_VERSION_NR_COMPRESSED_01; }
  else if( strcmp( fileHeader.id, MTX_ID_LEGACY_V01 ) == 0 ){ version = MTX_VERSION_NR_LEGACY_V01; }
  else if( strcmp( fileHeader.id, MTX_ID_LEGACY_V02 ) == 0 ){ version = MTX_VERSION_NR_LEGACY_V02; }

  if( version == 0 )
  {
    MTX_ERROR_MSG( "Unsupported compressed Matrix version." );
    fclose(fid);
    return FALSE;
  }

  if( version == MTX_VERSION_NR_LEGACY_V01 || version == MTX_VERSION_NR_LEGACY_V01 )
  {
    // NOT SUPPORTED HERE
    MTX_ERROR_MSG( "Unsupported compressed Matrix version." );
    fclose(fid);
    *nrows = 0;
    *ncols = 0;
    return FALSE;
  }

  // get the size of the header
  count = fread( &(fileHeader.headersize), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  // get if the matrix is real or complex
  count = fread( &(fileHeader.isReal), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  // get nrows
  count = fread( &(fileHeader.nrows), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  if( fileHeader.nrows == 0 )
  {
    MTX_ERROR_MSG( "if( fileHeader.nrows == 0 )" );
    fclose(fid);
    return FALSE;
  }
  // get ncols
  count = fread( &(fileHeader.ncols), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  if( fileHeader.ncols == 0 )
  {
    MTX_ERROR_MSG( "if( fileHeader.ncols == 0 )" );
    fclose(fid);
    return FALSE;
  }

  fclose(fid);
  *nrows = fileHeader.nrows;
  *ncols = fileHeader.ncols;
  *isReal = fileHeader.isReal;
  return TRUE;
}


BOOL MTX_ReadCompressed( MTX *M, const char *path )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned ncols = 0; // The number of compressed columns to read. if(M->isReal ) M->ncols else M->ncols*2.

#ifdef MTX_DEBUG
  time_t t0, t1; // time_t is defined on <time.h> and <sys/types.h> as long
  clock_t c0, c1; // clock_t is defined on <time.h> and <sys/types.h> as int
#endif

  const unsigned nk = MTX_NK; // number of byte columns per double column

  unsigned char prevByte = 0;
  unsigned char currByte = 0;
  unsigned nRepeatBytes = 0;
  unsigned n = 0; // counter
  unsigned p = 0; // counter

  unsigned char* bytes[MTX_NK];
  unsigned char* compressed[MTX_NK];
  unsigned char curr[MTX_NK];
  size_t count = 0; // number of bytes returned by fread

  char msg[512];

  FILE* fid = NULL;

  unsigned crc = 0; // the calculated crc
  unsigned filesize = 0; // the calculated file size
  unsigned commentLength = 0; // the length of the comment if present

  int version = 0; // matrix file version nr (0 is invalid)

  double dtmp; // a temporary double
  unsigned doubleWords[2] = {0,0}; // two 32 bit words that form a double

  _MTX_STRUCT_CompressedColumnHeader columnHeader; // column header information, just easier encapsulation

  _MTX_STRUCT_FileHeader fileHeader; // file header information

  MTX MtxRe; // For complex input, the real component column vector.
  MTX MtxIm; // For complex input, the imag component column vector.
  MTX *MtxCol; // A pointer to either MtxRe or MtxIm.

  // initializing
  MTX_Init( &MtxRe );
  MTX_Init( &MtxIm );

  if( !MTX_DetermineFileSize( path, &filesize ) )
  {
    MTX_ERROR_MSG( "MTX_DetermineFileSize returned FALSE." );
    return FALSE;
  }

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, path, "rb" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned an error condition." );
    return FALSE;
  }
#else
  fid = fopen( path, "rb" );
#endif
  if( !fid )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( msg, 512, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#else
    if( sprintf( msg, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#endif
    return FALSE;
  }

  // read the file identifiers
  count = fread( fileHeader.id, sizeof(char), MTX_ID_SIZE, fid );
  if( count != MTX_ID_SIZE )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  if( strcmp( fileHeader.id, MTX_ID_COMPRESSED_01 ) == 0 ){ version = MTX_VERSION_NR_COMPRESSED_01; }
  else if( strcmp( fileHeader.id, MTX_ID_LEGACY_V01 ) == 0 ){ version = MTX_VERSION_NR_LEGACY_V01; }
  else if( strcmp( fileHeader.id, MTX_ID_LEGACY_V02 ) == 0 ){ version = MTX_VERSION_NR_LEGACY_V02; }

  if( version == 0 )
  {
    MTX_ERROR_MSG( "Unsupported compressed matrix version." );
    fclose(fid);
    return FALSE;
  }

  if( version == MTX_VERSION_NR_LEGACY_V01 || version == MTX_VERSION_NR_LEGACY_V01 )
  {
    fclose(fid);
    return MTX_static_ReadCompressed_LegacyVersion( M, path );
  }

  // get the size of the header
  count = fread( &(fileHeader.headersize), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  // get if the matrix is real or complex
  count = fread( &(fileHeader.isReal), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  // get nrows
  count = fread( &(fileHeader.nrows), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  if( fileHeader.nrows == 0 )
  {
    MTX_ERROR_MSG( "if( fileHeader.nrows == 0 )" );
    fclose(fid);
    return FALSE;
  }
  // get ncols
  count = fread( &(fileHeader.ncols), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  if( fileHeader.ncols == 0 )
  {
    MTX_ERROR_MSG( "if( fileHeader.ncols == 0 )" );
    fclose(fid);
    return FALSE;
  }

  // get the filesize
  count = fread( &(fileHeader.filesize), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  // check the filesize
  if( filesize != fileHeader.filesize )
  {
    MTX_ERROR_MSG( "if( filesize != fileHeader.filesize )" );
    fclose(fid);
    return FALSE;
  }

  // get the crc
  count = fread( &(fileHeader.crc), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  // get the matrix comment if any
  commentLength = fileHeader.headersize - MTX_ID_SIZE - 5*sizeof(unsigned);
  if( commentLength != 0 && commentLength < MTX_MAX_COMMENT_LENGTH )
  {
    fileHeader.comment = (char*)malloc( sizeof(char)*(commentLength+1) );
    if( fileHeader.comment == NULL )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      fclose(fid);
      return FALSE;
    }
    M->comment = fileHeader.comment;
  }
  else
  {
    if( M->comment )
      free( M->comment );
    M->comment = NULL;
  }

  // If the input is complex, the real and imag component vectors are input.
  if( !fileHeader.isReal )
  {
    if( !MTX_Malloc( &MtxRe, fileHeader.nrows, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
    if( !MTX_Malloc( &MtxIm, fileHeader.nrows, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      MTX_Free( &MtxRe );
      MTX_Free( &MtxIm );
      return FALSE;
    }
  }

  // resize the matrix if needed
  if( M->nrows != fileHeader.nrows || M->ncols != fileHeader.ncols || (M->isReal != fileHeader.isReal) )
  {
    if( !MTX_Resize( M, fileHeader.nrows, fileHeader.ncols, fileHeader.isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      fclose(fid);
      MTX_Free( &MtxRe );
      MTX_Free( &MtxIm );
      return FALSE;
    }
  }

  // allocate memory for the RLE byte vectors
  // at most twice as long is possible with RLE
  for( k = 0; k < nk; k++ )
  {
    bytes[k] = (unsigned char*)malloc( M->nrows*sizeof(unsigned char)*2 );
    if( !bytes[k] )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      fclose(fid);
      MTX_Free( &MtxRe );
      MTX_Free( &MtxIm );
      return FALSE;
    }
    compressed[k] = (unsigned char*)malloc( M->nrows*sizeof(unsigned char)*2 );
    if( !compressed[k] )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      fclose(fid);
      MTX_Free( &MtxRe );
      MTX_Free( &MtxIm );
      return FALSE;
    }
  }



#ifdef MTX_DEBUG
  t0 = time(NULL);
  c0 = clock();
#endif

  if( M->isReal )
    ncols = M->ncols;
  else
    ncols = M->ncols*2;

  for( j = 0; j < ncols; j++ )
  {
    if( !M->isReal )
    {
      if( j == 0 || j%2 == 0 )
      {
        MtxCol = &MtxRe;
      }
      else
      {
        MtxCol = &MtxIm;
      }
    }

    // check for special case of a single valued column
    // get first compressed vector length
    count = fread( &(columnHeader.length[0]), sizeof(unsigned), 1, fid );
    if( count != 1 )
    {
      MTX_ERROR_MSG( "fread returned an error condition." );
      MTX_Free( &MtxRe );
      MTX_Free( &MtxIm );
      MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
      return FALSE;
    }
    MTX_static_updateCRC( (unsigned char*)&(columnHeader.length[0]), 4, &crc );
    if( columnHeader.length[0] == 0 )
    {
      count = fread( &dtmp, sizeof(double), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fread returned an error condition." );
        MTX_Free( &MtxRe );
        MTX_Free( &MtxIm );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      memcpy( curr, &dtmp, sizeof(double) );
      MTX_static_updateCRC( curr, 8, &crc );

      if( M->isReal )
      {
        for( i = 0; i < M->nrows; i++ )
          M->data[j][i] = dtmp;
      }
      else
      {
        for( i = 0; i < M->nrows; i++ )
        {
          MtxCol->data[0][i] = dtmp;
        }
        if( j%2 != 0 && j != 0 )
        {
          if( !MTX_SetComplexColumn( M, j/2, &MtxRe, &MtxIm ) )
          {
            MTX_ERROR_MSG( "MTX_SetComplexColumn returned FALSE." );
            MTX_Free( &MtxRe );
            MTX_Free( &MtxIm );
            return FALSE;
          }
        }
      }
      continue;
    }

    // get the rest of the compressed vector lengths
    for( k = 1; k < nk; k++ )
    {
      count = fread( &(columnHeader.length[k]), sizeof(unsigned), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fread returned an error condition." );
        MTX_Free( &MtxRe );
        MTX_Free( &MtxIm );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      MTX_static_updateCRC( (unsigned char*)&(columnHeader.length[k]), 4, &crc );
    }

    // get the indication that compression was used
    for( k = 0; k < nk; k++ )
    {
      count = fread( &(columnHeader.isCompressed[k]), sizeof(unsigned char), 1, fid );
      if( count != 1 )
      {
        MTX_ERROR_MSG( "fread returned an error condition." );
        MTX_Free( &MtxRe );
        MTX_Free( &MtxIm );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      MTX_static_updateCRC( &(columnHeader.isCompressed[k]), 1, &crc );
    }

    // get the compressed data
    for( k = 0; k < nk; k++ )
    {
      if( columnHeader.isCompressed[k] )
      {
        count = fread( compressed[k], sizeof(unsigned char), columnHeader.length[k], fid );
      }
      else
      {
        count = fread( bytes[k], sizeof(unsigned char), columnHeader.length[k], fid );
      }
      if( count != columnHeader.length[k] )
      {
        MTX_ERROR_MSG( "if( count != columnHeader.length[k] )" );
        MTX_Free( &MtxRe );
        MTX_Free( &MtxIm );
        MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
        return FALSE;
      }
      if( columnHeader.isCompressed[k] )
      {
        MTX_static_updateCRC( compressed[k], columnHeader.length[k], &crc );
      }
      else
      {
        MTX_static_updateCRC( bytes[k], columnHeader.length[k], &crc );
      }
    }


    // decompress the data
    // aa1bb2aa3bb0abb4
    // decompressed becomes
    // aaabbbbaaaaabbabbbbbb
    for( k = 0; k < nk; k++ )
    {
      if( columnHeader.isCompressed[k] )
      {
        n = 0;
        p = 0;
        prevByte = compressed[k][p]; p++;
        bytes[k][n] = prevByte; n++;
        while( p < columnHeader.length[k] )
        {
          currByte = compressed[k][p]; p++;
          bytes[k][n] = currByte; n++;

          if( currByte == prevByte )
          {
            nRepeatBytes = compressed[k][p];
            p++;

            nRepeatBytes = n+nRepeatBytes;
            for( n; n < nRepeatBytes; n++ )
            {
              if( n >= M->nrows )
              {
                MTX_ERROR_MSG( "if( n >= M->nrows ) - BAD DATA IN COMPRESSED MATRIX FILE." );
                // bad data!, this should not happen
                MTX_Free( &MtxRe );
                MTX_Free( &MtxIm );
                MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
                return FALSE;
              }
              bytes[k][n] = currByte;
            }
            if( p == columnHeader.length[k] )
              break;
            if( n >= M->nrows )
            {
              MTX_ERROR_MSG( "if( n >= M->nrows ) - BAD DATA IN COMPRESSED MATRIX FILE." );
              // bad data!, this should not happen
              MTX_Free( &MtxRe );
              MTX_Free( &MtxIm );
              MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
              return FALSE;
            }
            prevByte = compressed[k][p]; p++;
            bytes[k][n] = prevByte; n++;
          }
          else
          {
            prevByte = currByte;
          }
        }

        // check that the bytes vector is fully populated
        if( n != M->nrows )
        {
          MTX_ERROR_MSG( "if( n >= M->nrows ) - BAD DATA IN COMPRESSED MATRIX FILE." );
          MTX_Free( &MtxRe );
          MTX_Free( &MtxIm );
          MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
          return FALSE;
        }
      }
    }

    // reform the doubles using the byte data
    for( i = 0; i < M->nrows; i++ )
    {
      doubleWords[0] = bytes[3][i] << 24;
      doubleWords[0] |= bytes[2][i] << 16;
      doubleWords[0] |= bytes[1][i] << 8;
      doubleWords[0] |= bytes[0][i];

      doubleWords[1] = bytes[7][i] << 24;
      doubleWords[1] |= bytes[6][i] << 16;
      doubleWords[1] |= bytes[5][i] << 8;
      doubleWords[1] |= bytes[4][i];

      if( M->isReal )
      {
        memcpy( &(M->data[j][i]), doubleWords, sizeof(double) );
      }
      else
      {
        memcpy( &(MtxCol->data[0][i]), doubleWords, sizeof(double) );
      }
    }

    if( !M->isReal )
    {
      if( j%2 != 0 && j != 0 )
      {
        if( !MTX_SetComplexColumn( M, j/2, &MtxRe, &MtxIm ) )
        {
          MTX_ERROR_MSG( "MTX_SetComplexColumn returned FALSE." );
          MTX_Free( &MtxRe );
          MTX_Free( &MtxIm );
          return FALSE;
        }
      }
    }
  }

  // check the crc
  if( fileHeader.crc != crc )
  {
    MTX_ERROR_MSG( "if( fileHeader.crc != crc )" );
    MTX_Free( &MtxRe );
    MTX_Free( &MtxIm );
    MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );
    return FALSE;
  }

#ifdef MTX_DEBUG
  t1 = time(NULL);
  c1 = clock();
  printf("File Loaded and decompression RLE in: %.2f (s) (GDM)\n", (float) (c1 - c0)/CLOCKS_PER_SEC);
#endif

  // Cleanup
  MTX_Free( &MtxRe );
  MTX_Free( &MtxIm );
  MTX_static_SaveAndLoadCleanUp( fid, bytes, compressed, nk );

  return TRUE;
}


BOOL MTX_static_ReadCompressed_LegacyVersion( MTX* M, const char *path )
{
  unsigned i = 0;
  unsigned j = 0;
  int version = 0;

  FILE* fid = NULL;

  unsigned g = 0; // counter
  unsigned p = 0; // counter
  unsigned k = 0; // counter
  unsigned char nrepeat = 0;
  unsigned char method = 0;
  double d = 0.0;
  const unsigned kCompressionByDouble = 1;

  unsigned char* columnBuffer = NULL;
  unsigned char* compressedBuffer = NULL;
  unsigned char ucTmp[2];
  unsigned columnBufSize = 0;
  unsigned compressedSize = 0;

  char msg[512];

  size_t count = 0;

  _MTX_STRUCT_FileHeader fileHeader;

  fileHeader.nrows = 0;
  fileHeader.ncols = 0;
  fileHeader.filesize = 0;
  fileHeader.headersize = 0;
  fileHeader.crc = 0;
  fileHeader.comment = NULL;

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, path, "rb" ) != 0 )
  {
    MTX_ERROR_MSG( "fopen_s returned an error condition." );
    return FALSE;
  }
#else
  fid = fopen(path,"rb");
#endif
  if( !fid )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sprintf_s( msg, 512, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#else
    if( sprintf( msg, "Unable to open %s.", path ) > 0 )
      MTX_ERROR_MSG( msg );
#endif
    return FALSE;
  }

  // write the file identifiers
  count = fread( fileHeader.id, sizeof(char), MTX_ID_SIZE, fid );
  if( count != MTX_ID_SIZE )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  if( strcmp( fileHeader.id, MTX_ID_LEGACY_V01 ) == 0 )
  {
    version = MTX_VERSION_NR_LEGACY_V01;
  }
  else if( strcmp( fileHeader.id, MTX_ID_LEGACY_V02 ) == 0 )
  {
    version = MTX_VERSION_NR_LEGACY_V02;
  }
  else // version == 0
  {
    MTX_ERROR_MSG( "Unsupported compressed matrix version." );
    fclose(fid);
    return FALSE;
  }

  count = fread( &(fileHeader.nrows), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }
  count = fread( &(fileHeader.ncols), sizeof(unsigned), 1, fid );
  if( count != 1 )
  {
    MTX_ERROR_MSG( "fread returned an error condition." );
    fclose(fid);
    return FALSE;
  }

  if( fileHeader.nrows == 0 || fileHeader.ncols == 0 )
  {
    MTX_ERROR_MSG( "if( fileHeader.nrows == 0 || fileHeader.ncols == 0 )" );
    fclose(fid);
    return FALSE;
  }

  // read in version 2 header info
  if( version == 2 )
  {
    // info was used for column width determination, no longer needed
    // with better code :D
    for( j = 0; j < fileHeader.ncols; j++ )
    {
      count = fread( ucTmp, sizeof(unsigned char), 2, fid );
      if( count != 2 )
      {
        MTX_ERROR_MSG( "fread returned an error condition." );
        fclose(fid);
        return FALSE;
      }
    }
  }

  // resize the matrix accordingly
  if( M->nrows != fileHeader.nrows || M->ncols != fileHeader.ncols )
  {
    if( !MTX_Resize( M, fileHeader.nrows, fileHeader.ncols, TRUE ) ) //!!!! adress complex/real
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      fclose(fid);
      return FALSE;
    }
  }


  // here's where we go crazy
  // Run Length Encoding is used in decompressing the matrix
  // on a column by column basis

  // allocate a buffer large enough to store one column completely
  columnBufSize = sizeof(double)*(M->nrows);
  columnBuffer = (unsigned char*)malloc( columnBufSize );
  if( columnBuffer == NULL )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    fclose(fid);
    return FALSE;
  }

  for( j = 0; j < M->ncols; j++ )
  {
    // get the size of the compressedBuffer
    count = fread( &compressedSize, sizeof(unsigned), 1, fid );
    if( count != 1 )
    {
      MTX_ERROR_MSG( "fread returned an error condition." );
      if( columnBuffer ) 
        free(columnBuffer);
      fclose(fid);
      return FALSE;
    }

    if( compressedSize == columnBufSize )
    {
      // no compression just read in the doubles
      count = fread( M->data[j], sizeof(double), M->nrows, fid );
      if( count != M->nrows )
      {
        MTX_ERROR_MSG( "fread returned an error condition." );
        if( columnBuffer )
          free(columnBuffer);
        fclose(fid);
        return FALSE;
      }
      continue;
    }

    // allocate the compressedBuffer
    compressedBuffer = (unsigned char*)malloc( compressedSize );
    if( compressedBuffer == NULL )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      if( columnBuffer )
        free(columnBuffer);
      fclose(fid);      
      return FALSE;
    }

    memcpy( &(compressedBuffer[0]), &compressedSize, sizeof(unsigned) );

    // read in the compressed data
    count = fread( &(compressedBuffer[sizeof(unsigned)]), 1, (compressedSize - sizeof(unsigned)), fid );
    if( count != (compressedSize - sizeof(unsigned)) )
    {
      MTX_ERROR_MSG( "fread returned an error condition." );
      if( compressedBuffer )
        free(compressedBuffer);
      if( columnBuffer )
        free(columnBuffer);
      fclose(fid);
      return FALSE;
    }

    // determine compression method
    memcpy( &method, &(compressedBuffer[sizeof(unsigned)]), 1 );
    if( method != kCompressionByDouble )
    {
      MTX_ERROR_MSG( "if( method != kCompressionByDouble )" );
      if( compressedBuffer )
        free(compressedBuffer);
      if( columnBuffer )
        free(columnBuffer);
      fclose(fid);
      return FALSE;
    }

    // simple double RLE decompression
    k = 0;
    for( g = sizeof(unsigned)+1; g < compressedSize-sizeof(double); g+=sizeof(double) )
    {
      nrepeat = compressedBuffer[g];
      g++;
      memcpy( &d, &(compressedBuffer[g]), sizeof(double) );
      for( p = 0; p < nrepeat; p++)
      {
        memcpy( &(columnBuffer[k]), &d, sizeof(double) );
        k+=sizeof(double);
      }
    }

    // set the data
    for( i = 0; i < M->nrows; i++ )
      memcpy( &(M->data[j][i]), &(columnBuffer[i*sizeof(double)]), sizeof(double) );

    // remember to free the compressed buffer
    free(compressedBuffer);
    compressedBuffer = NULL;
    compressedSize = 0;
  }

  // remember to delete the columnBuffer
  free(columnBuffer);

  // we're done
  fclose(fid);

  return TRUE;
}

BOOL MTX_LoadAndSave( const char* infilepath, const char* outfilepath )
{
  MTX M;
  MTX_Init( &M );

  if( !MTX_ReadFromFile( &M, infilepath ) )
  {
    MTX_ERROR_MSG( "MTX_ReadFromFile returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }

  if( !MTX_SaveCompressed( &M, outfilepath ) )
  {
    MTX_ERROR_MSG( "MTX_SaveCompressed returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }

  MTX_Free( &M );
  return TRUE;
}

BOOL MTX_LoadAndSaveQuick( const char* infilepath )
{
  unsigned k;
  unsigned p;
  unsigned length;
  char outfilepath[1024];
  char *strptr;

  MTX M;
  MTX_Init( &M );

  if( !MTX_ReadFromFile( &M, infilepath ) )
  {
    MTX_ERROR_MSG( "MTX_ReadFromFile returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }

  length = (unsigned int)strlen(infilepath);
  if( length >= 1024 )
  {
    MTX_ERROR_MSG( "if( length >= 1024 )" );
    MTX_Free( &M );
    return FALSE;
  }

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( strcpy_s( outfilepath, 1024, infilepath ) != 0 )
  {
    MTX_ERROR_MSG( "strcpy_s returned an error condition." );
    MTX_Free( &M );
    return FALSE;
  }
#else
  strcpy( outfilepath, infilepath );
#endif
  // find the last instance of '.' in the input file name
  for( k = 0; k < length; k++ )
  {
    if( outfilepath[k] == '.' )
    {
      strptr = &(outfilepath[k]);
      p = k;
    }
  }
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( sprintf_s( strptr, length-p+1, ".mtx" ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    MTX_Free( &M );
    return FALSE;
  }
#else
  if( sprintf( strptr, ".mtx" ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }
#endif

  // save the data
  if( !MTX_SaveCompressed( &M, outfilepath ) )
  {
    MTX_ERROR_MSG( "MTX_SaveCompressed returned FALSE." );
    MTX_Free( &M );
    return FALSE;
  }

  MTX_Free( &M );
  return TRUE;
}

BOOL MTX_TimeWindow(
                    MTX* M, //!< Matrix to be altered
                    const unsigned timeColumn, //!< The column containing time
                    const double startTime, //!< The specified start time (inclusive)
                    const double duration, //!< The duration to include
                    const double rolloverTime )//!< The potential time at which system time rolls over
{
  unsigned i = 0;
  unsigned j = 0;
  const double endTime = startTime + duration;
  unsigned index = 0;
  double time;
  double dt;

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( timeColumn >= M->ncols )
  {
    MTX_ERROR_MSG( "if( timeColumn >= M->ncols )" );
    return FALSE;
  }

  i = 0;
  if( M->isReal )
    time = M->data[timeColumn][i];
  else
    time = M->cplx[timeColumn][i].re;

  i++;
  if( time >= startTime && time <= endTime )
  {
    // no need to copy the 0th row to the 0th row
    index++;
  }

  if( time < endTime )
  {
    while( i < M->nrows )
    {
      if( M->isReal )
        dt = M->data[timeColumn][i] - time;
      else
        dt = M->cplx[timeColumn][i].re - time;

      if( dt < 0.0 )
      {
        if( rolloverTime == 0.0 )
          break;
        dt += rolloverTime;
      }

      time += dt;

      if( time > endTime )
        break;

      if( time >= startTime && time <= endTime )
      {
        // copy this row if needed
        if( index != i )
        {
          for( j = 0; j < M->ncols; j++ )
          {
            if( M->isReal )
            {
              M->data[j][index] = M->data[j][i];
            }
            else
            {
              M->cplx[j][index].re = M->cplx[j][i].re;
              M->cplx[j][index].im = M->cplx[j][i].im;
            }
          }
        }
        index++;
      }
      i++;
    }
  }

  if( index == 0 )
  {
    MTX_Free( M );
  }
  else
  {
    // redimension the matrix accordingly
    if( !MTX_Redim( M, index, M->ncols ) )
    {
      MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
      return FALSE;
    }
  }

  return TRUE;
}

BOOL MTX_TimeLimit(
                   MTX* M, //!< Matrix to be altered
                   const unsigned timeColumn, //!< The column containing time
                   const double startTime, //!< The specified start time (inclusive)
                   const double endTime )//!< The duration to include
{
  double duration = endTime - startTime;

  if( duration < 0 )
  {
    MTX_ERROR_MSG( "if( duration < 0 )" );
    return FALSE;
  }

  return MTX_TimeWindow( M, timeColumn, startTime, duration, 0.0 );
}

BOOL MTX_TimeMatch(
                   MTX *A,
                   const unsigned timeColumnA,
                   MTX *B,
                   const unsigned timeColumnB,
                   const unsigned precision,
                   const double rolloverTime )
{
  unsigned index = 0; // index into the final time matched matrices
  unsigned indexA = 0; // index into matrix A
  unsigned indexB = 0; // index into matrix B
  unsigned col;

  double time1; // current time rounded to the specified precision for matrix A
  double time2; // current time rounded to the specified precision for matrix B
  double time1_prev; // previous time rounded to the specified precision for matrix A
  double time2_prev; // previous time rounded to the specified precision for matrix B
  double ctime1; // continuous time without rollovers equivalent for time1
  double ctime2; // continuous time without rollovers equivalent for time2
  double dt; // delta time

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( MTX_isNull( B ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( timeColumnA >= A->ncols || timeColumnB >= B->ncols )
  {
    MTX_ERROR_MSG( "if( timeColumnA >= A->ncols || timeColumnB >= B->ncols )" );
    return FALSE;
  }

  if( A->isReal )
    time1 = A->data[timeColumnA][indexA];
  else
    time1 = A->cplx[timeColumnA][indexA].re;

  MTX_static_round_value( &time1, precision );
  ctime1 = time1;

  if( B->isReal )
    time2 = B->data[timeColumnB][indexB];
  else
    time2 = B->cplx[timeColumnB][indexB].re;

  MTX_static_round_value( &time2, precision );
  ctime2 = time2;

  while( indexA < A->nrows && indexB < B->nrows )
  {
    if( ctime1 == ctime2 )
    {
      // copy row to A
      if( index != indexA )
      {
        for( col = 0; col < A->ncols; col++ )
        {
          if( A->isReal )
          {
            A->data[col][index] = A->data[col][indexA];
          }
          else
          {
            A->cplx[col][index].re = A->cplx[col][indexA].re;
            A->cplx[col][index].im = A->cplx[col][indexA].im;
          }
        }
      }

      // copy row to B
      if( index != indexB )
      {
        for( col = 0; col < B->ncols; col++ )
        {
          if( B->isReal )
          {
            B->data[col][index] = B->data[col][indexB];
          }
          else
          {
            B->cplx[col][index].re = B->cplx[col][indexB].re;
            B->cplx[col][index].im = B->cplx[col][indexB].im;
          }
        }
      }
      index++;
      indexA++;
      indexB++;

      time1_prev = time1;
      if( A->isReal )
        time1 = A->data[timeColumnA][indexA];
      else
        time1 = A->cplx[timeColumnA][indexA].re;

      MTX_static_round_value( &time1, precision );
      dt = time1 - time1_prev;
      if( dt < 0.0 )
      {
        if( rolloverTime == 0.0 )
          break; // rollovers not allowed
        dt += rolloverTime;
      }
      ctime1 += dt;

      time2_prev = time2;
      if( B->isReal )
        time2 = B->data[timeColumnB][indexB];
      else
        time2 = B->cplx[timeColumnB][indexB].re;

      MTX_static_round_value( &time2, precision );
      dt = time2 - time2_prev;
      if( dt < 0.0 )
      {
        if( rolloverTime == 0.0 )
          break; // rollovers not allowed
        dt += rolloverTime;
      }
      ctime2 += dt;
    }
    else if( ctime1 < ctime2 )
    {
      indexA++;

      time1_prev = time1;
      if( A->isReal )
        time1 = A->data[timeColumnA][indexA];
      else
        time1 = A->cplx[timeColumnA][indexA].re;

      MTX_static_round_value( &time1, precision );
      dt = time1 - time1_prev;
      if( dt < 0.0 )
      {
        if( rolloverTime == 0.0 )
          break; // rollovers not allowed
        dt += rolloverTime;
      }
      ctime1 += dt;
    }
    else
    {
      indexB++;

      time2_prev = time2;
      if( B->isReal )
        time2 = B->data[timeColumnB][indexB];
      else
        time2 = B->cplx[timeColumnB][indexB].re;

      MTX_static_round_value( &time2, precision );
      dt = time2 - time2_prev;
      if( dt < 0.0 )
      {
        if( rolloverTime == 0.0 )
          break; // rollovers not allowed
        dt += rolloverTime;
      }
      ctime2 += dt;
    }
  }

  // special case
  if( index == 0 )
  {
    MTX_Free( A );
    MTX_Free( B );
    return TRUE;
  }

  // redimension appropriately
  if( !MTX_Redim( A, index, A->ncols ) )
  {
    MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
    return FALSE;
  }
  if( !MTX_Redim( B, index, B->ncols ) )
  {
    MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
    return FALSE;
  }

  return TRUE;
}

BOOL MTX_Interpolate(
                     MTX *A, //!< The matrix with interpolation times
                     const unsigned timeColumnA, //!< The zero based column index for matrix A
                     MTX *B, //!< The matrix to be interpolated
                     const unsigned timeColumnB, //!< The zero based column index for matrix B
                     const double maxInterpolationInterval, //!< The largest interpolation interval allowed
                     const double rolloverTime )//!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
{
  unsigned i = 0;
  unsigned j = 0;

  unsigned index = 0,
    indexA = 0,
    indexB = 0;

  double time = 0.0,
    prev = 0.0,
    next = 0.0,
    interval = 0.0,
    slope = 0.0,
    slope_im = 0.0;

  indexA = 0;
  indexB = 1;
  while( indexA < A->nrows && indexB < B->nrows )
  {
    if( A->isReal )
      time = A->data[timeColumnA][indexA];
    else
      time = A->cplx[timeColumnA][indexA].re;

    if( B->isReal )
      prev = B->data[timeColumnB][indexB-1];
    else
      prev = B->cplx[timeColumnB][indexB-1].re;

    if( B->isReal )
      next = B->data[timeColumnB][indexB];
    else
      next = B->cplx[timeColumnB][indexB].re;

    interval = next - prev;
    if( interval < 0 ) // a rollover occurred
    {
      if( rolloverTime == 0.0 )
        break;
      interval += rolloverTime;
      next += rolloverTime;
    }

    if( time >= prev &&
      time <= next )
    {
      if( interval > maxInterpolationInterval )
      {
        indexB++;
        continue;
      }

      if( index != indexA )
      {
        if( A->isReal )
        {
          A->data[timeColumnA][index] = time;
        }
        else
        {
          A->cplx[timeColumnA][index].re = time;
        }

        // copy row to other A matrix
        for( j = 0; j < A->ncols; j++ )
        {
          if( A->isReal )
          {
            A->data[j][index] = A->data[j][indexA];
          }
          else
          {
            A->cplx[j][index].re = A->cplx[j][indexA].re;
            A->cplx[j][index].im = A->cplx[j][indexA].im;
          }
        }
      }

      // perform the interpolation
      for( j = 0; j < B->ncols; j++ )
      {
        if( j == timeColumnB )
        {
          if( B->isReal )
            B->data[timeColumnB][index] = time;
          else
            B->cplx[timeColumnB][index].re = time;

          continue;
        }
        if( B->isReal )
        {
          slope = (B->data[j][indexB] - B->data[j][indexB-1]) / interval;
          B->data[j][index] = slope * (time-prev) + B->data[j][indexB-1];
        }
        else
        {
          slope = (B->cplx[j][indexB].re - B->cplx[j][indexB-1].re) / interval;
          slope_im = (B->cplx[j][indexB].im - B->cplx[j][indexB-1].im) / interval;

          B->cplx[j][index].re = slope * (time-prev) + B->cplx[j][indexB-1].re;
          B->cplx[j][index].im = slope_im * (time-prev) + B->cplx[j][indexB-1].im;
        }
      }
      index++;
      indexA++;
      indexB++;
      continue;
    }

    if( time < prev )
      indexA++;
    else if( time > next )
      indexB++;
  }
  if( !MTX_Redim( A, index, A->ncols ) )
  {
    MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
    return FALSE;
  }
  if( !MTX_Redim( B, index, B->ncols ) )
  {
    MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
    return FALSE;
  }

  return TRUE;
}


BOOL MTX_InvertInPlaceClosedForm( MTX *M )
{
  unsigned n;
  double dtmp;
  
  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_isSquare( M ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }
  n = M->nrows;
  if( n > 3 )
  {
    MTX_ERROR_MSG( "if( M->nrows > 3 )" );
    return FALSE;
  }  

  if( M->isReal )
  {
    ////
    // quick special cases, (closed forms)
    // 1x1
    if( n == 1 )
    {
      // Note: No divide by zero check.
      M->data[0][0] = 1.0/M->data[0][0];
      return TRUE;
    }
    // 2x2
    if( n == 2 )
    {
      // A = [a b; c d]
      // A^-1 = 1.0/(ad-bc) * [d -b; -c a]
      double invdet;
      invdet = M->data[0][0] * M->data[1][1] - M->data[1][0] * M->data[0][1];
      
      // Note: No divide by zero check.
      invdet = 1.0/invdet;

      dtmp = M->data[0][0];
      M->data[0][0] = invdet * M->data[1][1];
      M->data[1][1] = invdet * dtmp;
      M->data[1][0] *= -invdet;
      M->data[0][1] *= -invdet;

      return TRUE;
    }
    // 3x3
    if( n == 3 )
    {
      // M = [r s t;
      // u v w;
      // x y z]
      //
      // det(M) = r(vz-yw) - s(uz-xw) + t(uy-xv)
      // det(M) = r(vz-yw) + s(xw-uz) + t(uy-xv)
      //
      // A = [a11 a12 a13;
      // a21 a22 a23;
      // a31 a32 a33];
      //
      // a11 = vz-yw a12 = ty-zs a13 = sw-vt
      // a21 = wx-uz a22 = rz-xt a23 = ut-rw
      // a31 = uy-xv a32 = xs-ry a33 = rv-us
      //
      // inv(M) = 1/det(M) * A
      double invdet;
      double r,s,t,u,v,w,x,y,z; // elements of original M for ease of use
      double vzyw; // redudant shared calculations
      double xwuz; // redudant shared calculations
      double uyxv; // redudant shared calculations

      r = M->data[0][0]; s = M->data[1][0]; t = M->data[2][0];
      u = M->data[0][1]; v = M->data[1][1]; w = M->data[2][1];
      x = M->data[0][2]; y = M->data[1][2]; z = M->data[2][2];

      // shared calculations
      vzyw = v*z - y*w;
      xwuz = x*w - u*z;
      uyxv = u*y - x*v;

      invdet = r*vzyw + s*xwuz + t*uyxv;

      // Note: No divide by zero check.
      invdet = 1.0/invdet;

      M->data[0][0] = invdet*vzyw; M->data[1][0] = invdet*(t*y-z*s); M->data[2][0] = invdet*(s*w-v*t);
      M->data[0][1] = invdet*xwuz; M->data[1][1] = invdet*(r*z-x*t); M->data[2][1] = invdet*(u*t-r*w);
      M->data[0][2] = invdet*uyxv; M->data[1][2] = invdet*(x*s-r*y); M->data[2][2] = invdet*(r*v-u*s);

      return TRUE;
    }
  }
  else
  {
    ////
    // quick special cases, (closed forms)
    // 1x1
    if( n == 1 )
    {
      // Note: No divide by zero check.
      dtmp = M->cplx[0][0].re*M->cplx[0][0].re + M->cplx[0][0].im*M->cplx[0][0].im;
      M->cplx[0][0].re = M->cplx[0][0].re / dtmp;
      M->cplx[0][0].im = -M->cplx[0][0].im / dtmp;
      return TRUE;
    }
    // 2x2
    if( n == 2 )
    {
      // A = [a b; c d]
      // A^-1 = 1.0/(ad-bc) * [d -b; -c a]

      stComplex invdet; // complex inverse determinant
      stComplex tmpcplx;

      // compute 1/det(M)
      if( !MTX_Det( M, &tmpcplx.re, &tmpcplx.im ) )
      {
        MTX_ERROR_MSG( "MTX_Det returned FALSE." );
        return FALSE;
      }
      // Note: No divide by zero check.
      dtmp = tmpcplx.re*tmpcplx.re + tmpcplx.im*tmpcplx.im;
      invdet.re = tmpcplx.re / dtmp;
      invdet.im = -tmpcplx.im / dtmp;

      // A = [d -b; -c a]
      tmpcplx.re = M->cplx[0][0].re;
      tmpcplx.im = M->cplx[0][0].im;
      M->cplx[0][0].re = M->cplx[1][1].re;
      M->cplx[0][0].im = M->cplx[1][1].im;
      M->cplx[1][1].re = tmpcplx.re;
      M->cplx[1][1].re = tmpcplx.im;
      M->cplx[1][0].re *= -1.0;
      M->cplx[1][0].im *= -1.0;
      M->cplx[0][1].re *= -1.0;
      M->cplx[0][1].im *= -1.0;

      // then A *= invdet
      if( !MTX_Multiply_ScalarComplex( M, invdet.re, invdet.im ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
        return FALSE;
      }

      return TRUE;
    }
    // 3x3
    if( n == 3 )
    {
      // M = [r s t;
      // u v w;
      // x y z]
      //
      // det(M) = r(vz-yw) - s(uz-xw) + t(uy-xv)
      // det(M) = r(vz-yw) + s(xw-uz) + t(uy-xv)
      //
      // A = [a11 a12 a13;
      // a21 a22 a23;
      // a31 a32 a33];
      //
      // a11 = vz-yw a12 = ty-zs a13 = sw-vt
      // a21 = wx-uz a22 = rz-xt a23 = ut-rw
      // a31 = uy-xv a32 = xs-ry a33 = rv-us
      //
      // inv(M) = 1/det(M) * A
      stComplex invdet;
      stComplex r,s,t,u,v,w,x,y,z; // elements of original M for ease of use
      stComplex vzyw; // redudant shared calculations
      stComplex xwuz; // redudant shared calculations
      stComplex uyxv; // redudant shared calculations
      stComplex cplxA; // tmp cplx
      stComplex cplxB; // tmp cplx

      r.re = M->cplx[0][0].re; s.re = M->cplx[1][0].re; t.re = M->cplx[2][0].re;
      u.re = M->cplx[0][1].re; v.re = M->cplx[1][1].re; w.re = M->cplx[2][1].re;
      x.re = M->cplx[0][2].re; y.re = M->cplx[1][2].re; z.re = M->cplx[2][2].re;

      r.im = M->cplx[0][0].im; s.im = M->cplx[1][0].im; t.im = M->cplx[2][0].im;
      u.im = M->cplx[0][1].im; v.im = M->cplx[1][1].im; w.im = M->cplx[2][1].im;
      x.im = M->cplx[0][2].im; y.im = M->cplx[1][2].im; z.im = M->cplx[2][2].im;

      // shared calculations

      // vzyw
      MTX_static_quick_complex_mult_ab( &v.re, &v.im, &z.re, &z.im, &cplxA.re, &cplxA.im ); // vz
      MTX_static_quick_complex_mult_ab( &y.re, &y.im, &w.re, &w.im, &cplxB.re, &cplxB.im ); // yw
      vzyw.re = cplxA.re - cplxB.re;
      vzyw.im = cplxA.im - cplxB.im;

      // xwuz
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &w.re, &w.im, &cplxA.re, &cplxA.im ); // xw
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &z.re, &z.im, &cplxB.re, &cplxB.im ); // uz
      xwuz.re = cplxA.re - cplxB.re;
      xwuz.im = cplxA.im - cplxB.im;

      // uyxv
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &y.re, &y.im, &cplxA.re, &cplxA.im ); // uy
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &v.re, &v.im, &cplxB.re, &cplxB.im ); // xv
      uyxv.re = cplxA.re - cplxB.re;
      uyxv.im = cplxA.im - cplxB.im;

      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &vzyw.re, &vzyw.im, &cplxA.re, &cplxA.im ); // r*vzyw
      MTX_static_quick_complex_mult_ab( &s.re, &s.im, &xwuz.re, &xwuz.im, &cplxB.re, &cplxB.im ); // s*xwuz
      MTX_static_quick_complex_mult_ab( &t.re, &t.im, &uyxv.re, &uyxv.im, &invdet.re, &invdet.im ); // invdet = t*uyxv

      // invdet += r*vzyw + s*xwuz;
      invdet.re += cplxA.re + cplxB.re;
      invdet.im += cplxA.im + cplxB.im;
      // Note: No divide by zero check.
      // compute invdet = 1/det
      dtmp = invdet.re*invdet.re + invdet.im*invdet.im;
      invdet.re = invdet.re/dtmp;
      invdet.im = -invdet.im/dtmp;

      M->cplx[0][0].re = vzyw.re;
      M->cplx[0][1].re = xwuz.re;
      M->cplx[0][2].re = uyxv.re;

      M->cplx[0][0].im = vzyw.im;
      M->cplx[0][1].im = xwuz.im;
      M->cplx[0][2].im = uyxv.im;

      // (t*y-z*s);
      MTX_static_quick_complex_mult_ab( &t.re, &t.im, &y.re, &y.im, &cplxA.re, &cplxA.im ); // ty
      MTX_static_quick_complex_mult_ab( &z.re, &z.im, &s.re, &s.im, &cplxB.re, &cplxB.im ); // zs
      M->cplx[1][0].re = cplxA.re - cplxB.re;
      M->cplx[1][0].im = cplxA.im - cplxB.im;

      // (r*z-x*t);
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &z.re, &z.im, &cplxA.re, &cplxA.im ); // rz
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &t.re, &t.im, &cplxB.re, &cplxB.im ); // xt
      M->cplx[1][1].re = cplxA.re - cplxB.re;
      M->cplx[1][1].im = cplxA.im - cplxB.im;

      // (x*s-r*y);
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &s.re, &s.im, &cplxA.re, &cplxA.im ); // xs
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &y.re, &y.im, &cplxB.re, &cplxB.im ); // ry
      M->cplx[1][2].re = cplxA.re - cplxB.re;
      M->cplx[1][2].im = cplxA.im - cplxB.im;

      // (s*w-v*t);
      MTX_static_quick_complex_mult_ab( &s.re, &s.im, &w.re, &w.im, &cplxA.re, &cplxA.im ); // sw
      MTX_static_quick_complex_mult_ab( &v.re, &v.im, &t.re, &t.im, &cplxB.re, &cplxB.im ); // vt
      M->cplx[2][0].re = cplxA.re - cplxB.re;
      M->cplx[2][0].im = cplxA.im - cplxB.im;

      // (u*t-r*w);
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &t.re, &t.im, &cplxA.re, &cplxA.im ); // ut
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &w.re, &w.im, &cplxB.re, &cplxB.im ); // rw
      M->cplx[2][1].re = cplxA.re - cplxB.re;
      M->cplx[2][1].im = cplxA.im - cplxB.im;

      // (r*v-u*s);
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &v.re, &v.im, &cplxA.re, &cplxA.im ); // rv
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &s.re, &s.im, &cplxB.re, &cplxB.im ); // us
      M->cplx[2][2].re = cplxA.re - cplxB.re;
      M->cplx[2][2].im = cplxA.im - cplxB.im;

      if( !MTX_Multiply_ScalarComplex( M, invdet.re, invdet.im ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
        return FALSE;
      }

      return TRUE;
    }
  }

  MTX_ERROR_MSG( "Unexpected." );
  return FALSE;
}


BOOL MTX_InvertClosedForm( const MTX *src, MTX *dst )
{
  unsigned n;
  double dtmp;

  if( dst == NULL )
  {
    MTX_ERROR_MSG( "NULL input matrix." );
    return FALSE;
  }
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  n = src->nrows;
  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }
  if( src->nrows > 3 )
  {
    MTX_ERROR_MSG( "if( src->nrows > 3 )" );
    return FALSE;
  }
  if( !MTX_Malloc( dst, src->nrows, src->ncols, src->isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }

  if( src->isReal )
  {
    ////
    // quick special cases, (closed forms)
    // 1x1
    if( n == 1 )
    {
      // Note: No divide by zero check.
      dst->data[0][0] = 1.0/src->data[0][0];
      return TRUE;
    }
    // 2x2
    if( n == 2 )
    {
      // A = [a b; c d]
      // A^-1 = 1.0/(ad-bc) * [d -b; -c a]

      double invdet;
      invdet = src->data[0][0] * src->data[1][1] - src->data[1][0] * src->data[0][1];
      
      // Note: No divide by zero check.
      invdet = 1.0/invdet;

      dst->data[0][0] =  invdet * src->data[1][1];
      dst->data[1][1] =  invdet * src->data[0][0];
      dst->data[1][0] = -invdet * src->data[1][0];
      dst->data[0][1] = -invdet * src->data[0][1];

      return TRUE;
    }
    // 3x3
    if( n == 3 )
    {
      // M = [r s t;
      // u v w;
      // x y z]
      //
      // det(M) = r(vz-yw) - s(uz-xw) + t(uy-xv)
      // det(M) = r(vz-yw) + s(xw-uz) + t(uy-xv)
      //
      // A = [a11 a12 a13;
      // a21 a22 a23;
      // a31 a32 a33];
      //
      // a11 = vz-yw a12 = ty-zs a13 = sw-vt
      // a21 = wx-uz a22 = rz-xt a23 = ut-rw
      // a31 = uy-xv a32 = xs-ry a33 = rv-us
      //
      // inv(M) = 1/det(M) * A
      double invdet;
      double r,s,t,u,v,w,x,y,z; // elements of original M for ease of use
      double vzyw; // redudant shared calculations
      double xwuz; // redudant shared calculations
      double uyxv; // redudant shared calculations

      r = src->data[0][0]; s = src->data[1][0]; t = src->data[2][0];
      u = src->data[0][1]; v = src->data[1][1]; w = src->data[2][1];
      x = src->data[0][2]; y = src->data[1][2]; z = src->data[2][2];

      // shared calculations
      vzyw = v*z - y*w;
      xwuz = x*w - u*z;
      uyxv = u*y - x*v;

      invdet = r*vzyw + s*xwuz + t*uyxv;

      // Note: No divide by zero check.
      invdet = 1.0/invdet;

      dst->data[0][0] = invdet*vzyw; dst->data[1][0] = invdet*(t*y-z*s); dst->data[2][0] = invdet*(s*w-v*t);
      dst->data[0][1] = invdet*xwuz; dst->data[1][1] = invdet*(r*z-x*t); dst->data[2][1] = invdet*(u*t-r*w);
      dst->data[0][2] = invdet*uyxv; dst->data[1][2] = invdet*(x*s-r*y); dst->data[2][2] = invdet*(r*v-u*s);

      return TRUE;
    }
  }
  else
  {
    ////
    // quick special cases, (closed forms)
    // 1x1
    if( n == 1 )
    {
      // Note: No divide by zero check.
      dtmp = src->cplx[0][0].re*src->cplx[0][0].re + src->cplx[0][0].im*src->cplx[0][0].im;
      dst->cplx[0][0].re = src->cplx[0][0].re / dtmp;
      dst->cplx[0][0].im = -src->cplx[0][0].im / dtmp;
      return TRUE;
    }
    // 2x2
    if( n == 2 )
    {
      // A = [a b; c d]
      // A^-1 = 1.0/(ad-bc) * [d -b; -c a]

      stComplex invdet; // complex inverse determinant
      stComplex tmpcplx;

      // compute 1/det(src)
      if( !MTX_Det( src, &tmpcplx.re, &tmpcplx.im ) )
      {
        MTX_ERROR_MSG( "MTX_Det returned FALSE." );
        return FALSE;
      }
      // Note: No divide by zero check.
      dtmp = tmpcplx.re*tmpcplx.re + tmpcplx.im*tmpcplx.im;
      invdet.re = tmpcplx.re / dtmp;
      invdet.im = -tmpcplx.im / dtmp;

      // A = [d -b; -c a]
      dst->cplx[0][0].re =  src->cplx[1][1].re;
      dst->cplx[0][0].im =  src->cplx[1][1].im;
      dst->cplx[1][1].re =  src->cplx[0][0].re;
      dst->cplx[1][1].re =  src->cplx[0][0].im;
      dst->cplx[1][0].re = -src->cplx[1][0].re;
      dst->cplx[1][0].im = -src->cplx[1][0].im;
      dst->cplx[0][1].re = -src->cplx[0][1].re;
      dst->cplx[0][1].im = -src->cplx[0][1].im;

      // then A *= invdet
      if( !MTX_Multiply_ScalarComplex( dst, invdet.re, invdet.im ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
        return FALSE;
      }

      return TRUE;
    }
    // 3x3
    if( n == 3 )
    {
      // M = [r s t;
      // u v w;
      // x y z]
      //
      // det(M) = r(vz-yw) - s(uz-xw) + t(uy-xv)
      // det(M) = r(vz-yw) + s(xw-uz) + t(uy-xv)
      //
      // A = [a11 a12 a13;
      // a21 a22 a23;
      // a31 a32 a33];
      //
      // a11 = vz-yw a12 = ty-zs a13 = sw-vt
      // a21 = wx-uz a22 = rz-xt a23 = ut-rw
      // a31 = uy-xv a32 = xs-ry a33 = rv-us
      //
      // inv(src) = 1/det(src) * A
      stComplex invdet;
      stComplex r,s,t,u,v,w,x,y,z; // elements of original M for ease of use
      stComplex vzyw; // redudant shared calculations
      stComplex xwuz; // redudant shared calculations
      stComplex uyxv; // redudant shared calculations
      stComplex cplxA; // tmp cplx
      stComplex cplxB; // tmp cplx

      r.re = src->cplx[0][0].re; s.re = src->cplx[1][0].re; t.re = src->cplx[2][0].re;
      u.re = src->cplx[0][1].re; v.re = src->cplx[1][1].re; w.re = src->cplx[2][1].re;
      x.re = src->cplx[0][2].re; y.re = src->cplx[1][2].re; z.re = src->cplx[2][2].re;

      r.im = src->cplx[0][0].im; s.im = src->cplx[1][0].im; t.im = src->cplx[2][0].im;
      u.im = src->cplx[0][1].im; v.im = src->cplx[1][1].im; w.im = src->cplx[2][1].im;
      x.im = src->cplx[0][2].im; y.im = src->cplx[1][2].im; z.im = src->cplx[2][2].im;

      // shared calculations

      // vzyw
      MTX_static_quick_complex_mult_ab( &v.re, &v.im, &z.re, &z.im, &cplxA.re, &cplxA.im ); // vz
      MTX_static_quick_complex_mult_ab( &y.re, &y.im, &w.re, &w.im, &cplxB.re, &cplxB.im ); // yw
      vzyw.re = cplxA.re - cplxB.re;
      vzyw.im = cplxA.im - cplxB.im;

      // xwuz
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &w.re, &w.im, &cplxA.re, &cplxA.im ); // xw
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &z.re, &z.im, &cplxB.re, &cplxB.im ); // uz
      xwuz.re = cplxA.re - cplxB.re;
      xwuz.im = cplxA.im - cplxB.im;

      // uyxv
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &y.re, &y.im, &cplxA.re, &cplxA.im ); // uy
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &v.re, &v.im, &cplxB.re, &cplxB.im ); // xv
      uyxv.re = cplxA.re - cplxB.re;
      uyxv.im = cplxA.im - cplxB.im;

      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &vzyw.re, &vzyw.im, &cplxA.re, &cplxA.im ); // r*vzyw
      MTX_static_quick_complex_mult_ab( &s.re, &s.im, &xwuz.re, &xwuz.im, &cplxB.re, &cplxB.im ); // s*xwuz
      MTX_static_quick_complex_mult_ab( &t.re, &t.im, &uyxv.re, &uyxv.im, &invdet.re, &invdet.im ); // invdet = t*uyxv

      // invdet += r*vzyw + s*xwuz;
      invdet.re += cplxA.re + cplxB.re;
      invdet.im += cplxA.im + cplxB.im;
      // Note: No divide by zero check.
      // compute invdet = 1/det
      dtmp = invdet.re*invdet.re + invdet.im*invdet.im;
      invdet.re = invdet.re/dtmp;
      invdet.im = -invdet.im/dtmp;

      dst->cplx[0][0].re = vzyw.re;
      dst->cplx[0][1].re = xwuz.re;
      dst->cplx[0][2].re = uyxv.re;

      dst->cplx[0][0].im = vzyw.im;
      dst->cplx[0][1].im = xwuz.im;
      dst->cplx[0][2].im = uyxv.im;

      // (t*y-z*s);
      MTX_static_quick_complex_mult_ab( &t.re, &t.im, &y.re, &y.im, &cplxA.re, &cplxA.im ); // ty
      MTX_static_quick_complex_mult_ab( &z.re, &z.im, &s.re, &s.im, &cplxB.re, &cplxB.im ); // zs
      dst->cplx[1][0].re = cplxA.re - cplxB.re;
      dst->cplx[1][0].im = cplxA.im - cplxB.im;

      // (r*z-x*t);
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &z.re, &z.im, &cplxA.re, &cplxA.im ); // rz
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &t.re, &t.im, &cplxB.re, &cplxB.im ); // xt
      dst->cplx[1][1].re = cplxA.re - cplxB.re;
      dst->cplx[1][1].im = cplxA.im - cplxB.im;

      // (x*s-r*y);
      MTX_static_quick_complex_mult_ab( &x.re, &x.im, &s.re, &s.im, &cplxA.re, &cplxA.im ); // xs
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &y.re, &y.im, &cplxB.re, &cplxB.im ); // ry
      dst->cplx[1][2].re = cplxA.re - cplxB.re;
      dst->cplx[1][2].im = cplxA.im - cplxB.im;

      // (s*w-v*t);
      MTX_static_quick_complex_mult_ab( &s.re, &s.im, &w.re, &w.im, &cplxA.re, &cplxA.im ); // sw
      MTX_static_quick_complex_mult_ab( &v.re, &v.im, &t.re, &t.im, &cplxB.re, &cplxB.im ); // vt
      dst->cplx[2][0].re = cplxA.re - cplxB.re;
      dst->cplx[2][0].im = cplxA.im - cplxB.im;

      // (u*t-r*w);
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &t.re, &t.im, &cplxA.re, &cplxA.im ); // ut
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &w.re, &w.im, &cplxB.re, &cplxB.im ); // rw
      dst->cplx[2][1].re = cplxA.re - cplxB.re;
      dst->cplx[2][1].im = cplxA.im - cplxB.im;

      // (r*v-u*s);
      MTX_static_quick_complex_mult_ab( &r.re, &r.im, &v.re, &v.im, &cplxA.re, &cplxA.im ); // rv
      MTX_static_quick_complex_mult_ab( &u.re, &u.im, &s.re, &s.im, &cplxB.re, &cplxB.im ); // us
      dst->cplx[2][2].re = cplxA.re - cplxB.re;
      dst->cplx[2][2].im = cplxA.im - cplxB.im;

      if( !MTX_Multiply_ScalarComplex( dst, invdet.re, invdet.im ) )
      {
        MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned FALSE." );
        return FALSE;
      }

      return TRUE;
    }
  }

  MTX_ERROR_MSG( "Unexpected." );
  return FALSE;
}





BOOL MTX_InvertInPlace( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  BOOL isPositiveDefinite = TRUE;
  unsigned n;
  double **ptrptrData;
  double val;
  double dtmp;
  double maxdif; // the maximum symmetric difference 
  MTX copyM;  

  // always init copyM
  MTX_Init( &copyM );

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  n = M->nrows;
  if( !MTX_isSquare( M ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }
  if( n == 0 )
  {
    MTX_ERROR_MSG( "if( n == 0 )" );
    return FALSE;
  }

  // use the closed form solutions for dimensions smaller than 4x4
  if( n < 4 )
  {
    return MTX_InvertInPlaceClosedForm( M );
  }

  if( M->isReal )
  {    
    // check diagonal for positive definiteness
    for( j = 0; j < n; j++ )
    {
      // checks for diagonal elements at or less than zero
      if( M->data[j][j] <= 0.0 )
      {
        isPositiveDefinite = FALSE;
        break;
      }
    }
    if( !isPositiveDefinite )
    {
      // use inplace robust inversion
      return MTX_InvertInPlaceRobust( M );
    }

    // check symmetric
    for( i = 0; i < n; i++ )
    {
      for( j = i+1; j < n; j++ )
      {
        val = M->data[j][i];
        maxdif = fabs( val )*1e-14;
        // Why 1e-14? it works well for most matrices expected.
        // Worst case is that some positive definite matrices will be inverted
        // using the robust algorithm.

        dtmp = fabs( val - M->data[i][j] );
        if( dtmp > maxdif )
        {
          // Why 1e-14? it works well for most matrices expected.
          // Worst case is that some positive definite matrices will be inverted
          // using the robust algorithm.
          isPositiveDefinite = FALSE;
          break;
        }
      }
      if( !isPositiveDefinite )
        break;
    }
    if( !isPositiveDefinite )
    {
      // use inplace robust inversion
      return MTX_InvertInPlaceRobust( M );
    }

    // make a copy to use in case robust inversion is needed
    if( !MTX_Copy( M, &copyM ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &copyM );
      return FALSE;
    }

    // Perform Choleski decomposition
    for( j = 0; j < n; j++ )
    {
      for( k = 0; k < j; k++ )
      {
        val = M->data[k][j];
        M->data[j][j] -= val * val;
      }

      val = M->data[j][j];
      if( val < 0.0 )
      {
        // use robust inversion on the copy
        if( !MTX_InvertInPlaceRobust( &copyM ) )
        {
          MTX_ERROR_MSG( "MTX_InvertInPlaceRobust returned FALSE." );
          MTX_Free(&copyM);
          return FALSE;
        }

        // switching the pointers rather than copying the result
        ptrptrData = M->data;
        M->data = copyM.data;
        copyM.data = ptrptrData;
        MTX_Free( &copyM );
        return TRUE;
      }

      M->data[j][j] = sqrt( val );

      for( i = j + 1; i < n; i++ )
      {
        for( k = 0; k < j; k++ )
          M->data[j][i] -= M->data[k][i] * M->data[k][j];

        val = M->data[j][j];
        if( fabs(val) < 1.0E-20 )
        {
          // use Robust Inversion
          if( !MTX_InvertInPlaceRobust( &copyM ) )
          {
            MTX_ERROR_MSG( "MTX_InvertInPlaceRobust returned FALSE." );
            MTX_Free(&copyM);
            return FALSE;
          }

          // switching the pointers rather than copying the result
          ptrptrData = M->data;
          M->data = copyM.data;
          copyM.data = ptrptrData;
          MTX_Free( &copyM );
          return TRUE;
        }

        M->data[j][i] /= val;
      }
    }

    // inversion of lower triangular matrix
    for( j = 0; j < n; j++ )
    {
      M->data[j][j] = 1.0 / M->data[j][j];

      for( i = j + 1; i < n; i++ )
      {
        M->data[j][i] *= -M->data[j][j] / M->data[i][i];

        val = M->data[i][i];
        for( k = j + 1; k < i; k++ )
          M->data[j][i] -= M->data[k][i] * M->data[j][k] / val;
      }
    }

    // construction of lower triangular inverse matrix
    for( j = 0; j < n; j++ )
    {
      for( i = j; i < n; i++ )
      {
        M->data[j][i] *= M->data[i][i];

        for( k = i + 1; k < n; k++ )
          M->data[j][i] += M->data[i][k] * M->data[j][k];
      }
    }

    // fill upper diagonal
    for( i = 1; i < n; i++ )
    {
      for( j = 0; j < i; j++ )
        M->data[i][j] = M->data[j][i];
    }

    MTX_Free(&copyM);
  }
  else
  {
    // use inplace robust inversion
    return MTX_InvertInPlaceRobust( M );
  }
  return TRUE;
}



BOOL MTX_Invert( const MTX *src, MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  BOOL isPositiveDefinite = TRUE;
  unsigned n;
  double val;
  double dtmp;
  double maxdif; // the maximum symmetric difference
  
  if( dst == NULL )
  {
    MTX_ERROR_MSG( "NULL input matrix." );
    return FALSE;
  }
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  n = src->nrows;
  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }
  if( n == 0 )
  {
    MTX_ERROR_MSG( "if( n == 0 )" );
    return FALSE;
  }
  
  // use the closed form solutions for dimensions smaller than 4x4
  if( n < 4 )
  {
    return MTX_InvertClosedForm( src, dst );
  }

  // make dst a copy.
  if( !MTX_Copy( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  if( src->isReal )
  {
    // check diagonal for positive definiteness
    for( j = 0; j < n; j++ )
    {
      // checks for diagonal elements close to zero or less than zero
      if( src->data[j][j] <= 0.0 )
      {
        isPositiveDefinite = FALSE;
        break;
      }
    }
    if( !isPositiveDefinite )
    {
      // use inplace robust inversion      
      return MTX_InvertInPlaceRobust( dst );
    }

    // check symmetric
    for( i = 0; i < n; i++ )
    {
      for( j = i+1; j < n; j++ )
      {
        val = src->data[j][i];
        maxdif = fabs( val )*1e-14;
        // Why 1e-14? it works well for most matrices expected.
        // Worst case is that some positive definite matrices will be inverted
        // using the robust algorithm.

        dtmp = fabs( val - src->data[i][j] );
        if( dtmp > maxdif )
        {
          // Why 1e-14? it works well for most matrices expected.
          // Worst case is that some positive definite matrices will be inverted
          // using the robust algorithm.
          isPositiveDefinite = FALSE;
          break;
        }
      }
      if( !isPositiveDefinite )
        break;
    }
    if( !isPositiveDefinite )
    {
      // use inplace robust inversion
      return MTX_InvertInPlaceRobust( dst );
    }

    // Perform Choleski decomposition on dst inplace
    for( j = 0; j < n; j++ )
    {
      for( k = 0; k < j; k++ )
      {
        val = dst->data[k][j];
        dst->data[j][j] -= val*val;
      }

      val = dst->data[j][j];
      if( val < 0.0 )
      {
        // use robust inversion
        if( !MTX_Copy( src, dst ) )
        {
          MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
          return FALSE;
        }
        if( !MTX_InvertInPlaceRobust( dst ) )
        {
          MTX_ERROR_MSG( "MTX_InvertInPlaceRobust returned FALSE." );          
          return FALSE;
        }
        return TRUE;
      }

      dst->data[j][j] = sqrt( val );

      for( i = j + 1; i < n; i++ )
      {
        for( k = 0; k < j; k++ )
          dst->data[j][i] -= dst->data[k][i] * dst->data[k][j];

        val = dst->data[j][j];
        if( fabs(val) < 1.0E-20 )
        {
          // use Robust Inversion
          if( !MTX_Copy( src, dst ) )
          {
            MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
            return FALSE;
          }
          if( !MTX_InvertInPlaceRobust( dst ) )
          {
            MTX_ERROR_MSG( "MTX_InvertInPlaceRobust returned FALSE." );            
            return FALSE;
          }
          return TRUE;
        }

        dst->data[j][i] /= val;
      }
    }

    // inversion of lower triangular matrix
    for( j = 0; j < n; j++ )
    {
      dst->data[j][j] = 1.0 / dst->data[j][j];

      for( i = j + 1; i < n; i++ )
      {
        dst->data[j][i] *= -dst->data[j][j] / dst->data[i][i];

        val = dst->data[i][i];
        for( k = j + 1; k < i; k++ )
          dst->data[j][i] -= dst->data[k][i] * dst->data[j][k] / val;
      }
    }

    // construction of lower triangular inverse matrix
    for( j = 0; j < n; j++ )
    {
      for( i = j; i < n; i++ )
      {
        dst->data[j][i] *= dst->data[i][i];

        for( k = i + 1; k < n; k++ )
          dst->data[j][i] += dst->data[i][k] * dst->data[j][k];
      }
    }

    // fill upper diagonal
    for( i = 1; i < n; i++ )
    {
      for( j = 0; j < i; j++ )
        dst->data[i][j] = dst->data[j][i];
    }    
  }
  else
  {
    // use inplace robust inversion
    return MTX_InvertInPlaceRobust( dst );
  }
  return TRUE;
}


BOOL MTX_InvertInPlaceRobust( MTX *M )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  const unsigned n = M->nrows;
  BOOL isFullRank = FALSE;
  unsigned *index = NULL;
  MTX A; // nxn, M is nxn
  MTX ColumnI; // nx1
  MTX X; // nx1

  // make A a copy of this matrix
  // A will be factorized
  MTX_Init( &A);

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_isSquare( M ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }
  if( n == 0 )
  {
    MTX_ERROR_MSG( "if( n == 0 )" );
    return FALSE;
  }

  // use the closed form solutions for dimensions smaller than 4x4
  if( n < 4 )
  {
    return MTX_InvertInPlaceClosedForm( M );
  }


  if( !MTX_Copy( M, &A) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &A );
    return FALSE;
  }
  
  index = (unsigned*)calloc( n, sizeof(unsigned) );
  if( !index )
  {
    MTX_ERROR_MSG( "calloc returned NULL." );
    MTX_Free( &A );
    return FALSE;
  }

  // need to get the index and Factorized A
  if( !MTX_static_Factorize( &isFullRank, n, index, &A ) )
  {
    MTX_ERROR_MSG( "MTX_static_Factorize returned FALSE." );
    MTX_Free( &A );
    if( index )
      free(index);
    return FALSE;
  }
  if( !isFullRank )
  {
    MTX_Free( &A );
    if( index )
      free(index);
    return FALSE;
  }

  MTX_Init( &ColumnI );
  MTX_Init( &X );

  // X is used to just point to data already
  // allocated in M so no need to call MTX_Free.
  X.nrows = n;
  X.ncols = 1;

  if( M->isReal )
  {
    X.isReal = TRUE;
    if( !MTX_Calloc( &ColumnI, n, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
      MTX_Free( &A );
      MTX_Free( &ColumnI );
      if(index)
        free(index);
      return FALSE;
    }

    for( i = 0; i < n; i++ )
    {
      // make X point to a data column in M
      X.data = &(M->data[i]);

      if( i != 0 )
        ColumnI.data[0][i-1] = 0.0;
      ColumnI.data[0][i] = 1.0;

      if( !MTX_static_SolveByGaussianElimination( &ColumnI, &X, &A, index ) )
      {
        MTX_ERROR_MSG( "MTX_static_SolveByGaussianElimination returned FALSE." );
        MTX_Free( &A );
        MTX_Free( &ColumnI );
        if(index)
          free(index);
        return FALSE;
      }
    }
  }
  else
  {
    X.isReal = FALSE;
    if( !MTX_Calloc( &ColumnI, n, 1, FALSE ) )
    {
      MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
      MTX_Free( &A );
      MTX_Free( &ColumnI );
      if(index)
        free(index);
      return FALSE;
    }

    for( i = 0; i < n; i++ )
    {
      // make X point to a data column in M
      X.cplx = &(M->cplx[i]);

      if( i != 0 )
      {
        ColumnI.cplx[0][i-1].re = 0.0;
        ColumnI.cplx[0][i-1].im = 0.0;
      }
      ColumnI.cplx[0][i].re = 1.0;
      ColumnI.cplx[0][i].im = 0.0;

      if( !MTX_static_SolveByGaussianElimination( &ColumnI, &X, &A, index ) )
      {
        MTX_ERROR_MSG( "MTX_static_SolveByGaussianElimination returned FALSE." );
        MTX_Free( &A );
        MTX_Free( &ColumnI );
        if(index)
          free(index);
        return FALSE;
      }
    }
  }

  if(index)
    free(index);

  MTX_Free( &A );
  MTX_Free( &ColumnI );

  return TRUE;
}

BOOL MTX_static_Factorize( BOOL *isFullRank, const unsigned n, unsigned* index, MTX *A )
{
  // [1] Chaney, Ward & David Kincaid, "Numerical Mathematics and Computing, 3rd Edition",
  // Cole Publishing Co., 1994, Belmont, CA, p.237)
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned tempi = 0;
  double *scale;

  double r = 0.0;
  double rmax = 0.0;
  double smax_re = 0.0;
  double smax_im = 0.0;
  double xmult = 0.0;
  double tempd = 0.0;

  stComplex cplx_xmult = {0.0,0.0};
  stComplex cplx = {0.0,0.0};

  BOOL isFullRankTmp = FALSE;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_isSquare( A ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  if( n == 0 )
  {
    MTX_ERROR_MSG( "if( n == 0 )" );
    return FALSE;
  }

  scale = (double*)malloc( sizeof(double)*n );
  if( !scale )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

  // The first loop determines the maximum element
  // in each column and the index of its first occurance
  for( i = 0; i < n; i++ )
  {
    if( !MTX_MaxAbsRowIndex( A, i, &(scale[i]), &k ) )
    {
      MTX_ERROR_MSG( "MTX_MaxAbsRowIndex returned FALSE." );
      if( scale )
        free(scale);
      return FALSE;
    }
    index[i] = i;
  }

  if( A->isReal )
  {
    // Second Loop
    // perform gaussian elimination to form the Lower and Upper matrices
    for( k = 0; k < n - 1; k++ )
    {
      // select the pivot row, j, based on rmax, the first occurance of the largest ratio
      rmax = 0;
      j = 0;
      for( i = k; i < n; i++ )
      {
        r = fabs( A->data[k][index[i]] ) / scale[index[i]];
        if( r > rmax )
        {
          rmax = r;
          j = i;
        }
      }

      tempi = index[j];
      index[j] = index[k];
      index[k] = tempi;

      for( i = k + 1; i < n; i++ )
      {
        xmult = A->data[k][index[i]] / A->data[k][index[k]];
        A->data[k][index[i]] = xmult;

        isFullRankTmp = FALSE;
        for( j = k + 1; j < n; j++ )
        {
          A->data[j][index[i]] = A->data[j][index[i]] - xmult*(A->data[j][index[k]]);

          // if the upper matrix every has all zeros in one row, no solution is available
          if( A->data[j][index[i]] != 0.0 )
            isFullRankTmp = TRUE;
        }
        if( !isFullRankTmp )
        {
          *isFullRank = FALSE;
          if( scale )
            free(scale);
          return TRUE;
        }
      }
    }
  }
  else
  {
    // Second Loop
    // perform gaussian elimination to form the Lower and Upper matrices
    for( k = 0; k < n - 1; k++ )
    {
      // select the pivot row, j, based on rmax, the first occurance of the largest ratio
      rmax = 0;
      j = 0;
      for( i = k; i < n; i++ )
      {
        r = sqrt( A->cplx[k][index[i]].re*A->cplx[k][index[i]].re + A->cplx[k][index[i]].im*A->cplx[k][index[i]].im );
        r = r / scale[index[i]];
        if( r > rmax )
        {
          rmax = r;
          j = i;
        }
      }

      tempi = index[j];
      index[j] = index[k];
      index[k] = tempi;

      for( i = k + 1; i < n; i++ )
      {
        MTX_static_quick_complex_divide(
          &(A->cplx[k][index[i]].re),
          &(A->cplx[k][index[i]].im),
          &(A->cplx[k][index[k]].re),
          &(A->cplx[k][index[k]].im),
          &cplx_xmult.re,
          &cplx_xmult.im );

        A->cplx[k][index[i]] = cplx_xmult;

        isFullRankTmp = FALSE;
        for( j = k + 1; j < n; j++ )
        {
          MTX_static_quick_complex_mult_ab(
            &cplx_xmult.re,
            &cplx_xmult.im,
            &(A->cplx[j][index[k]].re),
            &(A->cplx[j][index[k]].im),
            &cplx.re,
            &cplx.im );

          A->cplx[j][index[i]].re = A->cplx[j][index[i]].re - cplx.re;
          A->cplx[j][index[i]].im = A->cplx[j][index[i]].im - cplx.im;

          // if the upper matrix every has all zeros in one row, no solution is available
          if( A->cplx[j][index[i]].re != 0.0 || A->cplx[j][index[i]].im != 0.0 )
            isFullRankTmp = TRUE;
        }
        if( !isFullRankTmp )
        {
          *isFullRank = FALSE;
          if( scale )
            free(scale);
          return TRUE;
        }
      }
    }
  }

  if( scale )
    free(scale);

  *isFullRank = TRUE;
  return TRUE;
}


BOOL MTX_static_SolveByGaussianElimination(
  const MTX *b,
  MTX *X,
  const MTX *A, // factorized A
  unsigned *index )
{
  int i = 0; // signed value is needed for the reverse for loop
  int j = 0;
  int k = 0;
  double sum = 0.0;
  const int n = A->nrows;
  MTX B;
  stComplex cplx = {0.0,0.0};
  stComplex sum_cplx = {0.0,0.0};

  // make B a copy of b
  MTX_Init( &B);

  if( !MTX_Copy( b, &B ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &B );
    return FALSE;
  }

  if( A->isReal )
  {
    for( k = 0; k < n - 1; k++ )
    {
      for( i = k + 1; i < n; i++ )
      {
        B.data[0][index[i]] = B.data[0][index[i]] - A->data[k][index[i]]*B.data[0][index[k]];
      }
    }
    X->data[0][n-1] = B.data[0][index[n-1]] / A->data[n-1][index[n-1]];

    for( i = n - 2; i >= 0; i-- )
    {
      sum = B.data[0][index[i]];
      for( j = i + 1; j < n; j++ )
      {
        sum = sum - A->data[j][index[i]]*X->data[0][j];
      }
      X->data[0][i] = sum / A->data[i][index[i]];
    }
  }
  else
  {
    for( k = 0; k < n - 1; k++ )
    {
      for( i = k + 1; i < n; i++ )
      {
        MTX_static_quick_complex_mult_ab(
          &(A->cplx[k][index[i]].re),
          &(A->cplx[k][index[i]].im),
          &(B.cplx[0][index[k]].re),
          &(B.cplx[0][index[k]].im),
          &cplx.re,
          &cplx.im );

        B.cplx[0][index[i]].re = B.cplx[0][index[i]].re - cplx.re;
        B.cplx[0][index[i]].im = B.cplx[0][index[i]].im - cplx.im;
      }
    }

    MTX_static_quick_complex_divide(
      &(B.cplx[0][index[n-1]].re),
      &(B.cplx[0][index[n-1]].im),
      &(A->cplx[n-1][index[n-1]].re),
      &(A->cplx[n-1][index[n-1]].im),
      &(X->cplx[0][n-1].re),
      &(X->cplx[0][n-1].im) );

    for( i = n - 2; i >= 0; i-- )
    {
      sum_cplx.re = B.cplx[0][index[i]].re;
      sum_cplx.im = B.cplx[0][index[i]].im;

      for( j = i + 1; j < n; j++ )
      {
        MTX_static_quick_complex_mult_ab(
          &(A->cplx[j][index[i]].re),
          &(A->cplx[j][index[i]].im),
          &(X->cplx[0][j].re),
          &(X->cplx[0][j].im),
          &cplx.re,
          &cplx.im );

        sum_cplx.re = sum_cplx.re - cplx.re;
        sum_cplx.im = sum_cplx.im - cplx.im;
      }

      MTX_static_quick_complex_divide(
        &sum_cplx.re,
        &sum_cplx.im,
        &(A->cplx[i][index[i]].re),
        &(A->cplx[i][index[i]].im),
        &(X->cplx[0][i].re),
        &(X->cplx[0][i].im) );
    }
  }

  MTX_Free( &B );
  return TRUE;
}

BOOL MTX_ColumnMovAvg( const MTX *src, const unsigned col, const unsigned nlead, const unsigned nlag, MTX *dst )
{
  unsigned i = 0;
  unsigned k = 0;
  unsigned count = 0;

  double sum = 0.0;
  double sum_im = 0.0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( col >= src->ncols )
  {
    MTX_ERROR_MSG( "if( col >= src->ncols )" );
    return FALSE;
  }

  if( nlead+nlag > src->nrows )
  {
    MTX_ERROR_MSG( "if( nlead+nlag > src->nrows )" );
    return FALSE;
  }

  // resize or make real/complex as needed
  if( dst->isReal != src->isReal )
  {
    if( !MTX_Resize( dst, src->nrows, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  else if( dst->nrows != src->nrows || dst->ncols != 1 )
  {
    if( !MTX_Resize( dst, src->nrows, 1, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( src->isReal )
  {
    for( i = 0; i < src->nrows; i++ )
    {
      sum = src->data[col][i];
      count = 1;
      for( k = 1; k <= nlead; k++ )
      {
        if( i >= k )
        {
          sum += src->data[col][i-k];
          count++;
        }
      }
      for( k = 1; k <= nlag; k++ )
      {
        if( i < src->nrows-k )
        {
          sum += src->data[col][i+k];
          count++;
        }
      }
      dst->data[0][i] = sum / ((double)(count));
    }
  }
  else
  {
    for( i = 0; i < src->nrows; i++ )
    {
      sum = src->cplx[col][i].re;
      sum_im = src->cplx[col][i].im;
      count = 1;
      for( k = 1; k <= nlead; k++ )
      {
        if( i >= k )
        {
          sum += src->cplx[col][i-k].re;
          sum_im += src->cplx[col][i-k].im;
          count++;
        }
      }
      for( k = 1; k <= nlag; k++ )
      {
        if( i < src->nrows-k )
        {
          sum += src->cplx[col][i+k].re;
          sum_im += src->cplx[col][i+k].im;
          count++;
        }
      }
      dst->cplx[0][i].re = sum / ((double)(count));
      dst->cplx[0][i].im = sum / ((double)(count));
    }
  }

  return TRUE;
}


BOOL MTX_MovAvg( const MTX *src, const unsigned nlead, const unsigned nlag, MTX *dst )
{
  unsigned j = 0;

  MTX column;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  MTX_Init( &column );

  if( src->isReal != dst->isReal )
  {
    if( !MTX_Redim( dst, src->nrows, src->ncols ) )
    {
      MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
      return FALSE;
    }
  }
  else if( !MTX_isSameSize( src, dst ) )
  {
    if( !MTX_Redim( dst, src->nrows, src->ncols ) )
    {
      MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
      return FALSE;
    }
  }

  // Column is a fixed container that will point to dst data.
  column.isReal = dst->isReal;
  column.nrows = dst->nrows;
  column.ncols = 1;

  for( j = 0; j < src->ncols; j++ )
  {
    if( dst->isReal )
      column.data = &(dst->data[j]);
    else
      column.data = &(dst->data[j]);

    if( !MTX_ColumnMovAvg( src, j, nlead, nlag, &column ) ) // this is acting on dst indirectly
    {
      MTX_ERROR_MSG( "MTX_ColumnMovAvg returned FALSE." );
      return FALSE;
    }
  }

  // Note: No need to MTX_Free column.

  return TRUE;
}





BOOL MTX_ATAInverse( const MTX *A, MTX *InvATA )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  MTX colA;

  if( MTX_isNull( A ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // resize InvATA if needed
  if( MTX_isNull( InvATA ) )
  {
    if( !MTX_Resize( InvATA, A->ncols, A->ncols, A->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  else if( InvATA->nrows != A->ncols || InvATA->ncols != A->ncols )
  {
    if( !MTX_Resize( InvATA, A->ncols, A->ncols, A->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  // make a copy and a vector
  MTX_Init( &colA );

  for( i = 0; i < A->ncols; i++ )
  {
    for( j = 0; j < A->ncols; j++ )
    {
      if( i > j )
      {
        InvATA->data[j][i] = InvATA->data[i][j];
      }
      else
      {
        InvATA->data[j][i] = 0;
        for( k = 0; k < A->nrows; k++ )
        {
          InvATA->data[j][i] += A->data[i][k] * A->data[j][k];
        }
      }
    }
  }

  return MTX_InvertInPlace( InvATA );
}


BOOL MTX_LowerTriangularInverseInplace( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  // inversion of lower triangular matrix
  for( j = 0; j < src->nrows; j++ )
  {
    if( src->data[j][j] == 0.0 )
    {
      MTX_ERROR_MSG( "if( src->data[j][j] == 0.0 )" );
      return FALSE;
    }

    src->data[j][j] = 1.0 / src->data[j][j];

    for( i = j + 1; i < src->nrows; i++ )
    {
      if( src->data[i][i] == 0.0 )
      {
        MTX_ERROR_MSG( "if( src->data[j][j] == 0.0 )" );
        return FALSE;
      }

      src->data[j][i] *= -src->data[j][j] / src->data[i][i];

      for( k = j + 1; k < i; k++ )
        src->data[j][i] -= src->data[k][i] * src->data[j][k] / src->data[i][i];
    }
  }
  return TRUE;
}


void MTX_static_quick_complex_mult_ab(
                                      const double* a_re,
                                      const double* a_im,
                                      const double* b_re,
                                      const double* b_im,
                                      double *re,
                                      double *im )
{
  *re = (*a_re)*(*b_re) - (*a_im)*(*b_im);
  *im = (*a_re)*(*b_im) + (*a_im)*(*b_re);
}


void MTX_static_quick_complex_mult_abc(
                                       const double* a_re,
                                       const double* a_im,
                                       const double* b_re,
                                       const double* b_im,
                                       const double* c_re,
                                       const double* c_im,
                                       double *re,
                                       double *im )
{
  double r,i; // temps

  r = (*a_re)*(*b_re) - (*a_im)*(*b_im);
  i = (*a_re)*(*b_im) + (*a_im)*(*b_re);

  *re = r*(*c_re) - i*(*c_im);
  *im = r*(*c_im) + i*(*c_re);
}

void MTX_static_quick_complex_divide(
                                     const double* a_re, //!< The real part of a (input).
                                     const double* a_im, //!< The imag part of a (input).
                                     const double* b_re, //!< The real part of b (input).
                                     const double* b_im, //!< The imag part of b (input).
                                     double *re, //!< The real part of the result.
                                     double *im )//!< The imag part of the result.
{
  double dtmp;
  dtmp = (*b_re)*(*b_re) + (*b_im)*(*b_im);
  *re = ((*a_re) * (*b_re) + (*a_im) * (*b_im) ) / dtmp;
  *im = ((*a_im) * (*b_re) - (*a_re) * (*b_im) ) / dtmp;
}


// static
void MTX_static_Det_cleanup( unsigned *index, double *scale, MTX *U, MTX *magMtx )
{
  if( index )
  {
    free( index );
    index = NULL;
  }
  if( scale )
  {
    free( scale );
    scale = NULL;
  }
  if( U != NULL )
  {
    MTX_Free( U );
  }
  if( magMtx != NULL )
  {
    MTX_Free( magMtx );
  }
}

// Computes the determinant of the square matrix M.
// det(M) = det(LU) = det(L)*det(U)
BOOL MTX_Det( const MTX *M, double *re, double *im )
{
  unsigned n; // the number of rows in M
  double tmpre;
  double tmpim;
  int s; // a sign value 1 or -1.

  unsigned *index = NULL;
  double *scale = NULL;

  MTX U; // An Upper triangular matrix (may be permutated).
  MTX magMtx; // A matrix with magnitude of M.

  MTX_Init( &U );
  MTX_Init( &magMtx );

  if( MTX_isNull( M ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_isSquare( M ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  n = M->nrows;

  *re = 0.0;
  *im = 0.0;

  // special cases
  // use the direct solution for rank up to 3
  if( n == 1 )
  {
    if( M->isReal )
    {
      *re = M->data[0][0];
    }
    else
    {
      *re = M->cplx[0][0].re;
      *im = M->cplx[0][0].im;
    }
    return TRUE;
  }
  else if( n == 2 )
  {
    if( M->isReal )
    {
      *re = M->data[0][0]*M->data[1][1] - M->data[0][1]*M->data[1][0];
    }
    else
    {
      *re = M->cplx[0][0].re*M->cplx[1][1].re - M->cplx[0][0].im*M->cplx[1][1].im;
      *im = M->cplx[0][0].re*M->cplx[1][1].im + M->cplx[0][0].im*M->cplx[1][1].re;

      *re -= M->cplx[0][1].re*M->cplx[1][0].re - M->cplx[0][1].im*M->cplx[1][0].im;
      *im -= M->cplx[0][1].re*M->cplx[1][0].im + M->cplx[0][1].im*M->cplx[1][0].re;
    }
    return TRUE;
  }

  
  // special cases
  // use the direct solution for rank up to 3
  if( n == 3)
  {
    // det = a11*a22*a33 + a13*a21*a32 + a12*a23*a31 - a13*a22*a31 - a11*a23*a32 - a12*a21*a33 // row-column
    // det = a00*a11*a22 + a02*a10*a21 + a01*a12*a20 - a02*a11*a20 - a00*a12*a21 - a01*a10*a22 // row-column, zero-based
    // det = a00*a11*a22 + a20*a01*a12 + a10*a21*a02 - a20*a11*a02 - a00*a21*a12 - a10*a01*a22 // column-row

    if( M->isReal )
    {
      *re = M->data[0][0]*M->data[1][1]*M->data[2][2] +
        M->data[2][0]*M->data[0][1]*M->data[1][2] +
        M->data[1][0]*M->data[2][1]*M->data[0][2] -
        M->data[2][0]*M->data[1][1]*M->data[0][2] -
        M->data[0][0]*M->data[2][1]*M->data[1][2] -
        M->data[1][0]*M->data[0][1]*M->data[2][2];
    }
    else
    {
      MTX_static_quick_complex_mult_abc(
        &(M->cplx[0][0].re), &(M->cplx[0][0].im),
        &(M->cplx[1][1].re), &(M->cplx[1][1].im),
        &(M->cplx[2][2].re), &(M->cplx[2][2].im),
        &tmpre, &tmpim );
      *re = tmpre;
      *im = tmpim;

      MTX_static_quick_complex_mult_abc(
        &(M->cplx[2][0].re), &(M->cplx[2][0].im),
        &(M->cplx[0][1].re), &(M->cplx[0][1].im),
        &(M->cplx[1][2].re), &(M->cplx[1][2].im),
        &tmpre, &tmpim );
      *re += tmpre;
      *im += tmpim;

      MTX_static_quick_complex_mult_abc(
        &(M->cplx[1][0].re), &(M->cplx[1][0].im),
        &(M->cplx[2][1].re), &(M->cplx[2][1].im),
        &(M->cplx[0][2].re), &(M->cplx[0][2].im),
        &tmpre, &tmpim );
      *re += tmpre;
      *im += tmpim;

      MTX_static_quick_complex_mult_abc(
        &(M->cplx[2][0].re), &(M->cplx[2][0].im),
        &(M->cplx[1][1].re), &(M->cplx[1][1].im),
        &(M->cplx[0][2].re), &(M->cplx[0][2].im),
        &tmpre, &tmpim );
      *re -= tmpre;
      *im -= tmpim;

      MTX_static_quick_complex_mult_abc(
        &(M->cplx[0][0].re), &(M->cplx[0][0].im),
        &(M->cplx[2][1].re), &(M->cplx[2][1].im),
        &(M->cplx[1][2].re), &(M->cplx[1][2].im),
        &tmpre, &tmpim );
      *re -= tmpre;
      *im -= tmpim;

      MTX_static_quick_complex_mult_abc(
        &(M->cplx[1][0].re), &(M->cplx[1][0].im),
        &(M->cplx[0][1].re), &(M->cplx[0][1].im),
        &(M->cplx[2][2].re), &(M->cplx[2][2].im),
        &tmpre, &tmpim );
      *re -= tmpre;
      *im -= tmpim;
    }
    return TRUE;
  }
  else
  {
    stComplex det;
    stComplex xmult;

    unsigned i;
    unsigned j;
    unsigned k;
    unsigned tempi;    

    double r = 0.0,
      rmax = 0.0,
      smax = 0.0,
      tempd = 0.0;

    BOOL isFullRankTmp = FALSE;
    BOOL isFullRank = TRUE;

    // factorization by naive gaussian elimination (to perform LUFactorization)
    // The product of the columns of the U matrix is the determinant.
    // Reference
    // Chaney, Ward & David Kincaid, "Numerical Mathematics and Computing, 3rd Edition", Cole
    // Publishing Co., 1994, Belmont, CA

    // operate on a copy
    if( !MTX_Copy( M, &U ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_static_Det_cleanup( index, scale, &U, &magMtx );
      return FALSE;
    }

    index = (unsigned*)malloc( sizeof(unsigned)*n );
    if( !index )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      MTX_static_Det_cleanup( index, scale, &U, &magMtx );
      return FALSE;
    }

    scale = (double*)malloc( sizeof(double)*n );
    if( !index )
    {
      MTX_ERROR_MSG( "malloc returned NULL." );
      MTX_static_Det_cleanup( index, scale, &U, &magMtx );
      return FALSE;
    }

    // deal with the real and complex cases separately
    if( M->isReal )
    {
      // First Loop
      // initial form of the indexArray is determined
      // the scale factor array scaleArray is also determined
      // The indexArray will become [0 1 2.... n-1]
      // The scale factor array is the first absolue largest element of each row.
      for( i = 0; i < n; i++ )
      {
        index[i] = i;
        smax = 0;
        for( j = 0; j < n; j++ )
        {
          tempd = fabs( U.data[j][i] );
          if( smax < tempd )
          {
            smax = tempd;
          }
        }
        scale[i] = smax;
        if( scale[i] == 0.0 )
        {
          *re = 0.0;
          *im = 0.0;

          MTX_static_Det_cleanup( index, scale, &U, &magMtx );

          return TRUE;
        }
      }

      // Second Loop
      // perform gaussian elimination to form the Lower and Upper matrices
      for( k = 0; k < n - 1; k++ )
      {
        // select the pivot row, j, based on rmax, the last occurance of the largest ratio (last instead of first to match Matlab results)
        j = 0;
        rmax = 0;
        for( i = k; i < n; i++ )
        {
          r = fabs( U.data[k][index[i]] ) / scale[index[i]];

          if( r >= rmax )
          {
            rmax = r;
            j = i;
          }
        }

        tempi = index[j];
        index[j] = index[k];
        index[k] = tempi;

        for( i = k + 1; i < n; i++ )
        {
          xmult.re = U.data[k][index[i]] / U.data[k][index[k]];

          isFullRankTmp = FALSE;
          for( j = k + 1; j < n; j++ )
          {
            U.data[j][index[i]] = U.data[j][index[i]] - xmult.re*U.data[j][index[k]];

            // if the upper matrix every has all zeros in one row, no solution is available
            if( U.data[j][index[i]] != 0 )
              isFullRankTmp = TRUE;
          }
          if( !isFullRankTmp )
          {
            *re = 0.0;
            *im = 0.0;

            MTX_static_Det_cleanup( index, scale, &U, &magMtx );

            return TRUE;
          }
        }
      }

      //MTX_PrintAutoWidth( &U, "U.txt", 10, FALSE ); 

      // compute the product of the psychologically 'diagonal' terms
      det.re = 1;
      det.im = 0;
      for( j = 0; j < n; j++ )
      {
        det.re *= U.data[j][index[j]];        
      }

      /*
      since permutations change the sign of the determinant, calculate det(P)
      e.g. 
      P = [1 0 0 0 0;
           0 0 0 1 0;
           0 0 0 0 1;
           0 0 1 0 0;
           0 1 0 0 0];
      The following code uses the +/- rule recursively and to determine if 
      there is a sign change. 
      */
      s = 1; // det(P) is either 1 or -1
      for( j = 0; j < n-1; j++ )
      {
        if( index[j]%2!=0 )
          s *= -1;
        for( i=j+1; i < n; i++ )
        {
          if( index[i] > j )
            index[i]--;
        }
      }
      det.re *= s;

      *re = det.re;
      // *im is already 0
    }
    else
    {
      // Compute the magnitude of M.
      if( !MTX_Magnitude( M, &magMtx ) )
      {
        MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
        MTX_static_Det_cleanup( index, scale, &U, &magMtx );
        return FALSE;
      }

      // First Loop
      // initial form of the indexArray is determined
      // the scale factor array scaleArray is also determined
      // The indexArray will become [0 1 2.... n-1]
      // The scale factor array is the first absolute largest element of each row.
      for( i = 0; i < n; i++ )
      {
        index[i] = i;
        smax = 0;
        for( j = 0; j < n; j++ )
        {
          if( smax < magMtx.data[j][i] )
          {
            smax = magMtx.data[j][i];
          }
        }
        scale[i] = smax;
        if( scale[i] == 0.0 )
        {
          *re = 0.0;
          *im = 0.0;

          MTX_static_Det_cleanup( index, scale, &U, &magMtx );

          return TRUE;
        }
      }

      // Second Loop
      // perform gaussian elimination to form the Lower and Upper matrices
      for( k = 0; k < n - 1; k++ )
      {
        // select the pivot row, j, based on rmax, the last occurance of the largest ratio (last instead of first to match Matlab results)
        j = 0;
        rmax = 0;
        for( i = k; i < n; i++ )
        {
          r = magMtx.data[k][index[i]] / scale[index[i]];

          if( r >= rmax )
          {
            rmax = r;
            j = i;
          }
        }

        tempi = index[j];
        index[j] = index[k];
        index[k] = tempi;

        for( i = k + 1; i < n; i++ )
        {
          // compute: xmult = U.cplx[k][index[i]] / U.cplx[k][index[k]];
          MTX_static_quick_complex_divide(
            &(U.cplx[k][index[i]].re),
            &(U.cplx[k][index[i]].im),
            &(U.cplx[k][index[k]].re),
            &(U.cplx[k][index[k]].im),
            &(xmult.re),
            &(xmult.im)
            );

          isFullRankTmp = FALSE;
          for( j = k + 1; j < n; j++ )
          {
            // compute: xmult*U.cplx[j][index[k]]
            tmpre = xmult.re * U.cplx[j][index[k]].re - xmult.im * U.cplx[j][index[k]].im;
            tmpim = xmult.re * U.cplx[j][index[k]].im + xmult.im * U.cplx[j][index[k]].re;

            U.cplx[j][index[i]].re = U.cplx[j][index[i]].re - tmpre;
            U.cplx[j][index[i]].im = U.cplx[j][index[i]].im - tmpim;

            // if the upper matrix every has all zeros in one row, no solution is available
            if( U.cplx[j][index[i]].re != 0 )
              isFullRankTmp = TRUE;
            if( U.cplx[j][index[i]].im != 0 )
              isFullRankTmp = TRUE;
          }
          if( !isFullRankTmp )
          {
            *re = 0.0;
            *im = 0.0;

            MTX_static_Det_cleanup( index, scale, &U, &magMtx );
            return TRUE;
          }
        }
      }

      // compute the product of the psychologically 'diagonal' terms
      j = 0;
      det.re = U.cplx[j][index[j]].re;
      det.im = U.cplx[j][index[j]].im;
      for( j = 1; j < n; j++ )
      {
        tmpre = det.re * U.cplx[j][index[j]].re - det.im * U.cplx[j][index[j]].im;
        tmpim = det.re * U.cplx[j][index[j]].im + det.im * U.cplx[j][index[j]].re;

        det.re = tmpre;
        det.im = tmpim;        
      }

      /*
      since permutations change the sign of the determinant, calculate det(P)
      e.g. 
      P = [1 0 0 0 0;
           0 0 0 1 0;
           0 0 0 0 1;
           0 0 1 0 0;
           0 1 0 0 0];
      The following code uses the +/- rule recursively and to determine if 
      there is a sign change. 
      */
      s = 1; // det(P) is either 1 or -1
      for( j = 0; j < n-1; j++ )
      {
        if( index[j]%2!=0 )
          s *= -1;
        for( i=j+1; i < n; i++ )
        {
          if( index[i] > j )
            index[i]--;
        }
      }
      det.re *= s;
      det.im *= s;

      // the final result (output)
      *re = det.re;
      *im = det.im;
    }

    MTX_static_Det_cleanup( index, scale, &U, &magMtx );

    isFullRank = TRUE;
  }

  return TRUE;
}

BOOL MTX_LUFactorization( const MTX *src, BOOL *IsFullRank, MTX *P, MTX *L, MTX *U )
{
  // factorization by naive gaussian elimination
  // Reference
  // Chaney, Ward & David Kincaid, "Numerical Mathematics and Computing, 3rd Edition", Cole
  // Publishing Co., 1994, Belmont, CA
  unsigned n;
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned tempi = 0;
  unsigned *index = NULL;
  double *scale = NULL;
  double r = 0.0,
    rmax = 0.0,
    smax = 0.0,
    tempd = 0.0;

  stComplex xmult; // a real/complex row multiplier value.

  BOOL isFullRankTmp = FALSE;


  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "MTX_isSquare returned FALSE." );
    return FALSE;
  }

  n = src->nrows;

  // resize appropriately
  if( P->nrows != n || P->ncols != n || !P->isReal )
  {
    if( !MTX_Resize( P, n, n, TRUE ) ) // always real
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  if( L->nrows != n || L->ncols != n || (L->isReal != src->isReal) )
  {
    if( !MTX_Resize( L, n, n, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }
  if( U->nrows != n || U->ncols != n || (U->isReal != src->isReal) )
  {
    if( !MTX_Resize( U, n, n, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
      return FALSE;
    }
  }

  if( !MTX_Zero( P ) )
  {
    MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
    return FALSE;
  }
  if( !MTX_Zero( L ) )
  {
    MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
    return FALSE;
  }

  if( !MTX_Copy( src, U ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  index = (unsigned*)malloc( sizeof(unsigned)*n );
  if( !index )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    return FALSE;
  }

  scale = (double*)malloc( sizeof(double)*n );
  if( !scale )
  {
    MTX_ERROR_MSG( "malloc returned NULL." );
    if( index )
      free( index );
    return FALSE;
  }

  if( src->isReal )
  {
    // First Loop
    // initial form of the indexArray is determined
    // the scale factor array scaleArray is also determined
    // The indexArray will become [0 1 2.... n-1]
    // The scale factor array is the first absolue largest element of each row.
    for( i = 0; i < n; i++ )
    {
      index[i] = i;
      smax = 0;
      for( j = 0; j < n; j++ )
      {
        tempd = fabs( U->data[j][i] );
        if( smax < tempd )
        {
          smax = tempd;
        }
      }
      scale[i] = smax;
      if( scale[i] == 0.0 )
      {
        *IsFullRank = FALSE;
        if(index)
          free(index);
        if(scale)
          free(scale);
        if( !MTX_Zero(L) )
        {
          MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
          return FALSE;
        }
        if( !MTX_Zero(U) )
        {
          MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
          return FALSE;
        }

        return TRUE;
      }
    }

    // Second Loop
    // perform gaussian elimination to form the Lower and Upper matrices
    for( k = 0; k < n - 1; k++ )
    {
      // select the pivot row, j, based on rmax, the last occurance of the largest ratio (last instead of first to match Matlab results)
      j = 0;
      rmax = 0;
      for( i = k; i < n; i++ )
      {
        r = fabs( U->data[k][index[i]] ) / scale[index[i]];

        if( r >= rmax )
        {
          rmax = r;
          j = i;
        }
      }

      tempi = index[j];
      index[j] = index[k];
      index[k] = tempi;

      for( i = k + 1; i < n; i++ )
      {
        xmult.re = U->data[k][index[i]] / U->data[k][index[k]];
        L->data[k][index[i]] = xmult.re;

        isFullRankTmp = FALSE;
        for( j = k + 1; j < n; j++ )
        {
          U->data[j][index[i]] = U->data[j][index[i]] - xmult.re*U->data[j][index[k]];

          // if the upper matrix every has all zeros in one row, no solution is available
          if( U->data[j][index[i]] != 0 )
            isFullRankTmp = TRUE;
        }
        if( !isFullRankTmp )
        {
          *IsFullRank = FALSE;
          if(index)
            free(index);
          if(scale)
            free(scale);
          if( !MTX_Zero(L) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }
          if( !MTX_Zero(U) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }

          return TRUE;
        }
        U->data[k][index[i]] = 0.0;
      }
    }
  }
  else // deal with the complex case
  {
    double tmpre;
    double tmpim;
    MTX magMtx; // A matrix with magnitude of the src matrix.
    MTX_Init( &magMtx );

    // Compute the magnitude of the src matrix.
    if( !MTX_Magnitude( src, &magMtx ) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free( &magMtx );
      if(index)
        free(index);
      if(scale)
        free(scale);
      return FALSE;
    }

    // First Loop
    // initial form of the indexArray is determined
    // the scale factor array scaleArray is also determined
    // The indexArray will become [0 1 2.... n-1]
    // The scale factor array is the first absolute largest element of each row.
    for( i = 0; i < n; i++ )
    {
      index[i] = i;
      smax = 0;
      for( j = 0; j < n; j++ )
      {
        if( smax < magMtx.data[j][i] )
        {
          smax = magMtx.data[j][i];
        }
      }
      scale[i] = smax;
      if( scale[i] == 0.0 )
      {
        *IsFullRank = FALSE;

        if(index)
          free(index);
        if(scale)
          free(scale);
        MTX_Free(&magMtx);

        if( !MTX_Zero(L) )
        {
          MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
          return FALSE;
        }
        if( !MTX_Zero(U) )
        {
          MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
          return FALSE;
        }
        return TRUE;
      }
    }

    // Second Loop
    // perform gaussian elimination to form the Lower and Upper matrices
    for( k = 0; k < n - 1; k++ )
    {
      // select the pivot row, j, based on rmax, the last occurance of the largest ratio
      j = 0;
      rmax = 0;
      for( i = k; i < n; i++ )
      {
        r = magMtx.data[k][index[i]] / scale[index[i]];

        if( r >= rmax )
        {
          rmax = r;
          j = i;
        }
      }

      tempi = index[j];
      index[j] = index[k];
      index[k] = tempi;

      for( i = k + 1; i < n; i++ )
      {
        // compute: xmult = U->cplx[k][index[i]] / U->cplx[k][index[k]];
        MTX_static_quick_complex_divide(
          &(U->cplx[k][index[i]].re),
          &(U->cplx[k][index[i]].im),
          &(U->cplx[k][index[k]].re),
          &(U->cplx[k][index[k]].im),
          &(xmult.re),
          &(xmult.im)
          );
        L->cplx[k][index[i]].re = xmult.re;
        L->cplx[k][index[i]].im = xmult.im;

        isFullRankTmp = FALSE;
        for( j = k + 1; j < n; j++ )
        {
          // compute: xmult*U->cplx[j][index[k]]
          tmpre = xmult.re * U->cplx[j][index[k]].re - xmult.im * U->cplx[j][index[k]].im;
          tmpim = xmult.re * U->cplx[j][index[k]].im + xmult.im * U->cplx[j][index[k]].re;

          U->cplx[j][index[i]].re = U->cplx[j][index[i]].re - tmpre;
          U->cplx[j][index[i]].im = U->cplx[j][index[i]].im - tmpim;

          // if the upper matrix every has all zeros in one row, no solution is available
          if( U->cplx[j][index[i]].re != 0 )
            isFullRankTmp = TRUE;
          if( U->cplx[j][index[i]].im != 0 )
            isFullRankTmp = TRUE;
        }
        if( !isFullRankTmp )
        {
          *IsFullRank = FALSE;

          if(index)
            free(index);
          if(scale)
            free(scale);
          MTX_Free(&magMtx);

          if( !MTX_Zero(L) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }
          if( !MTX_Zero(U) )
          {
            MTX_ERROR_MSG( "MTX_Zero returned FALSE." );
            return FALSE;
          }

          return TRUE;
        }
        U->cplx[k][index[i]].re = 0.0;
        U->cplx[k][index[i]].im = 0.0;
      }
    }

    MTX_Free(&magMtx);
  }

  for( j = 0; j < n; j++ )
  {
    P->data[index[j]][j] = 1.0;

    if( L->isReal )
      L->data[j][index[j]] = 1.0;
    else
      L->cplx[j][index[j]].re = 1.0;
  }

  if(index)
    free(index);

  if(scale)
    free(scale);

  if( !MTX_PreMultiply_Inplace( L, P ) )
  {
    MTX_ERROR_MSG( "MTX_PreMultiply_Inplace returned FALSE." );
    return FALSE;
  }

  if( !MTX_PreMultiply_Inplace( U, P ) )
  {
    MTX_ERROR_MSG( "MTX_PreMultiply_Inplace returned FALSE." );
    return FALSE;
  }

  *IsFullRank = TRUE;

  return TRUE;
}

BOOL MTX_IndexedValues( const MTX *src, const MTX *row_index, const MTX *col_index, MTX *dst )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned row = 0;
  unsigned col = 0;

  if( MTX_isNull( row_index ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( col_index ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( dst->isReal != src->isReal || dst->nrows != row_index->nrows || dst->ncols != col_index->nrows )
  {
    if( !MTX_Malloc( dst, row_index->nrows, col_index->nrows, src->isReal ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < col_index->nrows; j++ )
  {
    for( i = 0; i < row_index->nrows; i++ )
    {
      if( col_index->data[0][j] < 0 )
      {
        MTX_ERROR_MSG( "if( col_index->data[0][j] < 0 )" );
        return FALSE;
      }
      col = (unsigned) col_index->data[0][j];
      if( col >= src->ncols )
      {
        MTX_ERROR_MSG( "if( col >= src->ncols )" );
        return FALSE;
      }

      if( row_index->data[0][i] < 0 )
      {
        MTX_ERROR_MSG( "if( row_index->data[0][i] < 0 )" );
        return FALSE;
      }
      row = (unsigned) row_index->data[0][i];
      if( row >= src->nrows )
      {
        MTX_ERROR_MSG( "if( row >= src->nrows )" );
        return FALSE;
      }

      if( dst->isReal )
      {
        dst->data[j][i] = src->data[col][row];
      }
      else
      {
        dst->cplx[j][i].re = src->cplx[col][row].re;
        dst->cplx[j][i].im = src->cplx[col][row].im;
      }
    }
  }

  return TRUE;
}


BOOL MTX_SetIndexedValues( MTX *dst, const MTX *row_index, const MTX *col_index, const MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned row = 0;
  unsigned col = 0;
  unsigned maxrow = 0;
  unsigned maxcol = 0;
  double maxrow_d = 0.0;
  double maxcol_d = 0.0;
  double dumd;

  if( MTX_isNull( row_index ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( col_index ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  // Check that the dimensions of the src matrix match the number of elemens in row_index and col_index.
  if( row_index->nrows != src->nrows )
  {
    MTX_ERROR_MSG( "if( row_index->nrows != src->nrows )" );
    return FALSE;
  }
  if( col_index->nrows != src->ncols )
  {
    MTX_ERROR_MSG( "if( col_index->nrows != src->ncols )" );
    return FALSE;
  }

  // Check the dimension of the destination matrix and resize if needed.
  if( !MTX_MaxColumn( row_index, 0, &maxrow_d, &dumd ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
    return FALSE;
  }
  if( maxrow_d < 0.0 )
  {
    MTX_ERROR_MSG( "if( maxrow_d < 0.0 )" );
    return FALSE;
  }
  maxrow = (unsigned) maxrow_d;
  if( !MTX_MaxColumn( col_index, 0, &maxcol_d, &dumd ) )
  {
    MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
    return FALSE;
  }
  if( maxcol_d < 0 )
  {
    MTX_ERROR_MSG( "if( maxcol_d < 0 )" );
    return FALSE;
  }
  maxcol = (unsigned) maxcol_d;
  if( maxrow >= dst->nrows )
  {
    if( maxcol >= dst->ncols )
    {
      if( !MTX_Redim( dst, maxrow+1, maxcol+1 ) )
      {
        MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
        return FALSE;
      }
    }
    else
    {
      if( !MTX_Redim( dst, maxrow+1, dst->ncols ) )
      {
        MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
        return FALSE;
      }
    }
  }
  else
  {
    if( maxcol >= dst->ncols )
    {
      if( !MTX_Redim( dst, dst->nrows, maxcol+1 ) )
      {
        MTX_ERROR_MSG( "MTX_Redim returned FALSE." );
        return FALSE;
      }
    }
  }

  if( !src->isReal && dst->isReal )
  {
    if( !MTX_ConvertRealToComplex( dst ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  for( j = 0; j < col_index->nrows; j++ )
  {
    for( i = 0; i < row_index->nrows; i++ )
    {
      if( col_index->data[0][j] < 0 )
      {
        MTX_ERROR_MSG( "if( col_index->data[0][j] < 0 )" );
        return FALSE;
      }
      col = (unsigned) col_index->data[0][j];
      if( col >= dst->ncols )
      {
        MTX_ERROR_MSG( "if( col >= dst->ncols )" );
        return FALSE;
      }

      if( row_index->data[0][i] < 0 )
      {
        MTX_ERROR_MSG( "if( row_index->data[0][i] < 0 )" );
        return FALSE;
      }
      row = (unsigned) row_index->data[0][i];
      if( row >= dst->nrows )
      {
        MTX_ERROR_MSG( "if( row >= dst->nrows )" );
        return FALSE;
      }

      if( dst->isReal )
      {
        dst->data[col][row] = src->data[j][i];
      }
      else
      {
        if( src->isReal )
        {
          dst->cplx[col][row].re = src->data[j][i];
          dst->cplx[col][row].im = 0.0;
        }
        else
        {
          dst->cplx[col][row].re = src->cplx[j][i].re;
          dst->cplx[col][row].im = src->cplx[j][i].im;
        }
      }
    }
  }

  return TRUE;
}

BOOL MTX_FFT2( const MTX *src, MTX *dst )
{
  MTX M;
  MTX W;
  MTX_Init( &M );
  MTX_Init( &W );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src is a NULL matrix." );
    return FALSE;
  }
  if( dst == NULL )
  {
    MTX_ERROR_MSG( "dst is NULL." );
    return FALSE;
  }

  if( !MTX_FFT( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_FFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( dst, &M ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_FFT( &M, &W ) )
  {
    MTX_ERROR_MSG( "MTX_FFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( &W, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  MTX_Free( &M );
  MTX_Free( &W );
  return TRUE;
}

BOOL MTX_FFT2_Inplace( MTX *src )
{
  MTX M;
  MTX W;
  MTX_Init( &M );
  MTX_Init( &W );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src is a NULL matrix." );
    return FALSE;
  }

  if( !MTX_FFT_Inplace( src ) )
  {
    MTX_ERROR_MSG( "MTX_FFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( src, &M ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_FFT( &M, &W ) )
  {
    MTX_ERROR_MSG( "MTX_FFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( &W, src ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  MTX_Free( &M );
  MTX_Free( &W );
  return TRUE;
}


BOOL MTX_IFFT2( const MTX *src, MTX *dst )
{
  MTX M;
  MTX W;
  MTX_Init( &M );
  MTX_Init( &W );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src is a NULL matrix." );
    return FALSE;
  }
  if( dst == NULL )
  {
    MTX_ERROR_MSG( "dst is NULL." );
    return FALSE;
  }

  if( !MTX_IFFT( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_IFFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( dst, &M ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_IFFT( &M, &W ) )
  {
    MTX_ERROR_MSG( "MTX_IFFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( &W, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  MTX_Free( &M );
  MTX_Free( &W );
  return TRUE;
}


BOOL MTX_IFFT2_Inplace( MTX *src )
{
  MTX M;
  MTX W;
  MTX_Init( &M );
  MTX_Init( &W );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src is a NULL matrix." );
    return FALSE;
  }

  if( !MTX_IFFT_Inplace( src ) )
  {
    MTX_ERROR_MSG( "MTX_IFFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( src, &M ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_IFFT( &M, &W ) )
  {
    MTX_ERROR_MSG( "MTX_IFFT returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  if( !MTX_Transpose( &W, src ) )
  {
    MTX_ERROR_MSG( "MTX_Transpose returned FALSE." );
    MTX_Free( &M );
    MTX_Free( &W );
    return FALSE;
  }
  MTX_Free( &M );
  MTX_Free( &W );
  return TRUE;
}


BOOL MTX_FFT( const MTX *src, MTX *dst )
{
  return MTX_static_fft( src, dst, TRUE );
}

BOOL MTX_IFFT( const MTX *src, MTX *dst )
{
  return MTX_static_fft( src, dst, FALSE );
}

BOOL MTX_FFT_Inplace( MTX *src )
{
  return MTX_static_fft_inplace( src, TRUE );
}

BOOL MTX_IFFT_Inplace( MTX *src )
{
  return MTX_static_fft_inplace( src, FALSE );
}





// static
BOOL MTX_static_fft(
                    const MTX *src, //!< The source matrix.
                    MTX *dst, //!< The result matrix (always complex).
                    BOOL isFwd //!< A boolean to indicate if this is a fwd transform or the inverse transform
                    )
{

  if( !MTX_Copy( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  // Use the inplace KISS FFT function.
  return MTX_static_fft_inplace( dst, isFwd );


  return TRUE;
}





// static
BOOL MTX_static_fft_inplace(
                            MTX *src, //!< The source matrix.
                            BOOL isFwd //!< A boolean to indicate if this is a fwd transform or the inverse transform
                            )
{
  unsigned j = 0;
  double length = 0.0; // The length of the fft to be performed (as a double).


  // This is the configuration for the KISS FFT. (it's a pointer)
  kiss_fft_cfg fft_config;

  // This is the data type pointer used by KISS FFT.
  // It is the same as stComplex*
  kiss_fft_cpx *pKissComplexData = NULL;

  // Get the fft length.
  length = src->nrows;


  // KISS will work inplace.
  if( src->isReal )
  {
    if( !MTX_ConvertRealToComplex( src ) )
    {
      MTX_ERROR_MSG( "MTX_ConvertRealToComplex returned FALSE." );
      return FALSE;
    }
  }

  // special case
  if( length == 1 )
  {
    return TRUE;
  }

  // Set up the FFT engine
  //
  if( isFwd )
  {
    // for FFT
    fft_config = kiss_fft_alloc( (unsigned)length, 0, 0, 0 );
  }
  else
  {
    // for IFFT
    fft_config = kiss_fft_alloc( (unsigned)length, 1, 0, 0 );
  }
  if( fft_config == NULL )
  {
    MTX_ERROR_MSG( "if( fft_config == NULL )" );
    return FALSE;
  }

  // Compute the fft of each of the columns.
  for( j = 0; j < src->ncols; j++ )
  {
    // The casting into the kiss struct pointer is allowed because the  
    // structs are defined identically.
    pKissComplexData = (kiss_fft_cpx *)src->cplx[j];

    // inplace FFT is allowed for kiss fft
    kiss_fft( fft_config, pKissComplexData, pKissComplexData );    
  }

  if( !isFwd )
  {
    // scaling must be done
    if( !MTX_Multiply_Scalar( src, 1.0/length ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
      return FALSE;
    }
  }

  // Free the KISS FFT configuration
  if( fft_config )
  {
    kiss_fft_free( fft_config );
    fft_config = NULL;
  }


  return TRUE;
}


BOOL MTX_sin( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  double re;
  double im;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = sin( src->data[j][i] );
      }
    }
  }
  else
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {      
        re = src->cplx[j][i].re;
        im = src->cplx[j][i].im;
        src->cplx[j][i].re = sin(re)*cosh(im);
        src->cplx[j][i].im = cos(re)*sinh(im);
      }
    }
  }
  return TRUE;

}

BOOL MTX_sinc( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  MTX copysrc;
  double re = 0.0;
  double im = 0.0;

  MTX_Init( &copysrc );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_Multiply_Scalar( src, PI ) )
  {
    MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
    return FALSE;
  }


  // make a copy
  if( !MTX_Copy( src, &copysrc ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  // compute the sin of the values
  if( !MTX_sin( src ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free( &copysrc );
    return FALSE;
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        if( copysrc.data[j][i] == 0.0 )
          src->data[j][i] = 1.0;
        else
          src->data[j][i] /= copysrc.data[j][i];
      }
    }
  }
  else
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        if( copysrc.cplx[j][i].re == 0.0 && copysrc.cplx[j][i].im == 0.0 )
        {
          src->cplx[j][i].re = 1.0;
          src->cplx[j][i].im = 0.0;
        }
        else
        {
          MTX_static_quick_complex_divide( &src->cplx[j][i].re, &src->cplx[j][i].im, &copysrc.cplx[j][i].re, &copysrc.cplx[j][i].im, &re, &im );
          src->cplx[j][i].re = re;
          src->cplx[j][i].im = im;
        }
      }
    }
  }

  MTX_Free(&copysrc);
  return TRUE;
}

BOOL MTX_sinh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;


  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = sinh( src->data[j][i] );
      }
    }    
  }
  else
  {
    // sinh = (exp(x) - exp(-x))/2.0
    MTX epX; // e^x
    MTX emX; // e-x

    MTX_Init( &epX );
    MTX_Init( &emX );

    if( !MTX_Copy( src, &epX ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free(&epX);
      return FALSE;
    }
    if( !MTX_Copy( src, &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }
    if( !MTX_Negate( &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }

    if( !MTX_Exp( &epX ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }
    if( !MTX_Exp( &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }

    if( !MTX_Subtract( src, &epX, &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Subtract returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }
    if( !MTX_Multiply_Scalar( src, 0.5 ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
      MTX_Free(&epX);
      MTX_Free(&emX);
      return FALSE;
    }

    MTX_Free(&epX);
    MTX_Free(&emX);
  }
  return TRUE;
}

BOOL MTX_asinh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  // asinh = ln (x+sqrt(1+x^2))
  MTX sqrtOnePlusX2;
  MTX_Init( &sqrtOnePlusX2 );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( !MTX_Copy(src,&sqrtOnePlusX2) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }
  if( !MTX_Sqr(&sqrtOnePlusX2) )
  {
    MTX_ERROR_MSG( "MTX_Sqr returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }
  if( !MTX_Increment(&sqrtOnePlusX2) )
  {
    MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }
  if( !MTX_Sqrt(&sqrtOnePlusX2) )
  {
    MTX_ERROR_MSG( "MTX_Sqrt returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }
  if( !MTX_Add_Inplace( src, &sqrtOnePlusX2 ) )
  {
    MTX_ERROR_MSG( "MTX_Add_Inplace returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }
  if( !MTX_Ln( src ) )
  {
    MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
    MTX_Free(&sqrtOnePlusX2);
    return FALSE;
  }

  MTX_Free(&sqrtOnePlusX2);
  return TRUE;
}


BOOL MTX_cos( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  double re;
  double im;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = cos( src->data[j][i] );
      }
    }
  }
  else
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {      
        re = src->cplx[j][i].re;
        im = src->cplx[j][i].im;
        src->cplx[j][i].re = cos(re)*cosh(im);
        src->cplx[j][i].im = -sin(re)*sinh(im);
      }
    }
  }
  return TRUE;

}

BOOL MTX_cosh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = cosh( src->data[j][i] );        
      }
    }
    return TRUE;
  }
  else
  {
    // cosh = (exp(x) + exp(-x))/2.0
    MTX epX; // e^x
    MTX emX; // e-x

    MTX_Init( &epX );
    MTX_Init( &emX );

    if( !MTX_Copy( src, &epX ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }
    if( !MTX_Copy( src, &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }
    if( !MTX_Negate( &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }

    if( !MTX_Exp( &epX ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }
    if( !MTX_Exp( &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }

    if( !MTX_Add( src, &epX, &emX ) )
    {
      MTX_ERROR_MSG( "MTX_Add returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }
    if( !MTX_Multiply_Scalar( src, 0.5 ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
      MTX_Free( &epX );
      MTX_Free( &emX );
      return FALSE;
    }

    MTX_Free(&epX);
    MTX_Free(&emX);
  }
  return TRUE;
}

BOOL MTX_acosh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  // acosh = ln( z + sqrt(z+1)*sqrt(z-1) )
  MTX sXp1;
  MTX sXm1;
  MTX_Init( &sXp1 );
  MTX_Init( &sXm1 );

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  if( !MTX_Copy(src,&sXp1) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free(&sXp1);
    return FALSE;
  }
  if( !MTX_Copy(src,&sXm1) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }

  // acosh = ln( z + sqrt(z+1)*sqrt(z-1) )

  if( !MTX_Increment(&sXp1) )
  {
    MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  if( !MTX_Decrement(&sXm1) )
  {
    MTX_ERROR_MSG( "MTX_Decrement returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  if( !MTX_Sqrt(&sXp1) )
  {
    MTX_ERROR_MSG( "MTX_Sqrt returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  if( !MTX_Sqrt(&sXm1) )
  {
    MTX_ERROR_MSG( "MTX_Sqrt returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  if( !MTX_DotMultiply_Inplace( &sXp1, &sXm1 ) )
  {
    MTX_ERROR_MSG( "MTX_DotMultiply_Inplace returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  if( !MTX_Add_Inplace( src, &sXp1 ) )
  {
    MTX_ERROR_MSG( "MTX_Add_Inplace returned FALSE." );
    MTX_Free(&sXp1);
    MTX_Free(&sXm1);
    return FALSE;
  }
  MTX_Free(&sXp1);
  MTX_Free(&sXm1);

  if( !MTX_Ln( src ) )
  {
    MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
    return FALSE;
  }
  return TRUE;
}


BOOL MTX_tan( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = tan( src->data[j][i] );        
      }
    }
    return TRUE;
  }
  else
  {
    MTX cos_src;
    MTX_Init(&cos_src);

    if( !MTX_Copy(src,&cos_src) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free(&cos_src);
      return FALSE;
    }

    if( !MTX_cos(&cos_src) )
    {
      MTX_ERROR_MSG( "MTX_cos returned FALSE." );
      MTX_Free(&cos_src);
      return FALSE;
    }

    if( !MTX_sin(src) )
    {
      MTX_ERROR_MSG( "MTX_sin returned FALSE." );
      MTX_Free(&cos_src);
      return FALSE;
    }

    if( !MTX_DotDivide_Inplace( src, &cos_src ) )
    {
      MTX_ERROR_MSG( "MTX_DotDivide_Inplace returned FALSE." );
      MTX_Free(&cos_src);
      return FALSE;
    }

    MTX_Free(&cos_src);
  }
  return TRUE;
}

BOOL MTX_tanh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = tanh( src->data[j][i] );        
      }
    }
    return TRUE;
  }
  else
  {
    // tanh = (exp(2x)-1)/(exp(2x)+1)
    MTX e2Xm1; // exp(2x)-1
    MTX e2Xp1; // exp(2x)+1

    MTX_Init( &e2Xm1 );
    MTX_Init( &e2Xp1 );

    if( !MTX_Copy( src, &e2Xm1 ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }
    if( !MTX_Multiply_Scalar( &e2Xm1, 2.0 ) )
    {
      MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }

    if( !MTX_Exp( &e2Xm1 ) )
    {
      MTX_ERROR_MSG( "MTX_Exp returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }
    if( !MTX_Copy( &e2Xm1, &e2Xp1 ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }

    if( !MTX_Decrement( &e2Xm1 ) )
    {
      MTX_ERROR_MSG( "MTX_Decrement returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }
    if( !MTX_Increment( &e2Xp1 ) )
    {
      MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }

    if( !MTX_DotDivide_Inplace( &e2Xm1, &e2Xp1 ) )
    {
      MTX_ERROR_MSG( "MTX_DotDivide_Inplace returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }

    if( !MTX_Copy( &e2Xm1, src ) )
    {
      MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
      MTX_Free( &e2Xm1 );
      MTX_Free( &e2Xp1 );
      return FALSE;
    }

    MTX_Free(&e2Xm1);
    MTX_Free(&e2Xp1);
  }
  return TRUE;
}


BOOL MTX_atanh( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;
  // atanh = 0.5*( ln((1+z)/(1-z)) )
  MTX oneMx; // 1-x

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }


  // atanh = 0.5*( ln((1+z)/(1-z)) )
  MTX_Init( &oneMx );

  if( !MTX_Copy( src, &oneMx ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }
  if( !MTX_Increment( src ) )
  {
    MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
    MTX_Free(&oneMx);
    return FALSE;
  }
  if( !MTX_Negate( &oneMx ) )
  {
    MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
    MTX_Free(&oneMx);
    return FALSE;
  }
  if( !MTX_Increment( &oneMx ) )
  {
    MTX_ERROR_MSG( "MTX_Increment returned FALSE." );
    MTX_Free(&oneMx);
    return FALSE;
  }

  if( !MTX_DotDivide_Inplace( src, &oneMx ) )
  {
    MTX_ERROR_MSG( "MTX_DotDivide_Inplace returned FALSE." );
    MTX_Free(&oneMx);
    return FALSE;
  }

  MTX_Free(&oneMx);

  if( !MTX_Ln(src) )
  {
    MTX_ERROR_MSG( "MTX_Ln returned FALSE." );
    return FALSE;
  }
  if( !MTX_Multiply_Scalar( src, 0.5 ) )
  {
    MTX_ERROR_MSG( "MTX_Multiply_Scalar returned FALSE." );
    return FALSE;
  }

  return TRUE;
}


BOOL MTX_cot( MTX *src )
{
  if( !MTX_tan( src ) )
  {
    MTX_ERROR_MSG( "MTX_tan returned FALSE." );
    return FALSE;
  }

  if( !MTX_Inv( src ) )
  {
    MTX_ERROR_MSG( "MTX_Inv returned FALSE." );
    return FALSE;
  }

  return TRUE;
}

BOOL MTX_coth( MTX *src )
{
  if( !MTX_tanh( src ) )
  {
    MTX_ERROR_MSG( "MTX_tanh returned FALSE." );
    return FALSE;
  }

  if( !MTX_Inv( src ) )
  {
    MTX_ERROR_MSG( "MTX_Inv returned FALSE." );
    return FALSE;
  }

  return TRUE;
}

BOOL MTX_Inv( MTX *src )
{
  unsigned i = 0;
  unsigned j = 0;

  if( MTX_isNull(src) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( src->isReal )
  {
    for( j = 0; j < src->ncols; j++ )
    {
      for( i = 0; i < src->nrows; i++ )
      {
        src->data[j][i] = 1.0 / src->data[j][i];
      }
    }
  }
  else
  {
    // 1/(A+Bi) = 1/(A+Bi) * (A-Bi)/(A-Bi) = conj/mag^2
    MTX magSrc;
    MTX_Init(&magSrc);

    if( !MTX_Magnitude(src,&magSrc) )
    {
      MTX_ERROR_MSG( "MTX_Magnitude returned FALSE." );
      MTX_Free(&magSrc);
      return FALSE;
    }
    if( !MTX_Sqr(&magSrc) )
    {
      MTX_ERROR_MSG( "MTX_Sqr returned FALSE." );
      MTX_Free(&magSrc);
      return FALSE;
    }
    if( !MTX_Conjugate(src) )
    {
      MTX_ERROR_MSG( "MTX_Conjugate returned FALSE." );
      MTX_Free(&magSrc);
      return FALSE;
    }
    if( !MTX_DotDivide_Inplace(src,&magSrc) )
    {
      MTX_ERROR_MSG( "MTX_DotDivide_Inplace returned FALSE." );
      MTX_Free(&magSrc);
      return FALSE;
    }

    MTX_Free(&magSrc);
  }
  return TRUE;
}


BOOL MTX_Colon( MTX *dst, const double start, const double increment, const double end )
{
  unsigned i = 0;
  unsigned nrows = 0;

  if( increment == 0.0 )
  {
    MTX_ERROR_MSG( "if( increment == 0.0 )" );
    return FALSE;
  }

  if( increment > 0.0 )
  {
    if( end < start )
    {
      MTX_ERROR_MSG( "if( end < start )" );
      return FALSE;
    }

    // Determine the number of rows needed.
    nrows = (unsigned)floor( (end-start)/increment ) + 1;

    if( !MTX_Malloc( dst, nrows, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }

    for( i = 0; i < nrows; i++ )
    {
      dst->data[0][i] = start + i*increment;
    }
  }
  else
  {
    if( start < end )
    {
      MTX_ERROR_MSG( "if( start < end )" );
      return FALSE;
    }

    // Determine the number of rows needed.
    nrows = abs((unsigned)floor( (end-start)/increment )) + 1;

    if( !MTX_Malloc( dst, nrows, 1, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }

    for( i = 0; i < nrows; i++ )
    {
      dst->data[0][i] = start + i*increment;
    }
  }

  return TRUE;
}

BOOL MTX_RemoveRowsAndColumns(
                              MTX *src, //!< The pointer to the matrix object.
                              const unsigned nrows, //!< The number of rows to remove (the length of the rows array).
                              const unsigned rows[], //!< The array of row indices to remove.
                              const unsigned ncols, //!< The number of columns to remove (the length of hte cols array).
                              const unsigned cols[]
)
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned col_index = 0;
  unsigned row_index = 0;
  MTX srcCopy;
  BOOL skip_row = FALSE;
  //char buffer[1024];

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  MTX_Init( &srcCopy );

  if( nrows > 0 )
  {
    if( rows == NULL )
    {
      MTX_ERROR_MSG( "rows is a NULL pointer." );
      return FALSE;
    }
  }

  if( ncols > 0 )
  {
    if( cols == NULL )
    {
      MTX_ERROR_MSG( "cols is a NULL pointer." );
      return FALSE;
    }
  }

  //MTX_PrintAutoWidth_ToBuffer( src, buffer, 1024, 6 );
  //printf( buffer );

  // First remove all the columns specified.
  for( j = 0; j < ncols; j++ )
  {
    col_index = cols[j];
    if( col_index-j >= src->ncols )
    {
      MTX_ERROR_MSG( "if( col_index-j >= src->ncols )" );
      return FALSE;
    }

    // The removal of columns is very efficient.
    // The index changes stepwise (minus j is needed).
    if( !MTX_RemoveColumn( src, col_index-j ) )
    {
      MTX_ERROR_MSG( "MTX_RemoveColumn returned FALSE." );
      return FALSE;
    }
  }

  //MTX_PrintAutoWidth_ToBuffer( src, buffer, 1024, 6 );
  //printf( buffer );

  // Make a copy of the src matrix (columns already removed).
  if( !MTX_Copy( src, &srcCopy ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  // Check the indices of the rows specified for removal.
  for( i = 0; i < nrows; i++ )
  {
    if( rows[i] >= src->nrows )
    {
      MTX_ERROR_MSG( "if( rows[i] >= src->nrows )" );
      return FALSE;
    }
  }

  // Resize the original matrix accordingly.
  if( !MTX_Resize( src, src->nrows-nrows, src->ncols, src->isReal ) )
  {
    MTX_ERROR_MSG( "MTX_Resize returned FALSE." );
    return FALSE;
  }

  // Copy back the data with the specified rows removed.
  for( i = 0; i < srcCopy.nrows; i++ )
  {
    skip_row = FALSE;
    for( row_index = 0; row_index < nrows; row_index++ )
    {
      if( i == rows[row_index] )
      {
        skip_row = TRUE;
        break;
      }
    }

    if( !skip_row )
    {
      if( src->isReal )
      {
        for( j = 0; j < src->ncols; j++ )
        {
          src->data[j][k] = srcCopy.data[j][i];
        }
      }
      else
      {
        for( j = 0; j < src->ncols; j++ )
        {
          src->cplx[j][k].re = srcCopy.cplx[j][i].re;
          src->cplx[j][k].im = srcCopy.cplx[j][i].im;
        }
      }
      k++;
    }
  }

  //MTX_PrintAutoWidth_ToBuffer( src, buffer, 1024, 6 );
  //printf( buffer );

  MTX_Free( &srcCopy );

  return TRUE;
}


// static
BOOL MTX_IsNAN( double value )
{
#ifdef WIN32
  if( _isnan( value ) )
    return TRUE;
  else
    return FALSE;
#else
  if( isnan( value ) )
    return TRUE;
  else
    return FALSE;
#endif
}


// static
BOOL MTX_IsPostiveINF( double value )
{
#ifdef WIN32
  if( _finite( value ) )
  {
    return FALSE;
  }
  else
  {
    if( value > 0 )
      return TRUE;
    else
      return FALSE;
  }
#else
  if( isfinite( value ) )
  {
    return FALSE;
  }
  else
  {
    if( value > 0 )
      return TRUE;
    else
      return FALSE;
  }
#endif
}


// static
BOOL MTX_IsNegativeINF( double value )
{
#ifdef WIN32
  if( _finite( value ) )
  {
    return FALSE;
  }
  else
  {
    if( value < 0 )
      return TRUE;
    else
      return FALSE;
  }
#else
  if( isfinite( value ) )
  {
    return FALSE;
  }
  else
  {
    if( value < 0 )
      return TRUE;
    else
      return FALSE;
  }
#endif
}

// Get a value from the uniform distribution between 0 and 1.
double MTX_static_get_rand_value()
{
  double val;
  val = ((double)rand()) / ((double)RAND_MAX);
  return val;
}

// REFERENCE: 
// Scheinerman, E. R (2006). "C++ for Mathematicians: An Introduction for Students and Professionals."
// Chapman and Hall/CRC, Taylor and Francis Group. pp 61-63.
//
// static 
double MTX_static_get_randn_value()
{
  /*
  "The do-while loop generates points (x,y) uniformly in the square [-1,1]^2
  until one that is interior to the unit dist is found. Each pass through
  the loop has a pi/4 chance of succeeding, so after just a few iterations
  we are assured of finding a point chosed uniformly from the unit disk.
  Once the point (x,y) has been found, the rest of the algorithm follows the 
  Box-Muller method." 
  ...
  "The algorithm is capable of producing two independant normal random variables." (mu*x and mu*y) 
  Thus, the use of the static values.
  */
  static BOOL has_saved = FALSE;
  static double saved;
  double x;
  double y;
  double r;
  double mu;

  if( has_saved )
  {
    has_saved = FALSE;
    return saved;
  }

  do
  {
    x = MTX_static_get_rand_value()*2.0 - 1.0;
    y = MTX_static_get_rand_value()*2.0 - 1.0;
    r = x*x + y*y;
  } while( r >= 1.0 );

  mu = sqrt( -2.0 * log( r ) / r );

  saved = mu*y;
  has_saved = TRUE;

  return mu*x;
}

BOOL MTX_randn(
  MTX* M,
  const unsigned nrows,
  const unsigned ncols,  
  const unsigned seed
  )
{
  unsigned j;
  unsigned i;

  if( M == NULL )
  {
    MTX_ERROR_MSG( "NULL input matrix." );
    return FALSE;
  }
  if( nrows == 0 || ncols == 0 )
  {
    MTX_ERROR_MSG( "if( nrows == 0 || ncols == 0 )" );
    return FALSE;
  }
  if( !MTX_Malloc( M, nrows, ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }
  srand( seed ); // seed the rand function.

  for( j = 0; j < ncols; j++ )  
  {
    for( i = 0; i < nrows; i++ )    
    {
      M->data[j][i] = MTX_static_get_randn_value();
    }
  }
  
  return TRUE;
}

BOOL MTX_rand(
  MTX* M,
  const unsigned nrows,
  const unsigned ncols,  
  const unsigned seed
  )
{  
  unsigned j;
  unsigned i;

  if( M == NULL )
  {
    MTX_ERROR_MSG( "NULL input matrix." );
    return FALSE;
  }
  if( nrows == 0 || ncols == 0 )
  {
    MTX_ERROR_MSG( "if( nrows == 0 || ncols == 0 )" );
    return FALSE;
  }
  if( !MTX_Malloc( M, nrows, ncols, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
    return FALSE;
  }
  srand( seed ); // seed the rand function.

  for( j = 0; j < ncols; j++ )  
  {
    for( i = 0; i < nrows; i++ )    
    {
      M->data[j][i] = MTX_static_get_rand_value();
    }
  }
  
  return TRUE;
}




#ifndef _MATRIX_NO_PLOTTING

BOOL MTX_PlotQuick( MTX* M, const char* bmpfilename, const unsigned x_col, const unsigned y_col )
{
  unsigned i = 0;
  unsigned j = 0;
  CPLOT_structSeries s; // The deep level series struct.
  CPLOT_structPlotOptions opt; // The plotting options.
  CPLOT P; // The plot 'object'.
  char xlabel[128];
  char ylabel[128];
  char slabel[128];

  if( MTX_isNull(M) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( bmpfilename == NULL )
  {
    MTX_ERROR_MSG( "bmpfilename is a NULL pointer." );
    return FALSE;
  }

  if( x_col > M->ncols )
  {
    MTX_ERROR_MSG( "if( x_col > M->ncols )" );
    return FALSE;
  }
  if( y_col > M->ncols )
  {
    MTX_ERROR_MSG( "if( y_col > M->ncols )" );
    return FALSE;
  }

  memset( &s, 0, sizeof(CPLOT_structSeries) );
  memset( &opt, 0, sizeof(CPLOT_structPlotOptions) );

  if( !CPLOT_PlotOptionsInit( &opt ) )
  {
    MTX_ERROR_MSG( "CPLOT_PlotOptionsInit returned FALSE." );
    return FALSE;
  }

  if( !CPLOT_Init( &P ) )
  {
    MTX_ERROR_MSG( "CPLOT_Init returned FALSE." );
    return FALSE;
  }

  opt.numberOfSeries = 1;
  opt.PlotSize_Height_cm = 8;
  opt.PlotSize_Width_cm = 10;
  opt.plotStatistics = 1;
  opt.title = (char*)bmpfilename;
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( sprintf_s( xlabel, 128, "column %d", x_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    return FALSE;
  }
#else
  if( sprintf( xlabel, "column %d", x_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }
#endif
  opt.x.label = xlabel;
  opt.x.isGridOn = TRUE;
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( sprintf_s( ylabel, 128, "column %d", y_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    return FALSE;
  }
#else
  if( sprintf( ylabel, "column %d", y_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }
#endif
  opt.y.label = ylabel;
  opt.y.isGridOn = TRUE;

  if( !CPLOT_SetPlotOptions( &P, &opt ) )
  {
    MTX_ERROR_MSG( "CPLOT_SetPlotOptions returned FALSE." );
    return FALSE;
  }

  s.color = CPLOT_BLUE;
  s.connected = TRUE;
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( sprintf_s( slabel, 128, "col %d vs %d", x_col, y_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf_s returned failure." );
    return FALSE;
  }
#else
  if( sprintf( slabel, "col %d vs %d", x_col, y_col ) < 0 )
  {
    MTX_ERROR_MSG( "sprintf returned failure." );
    return FALSE;
  }
#endif
  s.label = slabel;
  s.markOutlierData = TRUE;
  s.n = M->nrows;
  s.precision = 6;
  s.units = NULL;
  s.X = M->data[x_col];
  s.Y = M->data[y_col];

  if( !CPLOT_Plot( &P, &s ) )
  {
    MTX_ERROR_MSG( "CPLOT_Plot returned FALSE." );
    return FALSE;
  }

  if( !CPLOT_SaveToFile( &P, bmpfilename ) )
  {
    MTX_ERROR_MSG( "CPLOT_SaveToFile returned FALSE." );
    return FALSE;
  }

  return TRUE;
}


BOOL MTX_Plot(
              const char* bmpfilename, //!< The output RLE compressed BITMAP filename.
              const char* title, //!< The plot title (NULL if not used).
              const unsigned plot_height_cm, //!< The plot height in cm.
              const unsigned plot_width_cm, //!< The plot width in cm.
              const BOOL includeStats, //!< A boolean to indicate if statistics info should be included on the plot.
              const BOOL isXGridOn, //!< A boolean to indicate if the x grid lines are on.
              const BOOL isYGridOn, //!< A boolean to indicate if the y grid lines are on.
              const char* xlabel, //!< The x axis label (NULL if not used).
              const char* ylabel, //!< The y axis label (NULL if not used).
              const MTX_structAxisOptions opt_x, //!< Limits and ticks for x.
              const MTX_structAxisOptions opt_y, //!< Limits and ticks for y.
              const MTX_PLOT_structSeries* series, //!< A pointer to an array of series structs.
              const unsigned nrSeries //!< The number of series.
              )
{
  unsigned i = 0;

  CPLOT_structSeries s; // The deep level series struct.
  CPLOT_structPlotOptions opt; // The plotting options.
  CPLOT P; // The plot 'object'.

  double val = 0;
  double re = 0;
  double im = 0;
  double xmin = DBL_MAX;
  double xmax = -DBL_MAX;
  double ymin = DBL_MAX;
  double ymax = -DBL_MAX;

  typedef struct
  {
    double lowerlimit;
    double upperlimit;
    double tickstart;
    double ticksize;
    double tickend;
  }_structAxis;
  _structAxis x;
  _structAxis y;

  if( bmpfilename == NULL )
  {
    MTX_ERROR_MSG( "bmpfilename is a NULL pointer." );
    return FALSE;
  }

  if( plot_height_cm > 50 || plot_width_cm > 50 )
  {
    MTX_ERROR_MSG( "if( plot_height_cm > 50 || plot_width_cm > 50 )" );
    return FALSE;
  }

  memset( &s, 0, sizeof(CPLOT_structSeries) );
  memset( &opt, 0, sizeof(CPLOT_structPlotOptions) );

  if( !CPLOT_PlotOptionsInit( &opt ) )
  {
    MTX_ERROR_MSG( "CPLOT_PlotOptionsInit returned FALSE." );
    return FALSE;
  }

  if( !CPLOT_Init( &P ) )
  {
    MTX_ERROR_MSG( "CPLOT_Init returned FALSE." );
    return FALSE;
  }

  if( series == NULL )
  {
    MTX_ERROR_MSG( "series is a NULL pointer." );
    return FALSE;
  }

  if( nrSeries == 0 )
  {
    MTX_ERROR_MSG( "if( nrSeries == 0 )" );
    return FALSE;
  }

  // Check the input.
  for( i = 0; i < nrSeries; i++ )
  {
    if( MTX_isNull(series[i].M) )
    {
      MTX_ERROR_MSG( "NULL Matrix" );
      return FALSE;
    }
    if( !series[i].M->isReal )
    {
      MTX_ERROR_MSG( "if( !series[i].M->isReal )" );
      return FALSE; // only plot real data
    }

    if( series[i].x_col >= series[i].M->ncols )
    {
      MTX_ERROR_MSG( "if( series[i].x_col >= series[i].M->ncols )" );
      return FALSE;
    }
    if( series[i].y_col >= series[i].M->ncols )
    {
      MTX_ERROR_MSG( "if( series[i].y_col >= series[i].M->ncols )" );
      return FALSE;
    }
  }

  for( i = 0; i < nrSeries; i++ )
  {
    // Determine the maximum and minimum values for this series.
    // Why? because if we are plotting multiple series, we first have to
    // determine the windows maximum, minimums.
    if( !MTX_MinColumn( series[i].M, series[i].x_col, &re, &im ) )
    {
      MTX_ERROR_MSG( "MTX_MinColumn returned FALSE." );
      return FALSE;
    }
    if( re < xmin )
      xmin = re;
    if( !MTX_MaxColumn( series[i].M, series[i].x_col, &re, &im ) )
    {
      MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
      return FALSE;
    }
    if( re > xmax )
      xmax = re;

    if( !MTX_MinColumn( series[i].M, series[i].y_col, &re, &im ) )
    {
      MTX_ERROR_MSG( "MTX_MinColumn returned FALSE." );
      return FALSE;
    }
    if( re < ymin )
      ymin = re;
    if( !MTX_MaxColumn( series[i].M, series[i].y_col, &re, &im ) )
    {
      MTX_ERROR_MSG( "MTX_MaxColumn returned FALSE." );
      return FALSE;
    }
    if( re > ymax )
      ymax = re;
  }


  // First deal with determining the full window size
  // use the max min values already determined for all series if needed.
  // We do this because otherwise the plot window dimensions are determined
  // by the first series to be plotted.

  // Special case - all defaults indicated
  if( !opt_x.lowerlimit.doNotUseDefault &&
    !opt_x.upperlimit.doNotUseDefault &&
    !opt_x.tickstart.doNotUseDefault &&
    !opt_x.tickend.doNotUseDefault )
  {
    val = xmin;
    if( val < 0 )
    {
      val = -ceil( -val * 10.0 ) / 10.0;
    }
    else
    {
      val = floor( val * 10.0 ) / 10.0;
    }
    x.lowerlimit = val;

    val = xmax;
    if( val < 0 )
    {
      val = -floor( -val * 10.0 ) / 10.0;
    }
    else
    {
      val = ceil( val * 10.0 ) / 10.0;
    }
    x.upperlimit = val;


    if( x.lowerlimit == x.upperlimit )
    {
      x.lowerlimit -= x.lowerlimit/100.0;
      x.upperlimit += x.upperlimit/100.0;
    }

    x.tickstart = x.lowerlimit;
    x.ticksize = (x.upperlimit-x.lowerlimit)/5.0;
    x.tickend = x.upperlimit;
  }
  else
  {
    // deal with mixed or all user specified case

    if( opt_x.lowerlimit.doNotUseDefault )
      x.lowerlimit = opt_x.lowerlimit.val;
    else
      x.lowerlimit = xmin;

    if( opt_x.upperlimit.doNotUseDefault )
      x.upperlimit = opt_x.upperlimit.val;
    else
      x.upperlimit = xmax;

    if( opt_x.tickstart.doNotUseDefault )
      x.tickstart = opt_x.tickstart.val;
    else
      x.tickstart = x.lowerlimit;

    if( opt_x.tickend.doNotUseDefault )
      x.tickend = opt_x.tickend.val;
    else
      x.tickend = x.upperlimit;

    if( opt_x.ticksize.doNotUseDefault )
      x.ticksize = opt_x.ticksize.val;
    else
      x.ticksize = (x.tickend - x.tickstart)/5.0;
  }
  if( x.lowerlimit == x.upperlimit )
  {
    MTX_ERROR_MSG( "if( x.lowerlimit == x.upperlimit )" );
    return FALSE;
  }
  if( x.tickstart == x.tickend )
  {
    MTX_ERROR_MSG( "if( x.tickstart == x.tickend )" );
    return FALSE;
  }
  if( x.ticksize <= 0.0 )
  {
    MTX_ERROR_MSG( "if( x.ticksize <= 0.0 )" );
    return FALSE;
  }



  // Special case - all defaults indicated
  if( !opt_y.lowerlimit.doNotUseDefault &&
    !opt_y.upperlimit.doNotUseDefault &&
    !opt_y.tickstart.doNotUseDefault &&
    !opt_y.tickend.doNotUseDefault )
  {
    val = ymin;
    if( val < 0 )
    {
      val = -ceil( -val * 10.0 ) / 10.0;
    }
    else
    {
      val = floor( val * 10.0 ) / 10.0;
    }
    y.lowerlimit = val;

    val = ymax;
    if( val < 0 )
    {
      val = -floor( -val * 10.0 ) / 10.0;
    }
    else
    {
      val = ceil( val * 10.0 ) / 10.0;
    }
    y.upperlimit = val;


    if( y.lowerlimit == y.upperlimit )
    {
      y.lowerlimit -= y.lowerlimit/100.0;
      y.upperlimit += y.upperlimit/100.0;
    }

    y.tickstart = y.lowerlimit;
    y.ticksize = (y.upperlimit-y.lowerlimit)/10.0;
    y.tickend = y.upperlimit;
  }
  else
  {
    // deal with mixed or all user specified case

    if( opt_y.lowerlimit.doNotUseDefault )
      y.lowerlimit = opt_y.lowerlimit.val;
    else
      y.lowerlimit = ymin;

    if( opt_y.upperlimit.doNotUseDefault )
      y.upperlimit = opt_y.upperlimit.val;
    else
      y.upperlimit = ymax;

    if( opt_y.tickstart.doNotUseDefault )
      y.tickstart = opt_y.tickstart.val;
    else
      y.tickstart = y.lowerlimit;

    if( opt_y.tickend.doNotUseDefault )
      y.tickend = opt_y.tickend.val;
    else
      y.tickend = y.upperlimit;

    if( opt_y.ticksize.doNotUseDefault )
      y.ticksize = opt_y.ticksize.val;
    else
      y.ticksize = (y.tickend - y.tickstart)/10.0;
  }
  if( y.lowerlimit == y.upperlimit )
  {
    MTX_ERROR_MSG( "if( y.lowerlimit == y.upperlimit )" );
    return FALSE;
  }
  if( y.tickstart == y.tickend )
  {
    MTX_ERROR_MSG( "if( y.tickstart == y.tickend )" );
    return FALSE;
  }
  if( y.ticksize <= 0.0 )
  {
    MTX_ERROR_MSG( "if( y.ticksize <= 0.0 )" );
    return FALSE;
  }

  // All the values for lower, upper, and ticks are now set.

  opt.x.lowerlimit.val = x.lowerlimit;
  opt.x.upperlimit.val = x.upperlimit;
  opt.x.tickstart.val = x.tickstart;
  opt.x.ticksize.val = x.ticksize;
  opt.x.tickend.val = x.tickend;

  opt.x.lowerlimit.doNotUseDefault = TRUE;
  opt.x.upperlimit.doNotUseDefault = TRUE;
  opt.x.tickstart.doNotUseDefault = TRUE;
  opt.x.ticksize.doNotUseDefault = TRUE;
  opt.x.tickend.doNotUseDefault = TRUE;

  opt.y.lowerlimit.val = y.lowerlimit;
  opt.y.upperlimit.val = y.upperlimit;
  opt.y.tickstart.val = y.tickstart;
  opt.y.ticksize.val = y.ticksize;
  opt.y.tickend.val = y.tickend;

  opt.y.lowerlimit.doNotUseDefault = TRUE;
  opt.y.upperlimit.doNotUseDefault = TRUE;
  opt.y.tickstart.doNotUseDefault = TRUE;
  opt.y.ticksize.doNotUseDefault = TRUE;
  opt.y.tickend.doNotUseDefault = TRUE;

  opt.title = (char*)title;
  opt.x.label = (char*)xlabel;
  opt.y.label = (char*)ylabel;
  opt.x.isGridOn = isXGridOn;
  opt.y.isGridOn = isYGridOn;
  opt.plotStatistics = includeStats;
  opt.PlotSize_Height_cm = plot_height_cm;
  opt.PlotSize_Width_cm = plot_width_cm;
  opt.numberOfSeries = nrSeries;

  if( !CPLOT_SetPlotOptions( &P, &opt ) )
  {
    MTX_ERROR_MSG( "CPLOT_SetPlotOptions returned FALSE." );
    return FALSE;
  }

  for( i = 0; i < nrSeries; i++ )
  {
    s.color = (CPLOT_enumColor) series[i].color;
    s.connected = series[i].connected;
    s.label = series[i].label;
    s.markOutlierData = series[i].markOutlierData;
    s.precision = series[i].precision;
    s.units = series[i].units;
    s.n = series[i].M->nrows;
    s.X = series[i].M->data[series[i].x_col];
    s.Y = series[i].M->data[series[i].y_col];

    if( !CPLOT_Plot( &P, &s ) )
    {
      MTX_ERROR_MSG( "CPLOT_Plot returned FALSE." );
      return FALSE;
    }
  }

  if( !CPLOT_SaveToFile( &P, bmpfilename ) )
  {
    MTX_ERROR_MSG( "CPLOT_SaveToFile returned FALSE." );
    return FALSE;
  }

  return TRUE;
}

#endif // ifndef _MATRIX_NO_PLOTTING

#ifdef _DEBUG
/* 
 Take a filename and return a pointer to its final element.  This
 function is called on __FILE__ to fix a MSVC nit where __FILE__
 contains the full path to the file.  This is bad, because it
 confuses users to find the home directory of the person who
 compiled the binary in their warrning messages.
*/
const char* _shortfile(const char *fname)
{
  const char *cp1, *cp2, *r;
  cp1 = strrchr(fname, '/');
  cp2 = strrchr(fname, '\\');
  if (cp1 && cp2) 
  {
    r = (cp1<cp2)?(cp2+1):(cp1+1);
  } 
  else if (cp1) 
  {
    r = cp1+1;
  } 
  else if (cp2) 
  {
    r = cp2+1;
  } 
  else 
  {
    r = fname;
  }
  return r;
}
#else
const char* nullstring = "NULL";
const char* _shortfile(const char *fname)
{
  return nullstring;
}
#endif

BOOL MTX_AddIdentity( const MTX *src, MTX *dst )
{
  unsigned i;
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_Copy( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }
  if( src->isReal )
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      dst->data[i][i] += 1.0;
  }
  else
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      dst->cplx[i][i].re += 1.0;
  }
  return TRUE;  
}

BOOL MTX_AddIdentity_Inplace( MTX *src )
{
  unsigned i;
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( src->isReal )
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->data[i][i] += 1.0;
  }
  else
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->cplx[i][i].re += 1.0;
  }
  return TRUE;  
}


BOOL MTX_MinusIdentity( const MTX *src, MTX *dst )
{
  unsigned i;
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_Copy( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }
  if( src->isReal )
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      dst->data[i][i] -= 1.0;
  }
  else
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      dst->cplx[i][i].re -= 1.0;
  }
  return TRUE;  
}

BOOL MTX_MinusIdentity_Inplace( MTX *src )
{
  unsigned i;
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( src->isReal )
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->data[i][i] -= 1.0;
  }
  else
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->cplx[i][i].re -= 1.0;
  }
  return TRUE;  
}

BOOL MTX_IdentityMinus( const MTX *src, MTX *dst )
{
  unsigned i;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_Copy( src, dst ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }
  if( !MTX_Negate( dst ) )
  {
    MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
    return FALSE;
  } 
  if( dst->isReal )
  {
    for( i = 0; i < dst->nrows && i < dst->ncols; i++ )
      dst->data[i][i] += 1.0;
  }
  else
  {
    for( i = 0; i < dst->nrows && i < dst->ncols; i++ )
      dst->cplx[i][i].re += 1.0;
  }
  return TRUE;  
}

BOOL MTX_IdentityMinus_Inplace( MTX *src )
{
  unsigned i;

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }
  if( !MTX_Negate( src ) )
  {
    MTX_ERROR_MSG( "MTX_Negate returned FALSE." );
    return FALSE;
  } 
  if( src->isReal )
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->data[i][i] += 1.0;
  }
  else
  {
    for( i = 0; i < src->nrows && i < src->ncols; i++ )
      src->cplx[i][i].re += 1.0;
  }
  return TRUE;  
}

BOOL MTX_Hilbert( MTX *src, const unsigned N )
{
  unsigned i;
  unsigned j;

  if( !src )
  {
    MTX_ERROR_MSG( "NULL pointer input." );
    return FALSE;
  }

  if( src->nrows != N || src->ncols != N || !src->isReal )
  {
    if( !MTX_Malloc( src, N, N, TRUE ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned FALSE." );
      return FALSE;
    }
  }

  for( j = 1; j < src->ncols+1; j++ )
  {
    for( i = 1; i < src->nrows+1; i++ )
    {
      src->data[j-1][i-1] = 1.0 / ( i + j - 1.0 );
    }
  }

  return TRUE;
}


BOOL MTX_Swap( MTX* A, MTX *B )
{
  MTX C; // used for temporary pointer value storage. MTX_Free( &C ) is not required!
  if( A == NULL )
  {
    MTX_ERROR_MSG( "A is a NULL Matrix" );
    return FALSE;
  }
  if( B == NULL )
  {
    MTX_ERROR_MSG( "B is a NULL Matrix" );
    return FALSE;
  }

  // easy since pointer values are just exchanged.

  C.isReal = A->isReal;
  C.comment = A->comment;
  C.cplx = A->cplx;
  C.data = A->data;
  C.ncols = A->ncols;
  C.nrows = A->nrows;
  
  A->isReal = B->isReal;
  A->comment = B->comment;
  A->cplx = B->cplx;
  A->data = B->data;
  A->ncols = B->ncols;
  A->nrows = B->nrows;

  B->isReal = C.isReal;
  B->comment = C.comment;
  B->cplx = C.cplx;
  B->data = C.data;
  B->ncols = C.ncols;
  B->nrows = C.nrows;

  // C does not need MTX_Free
  return TRUE;
}


BOOL MTX_LDLt( 
  MTX* src,           //!< src = L*D*Lt
  MTX *L,             //!< src = L*D*Lt
  MTX* d,             //!< src = L*D*Lt, d it the vector diagonal of D.
  BOOL checkSymmetric //!< Option to enable/disable checking the src matrix for symmetry.
  )
{
  int i;
  int j;
  int k;
  int n;
  BOOL isSymmetric = TRUE; // assume true
  double val;
  double maxdif;
  double dtmp;

  if( src == NULL || L == NULL || d == NULL )
  {
    MTX_ERROR_MSG( "An input Matrix is NULL" );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "src Matrix is not square" );
    return FALSE;
  }

  if( !src->isReal )
  {
    MTX_ERROR_MSG( "Complex LDLt not supported yet" );
    return FALSE;
  }

  // get the square dimension
  n = src->ncols;

  if( checkSymmetric )
  {    
    // check symmetric
    for( i = 0; i < n; i++ )
    {
      for( j = i+1; j < n; j++ )
      {
        val = src->data[j][i];
        maxdif = fabs( val )*1e-14;
        // Why 1e-14? it works well for most matrices expected.
        
        dtmp = fabs( val - src->data[i][j] );
        if( dtmp > maxdif )
        {
          isSymmetric = FALSE;
          break;
        }
      }
      if( !isSymmetric )
        break;
    }
    if( !isSymmetric )
    {
      MTX_ERROR_MSG( "src is not symmetric" );
      return FALSE;
    }
  }

  if( !MTX_Copy( src, L ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }

  if( !MTX_Calloc( d, n, 1, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    return FALSE;
  }
  
  // Perform LDLt decomposition without square roots
  // refer http://en.wikipedia.org/wiki/Cholesky_decomposition
  // The algorithm below performs everything inplace on a copy of the input matrix.
  for( j = 0; j < n; j++ )
  {
    dtmp = 0;
    for( k = 0; k < j; k++ )
    {     
      val = L->data[k][j];
      dtmp += val * val * L->data[k][k];
    }
    L->data[j][j] -= dtmp; // the diagonal element of D

    for( i = j + 1; i < n; i++ )
    {
      for( k = 0; k < j; k++ )
        L->data[j][i] -= L->data[k][i] * L->data[k][j] * L->data[k][k];

      if( L->data[j][j] == 0.0 )
        return FALSE;

      L->data[j][i] /= L->data[j][j];
    }
  }

  // form the explicit diagonal vector and L matrix
  for( i = 0; i < n; i++ )
  {
    d->data[0][i] = L->data[i][i];
    L->data[i][i] = 1.0;

    for( j = i+1; j < n; j++ )
    {
      L->data[j][i] = 0.0;
    }
  }

  return TRUE;
}


BOOL MTX_UDUt( 
  MTX* src,           //!< src = U*D*Ut
  MTX *U,             //!< src = U*D*Ut
  MTX* d,             //!< src = U*D*Ut, d it the vector diagonal of D.
  BOOL checkSymmetric //!< Option to enable/disable checking the src matrix for symmetry.
  )
{
  int i;
  int j;
  int k;
  int n;
  BOOL isSymmetric = TRUE; // assume true
  double val;
  double maxdif;
  double dtmp;
  double alpha;
  double beta;

  if( src == NULL || U == NULL || d == NULL )
  {
    MTX_ERROR_MSG( "An input Matrix is NULL" );
    return FALSE;
  }

  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "NULL Matrix" );
    return FALSE;
  }

  if( !MTX_isSquare( src ) )
  {
    MTX_ERROR_MSG( "src Matrix is not square" );
    return FALSE;
  }

  if( !src->isReal )
  {
    MTX_ERROR_MSG( "Complex LDLt not supported yet" );
    return FALSE;
  }

  // get the square dimension
  n = src->ncols;

  if( checkSymmetric )
  {    
    // check symmetric
    for( i = 0; i < n; i++ )
    {
      for( j = i+1; j < n; j++ )
      {
        val = src->data[j][i];
        maxdif = fabs( val )*1e-14;
        // Why 1e-14? it works well for most matrices expected.
        
        dtmp = fabs( val - src->data[i][j] );
        if( dtmp > maxdif )
        {
          isSymmetric = FALSE;
          break;
        }
      }
      if( !isSymmetric )
        break;
    }
    if( !isSymmetric )
    {
      MTX_ERROR_MSG( "src is not symmetric" );
      return FALSE;
    }
  }

  if( !MTX_Copy( src, U ) )
  {
    MTX_ERROR_MSG( "MTX_Copy returned FALSE." );
    return FALSE;
  }
  
  if( !MTX_Calloc( d, n, 1, TRUE ) )
  {
    MTX_ERROR_MSG( "MTX_Calloc returned FALSE." );
    return FALSE;
  }
  
  // Perform UDUt decomposition without square roots, inplace as U is initialized as a copy.

  for( j = n-1; j >= 1; j-- )
  {
    dtmp = U->data[j][j];
    d->data[0][j] = dtmp;
    if( dtmp == 0.0 )
      return FALSE;

    alpha = 1.0/dtmp;
    for( k = 0; k <= j-1; k++ )
    {
      beta = U->data[j][k];
      U->data[j][k] = alpha*beta;
      for( i = 0; i <= k; i++ )
      {
        U->data[k][i] -= beta*U->data[j][i];        
      }
    }
  }
  d->data[0][0] = U->data[0][0];

  // form the explicit U matrix
  for( i = 0; i < n; i++ )
  {
    U->data[i][i] = 1.0;
    for( j = 0; j < i; j++ )
    {
      U->data[j][i] = 0.0;
    }
  }

  /* The algorithm below is from:
  Grewel, M.S (2001), "Kalman Filtering: Theory and Practice using Matlab, Second Edition", 
  John Wiley and Sons, ISBN: 0-471-39254-5, pp. 222

  It performs slower than the above algorithm so it is not used.

  for( j = n-1; j >=0; j-- )
  {
    for( i = j; i >=0; i-- )
    {
      dtmp = src->data[j][i];
      for( k = j+1; k < n; k++ )
      {
        dtmp -= U->data[k][i] * d->data[0][k] * U->data[k][j];
      }
      if( i==j )
      {
        d->data[0][j] = dtmp;
        U->data[j][j] = 1.0;        
      }
      else
      {
        U->data[j][i] = dtmp/d->data[0][j];        
      }
    }
  }
  */
  
  return TRUE;
}



static BOOL MTX_static_gammp(double a, double x, double* ans);
static BOOL MTX_static_gammq(double a, double x, double* ans);
static BOOL MTX_static_gser(double *gamser, double a, double x, double *gln);
static BOOL MTX_static_gcf(double *gammcf, double a, double x, double *gln);
static double MTX_static_gammln(double xx);


BOOL MTX_erf_Inplace( MTX* src )
{
  unsigned j = 0;
  unsigned i = 0;
  double x = 0;  
  
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src matrix is NULL" );
    return FALSE;
  }

  if( !src->isReal )
  {
    MTX_ERROR_MSG( "complex erf is not yet supported." );
    return FALSE;
  }

  for( j = 0; j < src->ncols; j++ )
  {

    for( i = 0; i < src->nrows; i++ )
    {
      x = src->data[j][i];

      if( x < 0.0 )
      {
        if( !MTX_static_gammp( 0.5, x*x, &x ) )
        {
          MTX_ERROR_MSG( "MTX_static_gammp returned FALSE." );
          return FALSE;
        }
        src->data[j][i] = -x;
      }
      else
      {
        if( !MTX_static_gammp( 0.5, x*x, &x ) )
        {
          MTX_ERROR_MSG( "MTX_static_gammp returned FALSE." );
          return FALSE;
        }
        src->data[j][i] = x;
      }
    } 
  }

  return TRUE;
}


BOOL MTX_erfc_Inplace( MTX* src )
{
  unsigned j = 0;
  unsigned i = 0;
  double x = 0;  
  
  if( MTX_isNull( src ) )
  {
    MTX_ERROR_MSG( "src matrix is NULL" );
    return FALSE;
  }

  if( !src->isReal )
  {
    MTX_ERROR_MSG( "complex erfc is not yet supported." );
    return FALSE;
  }

  for( j = 0; j < src->ncols; j++ )
  {

    for( i = 0; i < src->nrows; i++ )
    {
      x = src->data[j][i];

      if( x < 0.0 )
      {
        if( !MTX_static_gammp( 0.5, x*x, &x ) )
        {
          MTX_ERROR_MSG( "MTX_static_gammp returned FALSE." );
          return FALSE;
        }
        src->data[j][i] = 1+x;
      }
      else
      {
        if( !MTX_static_gammq( 0.5, x*x, &x ) )
        {
          MTX_ERROR_MSG( "MTX_static_gammp returned FALSE." );
          return FALSE;
        }
        src->data[j][i] = x;
      }
    } 
  }

  return TRUE;
}



// Returns the incomplete gamma function P(a,x).
//
// reference
// Press, W.H., S.A. Teukolsky, W.T. Vetterling,  and B.P. Flannery (1997), 
// "Numerical Recipes in C", CAMBRIDGE UNIVERSITY PRESS, ISBN 0 521 43108 5,
// pp. 214.
//
// static
BOOL MTX_static_gammp(double a, double x, double* ans)
{
  double gamser=0;
  double gammcf=0;
  double gln=0;
  if( x < 0.0 || a <= 0.0 )
  {
    MTX_ERROR_MSG( "Invalid arguments a or x.");
    return FALSE;
  }
  if( x < (a+1.0) ) 
  { 
    // Use the series representation.
    if( !MTX_static_gser( &gamser, a, x, &gln) )
    {
      MTX_ERROR_MSG( "MTX_static_gser returned FALSE." );
      return FALSE;
    }
    *ans = gamser;
    return TRUE;
  } 
  else 
  { 
    // Use the continued fraction representation
    if( !MTX_static_gcf( &gammcf, a, x, &gln ) )
    {
      MTX_ERROR_MSG( "MTX_static_gcf returned FALSE." );
      return FALSE;
    }
    *ans = 1.0-gammcf; // and take its complement.
    return TRUE;
  }  
}

// Returns the incomplete gamma function Q(a,x) == 1 - P(a,x).
//
// reference
// Press, W.H., S.A. Teukolsky, W.T. Vetterling,  and B.P. Flannery (1997), 
// "Numerical Recipes in C", CAMBRIDGE UNIVERSITY PRESS, ISBN 0 521 43108 5,
// pp. 214.
//
// static
BOOL MTX_static_gammq(double a, double x, double* ans)
{
  double gamser=0;
  double gammcf=0;
  double gln=0;
  if( x < 0.0 || a <= 0.0 ) 
  {
    MTX_ERROR_MSG("if( x < 0.0 || a <= 0.0 ) ");
    return FALSE;
  }
  if( x < (a+1.0) ) 
  { 
    //Use the series representation
    if( !MTX_static_gser(&gamser,a,x,&gln) )
    {
      MTX_ERROR_MSG( "MTX_static_gser returned FALSE." );
      return FALSE;
    }
    *ans = 1.0-gamser; // and take its complement.
    return TRUE;
  } 
  else 
  { 
    //Use the continued fraction representation.
    if( !MTX_static_gcf(&gammcf,a,x,&gln) )
    {
      MTX_ERROR_MSG( "MTX_static_gcf returned FALSE." );
      return FALSE;
    }
    *ans = gammcf;
    return TRUE;
  }
}


// Returns the incomplete gamma function P(a; x) evaluated by its series representation as gamser.
//
// reference
// Press, W.H., S.A. Teukolsky, W.T. Vetterling,  and B.P. Flannery (1997), 
// "Numerical Recipes in C", CAMBRIDGE UNIVERSITY PRESS, ISBN 0 521 43108 5,
// pp. 214.
//
// static 
BOOL MTX_static_gser(double *gamser, double a, double x, double *gln)
{
  int n;
  const int itmax = 10000; // Maximum allowed number of iterations.
  const double eps = DBL_EPSILON; // Relative accuracy.  
  
  // in float.h
  // #define DBL_EPSILON     2.2204460492503131e-016 /* smallest such that 1.0+DBL_EPSILON != 1.0 */  
  
  double sum;
  double del;
  double ap;

  // gamser and gln must be valid pointers
  // since this is an internal static function they will not be checked.
  
  *gln = MTX_static_gammln(a);

  if( x <= 0.0 )  
  {
    if( x < 0.0 )
    {
      MTX_ERROR_MSG( "x less than 0.");
      return FALSE;
    }
    *gamser=0.0;
    return TRUE;
  }
  else 
  {
    ap = a;
    del = sum = 1.0/a;
    for( n = 1; n <= itmax; n++) 
    {
      ap += 1.0;
      del *= x/ap;
      sum += del;
      if( fabs(del) < fabs(sum)*eps )
      {
        *gamser = sum * exp( -x+a*log(x)-(*gln) );
        return TRUE;
      }
    }
    MTX_ERROR_MSG( "a is too large for the number of iterations in MTX_static_gser." );    
    return FALSE;
  }
}


// Determines the incomplete gamma function Q(a,x) evaluated by 
// its continued fraction representation as gammcf.
// Also returns ln( Gamma(a) ) as gln.
//
// reference
// Press, W.H., S.A. Teukolsky, W.T. Vetterling,  and B.P. Flannery (1997), 
// "Numerical Recipes in C", CAMBRIDGE UNIVERSITY PRESS, ISBN 0 521 43108 5,
// pp. 214.
//
//static 
BOOL MTX_static_gcf(double *gammcf, double a, double x, double *gln)
{
  const int itmax = 10000; // Maximum allowed number of iterations.
  const double eps = DBL_EPSILON; // Relative accuracy (in float.h).
  const double fpmin  = DBL_MIN; // Number near the smallest representable double floating-point number (in float.h).

  // in float.h
  // #define DBL_EPSILON     2.2204460492503131e-016 /* smallest such that 1.0+DBL_EPSILON != 1.0 */  
  // #define DBL_MIN 2.2250738585072014e-308 /* min positive value */
  
  int i;
  double an,b,c,d,del,h;
  
  *gln = MTX_static_gammln(a);

  b = x+1.0-a; // Set up for evaluating continued fraction by modified Lentz's method with b0 = 0.
  c = 1.0/fpmin;
  d = 1.0/b;
  h = d;
  for( i = 1; i <= itmax; i++ ) 
  {
    // Iterate to convergence.
    an = -i*(i-a);
    b += 2.0;
    d = an*d+b;
    if( fabs(d) < fpmin ) 
      d = fpmin;
    c = b+an/c;
    if( fabs(c) < fpmin )
      c = fpmin;
    d = 1.0/d;
    del = d*c;
    h *= del;
    if( fabs(del-1.0) < eps ) 
      break;
  }
  if( i > itmax )
  {
    MTX_ERROR_MSG("a too large, too few iterations");
    return FALSE;
  }
  *gammcf = exp(-x+a*log(x)-(*gln))*h; // Put factors in front.
  return TRUE;
}


// returns the value of ln(Gamma(xx)) for xx > 0  
//
// reference
// Press, W.H., S.A. Teukolsky, W.T. Vetterling,  and B.P. Flannery (1997), 
// "Numerical Recipes in C", CAMBRIDGE UNIVERSITY PRESS, ISBN 0 521 43108 5,
// pp. 214.
//
//static 
double MTX_static_gammln(double xx)
{
  double x,y,tmp,ser;
  static double cof[6] = {76.18009172947146,-86.50532032941677,24.01409824083091,-1.231739572450155,0.1208650973866179e-2,-0.5395239384953e-5};
  int j;
  y = x = xx;
  tmp = x+5.5;
  tmp -= (x+0.5)*log(tmp);
  ser = 1.000000000190015;
  for( j = 0; j < 6; j++ )
  {
    y += 1.0;
    ser += cof[j]/y;
  }
  x = -tmp+log(2.5066282746310005*ser/x);
  return x;
}



/*
   for( i = 0; i < src->ncols; i++ )
    {  
      z = fabs(src->data[j][i]);
      
      t = 1.0 / (1.0 + 0.5*z);

      ans = t * exp(-z*z-1.26551223+t*(1.00002368+t*(0.37409196+t*(0.09678418+
            t*(-0.18628806+t*(0.27886807+t*(-1.13520398+t*(1.48851587+
            t*(-0.82215223+t*0.17087277)))))))));

      if( x >= 0.0 )
        src->data[j][i] = ans;
      else
        src->data[j][i] = 2.0 - ans;
    }
    */
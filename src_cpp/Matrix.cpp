/**
\file     Matrix.h
\brief    The Zenautics Matrix Class
\author   Glenn D. MacGougan (GDM)
\date     2009-02-08
\version  0.07 Beta

\b Version \b Information \n
This is the open source version (BSD license). The Professional Version
is avaiable via http://www.zenautics.com. The Professional Version
is highly optimized using SIMD for INTEL processors and includes 
optimization for multi-code processors.

\b License \b Information \n
Copyright (c) 2008, Glenn D. MacGougan, Zenautics Technologies Inc. \n

Redistribution and use in source and binary forms, with or without
modification, of the specified files is permitted provided the following 
conditions are met: \n

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

\b NOTES: \n
This code was developed using rigourous unit testing for every function 
and operation. Despite any rigorous development process, bugs are
inevitable. Please report bugs and suggested fixes to glenn @ zenautics.com.\n

\b Preprocessor Defines \n
The implementation of exception handling and the use of namespace is
not standardized among older compilers, the following deines may be 
necessary. \n
#define _MATRIX_NO_NAMESPACE // removes namespace support. \n
#define _MATRIX_NO_EXCEPTION // removes exception handling support. \n
*/

#include <stdlib.h>
#include <string.h>

#include "Matrix.h"
#include "cmatrix.h"

// deal with msvc empty projects
#ifndef WIN32
  #ifdef _WIN32
    #define WIN32
  #endif
#endif

#ifndef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif


#ifndef DEG2RAD
#define DEG2RAD   (0.017453292519943295769236907684886)  //!< PI/180.0
#endif

#ifndef RAD2DEG
#define RAD2DEG   (57.295779513082320876798154814105)    //!< 180.0/PI
#endif

namespace Zenautics
{
  /// A static double  value used for bad referencing. i.e. give me element 10 of 1x9 vector.
  static double staticglobal_BadDouble = 0.0;

  // A boolean used to ensure initialization of the mtx engine.
  bool Matrix::m_IsMTXInitialized = false;


#ifndef _MATRIX_NO_EXCEPTION

  MatrixException::MatrixException( const char* msg )
  {
    unsigned msgLength = 0;
    if( msg == NULL )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      strcpy_s( m_msg, 256, "Unknown Matrix Exception" );
#else
      strcpy( m_msg, "Unknown Matrix Exception" );
#endif
    }
    else
    {
      msgLength = (unsigned)strlen( msg );
      // note 255 here, not 256, 
      // just in case msgLength is 255 and we add '\0' to the end of m_msg.
#ifndef _CRT_SECURE_NO_DEPRECATE
      if( msgLength < 255 )
      {
        strncpy_s( m_msg, 256, msg, msgLength );
        m_msg[msgLength] = '\0';
      }
      else
      {
        strncpy_s( m_msg, 256, msg, 255 );
        m_msg[255] = '\0';
      }
#else
      if( msgLength < 255 )
      {
        strncpy( m_msg, msg, msgLength );
        m_msg[msgLength] = '\0';
      }
      else
      {
        strncpy( m_msg, msg, 255 );
        m_msg[255] = '\0';
      }
#endif
    }
    m_ExceptionString = m_msg;
  }

  MatrixException::MatrixException(const MatrixException& matrix_exception)
  {
    // This will copy only the matrix exception string.
    m_ExceptionString = matrix_exception.m_ExceptionString;
  }

  std::string MatrixException::GetExceptionMessage()
  {
    return m_ExceptionString;
  }

  MatrixException::operator const char*()
  {
    return m_ExceptionString.c_str();
  }
#endif


  // default constructor
  Matrix::Matrix()
    :m_MatrixElement(m_Matrix)
  {
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    MTX_Init( &m_Matrix );
  }


  // destructor
  Matrix::~Matrix()
  { 
    if( !MTX_Free( &m_Matrix ) )
    {
      MatrixError( "~Matrix", "Unable to free memory properly" );
    }
  }


  // vector style constructor
  Matrix::Matrix( const unsigned nrows )
    :m_MatrixElement(m_Matrix)
  { 
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    MTX_Init( &m_Matrix );
    if( !MTX_Calloc( &m_Matrix, nrows, 1, true ) )
    {
      char msg[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s( msg, 128, "Unable to allocate enough memory for Matrix as vector(%d)", nrows );
#else
      sprintf( msg, "Unable to allocate enough memory for Matrix as vector(%d)", nrows );
#endif
      MatrixError( "Matrix", msg );
    }
  }


  // matrix style constructor
  Matrix::Matrix( const unsigned nrows, const unsigned ncols, const bool isReal )
    :m_MatrixElement(m_Matrix)
  { 
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    MTX_Init( &m_Matrix );
    if( !MTX_Calloc( &m_Matrix, nrows, ncols, isReal ) )
    {
      char msg[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      if( isReal )
        sprintf_s( msg, 128, "Unable to allocate enough memory for Matrix(%d,%d)", nrows, ncols );
      else
        sprintf_s( msg, 128, "Unable to allocate enough memory for complex Matrix(%d,%d)", nrows, ncols );
#else
      if( isReal )
        sprintf( msg, "Unable to allocate enough memory for Matrix(%d,%d)", nrows, ncols );
      else
        sprintf( msg, "Unable to allocate enough memory for complex Matrix(%d,%d)", nrows, ncols );
#endif

      MatrixError( "Matrix", msg );
    }
  }


  // constructor reading data from file   
  Matrix::Matrix( const char* path, bool& itWorked )
    :m_MatrixElement(m_Matrix)
  {
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    MTX_Init( &m_Matrix );

    if( MTX_ReadFromFile( &m_Matrix, path ) )
      itWorked = true;
    else
      itWorked = false;  
  }

  // copy constructor
  Matrix::Matrix( const Matrix& mat )
    :m_MatrixElement(m_Matrix)
  {
    MTX_Init( &m_Matrix );
    if( !MTX_Copy( &(mat.m_Matrix), &m_Matrix ) )
    {
      MatrixError( "Matrix", "Copy constructor failed to copy input matrix." );
    }
  }

  // copy from a static matrix
  Matrix::Matrix(const double mat[], const unsigned nrows, const unsigned ncols )
    :m_MatrixElement(m_Matrix)
  {
    MTX_Init( &m_Matrix );
    if( mat == NULL )
    {
      MatrixError( "Matrix", "Input static double array(matrix) pointer is NULL" );
    }
    if( !MTX_SetFromStaticMatrix( &m_Matrix, mat, nrows, ncols ) )
    {
      MatrixError( "Matrix", "Failed to set the matrix from a static double array(matrix)" );
    }
  }

  // copy from a matrix string 
  Matrix::Matrix(const char* strMatrix)
    :m_MatrixElement(m_Matrix)
  {
    MTX_Init( &m_Matrix );
    if( !MTX_SetFromMatrixString( &m_Matrix, strMatrix ) )
    {
      MatrixError( "Matrix = \"string matrix\"", "Unable to set matrix from the string specified." );
    }    
  }


  // assignment operator (constructor)
  Matrix& Matrix::operator= (const Matrix& mat)
  {
    // trap assignment to self
    if( this == &mat )
      return *this;

    if( !MTX_Copy( &mat.m_Matrix, &m_Matrix ) )
    {
      MatrixError( "operator=", "Failed to copy input matrix" );
    }

    return *this;
  }


  Matrix& Matrix::operator= (const double value)
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, true ) )
    {
      MatrixError( "operator=double", "Unable to redimension to 1x1." );
    }

    if( !MTX_SetValue( &m_Matrix, 0, 0, value ) )
    {
      MatrixError( "operator=double", "Unable to set double value." );
    }

    return *this;
  }

  Matrix& Matrix::operator= (const std::complex<double> value)
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, false ) )
    {
      MatrixError( "operator=std::complex<double>", "Unable to redimension to 1x1." );
    }

    if( !MTX_SetComplexValue( &m_Matrix, 0, 0, value.real(), value.imag() ) )
    {
      MatrixError( "operator=std::complex<double>", "Unable to set the value." );
    }

    return *this;
  }

  Matrix& Matrix::operator=(const char* strMatrix)
  {
    if( !MTX_SetFromMatrixString( &m_Matrix, strMatrix ) )
    {
      MatrixError( "operator=string", "Unable to set matrix from the string specified." );
    }
    return *this;
  }

  bool Matrix::Clear()
  {
    if( MTX_Free( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_FREE returned false." );
      return false;    
    }
  }

  void Matrix::MatrixError( const char* error )
  {
    Clear();
    StaticMatrixError( error );
  }

  void Matrix::MatrixError( const char* function, const char* error )
  {
    Clear();
    StaticMatrixError( function, error );
  }


  // static
  void Matrix::StaticMatrixError( const char* error )
  {
    StaticMatrixError( "", error );
  }

  // static
  void Matrix::StaticMatrixError( const char* function, const char* error )
  {
    char msg[256];
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( strstr(function,"operator") != NULL )
      sprintf_s( msg, 256, "\nMatrix::%s, Error:\n%s\n", function, error );   
    else
      sprintf_s( msg, 256, "\nMatrix::%s(), Error:\n%s\n", function, error );
#else
    if( strstr(function,"operator") != NULL )
      sprintf( msg, "\nMatrix::%s, Error:\n%s\n", function, error );   
    else
      sprintf( msg, "\nMatrix::%s(), Error:\n%s\n", function, error );
#endif

#ifndef _MATRIX_NO_EXCEPTION

    throw MatrixException(msg);
    return;

#else

    printf( "%s\r\n", msg );   

    // no choice but to call exit!
    exit(1);

#endif    
  }


  bool Matrix::isEmpty() const
  {
    if( MTX_isNull( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Matrix::isConformal(const Matrix& mat) const
  {
    if( MTX_isConformalForMultiplication( &m_Matrix, &mat.m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_isConformalForMultiplication returned false." );
      return false;
    }
  }

  bool Matrix::isSameSize(const Matrix& mat) const
  {
    if( MTX_isSameSize( &m_Matrix, &mat.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_isSameSize returned false." );
      return false;
    }
  }

  bool Matrix::isSquare() const
  {
    if( MTX_isSquare( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_isSquare returned false." );
      return false;
    }
  }

  unsigned  Matrix::GetNrCols() const
  {
    return m_Matrix.ncols;
  }

  unsigned  Matrix::ncols() const
  {
    return m_Matrix.ncols;
  }

  unsigned Matrix::GetNrElems() const
  {
    return m_Matrix.ncols*m_Matrix.nrows;
  }

  unsigned Matrix::nelems() const
  {
    return m_Matrix.ncols*m_Matrix.nrows;
  }

  unsigned Matrix::GetNrRows() const
  {
    return m_Matrix.nrows;
  }

  unsigned Matrix::nrows() const
  {
    return m_Matrix.nrows;
  }

  unsigned Matrix::GetLength() const
  {
    if( m_Matrix.nrows > m_Matrix.ncols )
      return m_Matrix.nrows;
    else
      return m_Matrix.ncols;
  }

  double Matrix::real(const unsigned row, const unsigned col)
  {
    if( IndexCheck(row,col) )
    {
      if( m_Matrix.isReal )
      {
        return m_Matrix.data[col][row];
      }
      else
      {
        return m_Matrix.cplx[col][row].re;
      }
    }
    else
    {
      return 0.0;
    }
  }

  double Matrix::real(const unsigned index)
  {
    unsigned row = 0;
    unsigned col = 0;

    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 )
      {
        row = index;
      }
      else if( m_Matrix.nrows == 1 )
      {
        col = index;
      }
      else
      {
        // access the matrix as a singular column array
        col = index / m_Matrix.nrows;
        row = index - col*m_Matrix.nrows;
      }
      if( m_Matrix.isReal )
      {
        return m_Matrix.data[col][row];
      }
      else
      {
        return m_Matrix.cplx[col][row].re;
      }
    }
    else
    {
      return 0.0;
    }
  }

  double Matrix::imag(const unsigned row, const unsigned col)
  {
    if( IndexCheck(row,col) )
    {
      if( m_Matrix.isReal )
      {
        return 0.0;
      }
      else
      {
        return m_Matrix.cplx[col][row].im;
      }
    }
    else
    {
      return 0.0;
    }
  }

  double Matrix::imag(const unsigned index)
  {
    unsigned row = 0;
    unsigned col = 0;

    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 )
      {
        row = index;
      }
      else if( m_Matrix.nrows == 1 )
      {
        col = index;
      }
      else
      {
        // access the matrix as a singular column array
        col = index / m_Matrix.nrows;
        row = index - col*m_Matrix.nrows;
      }
      if( m_Matrix.isReal )
      {
        return 0.0;
      }
      else
      {
        return m_Matrix.cplx[col][row].im;
      }
    }
    else
    {
      return 0.0;
    }
  }

  bool Matrix::isStoredAsComplex()
  {
    if( m_Matrix.isReal )
      return false;
    else
      return true;
  }

  /// Is this a real matrix for accessing by (row,col) operator? e.g. double d = A(0,4).
  bool Matrix::isReal()
  {
    BOOL isItReal = 0;
    
    // not checking return value
    MTX_isReal(&m_Matrix,&isItReal);

    if( isItReal )
      return true;
    else
      return false;    
  }

  /// Is this a complex matrix for accessing by [row][col] operators? e.g. stComplex d = A[0][4].
  bool Matrix::isComplex()
  {
    return !isReal();
  }

  bool Matrix::isVector()
  {
    if( m_Matrix.nrows == 1 )
      return true;
    if( m_Matrix.ncols == 1 )
      return true;

    // otherwise
    return false;
  }

  bool Matrix::ReadFromFile( const char *path )
  {
    if( MTX_ReadFromFile( &m_Matrix, path ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ReadFromFile returned false." );
      return false;
    }
  }


  bool Matrix::ReadFromFile( std::string path )
  {
    return ReadFromFile( path.c_str() );
  }


  bool Matrix::Copy( Matrix& src )
  {
    if( MTX_Copy( &src.m_Matrix, &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Copy returned false." );
      return false;
    }
  }

  bool Matrix::Copy( const double& value )
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, true ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned false." );
      return false;
    }

    if( MTX_SetValue( &m_Matrix, 0, 0, value ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_SetValue returned false." );      
      return false;
    }
  }

  bool Matrix::Copy( const std::complex<double>& cplx )
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, false ) )
    {
      MTX_ERROR_MSG( "MTX_Malloc returned false." );      
      return false;
    }

    if( MTX_SetComplexValue( &m_Matrix, 0, 0, cplx.real(), cplx.imag() ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_SetComplexValue returned false." );      
      return false;
    }
  }

  bool Matrix::Save( const char* path )
  {
    if( MTX_SaveCompressed( &m_Matrix, path ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SaveCompressed returned false." );      
      return false;
    }
  }

  bool Matrix::Save( std::string path )
  {
    return Save( path.c_str() );
  }

  bool Matrix::Print( const char *path, const unsigned precision, bool append )
  {
    if( MTX_PrintAutoWidth( &m_Matrix, path, precision, append ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PrintAutoWidth returned false." );      
      return false;
    }
  }

  bool Matrix::Print( std::string path, const unsigned precision, bool append ) 
  {
    return Print( path.c_str(), precision );
  }

  bool Matrix::PrintStdout( const unsigned precision )
  {
    if( m_Matrix.ncols == 0 || m_Matrix.nrows == 0 )
    {
      printf( "\n" );
      return true;
    }
    if( MTX_PrintStdoutAutoWidth( &m_Matrix, precision ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_PrintStdoutAutoWidth returned false." );      
      return false;
    }
  }

  bool Matrix::PrintToBuffer( char* buffer, const unsigned maxlength, const unsigned precision )
  {
    if( MTX_PrintAutoWidth_ToBuffer( &m_Matrix, buffer, maxlength, precision ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PrintAutoWidth_ToBuffer returned false." );      
      return false;
    }
  }

  bool Matrix::PrintFixedWidth( const char* path, const unsigned width, const unsigned precision, bool append )
  {
    if( MTX_Print( &m_Matrix, path, width, precision, append ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Print returned false." );            
      return false;
    }
  }

  bool Matrix::PrintFixedWidth( std::string path, const unsigned width, const unsigned precision, bool append )
  {
    return PrintFixedWidth( path.c_str(), width, precision, append );
  }

  bool Matrix::PrintFixedWidthToBuffer( char* buffer, const unsigned maxlength, const unsigned width, const unsigned precision )
  {
    if( MTX_Print_ToBuffer( &m_Matrix, buffer, maxlength, width, precision ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Print_ToBuffer returned false." );            
      return false;
    }
  }

  bool Matrix::PrintDelimited( const char *path, const unsigned precision, const char delimiter, bool append )
  {
    if( MTX_PrintDelimited( &m_Matrix, path, precision, delimiter, append ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PrintDelimited returned false." );            
      return false;
    }
  }

  bool Matrix::PrintDelimited( std::string path, const unsigned precision, const char delimiter, bool append )
  {
    return PrintDelimited( path.c_str(), precision, delimiter, append );
  }
    

  bool Matrix::PrintDelimitedToBuffer( char *buffer, const unsigned maxlength, const unsigned precision, const char delimiter )
  {
    if( MTX_PrintDelimited_ToBuffer( &m_Matrix, buffer, maxlength, precision, delimiter ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PrintDelimited_ToBuffer returned false." );            
      return false;
    }
  }

  bool Matrix::PrintRowToString( const unsigned row, char *buffer, const unsigned maxlength, const int width, const int precision )
  {
    if( MTX_PrintRowToString( &m_Matrix, row, buffer, maxlength, width, precision ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_PrintRowToString returned false." );            
      return false;
    }
  }


  bool Matrix::RemoveColumn( const unsigned col )
  {
    if( MTX_RemoveColumn( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RemoveColumn returned false." );            
      return false;
    }
  }

  bool Matrix::RemoveColumnsAfterIndex( const unsigned col )
  {
    if( MTX_RemoveColumnsAfterIndex( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RemoveColumnsAfterIndex returned false." );            
      return false;
    }
  }

  bool Matrix::RemoveRowsAndColumns( const unsigned nrows, const unsigned rows[], const unsigned ncols, const unsigned cols[] )
  {
    if( MTX_RemoveRowsAndColumns( &m_Matrix, nrows, rows, ncols, cols ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_RemoveRowsAndColumns returned false." );            
      return false;
    }
  }

  bool Matrix::InsertColumn( const Matrix &src, const unsigned dst_col, const unsigned src_col )
  {
    if( MTX_InsertColumn( &m_Matrix, &src.m_Matrix, dst_col, src_col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_InsertColumn returned false." );            
      return false;
    }
  }

  bool Matrix::AddColumn( const Matrix &src, const unsigned src_col )
  {
    if( MTX_AddColumn( &m_Matrix, &src.m_Matrix, src_col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_AddColumn returned false." );            
      return false;
    }
  }

  bool Matrix::Concatonate( const Matrix &src )
  {
    if( MTX_Concatonate( &m_Matrix, &src.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Concatonate returned false." );
      return false;
    }
  }

  bool Matrix::Redim( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Redim( &m_Matrix, nrows, ncols ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Redim returned false." );
      return false;
    }
  }

  bool Matrix::Resize( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Resize( &m_Matrix, nrows, ncols, m_Matrix.isReal ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Resize returned false." );
      return false;
    }
  }


  bool Matrix::SetFromStaticMatrix( const double mat[], const unsigned nrows, const unsigned ncols )
  {
    if( MTX_SetFromStaticMatrix( &m_Matrix, mat, nrows, ncols ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SetFromStaticMatrix returned false." );
      return false;
    }
  }

  bool Matrix::SetFromMatrixString(const char* strMatrix)
  {
    if( MTX_SetFromMatrixString( &m_Matrix, strMatrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_SetFromStaticMatrix returned false." );
      return false;
    }
  }

  bool Matrix::CopyColumn( const unsigned src_col, Matrix &dst )
  {
    if( MTX_CopyColumn( &m_Matrix, src_col, &dst.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_CopyColumn returned false." );
      return false;
    }
  }

  bool Matrix::InsertSubMatrix( const Matrix &src, const unsigned dst_row, const unsigned dst_col )
  {
    if( MTX_InsertSubMatrix( &m_Matrix, &src.m_Matrix, dst_row, dst_col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_InsertSubMatrix returned false." );
      return false;
    }
  }

  bool Matrix::ExtractSubMatrix( 
    Matrix &dst,             //!< The destination matrix to contain the submatrix.
    const unsigned from_row, //!< The zero-based index for the from row.
    const unsigned from_col, //!< The zero-based index for the from column.
    const unsigned to_row,   //!< The zero-based index for the to row.
    const unsigned to_col    //!< The zero-based index for the to column.
    )
  {
    if( MTX_ExtractSubMatrix( &m_Matrix, &dst.m_Matrix, from_row, from_col, to_row, to_col ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ExtractSubMatrix returned false." );
      return false;
    }
  }

  bool Matrix::Zero()
  {
    if( MTX_Zero( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Zero returned false." );
      return false;
    }
  }

  bool Matrix::ZeroColumn( const unsigned col )
  {
    if( MTX_ZeroColumn( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ZeroColumn returned false." );
      return false;
    }
  }

  bool Matrix::ZeroRow( const unsigned row )
  {
    if( MTX_ZeroRow( &m_Matrix, row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ZeroRow returned false." );
      return false;
    }
  }

  bool Matrix::Swap( Matrix &M )
  {
    if( MTX_Swap( &m_Matrix, &M.m_Matrix ) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Matrix::Fill( const double value )
  {
    if( MTX_Fill( &m_Matrix, value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Fill returned false." );
      return false;
    }
  }

  bool Matrix::FillColumn( const unsigned col, const double value )
  {
    if( MTX_FillColumn( &m_Matrix, col, value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "FillColumn returned false." );
      return false;
    }
  }

  bool Matrix::FillRow( const unsigned row, const double value )
  {
    if( MTX_FillRow( &m_Matrix, row, value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_FillRow returned false." );
      return false;
    }
  }

  bool Matrix::FlipColumn( const unsigned col )
  {
    if( MTX_FlipColumn( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_FlipColumn returned false." );
      return false;
    }
  }

  bool Matrix::FlipRow( const unsigned row )
  {
    if( MTX_FlipRow( &m_Matrix, row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_FlipRow returned false." );
      return false;
    }
  }

  bool Matrix::Identity()
  {
    if( MTX_Identity( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Identity returned false." );
      return false;
    }
  }

  bool Matrix::Identity(const unsigned dimension)
  {
    if( MTX_Malloc( &m_Matrix, dimension, dimension, true ) )
    {
      if( MTX_Identity( &m_Matrix ) )
      {
        return true;
      }
      else
      {
        MTX_ERROR_MSG( "MTX_Identity returned false." );
        return false;
      }
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Malloc returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_ForceSymmetric()
  {
    if( MTX_ForceSymmetric( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ForceSymmetric returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Transpose()
  {
    if( MTX_TransposeInplace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TransposeInplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Round( const unsigned precision )
  {
    if( MTX_Round( &m_Matrix, precision ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Round returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Floor()
  {
    if( MTX_Floor( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Floor returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Ceil()
  {
    if( MTX_Ceil( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Ceil returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_erf()
  {
    if( MTX_erf_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_erf_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_erfinv()
  {
    if( MTX_erfinv_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_erfinv_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_erfc()
  {
    if( MTX_erfc_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_erfc_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Fix()
  {
    if( MTX_Fix( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Fix returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_AddScalar( const double scalar )
  {
    if( MTX_Add_Scalar( &m_Matrix, scalar ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Add_Scalar returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SubtractScalar( const double scalar )
  {
    if( MTX_Subtract_Scalar( &m_Matrix, scalar ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Subtract_Scalar returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_MultiplyScalar( const double scalar )
  {
    if( MTX_Multiply_Scalar( &m_Matrix, scalar ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Multiply_Scalar returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_DivideScalar( const double scalar )
  {
    if( MTX_Divide_Scalar( &m_Matrix, scalar ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Divide_Scalar returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_PowerScalar( const double scalar )
  {
    if( MTX_PowInplace( &m_Matrix, scalar, 0.0 ) ) 
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_PowInplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_AddScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Add_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Add_ScalarComplex returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SubtractScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Subtract_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Subtract_ScalarComplex returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_MultiplyScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Multiply_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Multiply_ScalarComplex returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_DivideScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Divide_ScalarComplex( &m_Matrix,  cplx.real(), cplx.imag() ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Divide_ScalarComplex returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_PowerScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_PowInplace( &m_Matrix, cplx.real(), cplx.imag() ) ) 
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_PowInplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Abs()
  {
    if( MTX_Abs( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Abs returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_acos()
  {
    if( MTX_acos( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_acos returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_acosd()
  {
    if( MTX_acos( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
      {
        return true;
      }
      else
      {
        MTX_ERROR_MSG( "MTX_Multiply_Scalar returned false." );
        return false;
      }
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_acos returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_acosh()
  {
    if( MTX_acosh( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_acosh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_angle()
  {
    if( MTX_angle( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_angle returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_asin()
  {
    if( MTX_asin( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_asin returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_asind()
  {
    if( MTX_asin( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
      {
        return true;
      }
      else
      {
        MTX_ERROR_MSG( "MTX_Multiply_Scalar returned false." );
        return false;
      }
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_asin returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_asinh()
  {
    if( MTX_asinh( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_asinh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_atan()
  {
    if( MTX_atan( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_atan returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_atand()
  {
    if( MTX_atan( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
      {
        return true;
      }
      else
      {
        MTX_ERROR_MSG( "MTX_Multiply_Scalar returned false." );
        return false;
      }
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_atan returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_atanh()
  {
    if( MTX_atanh( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_atanh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Sqr()
  {
    if( MTX_Sqr( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Sqr returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Sqrt()
  {
    if( MTX_Sqrt( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Sqrt returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Exp()
  {
    if( MTX_Exp( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Exp returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Ln()
  {
    if( MTX_Ln( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Ln returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Increment()
  {
    if( MTX_Increment( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Increment returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Decrement()
  {
    if( MTX_Decrement( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Decrement returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Add( const Matrix &B )
  {
    if( MTX_Add_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Add_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Subtract( const Matrix &B )
  {
    if( MTX_Subtract_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Subtract_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_PreMultiply( const Matrix &B )
  {
    if( MTX_PreMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PreMultiply_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_TranposePreMultiply( const Matrix &B )
  {
    if( MTX_TransposePreMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TransposePreMultiply_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_PostMultiply( const Matrix &B )
  {
    if( MTX_PostMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PostMultiply_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_PostMultiplyTranspose( const Matrix &B )
  {
    if( MTX_PostMultiplyTranspose_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_PostMultiplyTranspose_Inplace returned false." );
      return false;
    }
  }  

  bool Matrix::Inplace_DotMultiply( const Matrix &B )
  {
    if( MTX_DotMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_DotMultiply_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_DotDivide( const Matrix &B )
  {
    if( MTX_DotDivide_Inplace( &m_Matrix, &B.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_DotDivide_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortAscending()
  {
    if( MTX_SortAscending( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortAscending returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortDescending()
  {
    if( MTX_SortDescending( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortDescending returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortColumnAscending( const unsigned col )
  {
    if( MTX_SortColumnAscending( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortColumnAscending returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortColumnDescending( const unsigned col )
  {
    if( MTX_SortColumnDescending( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortColumnDescending returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortColumnIndexed( const unsigned col, Matrix &Index )
  {
    if( MTX_SortColumnIndexed( &m_Matrix, col, &Index.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortColumnIndexed returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_SortByColumn( const unsigned col )
  {
    if( MTX_SortByColumn( &m_Matrix, col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_SortByColumn returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_Invert()
  {
    if( MTX_InvertInPlace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_InvertInPlace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_InvertRobust()
  {
    if( MTX_InvertInPlaceRobust( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_InvertInPlaceRobust returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_LowerTriangularInverse()
  {
    if( MTX_LowerTriangularInverseInplace( &m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_LowerTriangularInverseInplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_FFT()
  {
    if( MTX_FFT_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_FFT_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_FFT2()
  {
    if( MTX_FFT2_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_FFT2_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_IFFT()
  {
    if( MTX_IFFT_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_IFFT_Inplace returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_IFFT2()
  {
    if( MTX_IFFT2_Inplace( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_IFFT2_Inplace returned false." );
      return false;
    }
  }
  
  bool Matrix::Add( const Matrix &B, const Matrix &C )
  {
    if( MTX_Add( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Add returned false." );
      return false;
    }
  }

  bool Matrix::Subtract( const Matrix &B, const Matrix &C )
  {
    if( MTX_Subtract( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Subtract returned false." );
      return false;
    }
  }

  bool Matrix::Multiply( const Matrix &B, const Matrix &C )
  {
    if( MTX_Multiply( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Multiply returned false." );
      return false;
    }
  }

  bool Matrix::TransposeMultiply( const Matrix &B, const Matrix &C )
  {
    if( MTX_TransposeMultiply( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TranposeMultiply returned false." );
      return false;
    }
  }

  bool Matrix::MultiplyTranspose( const Matrix &B, const Matrix &C )
  {
    if( MTX_MultiplyTranspose( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MultiplyTranspose returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_abs()
  {
    if( MTX_Abs( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Abs returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_colon( double start, double increment, double end )
  {
    if( MTX_Colon( &m_Matrix, start, increment, end) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Colon returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_conj()
  {
    if( MTX_Conjugate( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Conjugate returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_cos()
  {
    if( MTX_cos( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_cos returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_cosh()
  {
    if( MTX_cosh( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_cosh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_cot()
  {
    if( MTX_cot( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_cot returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_coth()
  {
    if( MTX_coth( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_coth returned false." );
      return false;
    }
  }
  
  bool Matrix::Inplace_imag()
  {
    if( MTX_ConvertComplexToImag( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ConvertComplexToImag returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_exp()
  {
    if( MTX_Exp( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Exp returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_eye( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Eye( &m_Matrix, nrows, ncols ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Eye returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_log2()
  {
    if( MTX_Ln( &m_Matrix ) )
    {
      if( MTX_Divide_Scalar( &m_Matrix, log(2.0) ) )
      {
        return true;
      }
      else
      {
        return false;
        MTX_ERROR_MSG( "MTX_Divide_Scalar returned false." );      
      }
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Ln returned false." );      
      return false;
    }
  }

  bool Matrix::Inplace_log10()
  {
    if( MTX_Ln( &m_Matrix ) )
    {
      if( MTX_Divide_Scalar( &m_Matrix, log(10.0) ) )
      {
        return true;
      }
      else
      {
        MTX_ERROR_MSG( "MTX_Divide_Scalar returned false." );
        return false;
      }
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Ln returned false." );
      return false;
    }
  }


  bool Matrix::Inplace_ones( const unsigned nrows, const unsigned ncols )
  {
    if( m_Matrix.nrows == nrows && m_Matrix.ncols == ncols && m_Matrix.isReal )
    { 
      if( !MTX_Fill( &m_Matrix, 1.0 ) )
      {
        MTX_ERROR_MSG( "MTX_Fill returned false." );
        return false;
      }
    }
    else
    {
      if( !MTX_Malloc( &m_Matrix, nrows, ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned false." );
        return false;
      }
      if( !MTX_Fill( &m_Matrix, 1.0 ) )
      {
        MTX_ERROR_MSG( "MTX_Fill returned false." );
        return false;
      }
    }
    return true;
  }

  bool Matrix::Inplace_real()
  {
    if( MTX_ConvertComplexToReal( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_ConvertComplexToReal returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_rand( const unsigned nrows, const unsigned ncols, const unsigned seed )
  {
    if( MTX_rand( &m_Matrix, nrows, ncols, seed ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_rand returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_randn( const unsigned nrows, const unsigned ncols, const unsigned seed )
  {
    if( MTX_randn( &m_Matrix, nrows, ncols, seed ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_randn returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_sin()
  {
    if( MTX_sin( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_sin returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_sinc()
  {
    if( MTX_sinc( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_sinc returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_sinh()
  {
    if( MTX_sinh( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_sinh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_sqrt()
  {
    if( MTX_Sqrt( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_Sqrt returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_tan()
  {
    if( MTX_tan( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_tan returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_tanh()
  {
    if( MTX_tanh( &m_Matrix ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_tanh returned false." );
      return false;
    }
  }

  bool Matrix::Inplace_zeros( const unsigned nrows, const unsigned ncols )
  {
    if( m_Matrix.nrows == nrows && m_Matrix.ncols == ncols && m_Matrix.isReal )
    { 
      if( !MTX_Fill( &m_Matrix, 0.0 ) )
      {
        MTX_ERROR_MSG( "MTX_Fill returned false." );
        return false;
      }
    }
    else
    {
      if( !MTX_Malloc( &m_Matrix, nrows, ncols, TRUE ) )
      {
        MTX_ERROR_MSG( "MTX_Malloc returned false." );
        return false;
      }
      if( !MTX_Fill( &m_Matrix, 0.0 ) )
      {
        MTX_ERROR_MSG( "MTX_Fill returned false." );
        return false;
      }
    }
    return true;
  }
  

  bool Matrix::GetStats_MaxAbs(unsigned &row, unsigned &col, double &value )
  {
    if( MTX_MaxAbsIndex( &m_Matrix, &value, &row, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxAbsIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Max(unsigned &row, unsigned &col, double &re, double &im )
  {
    if( MTX_MaxIndex( &m_Matrix, &re, &im, &row, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxVal(double &re, double &im )
  {
    if( MTX_Max( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Max returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxAbsCol(const unsigned col, double &value, unsigned &row )
  {
    if( MTX_MaxAbsColIndex( &m_Matrix, col, &value, &row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxAbsColIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxCol(const unsigned col, double &re, double &im, unsigned &row )
  {
    if( MTX_MaxColIndex( &m_Matrix, col, &re, &im, &row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxColIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxColVal(const unsigned col, double &re, double &im )
  {
    if( MTX_MaxColumn( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxColumn returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxAbsRow(const unsigned row, double &value, unsigned &col )
  {
    if( MTX_MaxAbsRowIndex( &m_Matrix, row, &value, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxAbsRowIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxRow(const unsigned row, double &re, double &im, unsigned &col )
  {
    if( MTX_MaxRowIndex( &m_Matrix, row, &re, &im, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxRowIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MaxRowVal(const unsigned row, double &re, double &im )
  {
    if( MTX_MaxRow( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MaxRow returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinAbs(unsigned &row, unsigned &col, double &value )
  {
    if( MTX_MinAbsIndex( &m_Matrix, &value, &row, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinAbsIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Min(unsigned &row, unsigned &col, double &re, double &im )
  {
    if( MTX_MinIndex( &m_Matrix, &re, &im, &row, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinVal(double &re, double &im )
  {
    if( MTX_Min( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Min returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinAbsCol(const unsigned col, double &value, unsigned &row )
  {
    if( MTX_MinAbsColIndex( &m_Matrix, col, &value, &row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinAbsColIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinCol(const unsigned col, double &re, double &im, unsigned &row )
  {
    if( MTX_MinColIndex( &m_Matrix, col, &re, &im, &row ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinColIndex returned false." );
      return false;
    }
  }


  bool Matrix::GetStats_MinColVal(const unsigned col, double &re, double &im )
  {
    if( MTX_MinColumn( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinColumn returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinAbsRow(const unsigned row, double &value, unsigned &col )
  {
    if( MTX_MinAbsRowIndex( &m_Matrix, row, &value, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinAbsRowIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinRow(const unsigned row, double &re, double &im, unsigned &col )
  {
    if( MTX_MinRowIndex( &m_Matrix, row, &re, &im, &col ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinRowIndex returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_MinRowVal(const unsigned row, double &re, double &im )
  {
    if( MTX_MinRow(  &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MinRow returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColRange( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnRange( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnRange returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowRange( const unsigned row, double &re, double &im )
  {
    if( MTX_RowRange( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowRange returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Range( double &re, double &im )
  {
    if( MTX_Range( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Range returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnSum( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnSum( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnSum returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowSum( const unsigned row, double &re, double &im )
  {
    if( MTX_RowSum( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowSum returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Sum( double &re, double &im )
  {
    if( MTX_Sum( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Sum returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnMean( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnMean( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnMean returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowMean( const unsigned row, double &re, double &im )
  {
    if( MTX_RowMean( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowMean returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Mean( double &re, double &im )
  {
    if( MTX_Mean( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Mean returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnStdev( const unsigned col, double &value )
  {
    if( MTX_ColumnStdev( &m_Matrix, col, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnStdev returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowStdev( const unsigned row, double &value )
  {
    if( MTX_RowStdev( &m_Matrix, row, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowStdev returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Stdev( double &value )
  {
    if( MTX_Stdev( &m_Matrix, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Stdev returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnVar( const unsigned col, double &value )
  {
    if( MTX_ColumnVar( &m_Matrix, col, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnVar returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowVar( const unsigned row, double &value )
  {
    if( MTX_RowVar( &m_Matrix, row, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowVar returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Var( double &value )
  {
    if( MTX_Var( &m_Matrix, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Var returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnNorm( const unsigned col, double &value )
  {
    if( MTX_ColumnNorm( &m_Matrix, col, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnNorm returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowNorm( const unsigned row, double &value )
  {
    if( MTX_RowNorm( &m_Matrix, row, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowNorm returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Norm( double &value )
  {
    if( MTX_Norm( &m_Matrix, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Norm returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnRMS( const unsigned col, double &value )
  {
    if( MTX_ColumnRMS( &m_Matrix, col, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnRMS returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowRMS( const unsigned row, double &value )
  {
    if( MTX_RowRMS( &m_Matrix, row, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowRMS returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RMS( double &value )
  {
    if( MTX_RMS( &m_Matrix, &value ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RMS returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnSkewness( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnSkewness( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnSkewness returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowSkewness( const unsigned row, double &re, double &im )
  {
    if( MTX_RowSkewness( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowSkewness returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Skewness( double &re, double &im )
  {
    if( MTX_Skewness( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Skewness returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_ColumnKurtosis( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnKurtosis( &m_Matrix, col, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnKurtosis returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_RowKurtosis( const unsigned row, double &re, double &im )
  {
    if( MTX_RowKurtosis( &m_Matrix, row, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_RowKurtosis returned false." );
      return false;
    }
  }

  bool Matrix::GetStats_Kurtosis( double &re, double &im )
  {
    if( MTX_Kurtosis( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Kurtosis returned false." );
      return false;
    }
  }

  bool Matrix::GetTrace( double &re, double &im )
  {
    if( MTX_Trace( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Trace returned false." );
      return false;
    }
  }

  bool Matrix::GetDeterminant( double &re, double &im )
  {
    if( MTX_Det( &m_Matrix, &re, &im ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Det returned false." );
      return false;
    }
  }

  bool Matrix::GetDiagonal( Matrix& DiagonalVector )
  {
    if( MTX_Diagonal( &m_Matrix, &DiagonalVector.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Diagonal returned false." );
      return false;
    }
  }

  bool Matrix::GetColumnMovAvg( const unsigned col, const unsigned lead, const unsigned lag, Matrix &MovAvg )
  {
    if( MTX_ColumnMovAvg( &m_Matrix, col, lead, lag, &MovAvg.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ColumnMovAvg returned false." );
      return false;
    }
  }

  bool Matrix::GetMovAvg( const unsigned lead, const unsigned lag, Matrix &MovAvg )
  {
    if( MTX_MovAvg( &m_Matrix, lead, lag, &MovAvg.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_MovAvg returned false." );
      return false;
    }
  }

  bool Matrix::GetATAInverse( Matrix &InvATA )
  {
    if( MTX_ATAInverse( &m_Matrix, &InvATA.m_Matrix ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_ATAInverse returned false." );
      return false;
    }
  }

  bool Matrix::GetLUFactorization( bool &isFullRank, Matrix &P, Matrix &L, Matrix &U )
  {
    BOOL b_isFullRank;
    if( MTX_LUFactorization( &m_Matrix, &b_isFullRank, &P.m_Matrix, &L.m_Matrix, &U.m_Matrix ) )
    {
      if( b_isFullRank )
        isFullRank = true;
      else
        isFullRank = false;

      return true;
    }
    else 
    {
      if( b_isFullRank )
        isFullRank = true;
      else
        isFullRank = false;

      MTX_ERROR_MSG( "MTX_LUFactorization returned false." );
      return false;
    }
  }

  bool Matrix::GetLDLt( 
    Matrix& L,   //!< A unit lower triangular matrix.
    Matrix& d,   //!< The diagonal vector from the diagonal of the D matrix.
    bool checkSymmetric //!< Enforce a symmetry check. Runs faster if disabled.
    )
  {
    if( MTX_LDLt( &m_Matrix, &L.m_Matrix, &d.m_Matrix, checkSymmetric ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_LDLt returned false." );
      return false;
    }
  }

  bool Matrix::GetUDUt( 
    Matrix& U,  //!< A unit upper triangular matrix.
    Matrix& d,  //!< The diagonal vector from the diagonal of the D matrix.
    bool checkSymmetric //!< Enforce a symmetry check. Runs faster if disabled.
    )
  {
    if( MTX_UDUt( &m_Matrix, &U.m_Matrix, &d.m_Matrix, checkSymmetric ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_UDUt returned false." );
      return false;
    }
  }

  bool Matrix::GetIndexedValues( Matrix& RowIndex, Matrix& ColIndex, Matrix& Result )
  {
    Matrix _rowIndex; // a copy if needed
    Matrix _colIndex; // a copy if needed
    if( !RowIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &RowIndex.m_Matrix, &_rowIndex.m_Matrix ) )
      {
        MTX_ERROR_MSG( "MTX_Real returned false." );
        return false;
      }
    }
    if( !ColIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &ColIndex.m_Matrix, &_colIndex.m_Matrix ) )
      {
        MTX_ERROR_MSG( "MTX_Real returned false." );
        return false;
      }
    }

    if( !_rowIndex.isEmpty() )
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_IndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &_colIndex.m_Matrix, &Result.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_IndexedValues returned false." );
          return false;
        }
      }
      else
      {
        if( MTX_IndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &ColIndex.m_Matrix, &Result.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_IndexedValues returned false." );
          return false;
        }
      }
    }
    else
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_IndexedValues( &m_Matrix, &RowIndex.m_Matrix, &_colIndex.m_Matrix, &Result.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_IndexedValues returned false." );
          return false;
        }
      }
      else
      {
        if( MTX_IndexedValues( &m_Matrix, &RowIndex.m_Matrix, &ColIndex.m_Matrix, &Result.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_IndexedValues returned false." );
          return false;
        }
      }
    }    
  }


  bool Matrix::SetIndexedValues( Matrix& RowIndex, Matrix& ColIndex, Matrix& SourceData )
  {
    Matrix _rowIndex; // a copy if needed
    Matrix _colIndex; // a copy if needed
    if( !RowIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &RowIndex.m_Matrix, &_rowIndex.m_Matrix ) )
      {
        MTX_ERROR_MSG( "MTX_Real returned false." );
        return false;
      }
    }
    if( !ColIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &ColIndex.m_Matrix, &_colIndex.m_Matrix ) )
      {
        MTX_ERROR_MSG( "MTX_Real returned false." );
        return false;
      }
    }

    if( !_rowIndex.isEmpty() )
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_SetIndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &_colIndex.m_Matrix, &SourceData.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_SetIndexedValues returned false." );
          return false;
        }
      }
      else
      {
        if( MTX_SetIndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &ColIndex.m_Matrix, &SourceData.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_SetIndexedValues returned false." );
          return false;
        }
      }
    }
    else
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_SetIndexedValues( &m_Matrix, &RowIndex.m_Matrix, &_colIndex.m_Matrix, &SourceData.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_SetIndexedValues returned false." );
          return false;
        }
      }
      else
      {
        if( MTX_SetIndexedValues( &m_Matrix, &RowIndex.m_Matrix, &ColIndex.m_Matrix, &SourceData.m_Matrix ) )
        {
          return true;
        }
        else
        {
          MTX_ERROR_MSG( "MTX_SetIndexedValues returned false." );
          return false;
        }
      }
    }    
  }

  bool Matrix::Find_EqualTo( 
    Matrix &IndexVector,    //!< Store the indexed values in this vector (nx1)
    const unsigned col,     //!< Search this column (zero-based index).
    const double value,     //!< Search for this value.
    const double tolerance  //!< Search with this tolerance.
    )
  {
    if( MTX_find_column_values_equalto( &m_Matrix, col, &IndexVector.m_Matrix, value, 0.0, tolerance ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_equalto() returned FALSE." );
      return false;
    }
  }

  bool Matrix::Find_EqualTo( 
    Matrix &IndexVector,            //!< Store the indexed values in this vector (nx1)
    const unsigned col,             //!< Search this column (zero-based index).
    const double value_re,          //!< Search for this complex value (re+i*im).
    const double value_im,          //!< Search for this complex value (re+i*im).
    const double tolerance          //!< Search with this tolerance.
    )
  {
    if( MTX_find_column_values_equalto( &m_Matrix, col, &IndexVector.m_Matrix, value_re, value_im, tolerance ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_equalto() returned FALSE." );
      return false;
    }
  }


  bool Matrix::Find_NotEqualTo( 
    Matrix &IndexVector,    //!< Store the indexed values in this vector (nx1)
    const unsigned col,     //!< Search this column (zero-based index).
    const double value,     //!< Search for this value.
    const double tolerance  //!< Search with this tolerance.
    )
  {
    if( MTX_find_column_values_not_equalto( &m_Matrix, col, &IndexVector.m_Matrix, value, 0.0, tolerance ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_not_equalto() returned FALSE." );
      return false;
    }
  }

  bool Matrix::Find_NotEqualTo( 
    Matrix &IndexVector,    //!< Store the indexed values in this vector (nx1)
    const unsigned col,     //!< Search this column (zero-based index).
    const double value_re,  //!< Search for this complex value (re+i*im).
    const double value_im,  //!< Search for this complex value (re+i*im).
    const double tolerance  //!< Search with this tolerance. No default parameter so there is no function overload confusion.
    )
  {
    if( MTX_find_column_values_not_equalto( &m_Matrix, col, &IndexVector.m_Matrix, value_re, value_im, tolerance ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_not_equalto() returned FALSE." );
      return false;
    }
  }


  bool Matrix::Find_LessThan( 
    Matrix &IndexVector, //!< Store the indexed values in this vector (nx1)
    const unsigned col,  //!< Search this column (zero-based index).
    const double value   //!< Search for this value.
    )   
  {
    if( MTX_find_column_values_less_than( &m_Matrix, col, &IndexVector.m_Matrix, value ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_less_than() returned FALSE." );
      return false;
    }
  }


  bool Matrix::Find_MoreThan( 
    Matrix &IndexVector, //!< Store the indexed values in this vector (nx1)
    const unsigned col,  //!< Search this column (zero-based index).
    const double value   //!< Search for this value.
    )   
  {
    if( MTX_find_column_values_more_than( &m_Matrix, col, &IndexVector.m_Matrix, value ) )
    {
      return true;
    }
    else
    {
      MTX_ERROR_MSG( "MTX_find_column_values_more_than() returned FALSE." );
      return false;
    }
  }



  std::string Matrix::GetMatrixComment()
  {
    std::string result;
    if( m_Matrix.comment != NULL )
      result = m_Matrix.comment;
    
    return result;
  }


  bool Matrix::TimeWindow( 
    const unsigned timeColumn, //!< The column containing time.
    const double startTime,    //!< The specified start time (inclusive).
    const double duration,     //!< The duration to include.
    const double rolloverTime  //!< The potential time at which system time rolls over.
    )
  {
    if( MTX_TimeWindow(
      &m_Matrix, 
      timeColumn, 
      startTime,
      duration,
      rolloverTime ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TimeWindow returned false." );
      return false;
    }
  }

  bool Matrix::TimeLimit( 
    const unsigned timeColumn, //!< The column containing time
    const double startTime,    //!< The specified start time (inclusive)
    const double endTime       //!< The duration to include
    )
  {
    if( MTX_TimeLimit(
      &m_Matrix,
      timeColumn,
      startTime,
      endTime ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TimeLimit returned false." );
      return false;
    }
  }

  bool Matrix::TimeMatch( 
    Matrix &A,                   //!< The matrix with interpolation times
    const unsigned timeColumnA,  //!< The zero based column index for matrix A
    Matrix &B,                   //!< The matrix to be interpolated
    const unsigned timeColumnB,  //!< The zero based column index for matrix B
    const unsigned precision,    //!< The rounding precision used for time matching, 0 = whole, 1 = 0.1, 2 = 0.01, etc
    const double rolloverTime    //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
    )
  {
    // check that the mtx engine is initialized
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        A.Clear();
        B.Clear();
        StaticMatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    if( MTX_TimeMatch(
      &A.m_Matrix,
      timeColumnA,
      &B.m_Matrix,
      timeColumnB,
      precision,
      rolloverTime ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_TimeMatch returned false." );
      return false;
    }
  }

  bool Matrix::Interpolate( 
    Matrix &A,                    //!< The matrix with interpolation times
    const unsigned timeColumnA,   //!< The zero based column index for matrix A
    Matrix &B,                    //!< The matrix to be interpolated
    const unsigned timeColumnB,   //!< The zero based column index for matrix B
    const double maxInterpolationInterval, //!< The largest interpolation interval allowed
    const double rolloverTime     //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
    )
  {
    // check that the mtx engine is initialized
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        A.Clear();
        B.Clear();
        StaticMatrixError( "Matrix", "Failed to initialize the MTX Engine." );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

    if( MTX_Interpolate(
      &A.m_Matrix,
      timeColumnA,
      &B.m_Matrix,
      timeColumnB,
      maxInterpolationInterval,
      rolloverTime ) )
    {
      return true;
    }
    else 
    {
      MTX_ERROR_MSG( "MTX_Interpolate returned false." );
      return false;
    }
  }

  // Return the column matrix specified by the column index. Returns (nrows x 1).
  Matrix  Matrix::Column(const unsigned col)
  {
    Matrix A;
    if( !MTX_CopyColumn( &m_Matrix, col, &A.m_Matrix ) )
    {
      MTX_ERROR_MSG( "MTX_CopyColumn returned false." );
      MatrixError( "Column", "Unable to copy the source matrix column." );
    }
    return A;
  }

  // Return the row matrix specified by the column index. Returns (ncols x 1).
  Matrix  Matrix::Row(const unsigned row)
  {
    Matrix A;
    if( !MTX_CopyRow(&m_Matrix, row, &A.m_Matrix ) )
    {
      MatrixError( "Column", "Unable to copy the source matrix column." );
    }
    return A;
  }

  // Return the tranpose of the matrix.
  Matrix  Matrix::Transpose()
  {
    Matrix A;
    if( !MTX_Transpose( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Column", "Unable to transpose the source matrix." );
    }
    return A;
  }

  // Return the tranpose of the matrix.
  Matrix  Matrix::T()
  {
    return Transpose();
  }

  // Return the diagonal of the matrix as a vector.
  Matrix  Matrix::Diagonal()
  {
    Matrix A;
    if( !MTX_Diagonal( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Diagonal", "Unable to get the diagonal of the source matrix." );
    }
    return A;
  }

  // Return the inverse of the matrix.
  Matrix  Matrix::Inverse()
  {
    Matrix A;
    if( !MTX_Invert( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Inverse", "Unable to invert the matrix." );
    }
    return A;
  }

  // Return the inverse of the matrix.
  Matrix  Matrix::Inv()// short version
  {
    return Inverse();
  }

  // Return the Fourier Transform of each column of the matrix. Power of two uses FFT, otherwise fast DFT.
  Matrix  Matrix::FFT()
  {
    Matrix A;
    if( !MTX_FFT( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "FFT", "Unable to perform the FFT." );
    }
    return A;
  }


  Matrix  Matrix::FFT2()
  {
    Matrix A;
    if( !MTX_FFT2( &m_Matrix, &A.m_Matrix ) )
    {
      MTX_ERROR_MSG( "MTX_FFT2 returned false." );
      return false;
    }
    return A;
  }

  // Return the inverse Fourier Transform of each column of the matrix. Power of two uses IFFT, otherwise fast IDFT.
  Matrix  Matrix::IFFT()
  {
    Matrix A;
    if( !MTX_IFFT( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "IFFT", "Unable to perform the IFFT." );
    }
    return A;
  }

  Matrix  Matrix::IFFT2()
  {
    Matrix A;
    if( !MTX_IFFT2( &m_Matrix, &A.m_Matrix ) )
    {
      MTX_ERROR_MSG( "MTX_IFFT2 returned false." );
      return false;
    }
    return A;
  }

  Matrix  Matrix::DotMultiply(const Matrix& B)
  {
    Matrix A;
    if( !MTX_isConformalForAddition( &m_Matrix, &B.m_Matrix ) )
    {
      MatrixError( "DotMultiply", "Not conformal for dot multiplication." );
    }
    else
    {
      if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
      {
        MatrixError( "DotMultiply", "MTX_Copy returned FALSE." );
      }        
      else
      {
        if( !MTX_DotMultiply_Inplace( &A.m_Matrix, &B.m_Matrix ) )
        {
          MatrixError( "MTX_DotMultiply_Inplace", "MTX_Copy returned FALSE." );
        }
      }
    }
    return A;
  }

  // Return the real part of the matrix.
  Matrix  Matrix::Real()
  {
    Matrix A;
    if( !MTX_Real( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Real", "Cannot get real part of the matrix." );
    }
    return A;
  }

  // Return the imaginary part of the matrix.
  Matrix  Matrix::Imag()
  {
    Matrix A;
    if( !MTX_Imag( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Imag", "Cannot get imaginary part of the matrix." );
    }
    return A;
  }

  // Return the complex conjugate of the matrix            
  Matrix  Matrix::conj()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Conj", "MTX_Copy returned FALSE." );
    }    
    if( !MTX_Conjugate( &A.m_Matrix ) )
    {
      MatrixError( "Conj", "MTX_Conj returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::AddIdentity()
  {
    Matrix A;
    if( !MTX_AddIdentity( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "AddIdentity", "MTX_AddIdentity returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::MinusIdentity()
  {
    Matrix A;
    if( !MTX_MinusIdentity( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "MinusIdentity", "MTX_MinusIdentity returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::IdentityMinusMe()
  {
    Matrix A;
    if( !MTX_IdentityMinus( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "IdentityMinusMe", "MTX_IdentityMinus returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::Negate()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Negate", "MTX_Copy returned FALSE." );
    }        
    if( !MTX_Negate( &A.m_Matrix ) )
    {
      MatrixError( "Negate", "MTX_Negate returned FALSE." );
    }
    return A;
  }
    

  // Return the square root of each element in the matrix.
  Matrix  Matrix::Sqrt()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Sqrt", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Sqrt( &A.m_Matrix ) )
    {
      MatrixError( "Sqrt", "MTX_Sqrt returned FALSE." );
    }
    return A;
  }

  // Return the exponent of each element in the matrix.
  Matrix  Matrix::Exp()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Exp", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Exp( &A.m_Matrix ) )
    {
      MatrixError( "Exp", "MTX_Exp returned FALSE." );
    }
    return A;
  }

  // Return the logarithm of each element in the matrix.
  Matrix  Matrix::Ln()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Ln", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Ln( &A.m_Matrix ) )
    {
      MatrixError( "Ln", "MTX_Ln returned FALSE." );
    }
    return A;
  }


  Matrix  Matrix::cos()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "cos", "MTX_Copy returned FALSE." );
    }
    if( !MTX_cos( &A.m_Matrix ) )
    {
      MatrixError( "cos", "MTX_cos returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::acos()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "acos", "MTX_Copy returned FALSE." );
    }
    if( !MTX_acos( &A.m_Matrix ) )
    {
      MatrixError( "acos", "MTX_acos returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::sin()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "sin", "MTX_Copy returned FALSE." );
    }
    if( !MTX_sin( &A.m_Matrix ) )
    {
      MatrixError( "sin", "MTX_sin returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::asin()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "asin", "MTX_Copy returned FALSE." );
    }
    if( !MTX_asin( &A.m_Matrix ) )
    {
      MatrixError( "asin", "MTX_asin returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::tan()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "tan", "MTX_Copy returned FALSE." );
    }
    if( !MTX_tan( &A.m_Matrix ) )
    {
      MatrixError( "tan", "MTX_tan returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::atan()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "atan", "MTX_Copy returned FALSE." );
    }
    if( !MTX_atan( &A.m_Matrix ) )
    {
      MatrixError( "atan", "MTX_atan returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::cosh()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "cosh", "MTX_Copy returned FALSE." );
    }
    if( !MTX_cosh( &A.m_Matrix ) )
    {
      MatrixError( "cosh", "MTX_cosh returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::sinh()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "sinh", "MTX_Copy returned FALSE." );
    }
    if( !MTX_sinh( &A.m_Matrix ) )
    {
      MatrixError( "sinh", "MTX_sinh returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::tanh()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "tanh", "MTX_Copy returned FALSE." );
    }
    if( !MTX_tanh( &A.m_Matrix ) )
    {
      MatrixError( "tanh", "MTX_tanh returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::cot()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "cot", "MTX_Copy returned FALSE." );
    }
    if( !MTX_cot( &A.m_Matrix ) )
    {
      MatrixError( "cot", "MTX_cot returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::coth()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "coth", "MTX_Copy returned FALSE." );
    }
    if( !MTX_coth( &A.m_Matrix ) )
    {
      MatrixError( "coth", "MTX_coth returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::abs()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "abs", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Abs( &A.m_Matrix ) )
    {
      MatrixError( "abs", "MTX_Abs returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::angle()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "angle", "MTX_Copy returned FALSE." );
    }
    if( !MTX_angle( &A.m_Matrix ) )
    {
      MatrixError( "angle", "MTX_angle returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::pow(const double power_re, const double power_im)
  {
    Matrix A;
    if( !MTX_Pow( &m_Matrix, &A.m_Matrix, power_re, power_im ) )
    {
      MatrixError( "pow", "MTX_Pow returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::round(const unsigned precision)
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "round", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Round( &A.m_Matrix, precision ) )
    {
      MatrixError( "round", "MTX_Round returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::floor()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "floor", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Floor( &A.m_Matrix ) )
    {
      MatrixError( "floor", "MTX_Floor returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::ceil()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "ceil", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Ceil( &A.m_Matrix ) )
    {
      MatrixError( "ceil", "MTX_Ceil returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::fix()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "fix", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Fix( &A.m_Matrix ) )
    {
      MatrixError( "fix", "MTX_Fix returned FALSE." );
    }
    return A;
  }

  Matrix  Matrix::dotInvert()
  {
    Matrix A;
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "dotInvert", "MTX_Copy returned FALSE." );
    }
    if( !MTX_Inv( &A.m_Matrix ) )
    {
      MatrixError( "dotInvert", "MTX_Inv returned FALSE." );
    }
    return A;
  }

  Matrix Matrix::oneMinusMe()
  {
    Matrix A;
    if( !MTX_OneMinus( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "oneMinusMe", "MTX_OneMinus returned FALSE." );
    }
    return A;
  }




  // Get a reference to an element in the matrix to set its value.
  Matrix::Element& Matrix::operator() (unsigned row, unsigned col)
  {
    if( IndexCheck(row,col) )
    {
      m_MatrixElement.m_row = row;
      m_MatrixElement.m_col = col;
    }
    else
    {
      // This code should not be reached!
      m_MatrixElement.m_row = 0;
      m_MatrixElement.m_col = 0;
    }
    return m_MatrixElement; 
  }


  // Get a reference to an element in the matrix as a column or row vector to set its value.
  Matrix::Element& Matrix::operator() (unsigned index)
  {
    if( IndexCheck( index ) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        m_MatrixElement.m_row = index;
        m_MatrixElement.m_col = 0;
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        m_MatrixElement.m_row = 0;
        m_MatrixElement.m_col = index;
      }
      else
      {
        // access the matrix as a singular column array
        m_MatrixElement.m_col = index / m_Matrix.nrows;
        m_MatrixElement.m_row = index - m_MatrixElement.m_col*m_Matrix.nrows;
      }
    }
    else
    {
      // This code should not be reached!
      m_MatrixElement.m_row = 0;
      m_MatrixElement.m_col = 0;
    }

    return m_MatrixElement;
  }


  Matrix::Element::Element(MTX& mtx)
    : m_mtx(mtx), m_row(0), m_col(0)
  {
    // Note that there is no index checking at this level. The 
    // index checking is performed at the Matrix operator() level.
  }

  Matrix::Element::~Element()
  {}

  const double Matrix::Element::real()
  {
    if( m_mtx.isReal )
    {
      return m_mtx.data[m_col][m_row];
    }
    else
    {
      return m_mtx.cplx[m_col][m_row].re;
    }
  }
  
  const double Matrix::Element::imag()
  {
    if( m_mtx.isReal )
    {
      return 0.0;
    }
    else
    {
      return m_mtx.cplx[m_col][m_row].im;
    }
  }

  const Matrix::Element& Matrix::Element::operator= (double v)
  {
    if( m_mtx.isReal )
    {
      m_mtx.data[m_col][m_row] = v;
    }
    else
    {
      m_mtx.cplx[m_col][m_row].re = v;
      m_mtx.cplx[m_col][m_row].im = 0.0;
    }
    return *this;
  }
  
  const Matrix::Element& Matrix::Element::operator= (std::complex<double> v)
  {
    if( m_mtx.isReal )
    {
      if( v.imag() != 0.0 )
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element::operator=", "Unable to convert matrix from real to complex" );
          return *this;
        }
      }
      else
      {
        m_mtx.data[m_col][m_row] = v.real();
        return *this;
      }
    }
    m_mtx.cplx[m_col][m_row].re = v.real();
    m_mtx.cplx[m_col][m_row].im = v.imag();
    return *this;
  }

  const Matrix::Element& Matrix::Element::operator= (Element v)
  {
    if( v.m_mtx.isReal )
    {
      if( m_mtx.isReal )
      {
        m_mtx.data[m_col][m_row] = v.m_mtx.data[v.m_col][v.m_row];
      }
      else
      {
        m_mtx.cplx[m_col][m_row].re = v.m_mtx.data[v.m_col][v.m_row];
        m_mtx.cplx[m_col][m_row].im = 0.0;
      }
    }
    else
    {
      if( m_mtx.isReal )
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element::operator=", "Unable to convert matrix from real to complex" );
          return *this;
        }        
      }
      
      m_mtx.cplx[m_col][m_row].re = v.m_mtx.cplx[v.m_col][v.m_row].re;
      m_mtx.cplx[m_col][m_row].im = v.m_mtx.cplx[v.m_col][v.m_row].im;      
    }
    return *this;
  }


  Matrix::Element::operator const std::complex<double>() const
  {
    if( m_mtx.isReal )
    {
      std::complex<double> v( m_mtx.data[m_col][m_row], 0.0 );
      return v;
    }
    else
    {
      std::complex<double> v( m_mtx.cplx[m_col][m_row].re, m_mtx.cplx[m_col][m_row].im );
      return v;
    }
  }


  void Matrix::Element::operator+= (const double scalar)
  {
    if( m_mtx.isReal )
    {
      m_mtx.data[m_col][m_row] += scalar;
    }
    else
    {
      m_mtx.cplx[m_col][m_row].re += scalar;
    }
  }

  void Matrix::Element::operator+= (const std::complex<double>& v)
  {
    if( m_mtx.isReal )
    {
      if( v.imag() != 0.0 )
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element::operator=", "Unable to convert matrix from real to complex" );
          return;
        }
      }
      else
      {
        m_mtx.data[m_col][m_row] += v.real();
        return;
      }
    }
    
    m_mtx.cplx[m_col][m_row].re += v.real();
    m_mtx.cplx[m_col][m_row].im += v.imag();    
  }
  
  void Matrix::Element::operator+= (const Element& v)
  {
    std::complex<double> cplx = (const std::complex<double>)v;
    *this += cplx;
  }


  void Matrix::Element::operator-= (const double scalar)
  {
    if( m_mtx.isReal )
    {
      m_mtx.data[m_col][m_row] -= scalar;
    }
    else
    {
      m_mtx.cplx[m_col][m_row].re -= scalar;
    }
  }

  void Matrix::Element::operator-= (const std::complex<double>& v)
  {
    if( m_mtx.isReal )
    {
      if( v.imag() != 0.0 )
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element::operator=", "Unable to convert matrix from real to complex" );
          return;
        }
      }
      else
      {
        m_mtx.data[m_col][m_row] -= v.real();
        return;
      }
    }
    
    m_mtx.cplx[m_col][m_row].re -= v.real();
    m_mtx.cplx[m_col][m_row].im -= v.imag();    
  }
  
  void Matrix::Element::operator-= (const Element& v)
  {
    std::complex<double> cplx = (const std::complex<double>)v;
    *this -= cplx;
  }


  void Matrix::Element::operator*= (const double scalar)
  {
    if( m_mtx.isReal )
    {
      m_mtx.data[m_col][m_row] *= scalar;
    }
    else
    {
      m_mtx.cplx[m_col][m_row].re *= scalar;
      m_mtx.cplx[m_col][m_row].im *= scalar;
    }
  }

  void Matrix::Element::operator*= (const std::complex<double>& v)
  {
    if( m_mtx.isReal )
    {
      if( v.imag() != 0.0 )
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element::operator=", "Unable to convert matrix from real to complex" );
          return;
        }
      }
      else
      {
        m_mtx.data[m_col][m_row] *= v.real();
        return;
      }
    }

    double re = m_mtx.cplx[m_col][m_row].re;
    double im = m_mtx.cplx[m_col][m_row].im;
    
    m_mtx.cplx[m_col][m_row].re = re*v.real() - im*v.imag();
    m_mtx.cplx[m_col][m_row].im = re*v.imag() + im*v.real();    
  }
  
  void Matrix::Element::operator*= (const Element& v)
  {
    std::complex<double> cplx = (const std::complex<double>)v;
    *this *= cplx;
  }


  void Matrix::Element::operator/= (const double scalar)
  {
    //if( scalar == 0.0 )
    //{
    //  MTX_Free( &m_mtx );
    //  Matrix::StaticMatrixError("Element/=","Divide by zero!");
    //}
    //else
    //{
      if( m_mtx.isReal )
      {
        m_mtx.data[m_col][m_row] /= scalar;
      }
      else
      {
        m_mtx.cplx[m_col][m_row].re /= scalar;
        m_mtx.cplx[m_col][m_row].im /= scalar;
      }
    //}
  }

  void Matrix::Element::operator/= (const std::complex<double>& v)
  {
    if( m_mtx.isReal )
    {
      if( v.imag() == 0.0 )
      {
      //  if( v.real() == 0.0 )
      //  {
      //    MTX_Free( &m_mtx );
      //    Matrix::StaticMatrixError( "Element/=", "Divide by zero." );
      //    return;
      //  }
      //  else
      //  {
          m_mtx.data[m_col][m_row] /= v.real();
          return;
      //  }
      }
      else
      {
        // This is on the fly conversion to a complex matrix!
        if( !MTX_ConvertRealToComplex( &m_mtx ) )
        {
          MTX_Free( &m_mtx );
          Matrix::StaticMatrixError( "Element/=", "Unable to convert matrix from real to complex" );
          return;
        }
      }
    }

    double d = v.real()*v.real() + v.imag()*v.imag(); 
    //if( d == 0.0 )
    //{ 
    //  MTX_Free( &m_mtx );
    //  Matrix::StaticMatrixError( "Element/=", "Divide by zero." );
    //}
    //else
    {
      double r = m_mtx.cplx[m_col][m_row].re;
      double i = m_mtx.cplx[m_col][m_row].im;
      m_mtx.cplx[m_col][m_row].re = (r * v.real() + i * v.imag()) / d;
      m_mtx.cplx[m_col][m_row].im = (i * v.real() - r * v.imag()) / d;
    }
  }
  
  void Matrix::Element::operator/= (const Element& v)
  {
    std::complex<double> cplx = (const std::complex<double>)v;
    *this /= cplx;
  }






  //friend 
  const std::complex<double> operator+ (const Matrix::Element& m, double scalar)
  {
    std::complex<double> v;
    v = (const std::complex<double>)m;
    v += scalar;
    return v;
  }

  //friend 
  const std::complex<double> operator+ (const Matrix::Element& a, const Matrix::Element& b)
  {
    std::complex<double> v1;
    std::complex<double> v2;
    v1 = (const std::complex<double>)a;
    v2 = (const std::complex<double>)b;
    v1 += v2;
    return v1;
  }
    
  //friend 
  const std::complex<double> operator+ (const Matrix::Element& a, const std::complex<double>& b)
  {
    std::complex<double> v;
    v = (const std::complex<double>)a;
    v += b;
    return v;
  }

  //friend 
  const std::complex<double> operator+ (double scalar, const Matrix::Element& m)
  {
    return (m+scalar);

  }

  //friend 
  const std::complex<double> operator+ (const std::complex<double>& b, const Matrix::Element& a)
  {
    return (a+b);
  }


  //friend 
  const std::complex<double> operator- (const Matrix::Element& m, double scalar)
  {
    std::complex<double> v;
    v = (const std::complex<double>)m;
    v -= scalar;
    return v;
  }

  //friend 
  const std::complex<double> operator- (const Matrix::Element& a, const Matrix::Element& b)
  {
    std::complex<double> v1;
    std::complex<double> v2;
    v1 = (const std::complex<double>)a;
    v2 = (const std::complex<double>)b;
    v1 -= v2;
    return v1;
  }
    
  //friend 
  const std::complex<double> operator- (const Matrix::Element& a, const std::complex<double>& b)
  {
    std::complex<double> v;
    v = (const std::complex<double>)a;
    v -= b;
    return v;
  }

  //friend 
  const std::complex<double> operator- (double scalar, const Matrix::Element& m)
  {
    return (m+(-1.0*scalar));

  }

  //friend 
  const std::complex<double> operator- (const std::complex<double>& b, const Matrix::Element& a)
  {
    std::complex<double> v = b;
    v -= (const std::complex<double>)a;
    return v;    
  }


  //friend 
  const std::complex<double> operator* (const Matrix::Element& m, double scalar)
  {
    std::complex<double> v;
    v = (const std::complex<double>)m;
    v *= scalar;
    return v;
  }

  //friend 
  const std::complex<double> operator* (const Matrix::Element& a, const Matrix::Element& b)
  {
    std::complex<double> v1;
    std::complex<double> v2;
    v1 = (const std::complex<double>)a;
    v2 = (const std::complex<double>)b;
    v1 *= v2;
    return v1;
  }
    
  //friend 
  const std::complex<double> operator* (const Matrix::Element& a, const std::complex<double>& b)
  {
    std::complex<double> v;
    v = (const std::complex<double>)a;
    v *= b;
    return v;
  }

  //friend 
  const std::complex<double> operator* (double scalar, const Matrix::Element& m)
  {
    return (m*scalar);
  }
      
  //friend 
  const std::complex<double> operator* (const std::complex<double>& b, const Matrix::Element& a)
  {
    return (a*b);
  }


  //friend 
  const std::complex<double> operator/ (const Matrix::Element& m, double scalar)
  {
    std::complex<double> v;
    v = (const std::complex<double>)m;
    v /= scalar;
    return v;
  }

  //friend 
  const std::complex<double> operator/ (const Matrix::Element& a, const Matrix::Element& b)
  {
    std::complex<double> v1;
    std::complex<double> v2;
    v1 = (const std::complex<double>)a;
    v2 = (const std::complex<double>)b;
    v1 /= v2;
    return v1;
  }
    
  //friend 
  const std::complex<double> operator/ (const Matrix::Element& a, const std::complex<double>& b)
  {
    std::complex<double> v;
    v = (const std::complex<double>)a;
    v /= b; 
    return v;
  }

  //friend 
  const std::complex<double> operator/ (double scalar, const Matrix::Element& m)
  {
    std::complex<double> v(scalar,0.0);
    v /= (const std::complex<double>)m;
    return v;
  }

  //friend 
  const std::complex<double> operator/ (const std::complex<double>& b, const Matrix::Element& a)
  {
    std::complex<double> v(b);
    v /= (const std::complex<double>)a;
    return v;
  }


  //friend 
  const bool operator== (const Matrix::Element& m, double scalar)
  {
    if( m.m_mtx.isReal )
    {
      if( m.m_mtx.data[m.m_col][m.m_row] == scalar )
        return true;
      else
        return false;
    }
    else
    {
      if( m.m_mtx.cplx[m.m_col][m.m_row].im != 0.0 )
        return false;
      if( m.m_mtx.cplx[m.m_col][m.m_row].re == scalar )
        return true;
      else
        return false;
    }
  }
  
  //friend 
  const bool operator== (const Matrix::Element& a, const Matrix::Element& b)
  {
    if( a.m_mtx.isReal )
    {
      if( b.m_mtx.isReal )
      {
        if( a.m_mtx.data[a.m_col][a.m_row] == b.m_mtx.data[b.m_col][b.m_row] )
          return true;
        else
          return false;
      }
      else
      {
        if( b.m_mtx.cplx[a.m_col][a.m_row].im != 0.0 )
          return false;
        if( a.m_mtx.data[a.m_col][a.m_row] == b.m_mtx.cplx[b.m_col][b.m_row].re )
          return true;
        else
          return false;
      }
    }
    else
    {
      if( b.m_mtx.isReal )
      {
        if( a.m_mtx.cplx[a.m_col][a.m_row].im != 0.0 )
          return false;
        if( a.m_mtx.cplx[a.m_col][a.m_row].re == b.m_mtx.data[b.m_col][b.m_row] )
          return true;
        else
          return false;
      }
      else
      {
        if( a.m_mtx.cplx[a.m_col][a.m_row].re == b.m_mtx.cplx[b.m_col][b.m_row].re &&
          a.m_mtx.cplx[a.m_col][a.m_row].im == b.m_mtx.cplx[b.m_col][b.m_row].im )
          return true;
        else
          return false;
      }
    }
  }

  //friend 
  const bool operator== (const Matrix::Element& a, const std::complex<double>& b)
  {
    if( a.m_mtx.isReal )
    {
      if( b.imag() != 0.0 )
        return false;
      if( a.m_mtx.data[a.m_col][a.m_row] == b.real() )
        return true;
      else
        return false;
    }
    else
    {
      if( a.m_mtx.cplx[a.m_col][a.m_row].re == b.real() && a.m_mtx.cplx[a.m_col][a.m_row].im == b.imag() )
        return true;
      else
        return false;
    }
  }

  //friend 
  const bool operator== (double scalar, const Matrix::Element& m)
  {
    return (m==scalar);
  }
      
  //friend 
  const bool operator== (const std::complex<double>& b, const Matrix::Element& a)
  {
    return (a==b);
  }



  bool Matrix::operator+= (const double scalar)
  {
    return Inplace_AddScalar( scalar );
  }
  
  bool Matrix::operator+= (const std::complex<double> cplx)
  {
    return Inplace_AddScalarComplex( cplx );
  }

  bool Matrix::operator-= (const double scalar)
  {
    return Inplace_SubtractScalar( scalar );
  }

  bool Matrix::operator-= (const std::complex<double> cplx)
  {
    return Inplace_SubtractScalarComplex( cplx );
  }

  bool Matrix::operator*= (const double scalar)
  {
    return Inplace_MultiplyScalar( scalar );
  }

  bool Matrix::operator*= (const std::complex<double> cplx)
  {
    return Inplace_MultiplyScalarComplex( cplx );
  }

  bool Matrix::operator/= (double scalar)
  {
    return Inplace_DivideScalar( scalar );
  }

  bool Matrix::operator/= (const std::complex<double> cplx)
  {
    return Inplace_DivideScalarComplex( cplx );
  }

  bool Matrix::operator+= (const Matrix& mat)
  {
    return Inplace_Add( mat );
  }

  bool Matrix::operator-= (const Matrix& mat)
  {
    return Inplace_Subtract( mat );
  }

  /* friend */
  Matrix operator++ (Matrix& mat, int)
  {
    if( !mat.Inplace_Increment() )
    {
      mat.MatrixError( "operator++", "Inplace_Increment() returned false." );
      return mat;
    }
    return mat;
  }

  /* friend */
  Matrix operator-- (Matrix& mat, int)
  {
    if( !mat.Inplace_Decrement() )
    {
      mat.MatrixError( "operator--", "Inplace_Decrement() returned false." );
      return mat;
    }
    return mat;
  }

  // matrix multiplication: A = B * C
  /* friend */
  Matrix operator* (Matrix& mat1, Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Multiply( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      mat1.Clear();
      A.Clear();
      mat2.MatrixError( "operator*", "MTX_Multiply() returned false." );
      return A;
    }
    return A; // return a copy
  }
  
  Matrix operator* (const Matrix& mat1, const Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Multiply( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      A.Clear();
      Matrix::StaticMatrixError( "operator*", "MTX_Multiply() returned false." );
      return A;
    }
    return A; // return a copy
  }



  // matrix addition: A = B + C
  /* friend */ 
  Matrix operator+ (Matrix& mat1, Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Add( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      A.Clear();
      mat1.Clear();
      mat2.MatrixError( "operator+", "MTX_Add() returned false." );
      return A;
    }
    return A; // return a copy
  }
  
  Matrix operator+ (const Matrix& mat1, const Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Add( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      A.Clear();
      Matrix::StaticMatrixError( "operator+", "MTX_Add() returned false." );
      return A;
    }
    return A; // return a copy
  }



  // matrix subtraction: A = B - C
  /* friend */ 
  Matrix operator- (Matrix& mat1, Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Subtract( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      mat1.Clear();
      mat2.MatrixError( "operator-", "MTX_Subtract() returned false." );
      return A;
    }

    return A; // return a copy
  }
  
  Matrix operator- (const Matrix& mat1, const Matrix& mat2)
  {
    Matrix A;
    if( !MTX_Subtract( &A.m_Matrix, &mat1.m_Matrix, &mat2.m_Matrix ) )
    {
      A.Clear();
      Matrix::StaticMatrixError( "operator-", "MTX_Subtract() returned false." );
      return A;
    }

    return A; // return a copy
  }


  // raise each element to a power
  /* friend */ 
  Matrix operator^ (Matrix& mat, const double scalar)
  {
    Matrix A;
    if( !MTX_Pow( &mat.m_Matrix, &A.m_Matrix, scalar, 0.0 ) )
    {
      mat.MatrixError( "operator^", "MTX_Pow() returned false." );
      return A;
    }

    return A; // return a copy
  }

  /* friend */ 
  Matrix operator+ (const double scalar, Matrix& mat)
  {
    Matrix A;
    if( !MTX_Copy( &mat.m_Matrix, &A.m_Matrix ) )
    {
      mat.MatrixError( "operator+", "MTX_Copy() returned false." );
      return A;
    }

    if( !A.Inplace_AddScalar( scalar ) )
    {
      mat.MatrixError( "Matrix operator+", "Inplace_AddScalar() returned false" );
      return A;
    }

    return A; // return a copy
  }

  /* friend */ 
  Matrix operator- (const double scalar, Matrix& mat)
  {
    Matrix A;

    if( !A.Redim( mat.GetNrRows(), mat.GetNrCols() ) )
    {
      mat.MatrixError( "operator-", "Redim() returned false." );
      return A;
    }

    if( !A.Fill( scalar ) )
    {
      mat.MatrixError( "operator-", "Fill() returned false." );
      return A;
    }

    if( !A.Inplace_Subtract( mat ) )
    {
      mat.MatrixError( "operator-", "Fill() returned false." );
      return A;
    }

    return A; // return a copy
  }


  /* friend */ 
  Matrix operator* (const double scalar, Matrix& mat)
  {
    Matrix A;
    if( !MTX_Copy( &mat.m_Matrix, &A.m_Matrix ) )
    {
      mat.MatrixError( "operator*", "MTX_Copy() returned false." );
      return A;
    }

    if( !A.Inplace_MultiplyScalar( scalar ) )
    {
      mat.MatrixError( "operator*", "Inplace_MultiplyScalar() returned false." );
      return A;
    }

    return A;
  }

  /* friend */ 
  Matrix operator/ (Matrix& mat, const double scalar)
  {
    Matrix A;
    if( !MTX_Copy( &mat.m_Matrix, &A.m_Matrix ) )
    {
      mat.MatrixError( "operator/", "MTX_Copy() returned false." );
      return A;
    }

    if( !A.Inplace_DivideScalar( scalar ) )
    {
      mat.MatrixError( "operator/", "Inplace_DivideScalar() returned false." );
      return A;
    }

    return A;
  }

  /* friend */ 
  Matrix operator/ (const double scalar, Matrix& mat)
  {
    Matrix A;
    if( !A.Redim( mat.GetNrRows(), mat.GetNrCols() ) )
    {
      mat.MatrixError( "operator/", "Redim() returned false." );
      return A;
    }

    if( !A.Fill( scalar ) )
    {
      mat.MatrixError( "operator/", "Fill() returned false." );
      return A;
    }

    if( !A.Inplace_DotDivide( mat ) )
    {
      mat.MatrixError( "operator/", "Inplace_DotDivide() returned false." );
      return A;
    }

    return A; // return a copy
  }





  //==========================
  //
#ifdef MATRIX_STREAM_SUPPORT

  // output of a matrix: cout << A;
  /* friend */ 
  ostream& operator<< (ostream& strm, const Matrix& mat)
  {
    mat.Print( strm, strm.precision(), strm.width() );
    return strm;
  }

#endif
  //
  //==========================


  bool Matrix::IndexCheck( const unsigned row, const unsigned col )
  {
    if( MTX_isNull(&m_Matrix) || row >= m_Matrix.nrows || col >= m_Matrix.ncols )
    {
      char msga[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s( msga, 128, "Matrix access violation (%d,%d) out of bounds (%d,%d).\n", row, col, m_Matrix.nrows, m_Matrix.ncols );
#else
      sprintf( msga, "Matrix access violation (%d,%d) out of bounds (%d,%d).\n", row, col, m_Matrix.nrows, m_Matrix.ncols );
#endif
      MatrixError( "IndexCheck", msga );
      MTX_ERROR_MSG( msga );
      return false;
    }
    return true;
  }

  bool Matrix::IndexCheck( const unsigned index )
  {
    unsigned n = 0; // The number of elements as a vector.
    if( MTX_isNull(&m_Matrix) )
      n = 0;
    else
      n = m_Matrix.nrows*m_Matrix.ncols;

    if( index >= n )
    {
      char msga[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s( msga, 128, "Matrix access violation (%d) out of bounds (%dx%d).\n", index, m_Matrix.nrows, m_Matrix.ncols );
#else
      sprintf( msga, "Matrix access violation (%d) out of bounds (%dx%d).\n", index, m_Matrix.nrows, m_Matrix.ncols );
#endif
      MatrixError( "IndexCheck", msga );
      MTX_ERROR_MSG( msga );
      return false;
    }
    return true;
  }




  Matrix::RealOnlyAccess::RealOnlyAccess( MTX& mtx, const unsigned row ) 
    : m_Matrix( mtx ), m_row( row ) 
  {}

  Matrix::RealOnlyAccess::~RealOnlyAccess()
  {}

  bool Matrix::RealOnlyAccess::IndexCheck( const unsigned row, const unsigned col )
  {
    if( MTX_isNull(&m_Matrix) || row >= m_Matrix.nrows || col >= m_Matrix.ncols )
    {
      char msga[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s( msga, 128, "Matrix access violation [%d][%d] out of bounds (%d,%d).\n", row, col, m_Matrix.nrows, m_Matrix.ncols );
#else
      sprintf( msga, "Matrix access violation (%d,%d) out of bounds (%d,%d).\n", row, col, m_Matrix.nrows, m_Matrix.ncols );
#endif
      MTX_Free( &m_Matrix );
      StaticMatrixError( "IndexCheck", msga );
      MTX_ERROR_MSG( msga );
      return false;
    }
    return true;
  }

  bool Matrix::RealOnlyAccess::IndexCheck( const unsigned index )
  {
    unsigned n = 0; // The number of elements as a vector.
    if( MTX_isNull(&m_Matrix) )
      n = 0;
    else
      n = m_Matrix.nrows*m_Matrix.ncols;

    if( index >= n )
    {
      char msga[128];
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s( msga, 128, "Matrix access violation [%d] out of bounds (%dx%d).\n", index, m_Matrix.nrows, m_Matrix.ncols );
#else
      sprintf( msga, "Matrix access violation (%d) out of bounds (%dx%d).\n", index, m_Matrix.nrows, m_Matrix.ncols );
#endif
      MTX_Free( &m_Matrix );
      StaticMatrixError( "IndexCheck", msga );
      MTX_ERROR_MSG( msga );
      return false;
    }
    return true;
  }

  double& Matrix::RealOnlyAccess::operator [] (const unsigned col)
  { 
    if( IndexCheck( m_row, col ) )
    {
      if( m_Matrix.isReal )
      {
        return m_Matrix.data[col][m_row];
      }
      else
      {
        return m_Matrix.cplx[col][m_row].re;
      }
    }
    else
    {
      staticglobal_BadDouble = 0.0;
      return staticglobal_BadDouble;
    }    
  }

  Matrix::RealOnlyAccess Matrix::operator[] (const unsigned row)
  {
    return RealOnlyAccess( m_Matrix, row );
  }

  Matrix::RealOnlyAccess& Matrix::RealOnlyAccess::operator=(const double value)
  {
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[0][index] = value;
        }
        else
        {
          m_Matrix.cplx[0][index].re = value;
          m_Matrix.cplx[0][index].im = 0.0;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[index][0] = value;
        }
        else
        {
          m_Matrix.cplx[index][0].re = value;
          m_Matrix.cplx[index][0].im = 0.0;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          m_Matrix.data[col][row] = value;
        }
        else
        {
          m_Matrix.cplx[col][row].re = value;
          m_Matrix.cplx[col][row].im = 0.0;
        }
      }
    }
    return *this;
  }

  Matrix::RealOnlyAccess& Matrix::RealOnlyAccess::operator=(RealOnlyAccess& rhs)
  {
    // Matrix A(1); Matrix B(1);
    // A[0] = B[0];

    // Get the value of the rhs.
    double val = (const double)rhs;

    // Use the operator= double overload.
    *this = val;
    return *this;
  }

  Matrix::RealOnlyAccess& Matrix::RealOnlyAccess::operator=(Matrix::Element& rhs)
  {
    // Matrix A(1); Matrix B(1);
    // A[0] = B(0);
    *this = rhs.real();
    return *this;
  }


  Matrix::RealOnlyAccess::operator const double()
  {
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          return m_Matrix.data[0][index];
        }
        else
        {
          return m_Matrix.cplx[0][index].re;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          return m_Matrix.data[index][0];
        }
        else
        {
          return m_Matrix.cplx[index][0].re;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          return m_Matrix.data[col][row];
        }
        else
        {
          return m_Matrix.cplx[col][row].re;
        }
      }
    }
    return 0.0; // Should not be reached!
  }

  bool Matrix::RealOnlyAccess::operator+= (const double scalar)
  {
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[0][index] += scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[0][index].re += scalar;
          return true;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[index][0] += scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[index][0].re += scalar;
          return true;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          m_Matrix.data[col][row] += scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[col][row].re += scalar;
          return true;
        }
      }
    }
    MTX_ERROR_MSG( "Unexpected." );
    return false; // Should not be reached!
  }
  
  bool Matrix::RealOnlyAccess::operator-= (const double scalar)
  {
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[0][index] -= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[0][index].re -= scalar;
          return true;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[index][0] -= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[index][0].re -= scalar;
          return true;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          m_Matrix.data[col][row] -= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[col][row].re -= scalar;
          return true;
        }
      }
    }
    MTX_ERROR_MSG( "Unexpected." );
    return false; // Should not be reached!
  }

  bool Matrix::RealOnlyAccess::operator*= (const double scalar)
  {
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[0][index] *= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[0][index].re *= scalar;
          return true;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[index][0] *= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[index][0].re *= scalar;
          return true;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          m_Matrix.data[col][row] *= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[col][row].re *= scalar;
          return true;
        }
      }
    }
    MTX_ERROR_MSG( "Unexpected." );
    return false; // Should not be reached!
  }
  
  bool Matrix::RealOnlyAccess::operator/= (double scalar)
  {
    //if( scalar == 0.0 )
    //{
    //  StaticMatrixError( "Divide by zero not allowed (real vector element division). Matrix X(10); X[0]/0.0!" );
    //  return false; // should not reach this code.
    //}
      
    // This is indexing the matrix as a vector.
    const unsigned index = m_row;
    if( IndexCheck(index) )
    {
      if( m_Matrix.ncols == 1 ) // column array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[0][index] /= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[0][index].re /= scalar;
          return true;
        }
      }
      else if( m_Matrix.nrows == 1 ) // row array
      {
        if( m_Matrix.isReal )
        {
          m_Matrix.data[index][0] /= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[index][0].re /= scalar;
          return true;
        }
      }
      else
      {
        // access the matrix as a singular column array
        unsigned col = index / m_Matrix.nrows;
        unsigned row = index - col*m_Matrix.nrows;

        if( m_Matrix.isReal )
        {
          m_Matrix.data[col][row] /= scalar;
          return true;
        }
        else
        {
          m_Matrix.cplx[col][row].re /= scalar;
          return true;
        }
      }
    }
    MTX_ERROR_MSG( "Unexpected." );
    return false; // Should not be reached!
  }


  bool Matrix::Hilbert( const unsigned N )
  {
    if( !MTX_Hilbert( &m_Matrix, N ) )
    {
      MTX_ERROR_MSG( "MTX_Hilbert returned FALSE." );
      return false;
    }
    else
    {
      return true;
    }
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data (if this is the same as y_col, then the index is plotted as x).
    const unsigned y_col,              //!< The column index (0toN-1) with the y series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label,    //!< The series label.
    const std::string units,           //!< The series data units.      
    const bool isXGridOn,             //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,             //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,          //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats,    //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm,    //!< The plot height in cm.
    const unsigned plot_width_cm      //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    MTX_PLOT_structSeries series;
    char buffer[256];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( series_label.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col );
#endif
      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label;
    }    

    series.connected = TRUE;
    series.markOutlierData = FALSE;
    series.precision = precisionStats;
    series.x_col = x_col;
    series.y_col = y_col;
    series.color = MTX_BLUE;
    series.M = &m_Matrix;
    series.label = (char*)slabel_1.c_str();
    series.units = (char*)units.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      &series,
      1
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
    const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
    const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label_1,  //!< The series label.
    const std::string units_1,         //!< The series data units.
    const std::string series_label_2,  //!< The series label.
    const std::string units_2,         //!< The series data units.
    const bool isXGridOn,           //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,           //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,        //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats,  //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm,  //!< The plot height in cm.
    const unsigned plot_width_cm    //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    std::string slabel_2;
    char buffer[256];

    MTX_PLOT_structSeries series[2];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( series_label_1.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_1 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_1 );
#endif
      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label_1;
    }
    if( series_label_2.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_2 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_2 );
#endif

      slabel_2 = buffer;
    }
    else
    {
      slabel_2 = series_label_2;
    }
    
    series[0].connected = TRUE;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = x_col;
    series[0].y_col = y_col_1;
    series[0].color = MTX_BLUE;
    series[0].M = &m_Matrix;
    series[0].label = (char*)slabel_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = TRUE;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = x_col;
    series[1].y_col = y_col_2;
    series[1].color = MTX_LIMEGREEN;
    series[1].M = &m_Matrix;
    series[1].label = (char*)slabel_2.c_str();
    series[1].units = (char*)units_2.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      2
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
    const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
    const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
    const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label_1,  //!< The series label.
    const std::string units_1,         //!< The series data units.
    const std::string series_label_2,  //!< The series label.
    const std::string units_2,         //!< The series data units.
    const std::string series_label_3,  //!< The series label.
    const std::string units_3,         //!< The series data units.            
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm  //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    std::string slabel_2;
    std::string slabel_3;
    char buffer[256];

    MTX_PLOT_structSeries series[3];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( series_label_1.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_1 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_1 );
#endif
      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label_1;
    }
    if( series_label_2.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_2 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_2 );
#endif

      slabel_2 = buffer;
    }
    else
    {
      slabel_2 = series_label_2;
    }
    if( series_label_3.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_3 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_3 );
#endif
      slabel_3 = buffer;
    }
    else
    {
      slabel_3 = series_label_3;
    }
    
    series[0].connected = TRUE;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = x_col;
    series[0].y_col = y_col_1;
    series[0].color = MTX_BLUE;
    series[0].M = &m_Matrix;
    series[0].label = (char*)slabel_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = TRUE;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = x_col;
    series[1].y_col = y_col_2;
    series[1].color = MTX_LIMEGREEN;
    series[1].M = &m_Matrix;
    series[1].label = (char*)slabel_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = TRUE;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = x_col;
    series[2].y_col = y_col_3;
    series[2].color = MTX_RED;
    series[2].M = &m_Matrix;
    series[2].label = (char*)slabel_3.c_str();
    series[2].units = (char*)units_3.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      3
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
    const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
    const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
    const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
    const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label_1,  //!< The series label.
    const std::string units_1,         //!< The series data units.
    const std::string series_label_2,  //!< The series label.
    const std::string units_2,         //!< The series data units.
    const std::string series_label_3,  //!< The series label.
    const std::string units_3,         //!< The series data units.            
    const std::string series_label_4,  //!< The series label.
    const std::string units_4,         //!< The series data units.            
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm  //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    std::string slabel_2;
    std::string slabel_3;
    std::string slabel_4;
    char buffer[256];

    MTX_PLOT_structSeries series[4];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( series_label_1.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_1 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_1 );
#endif

      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label_1;
    }
    if( series_label_2.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_2 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_2 );
#endif
      slabel_2 = buffer;
    }
    else
    {
      slabel_2 = series_label_2;
    }
    if( series_label_3.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_3 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_3 );
#endif

      slabel_3 = buffer;
    }
    else
    {
      slabel_3 = series_label_3;
    }
    if( series_label_4.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_4 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_4 );
#endif
      slabel_4 = buffer;
    }
    else
    {
      slabel_4 = series_label_4;
    }
    
    series[0].connected = TRUE;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = x_col;
    series[0].y_col = y_col_1;
    series[0].color = MTX_BLUE;
    series[0].M = &m_Matrix;
    series[0].label = (char*)slabel_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = TRUE;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = x_col;
    series[1].y_col = y_col_2;
    series[1].color = MTX_LIMEGREEN;
    series[1].M = &m_Matrix;
    series[1].label = (char*)slabel_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = TRUE;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = x_col;
    series[2].y_col = y_col_3;
    series[2].color = MTX_RED;
    series[2].M = &m_Matrix;
    series[2].label = (char*)slabel_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = TRUE;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = x_col;
    series[3].y_col = y_col_4;
    series[3].color = MTX_PURPLE;
    series[3].M = &m_Matrix;
    series[3].label = (char*)slabel_4.c_str();
    series[3].units = (char*)units_4.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      4
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
    const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
    const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
    const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
    const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.      
    const unsigned y_col_5,            //!< The column index (0toN-1) with the y_5 series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label_1,  //!< The series label.
    const std::string units_1,         //!< The series data units.
    const std::string series_label_2,  //!< The series label.
    const std::string units_2,         //!< The series data units.
    const std::string series_label_3,  //!< The series label.
    const std::string units_3,         //!< The series data units.            
    const std::string series_label_4,  //!< The series label.
    const std::string units_4,         //!< The series data units.            
    const std::string series_label_5,  //!< The series label.
    const std::string units_5,         //!< The series data units.                
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm  //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    std::string slabel_2;
    std::string slabel_3;
    std::string slabel_4;
    std::string slabel_5;
    char buffer[256];

    MTX_PLOT_structSeries series[5];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( series_label_1.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_1 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_1 );
#endif

      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label_1;
    }
    if( series_label_2.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_2 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_2 );
#endif

      slabel_2 = buffer;
    }
    else
    {
      slabel_2 = series_label_2;
    }
    if( series_label_3.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_3 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_3 );
#endif
      slabel_3 = buffer;
    }
    else
    {
      slabel_3 = series_label_3;
    }
    if( series_label_4.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_4 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_4 );
#endif

      slabel_4 = buffer;
    }
    else
    {
      slabel_4 = series_label_4;
    }
    if( series_label_5.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_5 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_5 );
#endif
      slabel_5 = buffer;
    }
    else
    {
      slabel_5 = series_label_5;
    }
    
    series[0].connected = TRUE;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = x_col;
    series[0].y_col = y_col_1;
    series[0].color = MTX_BLUE;
    series[0].M = &m_Matrix;
    series[0].label = (char*)slabel_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = TRUE;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = x_col;
    series[1].y_col = y_col_2;
    series[1].color = MTX_LIMEGREEN;
    series[1].M = &m_Matrix;
    series[1].label = (char*)slabel_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = TRUE;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = x_col;
    series[2].y_col = y_col_3;
    series[2].color = MTX_RED;
    series[2].M = &m_Matrix;
    series[2].label = (char*)slabel_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = TRUE;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = x_col;
    series[3].y_col = y_col_4;
    series[3].color = MTX_PURPLE;
    series[3].M = &m_Matrix;
    series[3].label = (char*)slabel_4.c_str();
    series[3].units = (char*)units_4.c_str();

    series[4].connected = TRUE;
    series[4].markOutlierData = FALSE;
    series[4].precision = precisionStats;
    series[4].x_col = x_col;
    series[4].y_col = y_col_5;
    series[4].color = MTX_ORANGE;
    series[4].M = &m_Matrix;
    series[4].label = (char*)slabel_5.c_str();
    series[4].units = (char*)units_5.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      5
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }
    return true;
  }

  bool Matrix::Plot( 
    const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
    const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
    const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
    const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
    const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.      
    const unsigned y_col_5,            //!< The column index (0toN-1) with the y_5 series data.      
    const unsigned y_col_6,            //!< The column index (0toN-1) with the y_6 series data.      
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    const std::string series_label_1,  //!< The series label.
    const std::string units_1,         //!< The series data units.
    const std::string series_label_2,  //!< The series label.
    const std::string units_2,         //!< The series data units.
    const std::string series_label_3,  //!< The series label.
    const std::string units_3,         //!< The series data units.            
    const std::string series_label_4,  //!< The series label.
    const std::string units_4,         //!< The series data units.            
    const std::string series_label_5,  //!< The series label.
    const std::string units_5,         //!< The series data units.                
    const std::string series_label_6,  //!< The series label.
    const std::string units_6,         //!< The series data units.                
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm  //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;
    std::string slabel_1;
    std::string slabel_2;
    std::string slabel_3;
    std::string slabel_4;
    std::string slabel_5;
    std::string slabel_6;
    char buffer[256];

    MTX_PLOT_structSeries series[6];

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( series_label_1.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_1 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_1 );
#endif
      slabel_1 = buffer;
    }
    else
    {
      slabel_1 = series_label_1;
    }
    if( series_label_2.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_2 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_2 );
#endif
      slabel_2 = buffer;
    }
    else
    {
      slabel_2 = series_label_2;
    }
    if( series_label_3.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_3 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_3 );
#endif
      slabel_3 = buffer;
    }
    else
    {
      slabel_3 = series_label_3;
    }
    if( series_label_4.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_4 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_4 );
#endif
      slabel_4 = buffer;
    }
    else
    {
      slabel_4 = series_label_4;
    }
    if( series_label_5.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_5 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_5 );
#endif
      slabel_5 = buffer;
    }
    else
    {
      slabel_5 = series_label_5;
    }
    if( series_label_6.compare( "" ) == 0 )
    {
#ifndef _CRT_SECURE_NO_DEPRECATE
      sprintf_s ( buffer, 256, "Col %d vs %d", x_col, y_col_6 );
#else
      sprintf( buffer, "Col %d vs %d", x_col, y_col_6 );
#endif
      slabel_6 = buffer;
    }
    else
    {
      slabel_6 = series_label_6;
    }
    
    series[0].connected = TRUE;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = x_col;
    series[0].y_col = y_col_1;
    series[0].color = MTX_BLUE;
    series[0].M = &m_Matrix;
    series[0].label = (char*)slabel_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = TRUE;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = x_col;
    series[1].y_col = y_col_2;
    series[1].color = MTX_LIMEGREEN;
    series[1].M = &m_Matrix;
    series[1].label = (char*)slabel_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = TRUE;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = x_col;
    series[2].y_col = y_col_3;
    series[2].color = MTX_RED;
    series[2].M = &m_Matrix;
    series[2].label = (char*)slabel_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = TRUE;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = x_col;
    series[3].y_col = y_col_4;
    series[3].color = MTX_PURPLE;
    series[3].M = &m_Matrix;
    series[3].label = (char*)slabel_4.c_str();
    series[3].units = (char*)units_4.c_str();

    series[4].connected = TRUE;
    series[4].markOutlierData = FALSE;
    series[4].precision = precisionStats;
    series[4].x_col = x_col;
    series[4].y_col = y_col_5;
    series[4].color = MTX_ORANGE;
    series[4].M = &m_Matrix;
    series[4].label = (char*)slabel_5.c_str();
    series[4].units = (char*)units_5.c_str();

    series[5].connected = TRUE;
    series[5].markOutlierData = FALSE;
    series[5].precision = precisionStats;
    series[5].x_col = x_col;
    series[5].y_col = y_col_6;
    series[5].color = MTX_GREEN;
    series[5].M = &m_Matrix;
    series[5].label = (char*)slabel_6.c_str();
    series[5].units = (char*)units_6.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      6
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  //friend
  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X,                         //!< The x series must be [Nx1].
    Matrix &Y,                         //!< The x series must be [Nx1].
    const std::string series_label,    //!< The series label.
    const std::string units,           //!< The series units.
    const bool isConnected,     //!< Are the data points connected.
    const MTX_enumColor color,  //!< The color of the data points/line.
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm   //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series;

    Matrix M;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( X.ncols() > 1 && X.nrows() == 1 )
    {
      M = X.Transpose();
    }
    else
    {
      M = X;
    }
    if( Y.ncols() > 1 && Y.nrows() == 1 )
    {
      if( !M.Concatonate( Y.Transpose() ) )
        return false;
    }
    else
    {
      if( !M.Concatonate( Y ) )
        return false;
    }

    series.connected = isConnected;
    series.markOutlierData = FALSE;
    series.precision = precisionStats;
    series.x_col = 0;
    series.y_col = 1;
    series.color = color;
    series.M = &(M.m_Matrix);
    series.label = (char*)series_label.c_str();
    series.units = (char*)units.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      &series,
      1
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }


  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X_1,                         //!< The x series must be [Nx1].
    Matrix &Y_1,                         //!< The x series must be [Nx1].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The x series must be [Nx1].
    Matrix &Y_2,                         //!< The x series must be [Nx1].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    const bool isConnected_1,     //!< Are the data points connected.
    const MTX_enumColor color_1,  //!< The color of the data points/line.
    const bool isConnected_2,     //!< Are the data points connected.
    const MTX_enumColor color_2,  //!< The color of the data points/line.    
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm  //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series[2];

    Matrix M1;
    Matrix M2;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;

    if( X_1.ncols() > 1 && X_1.nrows() == 1 )
    {
      M1 = X_1.Transpose();
    }
    else
    {
      M1 = X_1;
    }
    if( Y_1.ncols() > 1 && Y_1.nrows() == 1 )
    {
      if( !M1.Concatonate( Y_1.Transpose() ) )
        return false;
    }
    else
    {
      if( !M1.Concatonate( Y_1 ) )
        return false;
    }

    if( X_2.ncols() > 2 && X_2.nrows() == 2 )
    {
      M2 = X_2.Transpose();
    }
    else
    {
      M2 = X_2;
    }
    if( Y_2.ncols() > 2 && Y_2.nrows() == 2 )
    {
      if( !M2.Concatonate( Y_2.Transpose() ) )
        return false;
    }
    else
    {
      if( !M2.Concatonate( Y_2 ) )
        return false;
    }

    series[0].connected = isConnected_1;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = 0;
    series[0].y_col = 1;
    series[0].color = color_1;
    series[0].M = &(M1.m_Matrix);
    series[0].label = (char*)series_label_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = isConnected_2;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = 0;
    series[1].y_col = 1;
    series[1].color = color_2;
    series[1].M = &(M2.m_Matrix);
    series[1].label = (char*)series_label_2.c_str();
    series[1].units = (char*)units_2.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      2
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X_1,                         //!< The x series must be [Nx1].
    Matrix &Y_1,                         //!< The x series must be [Nx1].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The x series must be [Nx1].
    Matrix &Y_2,                         //!< The x series must be [Nx1].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The x series must be [Nx1].
    Matrix &Y_3,                         //!< The x series must be [Nx1].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    const bool isConnected_1,     //!< Are the data points connected.
    const MTX_enumColor color_1,  //!< The color of the data points/line.
    const bool isConnected_2,     //!< Are the data points connected.
    const MTX_enumColor color_2,  //!< The color of the data points/line.    
    const bool isConnected_3,     //!< Are the data points connected.
    const MTX_enumColor color_3,  //!< The color of the data points/line.        
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm   //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series[3];

    Matrix M1;
    Matrix M2;
    Matrix M3;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( X_1.ncols() > 1 && X_1.nrows() == 1 )
    {
      M1 = X_1.Transpose();
    }
    else
    {
      M1 = X_1;
    }
    if( Y_1.ncols() > 1 && Y_1.nrows() == 1 )
    {
      if( !M1.Concatonate( Y_1.Transpose() ) )
        return false;
    }
    else
    {
      if( !M1.Concatonate( Y_1 ) )
        return false;
    }

    if( X_2.ncols() > 2 && X_2.nrows() == 2 )
    {
      M2 = X_2.Transpose();
    }
    else
    {
      M2 = X_2;
    }
    if( Y_2.ncols() > 2 && Y_2.nrows() == 2 )
    {
      if( !M2.Concatonate( Y_2.Transpose() ) )
        return false;
    }
    else
    {
      if( !M2.Concatonate( Y_2 ) )
        return false;
    }

    if( X_3.ncols() > 3 && X_3.nrows() == 3 )
    {
      M3 = X_3.Transpose();
    }
    else
    {
      M3 = X_3;
    }
    if( Y_3.ncols() > 3 && Y_3.nrows() == 3 )
    {
      if( !M3.Concatonate( Y_3.Transpose() ) )
        return false;
    }
    else
    {
      if( !M3.Concatonate( Y_3 ) )
        return false;
    }

    series[0].connected = isConnected_1;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = 0;
    series[0].y_col = 1;
    series[0].color = color_1;
    series[0].M = &(M1.m_Matrix);
    series[0].label = (char*)series_label_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = isConnected_2;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = 0;
    series[1].y_col = 1;
    series[1].color = color_2;
    series[1].M = &(M2.m_Matrix);
    series[1].label = (char*)series_label_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = isConnected_3;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = 0;
    series[2].y_col = 1;
    series[2].color = color_3;
    series[2].M = &(M3.m_Matrix);
    series[2].label = (char*)series_label_3.c_str();
    series[2].units = (char*)units_3.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      3
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X_1,                         //!< The x series must be [Nx1].
    Matrix &Y_1,                         //!< The x series must be [Nx1].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The x series must be [Nx1].
    Matrix &Y_2,                         //!< The x series must be [Nx1].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The x series must be [Nx1].
    Matrix &Y_3,                         //!< The x series must be [Nx1].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The x series must be [Nx1].
    Matrix &Y_4,                         //!< The x series must be [Nx1].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    const bool isConnected_1,     //!< Are the data points connected.
    const MTX_enumColor color_1,  //!< The color of the data points/line.
    const bool isConnected_2,     //!< Are the data points connected.
    const MTX_enumColor color_2,  //!< The color of the data points/line.    
    const bool isConnected_3,     //!< Are the data points connected.
    const MTX_enumColor color_3,  //!< The color of the data points/line.        
    const bool isConnected_4,     //!< Are the data points connected.
    const MTX_enumColor color_4,  //!< The color of the data points/line.        
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm   //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series[4];

    Matrix M1;
    Matrix M2;
    Matrix M3;
    Matrix M4;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( X_1.ncols() > 1 && X_1.nrows() == 1 )
    {
      M1 = X_1.Transpose();
    }
    else
    {
      M1 = X_1;
    }
    if( Y_1.ncols() > 1 && Y_1.nrows() == 1 )
    {
      if( !M1.Concatonate( Y_1.Transpose() ) )
        return false;
    }
    else
    {
      if( !M1.Concatonate( Y_1 ) )
        return false;
    }

    if( X_2.ncols() > 2 && X_2.nrows() == 2 )
    {
      M2 = X_2.Transpose();
    }
    else
    {
      M2 = X_2;
    }
    if( Y_2.ncols() > 2 && Y_2.nrows() == 2 )
    {
      if( !M2.Concatonate( Y_2.Transpose() ) )
        return false;
    }
    else
    {
      if( !M2.Concatonate( Y_2 ) )
        return false;
    }

    if( X_3.ncols() > 3 && X_3.nrows() == 3 )
    {
      M3 = X_3.Transpose();
    }
    else
    {
      M3 = X_3;
    }
    if( Y_3.ncols() > 3 && Y_3.nrows() == 3 )
    {
      if( !M3.Concatonate( Y_3.Transpose() ) )
        return false;
    }
    else
    {
      if( !M3.Concatonate( Y_3 ) )
        return false;
    }

    if( X_4.ncols() > 4 && X_4.nrows() == 4 )
    {
      M4 = X_4.Transpose();
    }
    else
    {
      M4 = X_4;
    }
    if( Y_4.ncols() > 4 && Y_4.nrows() == 4 )
    {
      if( !M4.Concatonate( Y_4.Transpose() ) )
        return false;
    }
    else
    {
      if( !M4.Concatonate( Y_4 ) )
        return false;
    }

    series[0].connected = isConnected_1;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = 0;
    series[0].y_col = 1;
    series[0].color = color_1;
    series[0].M = &(M1.m_Matrix);
    series[0].label = (char*)series_label_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = isConnected_2;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = 0;
    series[1].y_col = 1;
    series[1].color = color_2;
    series[1].M = &(M2.m_Matrix);
    series[1].label = (char*)series_label_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = isConnected_3;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = 0;
    series[2].y_col = 1;
    series[2].color = color_3;
    series[2].M = &(M3.m_Matrix);
    series[2].label = (char*)series_label_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = isConnected_4;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = 0;
    series[3].y_col = 1;
    series[3].color = color_4;
    series[3].M = &(M4.m_Matrix);
    series[3].label = (char*)series_label_4.c_str();
    series[3].units = (char*)units_4.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      4
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X_1,                         //!< The x series must be [Nx1].
    Matrix &Y_1,                         //!< The x series must be [Nx1].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The x series must be [Nx1].
    Matrix &Y_2,                         //!< The x series must be [Nx1].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The x series must be [Nx1].
    Matrix &Y_3,                         //!< The x series must be [Nx1].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The x series must be [Nx1].
    Matrix &Y_4,                         //!< The x series must be [Nx1].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    Matrix &X_5,                         //!< The x series must be [Nx1].
    Matrix &Y_5,                         //!< The x series must be [Nx1].
    const std::string series_label_5,    //!< The series label.
    const std::string units_5,           //!< The series units.
    const bool isConnected_1,     //!< Are the data points connected.
    const MTX_enumColor color_1,  //!< The color of the data points/line.
    const bool isConnected_2,     //!< Are the data points connected.
    const MTX_enumColor color_2,  //!< The color of the data points/line.    
    const bool isConnected_3,     //!< Are the data points connected.
    const MTX_enumColor color_3,  //!< The color of the data points/line.        
    const bool isConnected_4,     //!< Are the data points connected.
    const MTX_enumColor color_4,  //!< The color of the data points/line.        
    const bool isConnected_5,     //!< Are the data points connected.
    const MTX_enumColor color_5,  //!< The color of the data points/line.        
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm   //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series[5];

    Matrix M1;
    Matrix M2;
    Matrix M3;
    Matrix M4;
    Matrix M5;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( X_1.ncols() > 1 && X_1.nrows() == 1 )
    {
      M1 = X_1.Transpose();
    }
    else
    {
      M1 = X_1;
    }
    if( Y_1.ncols() > 1 && Y_1.nrows() == 1 )
    {
      if( !M1.Concatonate( Y_1.Transpose() ) )
        return false;
    }
    else
    {
      if( !M1.Concatonate( Y_1 ) )
        return false;
    }

    if( X_2.ncols() > 2 && X_2.nrows() == 2 )
    {
      M2 = X_2.Transpose();
    }
    else
    {
      M2 = X_2;
    }
    if( Y_2.ncols() > 2 && Y_2.nrows() == 2 )
    {
      if( !M2.Concatonate( Y_2.Transpose() ) )
        return false;
    }
    else
    {
      if( !M2.Concatonate( Y_2 ) )
        return false;
    }

    if( X_3.ncols() > 3 && X_3.nrows() == 3 )
    {
      M3 = X_3.Transpose();
    }
    else
    {
      M3 = X_3;
    }
    if( Y_3.ncols() > 3 && Y_3.nrows() == 3 )
    {
      if( !M3.Concatonate( Y_3.Transpose() ) )
        return false;
    }
    else
    {
      if( !M3.Concatonate( Y_3 ) )
        return false;
    }

    if( X_4.ncols() > 4 && X_4.nrows() == 4 )
    {
      M4 = X_4.Transpose();
    }
    else
    {
      M4 = X_4;
    }
    if( Y_4.ncols() > 4 && Y_4.nrows() == 4 )
    {
      if( !M4.Concatonate( Y_4.Transpose() ) )
        return false;
    }
    else
    {
      if( !M4.Concatonate( Y_4 ) )
        return false;
    }

    if( X_5.ncols() > 5 && X_5.nrows() == 5 )
    {
      M5 = X_5.Transpose();
    }
    else
    {
      M5 = X_5;
    }
    if( Y_5.ncols() > 5 && Y_5.nrows() == 5 )
    {
      if( !M5.Concatonate( Y_5.Transpose() ) )
        return false;
    }
    else
    {
      if( !M5.Concatonate( Y_5 ) )
        return false;
    }

    series[0].connected = isConnected_1;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = 0;
    series[0].y_col = 1;
    series[0].color = color_1;
    series[0].M = &(M1.m_Matrix);
    series[0].label = (char*)series_label_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = isConnected_2;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = 0;
    series[1].y_col = 1;
    series[1].color = color_2;
    series[1].M = &(M2.m_Matrix);
    series[1].label = (char*)series_label_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = isConnected_3;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = 0;
    series[2].y_col = 1;
    series[2].color = color_3;
    series[2].M = &(M3.m_Matrix);
    series[2].label = (char*)series_label_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = isConnected_4;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = 0;
    series[3].y_col = 1;
    series[3].color = color_4;
    series[3].M = &(M4.m_Matrix);
    series[3].label = (char*)series_label_4.c_str();
    series[3].units = (char*)units_4.c_str();

    series[4].connected = isConnected_5;
    series[4].markOutlierData = FALSE;
    series[4].precision = precisionStats;
    series[4].x_col = 0;
    series[4].y_col = 1;
    series[4].color = color_5;
    series[4].M = &(M5.m_Matrix);
    series[4].label = (char*)series_label_5.c_str();
    series[4].units = (char*)units_5.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      5
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

  bool Plot(
    const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
    const std::string title,           //!< The plot title.
    const std::string xlabel,          //!< The x-axis label.
    const std::string ylabel,          //!< The y-axis label.
    Matrix &X_1,                         //!< The x series must be [Nx1].
    Matrix &Y_1,                         //!< The x series must be [Nx1].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The x series must be [Nx1].
    Matrix &Y_2,                         //!< The x series must be [Nx1].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The x series must be [Nx1].
    Matrix &Y_3,                         //!< The x series must be [Nx1].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The x series must be [Nx1].
    Matrix &Y_4,                         //!< The x series must be [Nx1].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    Matrix &X_5,                         //!< The x series must be [Nx1].
    Matrix &Y_5,                         //!< The x series must be [Nx1].
    const std::string series_label_5,    //!< The series label.
    const std::string units_5,           //!< The series units.
    Matrix &X_6,                         //!< The x series must be [Nx1].
    Matrix &Y_6,                         //!< The x series must be [Nx1].
    const std::string series_label_6,    //!< The series label.
    const std::string units_6,           //!< The series units.
    const bool isConnected_1,     //!< Are the data points connected.
    const MTX_enumColor color_1,  //!< The color of the data points/line.
    const bool isConnected_2,     //!< Are the data points connected.
    const MTX_enumColor color_2,  //!< The color of the data points/line.    
    const bool isConnected_3,     //!< Are the data points connected.
    const MTX_enumColor color_3,  //!< The color of the data points/line.        
    const bool isConnected_4,     //!< Are the data points connected.
    const MTX_enumColor color_4,  //!< The color of the data points/line.        
    const bool isConnected_5,     //!< Are the data points connected.
    const MTX_enumColor color_5,  //!< The color of the data points/line.        
    const bool isConnected_6,     //!< Are the data points connected.
    const MTX_enumColor color_6,  //!< The color of the data points/line.        
    const bool isXGridOn,       //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn,       //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats,    //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats, //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm, //!< The plot height in cm.
    const unsigned plot_width_cm   //!< The plot width in cm.
    )
  {
    MTX_structAxisOptions x;
    MTX_structAxisOptions y;

    MTX_PLOT_structSeries series[6];

    Matrix M1;
    Matrix M2;
    Matrix M3;
    Matrix M4;
    Matrix M5;
    Matrix M6;

    x.lowerlimit.doNotUseDefault = FALSE;
    x.ticksize.doNotUseDefault = FALSE;
    x.tickend.doNotUseDefault = FALSE;
    x.tickstart.doNotUseDefault = FALSE;
    x.upperlimit.doNotUseDefault = FALSE;
    
    y.lowerlimit.doNotUseDefault = FALSE;
    y.ticksize.doNotUseDefault = FALSE;
    y.tickend.doNotUseDefault = FALSE;
    y.tickstart.doNotUseDefault = FALSE;
    y.upperlimit.doNotUseDefault = FALSE;
    
    if( X_1.ncols() > 1 && X_1.nrows() == 1 )
    {
      M1 = X_1.Transpose();
    }
    else
    {
      M1 = X_1;
    }
    if( Y_1.ncols() > 1 && Y_1.nrows() == 1 )
    {
      if( !M1.Concatonate( Y_1.Transpose() ) )
        return false;
    }
    else
    {
      if( !M1.Concatonate( Y_1 ) )
        return false;
    }

    if( X_2.ncols() > 2 && X_2.nrows() == 2 )
    {
      M2 = X_2.Transpose();
    }
    else
    {
      M2 = X_2;
    }
    if( Y_2.ncols() > 2 && Y_2.nrows() == 2 )
    {
      if( !M2.Concatonate( Y_2.Transpose() ) )
        return false;
    }
    else
    {
      if( !M2.Concatonate( Y_2 ) )
        return false;
    }

    if( X_3.ncols() > 3 && X_3.nrows() == 3 )
    {
      M3 = X_3.Transpose();
    }
    else
    {
      M3 = X_3;
    }
    if( Y_3.ncols() > 3 && Y_3.nrows() == 3 )
    {
      if( !M3.Concatonate( Y_3.Transpose() ) )
        return false;
    }
    else
    {
      if( !M3.Concatonate( Y_3 ) )
        return false;
    }

    if( X_4.ncols() > 4 && X_4.nrows() == 4 )
    {
      M4 = X_4.Transpose();
    }
    else
    {
      M4 = X_4;
    }
    if( Y_4.ncols() > 4 && Y_4.nrows() == 4 )
    {
      if( !M4.Concatonate( Y_4.Transpose() ) )
        return false;
    }
    else
    {
      if( !M4.Concatonate( Y_4 ) )
        return false;
    }

    if( X_5.ncols() > 5 && X_5.nrows() == 5 )
    {
      M5 = X_5.Transpose();
    }
    else
    {
      M5 = X_5;
    }
    if( Y_5.ncols() > 5 && Y_5.nrows() == 5 )
    {
      if( !M5.Concatonate( Y_5.Transpose() ) )
        return false;
    }
    else
    {
      if( !M5.Concatonate( Y_5 ) )
        return false;
    }

    if( X_6.ncols() > 6 && X_6.nrows() == 6 )
    {
      M6 = X_6.Transpose();
    }
    else
    {
      M6 = X_6;
    }
    if( Y_6.ncols() > 6 && Y_6.nrows() == 6 )
    {
      if( !M6.Concatonate( Y_6.Transpose() ) )
        return false;
    }
    else
    {
      if( !M6.Concatonate( Y_6 ) )
        return false;
    }

    series[0].connected = isConnected_1;
    series[0].markOutlierData = FALSE;
    series[0].precision = precisionStats;
    series[0].x_col = 0;
    series[0].y_col = 1;
    series[0].color = color_1;
    series[0].M = &(M1.m_Matrix);
    series[0].label = (char*)series_label_1.c_str();
    series[0].units = (char*)units_1.c_str();

    series[1].connected = isConnected_2;
    series[1].markOutlierData = FALSE;
    series[1].precision = precisionStats;
    series[1].x_col = 0;
    series[1].y_col = 1;
    series[1].color = color_2;
    series[1].M = &(M2.m_Matrix);
    series[1].label = (char*)series_label_2.c_str();
    series[1].units = (char*)units_2.c_str();

    series[2].connected = isConnected_3;
    series[2].markOutlierData = FALSE;
    series[2].precision = precisionStats;
    series[2].x_col = 0;
    series[2].y_col = 1;
    series[2].color = color_3;
    series[2].M = &(M3.m_Matrix);
    series[2].label = (char*)series_label_3.c_str();
    series[2].units = (char*)units_3.c_str();

    series[3].connected = isConnected_4;
    series[3].markOutlierData = FALSE;
    series[3].precision = precisionStats;
    series[3].x_col = 0;
    series[3].y_col = 1;
    series[3].color = color_4;
    series[3].M = &(M4.m_Matrix);
    series[3].label = (char*)series_label_4.c_str();
    series[3].units = (char*)units_4.c_str();

    series[4].connected = isConnected_5;
    series[4].markOutlierData = FALSE;
    series[4].precision = precisionStats;
    series[4].x_col = 0;
    series[4].y_col = 1;
    series[4].color = color_5;
    series[4].M = &(M5.m_Matrix);
    series[4].label = (char*)series_label_5.c_str();
    series[4].units = (char*)units_5.c_str();

    series[5].connected = isConnected_6;
    series[5].markOutlierData = FALSE;
    series[5].precision = precisionStats;
    series[5].x_col = 0;
    series[5].y_col = 1;
    series[5].color = color_6;
    series[5].M = &(M6.m_Matrix);
    series[5].label = (char*)series_label_6.c_str();
    series[5].units = (char*)units_6.c_str();
    
    if( !MTX_Plot(
      bmpfilename.c_str(), 
      title.c_str(), 
      plot_height_cm, 
      plot_width_cm, 
      includeStats, 
      isXGridOn, 
      isYGridOn, 
      xlabel.c_str(), 
      ylabel.c_str(),
      x,
      y,
      series,
      6
      ) )
    {
      MTX_ERROR_MSG( "MTX_Plot returned FALSE." );
      return false;
    }

    return true;
  }

}

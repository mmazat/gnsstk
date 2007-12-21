/**
\file     Matrix.cpp
\brief    The Zenautics Matrix Class
\author   Glenn D. MacGougan (GDM)
\date     2007-12-21
\version  1.11

\b LICENSE \b INFORMATION \n
Copyright (c) 2007, Glenn D. MacGougan, Zenautics Technologies Inc. \n

Redistribution pertains only to the following files and their contents. \n
- Matrix.h\n
- Matrix.cpp\n
- cmatrix.h\n
- cmatrix_basic.lib (for windows), cmatrix_basic_lib.a (for linux)\n

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
inevitable. Please report bugs and suggested fixes to glenn@zenautics.com.\n
*/


#include <stdlib.h>
#include <string.h>

#include "Matrix.h"
#include "cmatrix.h"

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










  // default constructor
  Matrix::Matrix()
    :m_MatrixElement(m_Matrix)
  {
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
    if( !m_IsMTXInitialized )
    {
      if( !MTX_Initialize_MTXEngine() )
      {
        m_IsMTXInitialized = false;
        MatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
      }
      else
      {
        m_IsMTXInitialized = true;
      }
    }

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
        MatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
        MatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
        MatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
      return true;
    else
      return false;    
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


#ifdef MATRIX_USE_EXCEPTION_HANDLING

    throw MatrixException(msg);
    return;

#else

#ifdef USING_MFC
    CString errMsg = msg;
    AfxMessageBox(errMsg);   
#else
    printf( "%s\r\n", msg );   
#endif

    // no choice but to call exit!
    exit(1);
#endif    
  }


  bool Matrix::isEmpty() const
  {
    if( MTX_isNull( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::isConformal(const Matrix& mat) const
  {
    if( MTX_isConformalForMultiplication( &m_Matrix, &mat.m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::isSameSize(const Matrix& mat) const
  {
    if( MTX_isSameSize( &m_Matrix, &mat.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::isSquare() const
  {
    if( MTX_isSquare( &m_Matrix ) )
      return true;
    else 
      return false;
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
      return true;
    else 
      return false;
  }


  bool Matrix::ReadFromFile( std::string path )
  {
    return ReadFromFile( path.c_str() );
  }


  bool Matrix::Copy( Matrix& src )
  {
    if( MTX_Copy( &src.m_Matrix, &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Copy( const double& value )
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, true ) )
      return false;

    if( MTX_SetValue( &m_Matrix, 0, 0, value ) )
      return true;
    else
      return false;
  }

  bool Matrix::Copy( const std::complex<double>& cplx )
  {
    if( !MTX_Malloc( &m_Matrix, 1, 1, false ) )
      return false;

    if( MTX_SetComplexValue( &m_Matrix, 0, 0, cplx.real(), cplx.imag() ) )
      return true;
    else
      return false;
  }

  bool Matrix::Save( const char* path )
  {
    if( MTX_SaveCompressed( &m_Matrix, path ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Save( std::string path )
  {
    return Save( path.c_str() );
  }

  bool Matrix::Print( const char *path, const unsigned precision, bool append )
  {
    if( MTX_PrintAutoWidth( &m_Matrix, path, precision, append ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Print( std::string path, const unsigned precision, bool append ) 
  {
    return Print( path.c_str(), precision );
  }

  bool Matrix::PrintStdout( const unsigned precision )
  {
    if( MTX_PrintStdoutAutoWidth( &m_Matrix, precision ) )
      return true;
    else
      return false;
  }

  bool Matrix::PrintToBuffer( char* buffer, const unsigned maxlength, const unsigned precision )
  {
    if( MTX_PrintAutoWidth_ToBuffer( &m_Matrix, buffer, maxlength, precision ) )
      return true;
    else 
      return false;
  }

  bool Matrix::PrintFixedWidth( const char* path, const unsigned width, const unsigned precision, bool append )
  {
    if( MTX_Print( &m_Matrix, path, width, precision, append ) )
      return true;
    else 
      return false;
  }

  bool Matrix::PrintFixedWidth( std::string path, const unsigned width, const unsigned precision, bool append )
  {
    return PrintFixedWidth( path.c_str(), width, precision, append );
  }

  bool Matrix::PrintFixedWidthToBuffer( char* buffer, const unsigned maxlength, const unsigned width, const unsigned precision )
  {
    if( MTX_Print_ToBuffer( &m_Matrix, buffer, maxlength, width, precision ) )
      return true;
    else 
      return false;
  }

  bool Matrix::PrintDelimited( const char *path, const unsigned precision, const char delimiter, bool append )
  {
    if( MTX_PrintDelimited( &m_Matrix, path, precision, delimiter, append ) )
      return true;
    else 
      return false;
  }

  bool Matrix::PrintDelimited( std::string path, const unsigned precision, const char delimiter, bool append )
  {
    return PrintDelimited( path.c_str(), precision, delimiter, append );
  }
    

  bool Matrix::PrintDelimitedToBuffer( char *buffer, const unsigned maxlength, const unsigned precision, const char delimiter )
  {
    if( MTX_PrintDelimited_ToBuffer( &m_Matrix, buffer, maxlength, precision, delimiter ) )
      return true;
    else 
      return false;
  }

  bool Matrix::PrintRowToString( const unsigned row, char *buffer, const unsigned maxlength, const int width, const int precision )
  {
    if( MTX_PrintRowToString( &m_Matrix, row, buffer, maxlength, width, precision ) )
      return true;
    else
      return false;
  }


  bool Matrix::RemoveColumn( const unsigned col )
  {
    if( MTX_RemoveColumn( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::RemoveColumnsAfterIndex( const unsigned col )
  {
    if( MTX_RemoveColumnsAfterIndex( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::RemoveRowsAndColumns( const unsigned nrows, const unsigned rows[], const unsigned ncols, const unsigned cols[] )
  {
    if( MTX_RemoveRowsAndColumns( &m_Matrix, nrows, rows, ncols, cols ) )
      return true;
    else
      return false;
  }

  bool Matrix::InsertColumn( const Matrix &src, const unsigned dst_col, const unsigned src_col )
  {
    if( MTX_InsertColumn( &m_Matrix, &src.m_Matrix, dst_col, src_col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::AddColumn( const Matrix &src, const unsigned src_col )
  {
    if( MTX_AddColumn( &m_Matrix, &src.m_Matrix, src_col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Concatonate( const Matrix &src )
  {
    if( MTX_Concatonate( &m_Matrix, &src.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Redim( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Redim( &m_Matrix, nrows, ncols ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Resize( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Resize( &m_Matrix, nrows, ncols, m_Matrix.isReal ) )
      return true;
    else 
      return false;
  }


  bool Matrix::SetFromStaticMatrix( const double mat[], const unsigned nrows, const unsigned ncols )
  {
    if( MTX_SetFromStaticMatrix( &m_Matrix, mat, nrows, ncols ) )
      return true;
    else 
      return false;
  }

  bool Matrix::SetFromMatrixString(const char* strMatrix)
  {
    if( MTX_SetFromMatrixString( &m_Matrix, strMatrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::CopyColumn( const unsigned src_col, Matrix &dst )
  {
    if( MTX_CopyColumn( &m_Matrix, src_col, &dst.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::InsertSubMatrix( const Matrix &src, const unsigned dst_row, const unsigned dst_col )
  {
    if( MTX_InsertSubMatrix( &m_Matrix, &src.m_Matrix, dst_row, dst_col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Zero()
  {
    if( MTX_Zero( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::ZeroColumn( const unsigned col )
  {
    if( MTX_ZeroColumn( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::ZeroRow( const unsigned row )
  {
    if( MTX_ZeroRow( &m_Matrix, row ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Fill( const double value )
  {
    if( MTX_Fill( &m_Matrix, value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::FillColumn( const unsigned col, const double value )
  {
    if( MTX_FillColumn( &m_Matrix, col, value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::FillRow( const unsigned row, const double value )
  {
    if( MTX_FillRow( &m_Matrix, row, value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::FlipColumn( const unsigned col )
  {
    if( MTX_FlipColumn( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::FlipRow( const unsigned row )
  {
    if( MTX_FlipRow( &m_Matrix, row ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Identity()
  {
    if( MTX_Identity( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Identity(const unsigned dimension)
  {
    if( MTX_Malloc( &m_Matrix, dimension, dimension, true ) )
    {
      if( MTX_Identity( &m_Matrix ) )
        return true;
      else
        return false;
    }
    else
    {
      return false;
    }
  }

  bool Matrix::Inplace_Transpose()
  {
    if( MTX_TransposeInplace( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Round( const unsigned precision )
  {
    if( MTX_Round( &m_Matrix, precision ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Floor()
  {
    if( MTX_Floor( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Ceil()
  {
    if( MTX_Ceil( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Fix()
  {
    if( MTX_Fix( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_AddScalar( const double scalar )
  {
    if( MTX_Add_Scalar( &m_Matrix, scalar ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SubtractScalar( const double scalar )
  {
    if( MTX_Subtract_Scalar( &m_Matrix, scalar ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_MultiplyScalar( const double scalar )
  {
    if( MTX_Multiply_Scalar( &m_Matrix, scalar ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_DivideScalar( const double scalar )
  {
    if( MTX_Divide_Scalar( &m_Matrix, scalar ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_PowerScalar( const double scalar )
  {
    if( MTX_PowInplace( &m_Matrix, scalar, 0.0 ) ) 
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_AddScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Add_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SubtractScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Subtract_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_MultiplyScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Multiply_ScalarComplex( &m_Matrix, cplx.real(), cplx.imag() ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_DivideScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_Divide_ScalarComplex( &m_Matrix,  cplx.real(), cplx.imag() ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_PowerScalarComplex( const std::complex<double> cplx )
  {
    if( MTX_PowInplace( &m_Matrix, cplx.real(), cplx.imag() ) ) 
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_Abs()
  {
    if( MTX_Abs( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_acos()
  {
    if( MTX_acos( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_acosd()
  {
    if( MTX_acos( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
        return true;
      else
        return false;
    }
    else 
    {
      return false;
    }
  }

  bool Matrix::Inplace_acosh()
  {
    if( MTX_acosh( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_angle()
  {
    if( MTX_angle( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_asin()
  {
    if( MTX_asin( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_asind()
  {
    if( MTX_asin( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
        return true;
      else
        return false;
    }
    else 
    {
      return false;
    }
  }

  bool Matrix::Inplace_asinh()
  {
    if( MTX_asinh( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_atan()
  {
    if( MTX_atan( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_atand()
  {
    if( MTX_atan( &m_Matrix ) )
    {
      if( MTX_Multiply_Scalar( &m_Matrix, RAD2DEG ) )
        return true;
      else
        return false;
    }
    else 
    {
      return false;
    }
  }

  bool Matrix::Inplace_atanh()
  {
    if( MTX_atanh( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_Sqr()
  {
    if( MTX_Sqr( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Sqrt()
  {
    if( MTX_Sqrt( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Exp()
  {
    if( MTX_Exp( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Ln()
  {
    if( MTX_Ln( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Increment()
  {
    if( MTX_Increment( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Decrement()
  {
    if( MTX_Decrement( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Add( const Matrix &B )
  {
    if( MTX_Add_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Subtract( const Matrix &B )
  {
    if( MTX_Subtract_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_PreMultiply( const Matrix &B )
  {
    if( MTX_PreMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_PostMultiply( const Matrix &B )
  {
    if( MTX_PostMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_DotMultiply( const Matrix &B )
  {
    if( MTX_DotMultiply_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_DotDivide( const Matrix &B )
  {
    if( MTX_DotDivide_Inplace( &m_Matrix, &B.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortAscending()
  {
    if( MTX_SortAscending( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortDescending()
  {
    if( MTX_SortDescending( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortColumnAscending( const unsigned col )
  {
    if( MTX_SortColumnAscending( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortColumnDescending( const unsigned col )
  {
    if( MTX_SortColumnDescending( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortColumnIndexed( const unsigned col, Matrix &Index )
  {
    if( MTX_SortColumnIndexed( &m_Matrix, col, &Index.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_SortByColumn( const unsigned col )
  {
    if( MTX_SortByColumn( &m_Matrix, col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_Invert()
  {
    if( MTX_InvertInPlace( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_InvertRobust()
  {
    if( MTX_InvertInPlaceRobust( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_LowerTriangularInverse()
  {
    if( MTX_LowerTriangularInverseInplace( &m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Inplace_FFT()
  {
    if( MTX_FFT_Inplace( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_IFFT()
  {
    if( MTX_IFFT_Inplace( &m_Matrix ) )
      return true;
    else
      return false;
  }


  bool Matrix::Add( const Matrix &B, const Matrix &C )
  {
    if( MTX_Add( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
      return true;
    else 
      return false;

  }

  bool Matrix::Subtract( const Matrix &B, const Matrix &C )
  {
    if( MTX_Subtract( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::Multiply( const Matrix &B, const Matrix &C )
  {
    if( MTX_Multiply( &m_Matrix, &B.m_Matrix, &C.m_Matrix ) )
      return true;
    else 
      return false;
  }


  bool Matrix::Inplace_abs()
  {
    if( MTX_Abs( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_colon( double start, double increment, double end )
  {
    if( MTX_Colon( &m_Matrix, start, increment, end) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_conj()
  {
    if( MTX_Conjugate( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_cos()
  {
    if( MTX_cos( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_cosh()
  {
    if( MTX_cosh( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_cot()
  {
    if( MTX_cot( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_coth()
  {
    if( MTX_coth( &m_Matrix ) )
      return true;
    else
      return false;
  }
  
  bool Matrix::Inplace_imag()
  {
    if( MTX_ConvertComplexToImag( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_exp()
  {
    if( MTX_Exp( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_eye( const unsigned nrows, const unsigned ncols )
  {
    if( MTX_Eye( &m_Matrix, nrows, ncols ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_log2()
  {
    if( MTX_Ln( &m_Matrix ) )
    {
      if( MTX_Divide_Scalar( &m_Matrix, log(2.0) ) )
        return true;
      else
        return false;
    }
    else
    {
      return false;
    }
  }

  bool Matrix::Inplace_log10()
  {
    if( MTX_Ln( &m_Matrix ) )
    {
      if( MTX_Divide_Scalar( &m_Matrix, log(10.0) ) )
        return true;
      else
        return false;
    }
    else
    {
      return false;
    }
  }


  bool Matrix::Inplace_ones( const unsigned nrows, const unsigned ncols )
  {
    if( m_Matrix.nrows == nrows && m_Matrix.ncols == ncols && m_Matrix.isReal )
    { 
      if( !MTX_Fill( &m_Matrix, 1.0 ) )
        return false;
    }
    else
    {
      if( !MTX_Malloc( &m_Matrix, nrows, ncols, TRUE ) )
        return false;
      if( !MTX_Fill( &m_Matrix, 1.0 ) )
        return false;
    }
    return true;
  }

  bool Matrix::Inplace_real()
  {
    if( MTX_ConvertComplexToReal( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_sin()
  {
    if( MTX_sin( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_sinc()
  {
    if( MTX_sinc( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_sinh()
  {
    if( MTX_sinh( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_sqrt()
  {
    if( MTX_Sqrt( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_tan()
  {
    if( MTX_tan( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_tanh()
  {
    if( MTX_tanh( &m_Matrix ) )
      return true;
    else
      return false;
  }

  bool Matrix::Inplace_zeros( const unsigned nrows, const unsigned ncols )
  {
    if( m_Matrix.nrows == nrows && m_Matrix.ncols == ncols && m_Matrix.isReal )
    { 
      if( !MTX_Fill( &m_Matrix, 0.0 ) )
        return false;
    }
    else
    {
      if( !MTX_Malloc( &m_Matrix, nrows, ncols, TRUE ) )
        return false;
      if( !MTX_Fill( &m_Matrix, 0.0 ) )
        return false;
    }
    return true;
  }
  

  bool Matrix::GetStats_MaxAbs(unsigned &row, unsigned &col, double &value )
  {
    if( MTX_MaxAbsIndex( &m_Matrix, &value, &row, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Max(unsigned &row, unsigned &col, double &re, double &im )
  {
    if( MTX_MaxIndex( &m_Matrix, &re, &im, &row, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxVal(double &re, double &im )
  {
    if( MTX_Max( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxAbsCol(const unsigned col, double &value, unsigned &row )
  {
    if( MTX_MaxAbsColIndex( &m_Matrix, col, &value, &row ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxCol(const unsigned col, double &re, double &im, unsigned &row )
  {
    if( MTX_MaxColIndex( &m_Matrix, col, &re, &im, &row ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxColVal(const unsigned col, double &re, double &im )
  {
    if( MTX_MaxColumn( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxAbsRow(const unsigned row, double &value, unsigned &col )
  {
    if( MTX_MaxAbsRowIndex( &m_Matrix, row, &value, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxRow(const unsigned row, double &re, double &im, unsigned &col )
  {
    if( MTX_MaxRowIndex( &m_Matrix, row, &re, &im, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MaxRowVal(const unsigned row, double &re, double &im )
  {
    if( MTX_MaxRow( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinAbs(unsigned &row, unsigned &col, double &value )
  {
    if( MTX_MinAbsIndex( &m_Matrix, &value, &row, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Min(unsigned &row, unsigned &col, double &re, double &im )
  {
    if( MTX_MinIndex( &m_Matrix, &re, &im, &row, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinVal(double &re, double &im )
  {
    if( MTX_Min( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinAbsCol(const unsigned col, double &value, unsigned &row )
  {
    if( MTX_MinAbsColIndex( &m_Matrix, col, &value, &row ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinCol(const unsigned col, double &re, double &im, unsigned &row )
  {
    if( MTX_MinColIndex( &m_Matrix, col, &re, &im, &row ) )
      return true;
    else 
      return false;
  }


  bool Matrix::GetStats_MinColVal(const unsigned col, double &re, double &im )
  {
    if( MTX_MinColumn( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinAbsRow(const unsigned row, double &value, unsigned &col )
  {
    if( MTX_MinAbsRowIndex( &m_Matrix, row, &value, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinRow(const unsigned row, double &re, double &im, unsigned &col )
  {
    if( MTX_MinRowIndex( &m_Matrix, row, &re, &im, &col ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_MinRowVal(const unsigned row, double &re, double &im )
  {
    if( MTX_MinRow(  &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColRange( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnRange( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowRange( const unsigned row, double &re, double &im )
  {
    if( MTX_RowRange( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Range( double &re, double &im )
  {
    if( MTX_Range( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnSum( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnSum( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowSum( const unsigned row, double &re, double &im )
  {
    if( MTX_RowSum( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Sum( double &re, double &im )
  {
    if( MTX_Sum( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnMean( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnMean( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowMean( const unsigned row, double &re, double &im )
  {
    if( MTX_RowMean( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Mean( double &re, double &im )
  {
    if( MTX_Mean( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnStdev( const unsigned col, double &value )
  {
    if( MTX_ColumnStdev( &m_Matrix, col, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowStdev( const unsigned row, double &value )
  {
    if( MTX_RowStdev( &m_Matrix, row, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Stdev( double &value )
  {
    if( MTX_Stdev( &m_Matrix, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnVar( const unsigned col, double &value )
  {
    if( MTX_ColumnVar( &m_Matrix, col, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowVar( const unsigned row, double &value )
  {
    if( MTX_RowVar( &m_Matrix, row, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Var( double &value )
  {
    if( MTX_Var( &m_Matrix, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnNorm( const unsigned col, double &value )
  {
    if( MTX_ColumnNorm( &m_Matrix, col, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowNorm( const unsigned row, double &value )
  {
    if( MTX_RowNorm( &m_Matrix, row, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Norm( double &value )
  {
    if( MTX_Norm( &m_Matrix, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnRMS( const unsigned col, double &value )
  {
    if( MTX_ColumnRMS( &m_Matrix, col, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowRMS( const unsigned row, double &value )
  {
    if( MTX_RowRMS( &m_Matrix, row, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RMS( double &value )
  {
    if( MTX_RMS( &m_Matrix, &value ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnSkewness( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnSkewness( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowSkewness( const unsigned row, double &re, double &im )
  {
    if( MTX_RowSkewness( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Skewness( double &re, double &im )
  {
    if( MTX_Skewness( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_ColumnKurtosis( const unsigned col, double &re, double &im )
  {
    if( MTX_ColumnKurtosis( &m_Matrix, col, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_RowKurtosis( const unsigned row, double &re, double &im )
  {
    if( MTX_RowKurtosis( &m_Matrix, row, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetStats_Kurtosis( double &re, double &im )
  {
    if( MTX_Kurtosis( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetTrace( double &re, double &im )
  {
    if( MTX_Trace( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetDeterminant( double &re, double &im )
  {
    if( MTX_Det( &m_Matrix, &re, &im ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetDiagonal( Matrix& DiagonalVector )
  {
    if( MTX_Diagonal( &m_Matrix, &DiagonalVector.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetColumnMovAvg( const unsigned col, const unsigned lead, const unsigned lag, Matrix &MovAvg )
  {
    if( MTX_ColumnMovAvg( &m_Matrix, col, lead, lag, &MovAvg.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetMovAvg( const unsigned lead, const unsigned lag, Matrix &MovAvg )
  {
    if( MTX_MovAvg( &m_Matrix, lead, lag, &MovAvg.m_Matrix ) )
      return true;
    else 
      return false;
  }

  bool Matrix::GetATAInverse( Matrix &InvATA )
  {
    if( MTX_ATAInverse( &m_Matrix, &InvATA.m_Matrix ) )
      return true;
    else 
      return false;
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
        return false;
    }
    if( !ColIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &ColIndex.m_Matrix, &_colIndex.m_Matrix ) )
        return false;
    }

    if( !_rowIndex.isEmpty() )
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_IndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &_colIndex.m_Matrix, &Result.m_Matrix ) )
          return true;
        else
          return false;
      }
      else
      {
        if( MTX_IndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &ColIndex.m_Matrix, &Result.m_Matrix ) )
          return true;
        else
          return false;
      }
    }
    else
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_IndexedValues( &m_Matrix, &RowIndex.m_Matrix, &_colIndex.m_Matrix, &Result.m_Matrix ) )
          return true;
        else
          return false;
      }
      else
      {
        if( MTX_IndexedValues( &m_Matrix, &RowIndex.m_Matrix, &ColIndex.m_Matrix, &Result.m_Matrix ) )
          return true;
        else
          return false;
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
        return false;
    }
    if( !ColIndex.m_Matrix.isReal )
    {
      if( !MTX_Real( &ColIndex.m_Matrix, &_colIndex.m_Matrix ) )
        return false;
    }

    if( !_rowIndex.isEmpty() )
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_SetIndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &_colIndex.m_Matrix, &SourceData.m_Matrix ) )
          return true;
        else
          return false;
      }
      else
      {
        if( MTX_SetIndexedValues( &m_Matrix, &_rowIndex.m_Matrix, &ColIndex.m_Matrix, &SourceData.m_Matrix ) )
          return true;
        else
          return false;
      }
    }
    else
    {
      if( !_colIndex.isEmpty() )
      {
        if( MTX_SetIndexedValues( &m_Matrix, &RowIndex.m_Matrix, &_colIndex.m_Matrix, &SourceData.m_Matrix ) )
          return true;
        else
          return false;
      }
      else
      {
        if( MTX_SetIndexedValues( &m_Matrix, &RowIndex.m_Matrix, &ColIndex.m_Matrix, &SourceData.m_Matrix ) )
          return true;
        else
          return false;
      }
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
      return true;
    else 
      return false;
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
      return true;
    else 
      return false;
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
        StaticMatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
      return true;
    else 
      return false;
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
        StaticMatrixError( "Matrix", "Failed to initialize the MTX Engine. Try commenting out #define MTX_SIMD_OPTIMIZED in cmatrix.h" );
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
      return true;
    else 
      return false;
  }



  // Return the column matrix specified by the column index. Returns (nrows x 1).
  Matrix  Matrix::Column(const unsigned col)
  {
    Matrix A;
    if( !MTX_CopyColumn( &m_Matrix, col, &A.m_Matrix ) )
    {
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
    if( !MTX_Copy( &m_Matrix, &A.m_Matrix ) )
    {
      MatrixError( "Inverse", "Unable to copy the source matrix." );
    }
    if( !MTX_InvertInPlace( &A.m_Matrix ) )
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



  /// Get a reference to an element in the matrix to set its value.
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


  /// Get a reference to an element in the matrix as a column or row vector to set its value.
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
      StaticMatrixError( "IndexCheck", msga );
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
      StaticMatrixError( "IndexCheck", msga );
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
    return false; // Should not be reached!
  }
  

}


//============================================================================
/// \file     Matrix.h
/// \brief    The matrix class
/// \author   Glenn D. MacGougan (GDM)
/// \date     2007-03-19
/// \version  1.04
///
/// \b LICENSE \b INFORMATION \n
/// Copyright (c) 2007, Glenn D. MacGougan, Zenautics Technologies Inc. \n
///
/// Redistribution pertains only to the following files and their contents. \n
/// - Matrix.h\n
/// - Matrix.cpp\n
/// - cmatrix.h\n
/// - cmatrix_basic.lib (for windows), cmatrix_basic_lib.a (for linux)\n
///
/// Redistribution and use in source and binary forms, with or without
/// modification, of the specified files is permitted provided the following 
/// conditions are met: \n
///
/// - Redistributions of source code must retain the above copyright
///   notice, this list of conditions and the following disclaimer. \n
/// - Redistributions in binary form must reproduce the above copyright
///   notice, this list of conditions and the following disclaimer in the
///   documentation and/or other materials provided with the distribution. \n
/// - The name(s) of the contributor(s) may not be used to endorse or promote 
///   products derived from this software without specific prior written 
///   permission. \n
///
/// THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
/// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
/// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
/// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
/// SUCH DAMAGE.
///
/// \b NOTES: \n
/// This code was developed using rigourous unit testing for every function 
/// and operation. Despite any rigorous development process, bugs are
/// inevitable. Please report bugs and suggested fixes to glenn_at_zenautics.com.\n
//============================================================================

#ifndef _ZENAUTICS_MATRIX_H_
#define _ZENAUTICS_MATRIX_H_

#include <stdio.h>
#include <complex> // This is needed for the Standard Template Library complex<double> type.
#include <string>
#include "cmatrix.h" // The core matrix engine is written in 'c'.

namespace Zenautics
{

#define MATRIX_USE_EXCEPTION_HANDLING
#ifdef MATRIX_USE_EXCEPTION_HANDLING

  /**
  * \class   MatrixException
  * \brief   A class for exceptions thrown by the Matrix class.
  * 
  * The MATRIX_USE_EXCEPTION_HANDLING define enables the use of exception 
  * handling using try{} catch{}. A MatrixException is thrown. The use of 
  * exception handling is very highly recommended. When it is turned off, 
  * the matrix will try to output a message and then call 'exit(1)'.
  *
  * \code
  * int main()
  * { 
  *   try
  *   {
  *     Matrix A(2,2);
  *     double d = A(3,1).real(); // causes an out of bounds exception
  *   }
  *   catch( MatrixException& matrix_exception )
  *   {
  *     cout << matrix_exception << endl;
  *   }
  *   catch ( ... )
  *   {
  *     cout << "Caught unknown exception" << endl;
  *   }
  *   return 0;
  * }
  * \endcode
  */
  class MatrixException
  {  
  public: 
    /// \brief  The constructor.
    MatrixException( const char* msg );

    /// \brief  The copy constructor.
    MatrixException(const MatrixException& matrix_exception);

    /// \brief  The destuctor.
    virtual ~MatrixException() { /* nothing needed yet */ };

    /// \brief  Return a copy of the exception message.
    std::string GetExceptionMessage();

    /// \brief  Overload the casting operator to a string.
    operator const char*();

  public:
    /// The matrix exception string.  
    std::string m_ExceptionString;

  private:
    /// The matrix exception character string.  
    char m_msg[256];    
  };

#endif


  /**
  * \class   Matrix
  * \brief   The two dimensional matrix class. 
  *          Both real and complex data are inherently supported.
  * 
  * The matrix class supports advanced real and complex functionality. It is
  * optimized for columnwise operations.
  *
  * Refer to example_main.cpp for a complete example program using the Matrix.
  */
  class Matrix
  {  
  public: // Constructors / Destructor

    /// \brief  This function enables or disables a global flag
    ///         that forces single element matrices to be treated
    ///         as scalars. This is enabled by default. 
    static void Treat1x1MatricesAsScalar( bool enable = true );

    /// \brief  The default constructor (no data allocated yet).
    Matrix();                                             

    /// \brief  A vector style constructor.
    ///
    /// Matrix A(nrows);  creates an nrowsx1 real 'vector'.
    /// A complex vector must be created using Matrix A(nrows,ncols,false);
    Matrix(const unsigned nrows);

    /// \brief  A matrix style constructor.
    ///
    /// Matrix A(nrows,ncols); creates an nrowsxncols real 'matrix'. A real matrix is assumed.
    /// Matrix A(nrows,ncols,false); creates an nrowsxncols complex 'matrix'. A real matrix is assumed.
    Matrix(const unsigned nrows, const unsigned ncols, const bool isReal=true);

    /// \brief  The copy constructor.
    Matrix(const Matrix& mat);                          

    /// \brief  A constructor reading data from a file.
    Matrix(const char* path, bool& itWorked);

    /// \brief  The constructor as a copy from a static matrix.
    Matrix(const double mat[], const unsigned nrows, const unsigned ncols=1 ); 

    /// \brief  The destructor.
    virtual ~Matrix();

    /// \brief  The assignment operator from another matrix.
    ///
    /// e.g. Matrix B; Matrix A; B = "[1 2 3; 4 5 6]"; A = B; // A == [1 2 3; 4 5 6], A is (2x3)
    Matrix& operator=(const Matrix& mat);

    /// \brief  The assignment operator from a scalar double value.
    ///
    /// e.g. Matrix A; A = 2.0; // A is (1x1).
    Matrix& operator=(const double value);

    
    /// \brief  The assignment operator from a std::complex<double> value.
    ///
    /// e.g. Matrix A; A = 2.0; // A is (1x1).
    Matrix& operator=(const std::complex<double> value);

    /// \brief  The assignement operator from a string matrix.
    ///
    /// There are two general possible interpretations of the string input.
    ///
    /// (1) Square bracket delimited matrix. \n
    /// Matrix A; A = "[1 2 3; 4 5 6]" or A = "[1, 2, 3; 4, 5, 6]" \n
    /// In this case '[' donates the start of a matrix and ']' denotes the end. \n
    /// Row vectors [1 2 3] and [4 5 6] are separated by ';'.  \n
    /// Commas can delimit row vector data but are not needed. \n
    /// Complex input: e.g. A = "[1+1i 2+3j 1-2i; 4 5 6]" or A = "[1+1i, 2+3j, 1-2i; 4, 5, 6]" \n
    ///
    /// (2) Free form delimited matrix. \n
    /// Matrix A; A = "1 2 3 \n 4 5 6 \n" \n
    /// In this case, the newline delimits different rows of the matrix. (\\r\\n also works). \n
    /// Row vectors can still be delimited by ';' as well. \n
    /// B = "1 2 3; 4 5 6;\n 7 8 9" will set a 3x3 matrix == [1 2 3; 4 5 6; 7 8 9]. \n
    /// Commas can delimit row vector data but are not needed. \n
    /// Complex input: e.g. A = "[1+1i 2+3j 1-2i\n 4 5 6]" or  \n
    ///                     A = "1+1i, 2+3j, 1-2i\n 4, 5, 6" or  \n
    ///                     A = "1+1i 2+3j 1-2i; 4, 5, 6". \n
    /// All result in A = [1+1i 2+3i 1-2i; 4 5 6]; \n
    Matrix& operator=(const char* strMatrix); //!< The assignment operator from a matrix string. 

  public:

    /// \brief  Clear the matrix memory. Set the matrix to size 0x0.
    bool Clear();

  public: // Matrix Qualifiers

    /// \brief  Is this matrix empty();
    bool isEmpty() const;

    /// \brief  Is the matrix mat conformal for multiplication (*this * mat)?
    bool isConformal(const Matrix& mat) const;

    /// \brief  Is this matrix the same size as mat?
    bool isSameSize(const Matrix& mat) const;  

    /// \brief  Is this a square matrix?
    bool isSquare() const;

    /// Check if this a real matrix.
    bool isReal() const;

    /// Check if this a complex matrix.
    bool isComplex() const; 

    unsigned GetNrCols() const;   //!< return no. of cols
    unsigned ncols() const;       //!< return no. of cols
    unsigned GetNrElems() const;  //!< return total no. of elements
    unsigned nelems() const;      //!< return total no. of elements
    unsigned GetNrRows() const;   //!< return no. of rows         
    unsigned nrows() const;       //!< return no. of rows         


    /// \brief  Return the real part of the matrix at this row and column.
    double real(const unsigned row, const unsigned col);

    /// \brief  Return the real part of the matrix at this vector index.
    double real(const unsigned index);

    /// \brief  Return the imaginary part of the matrix at this row and column.
    double imag(const unsigned row, const unsigned col);

    /// \brief  Return the imaginary part of the matrix at this vector index.
    double imag(const unsigned index);


  public: // Input Operations

    /// \brief  Read the marix from an ASCII file with the path given by the 'c' style string
    /// (with support for many delimiters, whitespace, or ',', or ';', or many others) 
    /// or a compressed BINARY matrix file used in the Save function.
    /// Complex and real data input are supported.
    ///
    /// \return true if successful, false otherwise
    bool ReadFromFile( const char *path );
    
    /// \brief  Read the marix from an ASCII file with the path given by the std::string
    /// (with support for many delimiters, whitespace, or ',', or ';', or many others) 
    /// or a compressed BINARY matrix file used in the Save function.
    /// Complex and real data input are supported.
    ///
    /// \return true if successful, false otherwise    
    bool ReadFromFile( std::string path )
    {
      return ReadFromFile( path.c_str() );
    }


    /**  
    * \brief  A safe function for performing a copy of another matrix.
    *
    * \code
    * Matrix A(2,2);
    * A[0][0] = 1.0;
    * A[0][1] = 2.0;
    * A[1][0] = 3.0;
    * A[1][1] = 4.0;
    * Matrix B;
    * if( !B.Copy(A) )
    *   return false;
    * \endcode
    *
    * \return true if successful, false otherwise
    */
    bool Copy( Matrix& src );


    /**      
    * \brief  A safe function for setting the matrix from a double.
    *
    * \code
    * double d = 10.0;
    * Matrix A;
    * if( !A.Copy(d) )
    *   return false;
    * \endcode
    *
    * \return true if successful, false otherwise
    */
    bool Copy( const double& value );


    /**      
    * \brief  A safe function for setting the matrix from a std::complex<double>.
    *
    * \code
    * std::complex<double> cplx(1.0,2.0);
    * Matrix A;
    * if( !A.Copy(cplx) )
    *   return false;
    * \endcode
    *
    * \return true if successful, false otherwise
    */
    bool Copy( const std::complex<double>& cplx );


  public: // Output Operations

    /// \brief  Saves a matrix to the specified file path (a 'c' style string)
    /// using a proprietary compressed format.
    /// ADVANCED EDITION ONLY. BASIC EDITION will return false.
    ///
    /// \return true if successful, false otherwise
    bool Save( const char* path );

    /// \brief  Saves a matrix to the specified file path (a std::string)
    /// using a proprietary compressed format.
    /// ADVANCED EDITION ONLY. BASIC EDITION will return false.
    ///
    /// \return true if successful, false otherwise
    bool Save( std::string path )
    {
      return Save( path.c_str() );
    }

    /// \brief  Print the matrix to a file with automatically determined column width 
    /// and the specified precision, uses "%'blank''-'autowidth.precision'g'", to 
    /// the 'c' style path string provided.
    ///
    /// \return true if successful, false otherwise
    bool Print( const char *path, const unsigned precision, bool append = false );

    /// \brief  Print the matrix to a file with automatically determined column width 
    /// and the specified precision, uses "%'blank''-'autowidth.precision'g'", to 
    /// the std:string path provided.
    ///
    /// \return true if successful, false otherwise
    bool Print( std::string path, const unsigned precision, bool append = false ) 
    {
      return Print( path.c_str(), precision );
    }

    /**
    * \brief  Print the matrix to the standard output (stdout) with automatically 
    * determined column width and the specified precision, 
    * uses "%'blank''-'autowidth.precision'g'".
    *
    *  e.g. \n
    *  \code
    *  Matrix A;
    *  A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    *  bool result = A.PrintStdout(6); // Print to stdout with automatic width determination.
    *  // 0123456789012345678901234567890
    *  // results in:
    *  // 0123456789012345678901234567890
    *  //  1.123  0  2.123 -1
    *  //  3.123  0  4.123 -1
    *  \endcode
    *    
    * \return true if successful, false otherwise
    */
    bool PrintStdout( const unsigned precision );

    /**
    *  \brief  Print the matrix to a buffer of maxlength with automatically determined column width 
    *  and the specified precision, uses "%'blank''-'autowidth.precision'g'"
    *
    *  e.g. \n
    *  \code
    *  Matrix A;
    *  A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    *  char buffer[256]; 
    *  bool result = A.PrintToBuffer( buffer, 256, 6); // Print to a buffer with automatic width determination.
    *  cout << buffer << endl;
    *  // 0123456789012345678901234567890
    *  // results in:
    *  // 0123456789012345678901234567890
    *  //  1.123  0  2.123 -1
    *  //  3.123  0  4.123 -1
    *  \endcode
    *    
    *  \return true if successful, false otherwise
    */
    bool PrintToBuffer( char* buffer, const unsigned maxlength, const unsigned precision );


    /// \brief  Print the matrix to a file with specifed width and precision
    /// PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
    /// to file specified with the 'c' style path string provided.
    ///
    /// \return true if successful, false otherwise
    bool PrintFixedWidth( const char* path, const unsigned width, const unsigned precision, bool append = false );


    /// \brief  Print the matrix to a file with specifed width and precision
    /// PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
    /// to file specified with the std::string path string provided.
    ///
    /// \return true if successful, false otherwise
    bool PrintFixedWidth( std::string path, const unsigned width, const unsigned precision, bool append = false )
    {
      return PrintFixedWidth( path.c_str(), width, precision, append );
    }

    /**
    *  \brief  Print the matrix to a buffer of maxlength with specifed width and precision
    *  PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
    *
    *  e.g. \n
    *  \code
    *  Matrix A;
    *  A = "[1.123 2.123 -1; 3.123 4.123 -1]";  // Set A using string notation.
    *  char buffer[256]; 
    *  bool result = A.PrintFixedWidthToBuffer( buffer, 256, 10, 6 ); // Print to a buffer with fixed width.
    *  cout << buffer << endl;
    *  // 0123456789012345678901234567890
    *  // results in:
    *  //  1.123     2.123    -1
    *  //  3.123     4.123    -1
    *  \endcode
    *    
    *  \return true if successful, false otherwise
    */
    bool PrintFixedWidthToBuffer( char* buffer, const unsigned maxlength, const unsigned width, const unsigned precision );

    /// \brief  Print the matrix to a file path specified by the 'c' style string 
    /// with specifed precision and delimiter.
    ///
    /// \return true if successful, false otherwise
    bool PrintDelimited( const char *path, const unsigned precision, const char delimiter, bool append = false );

    /// \brief  Print the matrix to a file path specified by the std::string 
    /// with specifed precision and delimiter.
    ///
    /// \return true if successful, false otherwise
    bool PrintDelimited( std::string path, const unsigned precision, const char delimiter, bool append = false )
    {
      return PrintDelimited( path.c_str(), precision, delimiter, append );
    }
    
    /**
    *  \brief  Print the matrix to a 'c' style string buffer of maxlength with specifed precision and delimiter.
    *
    *  e.g. \n
    *  \code
    *  Matrix A;
    *  A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    *  char buffer[256]; 
    *  bool result = A.PrintDelimitedToBuffer( buffer, 256, 6, ',' ); // Print to a buffer using comma delimiters.
    *  cout << buffer << endl;
    *  // results in:
    *  // 1.123,2.123
    *  // 3.123,4.123
    *  \endcode
    *    
    *  \return true if successful, false otherwise
    */
    bool PrintDelimitedToBuffer( char *buffer, const unsigned maxlength, const unsigned precision, const char delimiter );

    /**
    *  \brief  Print a row to a 'c' style string buffer.
    *  
    *  e.g. \n
    *  \code
    *  Matrix A;
    *  A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    *  char buffer[256]; 
    *  bool result = A.PrintRowToString( 1, buffer, 256, 4, 6 ); // Print the second row to the char buffer.
    *  cout << buffer << endl;
    *  // results in:
    *  // 3.123   4.123
    *  \endcode
    * 
    *  \return true if successful, false otherwise
    */
    bool PrintRowToString( const unsigned row, char *buffer, const unsigned maxlength, const int width, const int precision );


  public: // Change the dimensions of the matrix

    /// \brief  Remove a single column from the matrix.
    ///
    /// \return true if successful, false otherwise.  
    bool RemoveColumn( const unsigned col );

    /// \brief  Remove all the columns 'after' the column index given.
    ///
    /// \return true if successful, false otherwise.  
    bool RemoveColumnsAfterIndex( const unsigned col );

    /** \brief  Remove the rows and columns specified by the indices in the rows[] and cols[] arrays.
    *
    * \code
    * Matrix A(4,4);
    * unsigned rows[2];
    * unsigned cols[2];
    * rows[0] = 0; // remove row 0
    * rows[1] = 2; // remove row 2
    * cols[0] = 0; // remove column 0
    * cols[1] = 2; // romve column 2
    * A.RemoveRowsAndColumns( 2, (unsigned int *)rows, 2, (unsigned int *)cols );
    * // A is now a 2x2 matrix
    * \endcode
    *
    * \return true if successful, false otherwise.  
    */
    bool RemoveRowsAndColumns( const unsigned nrows, const unsigned rows[], const unsigned ncols, const unsigned cols[] );

    /// \brief  Insert a column matrix into the matrix.
    ///
    /// \return true if successful, false otherwise.  
    bool InsertColumn( const Matrix &src, const unsigned dst_col, const unsigned src_col );

    /// \brief  Add a column to the end of the matrix.
    ///
    /// \return true if successful, false otherwise.  
    bool AddColumn( const Matrix &src, const unsigned src_col );

    /// \brief  Combine two matrices with the same nrows, A becomes A|B.
    ///
    /// \return true if successful, false otherwise.  
    bool Concatonate( const Matrix &src );

    /// \brief  Redimension the matrix, original data is saved in place, new 
    /// data is set to zero.
    /// The default value for ncols allows redimensioning as a vector.
    ///
    /// \return true if successful, false otherwise.  
    bool Redim( const unsigned nrows, const unsigned ncols=1 );

    /// \brief  Resize the matrix, original data is lost, new data is set to zero.
    /// The default value for ncols allows resizing as a vector.
    ///
    /// \return true if successful, false otherwise.  
    bool Resize( const unsigned nrows, const unsigned ncols=1 );


  public: // Setting matrix values

    /// \brief  Set the matrix from the static 'c' style matrix indexed by mat[i*ncols + j].
    ///
    /// \return true if successful, false otherwise.    
    bool SetFromStaticMatrix( const double mat[], const unsigned nrows, const unsigned ncols );

    /**
    * \brief  Setting the matrix values from a string matrix.
    *
    * There are two general possible interpretations of the string input.
    *
    * (1) Square bracket delimited matrix. \n
    * \code
    * bool result;
    * Matrix A;
    * result = A.SetFromMatrixString("[1 2 3; 4 5 6]");
    * // equivalently 
    * result = A.SetFromMatrixString("[1, 2, 3; 4, 5, 6]");
    * \endcode
    * In this case '[' donates the start of a matrix and ']' denotes the end. \n
    * Row vectors [1 2 3] and [4 5 6] are separated by ';'.  \n
    * Commas can delimit row vector data but are not needed. \n
    * Complex input: \n
    * \code 
    * bool result;
    * Matrix A;
    * result = A.SetFromMatrixString("[1+1i 2+3j 1-2i; 4 5 6]");
    * \endcode
    *
    * (2) Free form delimited matrix. \n
    * \code
    * bool result;
    * Matrix A; 
    * result = A.SetFromMatrixString("1 2 3 \n 4 5 6 \n");
    * \endcode
    * In this case, the newline delimits different rows of the matrix. (\\r\\n also works). \n
    * Row vectors can still be delimited by ';' as well. \n
    * \code
    * bool result;
    * Matrix B;
    * B.SetFromMatrixString("1 2 3; 4 5 6;\n 7 8 9"); // will set a 3x3 matrix == [1 2 3; 4 5 6; 7 8 9].
    * \endcode
    * Commas can delimit row vector data but are not needed. \n
    * Complex input: \n
    * \code
    * A.SetFromMatrixString("[1+1i 2+3j 1-2i\n 4 5 6]"); // or
    * A.SetFromMatrixString("1+1i, 2+3j, 1-2i\n 4, 5, 6"); // or
    * A.SetFromMatrixString("1+1i 2+3j 1-2i; 4, 5, 6"); 
    * \endcode
    * All result in \n
    * A = [1+1i 2+3i 1-2i; 4 5 6]; \n
    *
    * \return true if successful, false otherwise.
    */
    bool SetFromMatrixString(const char* strMatrix);


    /// \breif  Copy the src data in column col to dst matrix, resize dst if possible & necessary.
    ///
    /// \return true if successful, false otherwise.    
    bool CopyColumn( const unsigned src_col, Matrix &dst );

    /// \brief  Insert a submatrix (src) into dst, starting at indices dst(row,col).
    ///
    /// \return true if successful, false otherwise.    
    bool InsertSubMatrix( const Matrix &src, const unsigned dst_row, const unsigned dst_col );

    /// \brief  Zero the entire matrix.
    ///
    /// \return true if successful, false otherwise.  
    bool Zero();

    /// \brief  Zero all elements in a specified column.
    ///
    /// \return true if successful, false otherwise.    
    bool ZeroColumn( const unsigned col );

    /// \brief  Zero all elements in a specified row.
    ///
    /// \return true if successful, false otherwise.    
    bool ZeroRow( const unsigned row );

    /// \brief  Fill the matrix with the given value.
    ///
    /// \return true if successful, false otherwise.    
    bool Fill( const double value );

    /// \brief  Fill the matrix column with the given value.
    ///
    /// \return true if successful, false otherwise.    
    bool FillColumn( const unsigned col, const double value );

    /// \brief  Fills the matrix row with the given value.
    ///
    /// \return true if successful, false otherwise.    
    bool FillRow( const unsigned row, const double value );

    /// \brief  Reverse the order of elements of a column.
    ///
    /// \return true if successful, false otherwise.    
    bool FlipColumn( const unsigned col );

    /// \brief  Reverse the order of elements of a row.
    ///
    /// \return true if successful, false otherwise.    
    bool FlipRow( const unsigned row );

    /// \brief  Set the matrix to identity using the current dimensions.
    ///
    /// \return true if successful, false otherwise.    
    bool Identity();

    /// \brief  Set the matrix to identity using the specified dimension (nxn).
    ///
    /// \return true if successful, false otherwise.        
    bool Identity(const unsigned dimension);



  public: // Inplace Operations

    /// \brief  Transpose the matrix as an inplace operation.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Transpose();

    /// \brief  Round the matrix elements to the specified presision. \n
    /// e.g. precision = 0    1.8    -> 2     \n
    /// e.g. precision = 1,   1.45   -> 1.5   \n
    /// e.g. precision = 2    1.456  -> 1.46  \n
    /// e.g. precision = 3,   1.4566 -> 1.457 \n
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Round( const unsigned precision );                             

    /// \brief  Round the matrix elements to the nearest integers towards minus infinity.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Floor();

    /// \brief  Round the matrix elements to the nearest integers towards infinity.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Ceil();

    /// \brief  Rounds the matrix elements of X to the nearest integers towards zero.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Fix();

    /// \brief  Add a scaler double (ie: M += 5).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_AddScalar( const double scalar );

    /// \brief  Subtract a scaler double (ie: M += 5).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_SubtractScalar( const double scalar );

    /// \brief  Multiply by scaler double (ie: M *= 5).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_MultiplyScalar( const double scalar );

    /// \brief  Divide by scaler double (ie: M /= 5).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_DivideScalar( const double scalar );

    /// \brief  Raise the matrix to a power scaler double (ie: M ^= 5).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_PowerScalar( const double scalar );

    /// \brief  Add a scaler double (ie: M += (5+2i)).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_AddScalarComplex( const std::complex<double> cplx );

    /// \brief  Subtract a scaler double (ie: M -= (5+2i)).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_SubtractScalarComplex( const std::complex<double> cplx );

    /// \brief  Multiply by scaler double (ie: M *= (5+2i)).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_MultiplyScalarComplex( const std::complex<double> cplx );

    /// \brief  Divide by scaler double (ie: M /= (5+2i)).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_DivideScalarComplex( const std::complex<double> cplx );

    /// \brief  Raise the matrix to a power scaler double (ie: M ^= (5+2i)).
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_PowerScalarComplex( const std::complex<double> cplx );

    /// \brief  Compute the absolute value of each element in the matrix.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Abs();

    /// \brief  Compute the value^2 of each element in the matrix.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Sqr();

    /// \brief  Computes the sqrt(value) of each element in the matrix.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Sqrt();

    /// \brief  Computes the exp(value) of each element in the matrix.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Exp();

    /// \brief  Computes the natural logarithm, ln(value) of each element in the matrix.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Ln();

    /// \brief  Add +1.0 to all elements, e.g. M++.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Increment();

    /// \brief  Add +1.0 to all elements, e.g. M--.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_Decrement();

    /// \brief  Add matrix B to this matrix inplace. A += B, inplace.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_Add( const Matrix &B );

    /// \brief  Subtract matrix B from this matrix inplace. A -= B, inplace.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_Subtract( const Matrix &B );

    /// \brief  Pre-Multiply this matrix by B. A = B*A, inplace.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_PreMultiply( const Matrix &B );

    /// \brief  Post-Multiply this matrix by B. A = A*B, inplace.
    ///
    /// \return true if successful, false otherwise.  
    bool Inplace_PostMultiply( const Matrix &B );

    /// \brief  Dot multiply A .*= B, inplace. A and B must have the same dimensions.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_DotMultiply( const Matrix &B );

    /// \brief  Dot divide A ./= B, inplace. A and B must have the same dimensions.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_DotDivide( const Matrix &B );

    /// \brief  Sorts each column of the matrix in ascending order.
    /// If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortAscending();

    /// \brief  Sorts each column of M in descending order.
    /// If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortDescending();

    /// \brief  Sorts a specific column in ascending order.
    /// If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortColumnAscending( const unsigned col );

    /// \brief  Sorts a specific column in descending order.
    /// If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortColumnDescending( const unsigned col );

    /// \brief  Sorts a specific column in ascending order and fills a MTX 
    /// column vector with the sorted index. The index vector will be resized 
    /// if index->nrows != M->nrows. If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortColumnIndexed( const unsigned col, Matrix &Index );

    /// \brief  Sorts the entire matrix by a specific column.
    /// If complex, sorts based on magnitude.
    ///
    /// \return true if successful, false otherwise.
    bool Inplace_SortByColumn( const unsigned col );

    /// \brief  Computes the inplace inverse of the matrix.
    ///
    /// Uses fast closed form solutions for:
    /// 1x1, 2x2, 3x3
    ///
    /// Otherwise, the matrix is first tested to determine if it is a symmetric 
    /// positive-definite matrix. If so, Cholesky decomposition is used
    /// to facilitate the inversion of a lower triangular matrix. If the
    /// matrix is not symmetric and positive-definite robust inversion
    /// using gaussing elimination is attempted.
    /// 
    /// If the matrix is singular, the original matrix is unchanged.
    ///
    /// \return true if successful, false if empty, singular or not square.
    bool Inplace_Invert();

    /// \brief  Perfroms an inplace inverse using Gaussian Elimination methods.
    ///
    /// \return true if successful, false if empty, singular or not square.
    bool Inplace_InvertRobust();

    /// \brief  Compute the inplace inverse of a unit lower triangular matrix. 
    /// An example unit lower triangular matrix is: \n
    ///      A = [     1    0    0;          \n
    ///               -2    2    0;          \n
    ///                4   -3    3 ]; with   \n
    /// inv(A) = [     1    0    0;          \n
    ///                1  1/2    0;          \n
    ///             -1/3  1/2  1/3 ];        \n
    ///
    /// \return true if successful, false if empty, singular or not square.
    bool Inplace_LowerTriangularInverse();

    /// \brief  Compute the inplace Fourier Transform of each column of the matrix.
    /// \return   true if successful, false if unable to perform the FFT.
    bool Inplace_FFT();

    /// \brief  Compute the inplace inverse Fourier Transform of each column of the matrix.
    /// \return   true if successful, false if unable to perform the FFT.
    bool Inplace_IFFT();


  public: // Safe operations that set the matrix. Safe in that they return a boolean.

    /// \brief  Add A = B+C. The result, A, is stored in this matrix. 
    /// e.g. Matrix A; A.Add(B,C);
    ///
    /// \return true if successful, false otherwise.
    bool Add( const Matrix &B, const Matrix &C );

    /// \brief  Subtract A = B-C. The result, A, is stored in this matrix. 
    /// e.g. Matrix A; A.Subtract(B,C);
    ///
    /// \return true if successful, false otherwise.
    bool Subtract( const Matrix &B, const Matrix &C );

    /// \brief  Multiply A = B*C. The result, A, is stored in this matrix. 
    /// e.g. Matrix A; A.Multiply(B,C);
    ///
    /// \return true if successful, false otherwise.
    bool Multiply( const Matrix &B, const Matrix &C );


  public: // Matlab functions

    /// \brief  Compute the absolute value of each element of the matrix inplace.
    ///
    /// \return true if successful, false otherwise.    
    bool Inplace_abs();

    /// \brief  Create a column vector [start:increment:end) beginning at start
    /// with step size of increment until less than or equal to end. 
    /// Note that arguments must be real scalars. \n
    /// e.g. a = 2:2:9   = [2; 4; 6; 8;] \n
    /// e.g. b = 2:-2:-9 = [2; 0; -2; -4; -6; -9;] \n
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_colon( double start, double increment, double end );

    /// \brief  Compute the cosine of each element of the matrix inplace. This 
    /// function assumes radian values in the matrix.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_cos();

    /// \brief  Complex conjugate. z = x+yi. conj(z) = x-yi.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_conj();

    /// \brief  Imaginary part of the complex matrix. z = x+yi. real(z) = y.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_imag();

    /// \brief  Create a matrix of nrows by ncols filled with 1.0.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_ones( const unsigned nrows, const unsigned ncols );

    /// \brief  Real part of the complex matrix. z = x+yi. real(z) = x.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_real();

    /// \brief  Compute the sine of each element of the matrix inplace. This 
    /// function assumes radian values in the matrix.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_sin();

    /// \brief  Compute the sqrt of each element of the matrix inplace. This 
    /// function assumes radian values in the matrix.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_sqrt();

    /// \brief  Create a matrix of nrows by ncols filled with 0.0.
    ///
    /// \return true if successful, false otherwise.        
    bool Inplace_zeros( const unsigned nrows, const unsigned ncols );






  public: // Statistics

    /// \brief  Computes the value of the largest absolute element and its index.
    ///
    /// \return true if successful, false otherwise.
    bool GetStats_MaxAbs(unsigned &row, unsigned &col, double &value );

    /// \brief  Computes the value (re+im*j) of the maximum element and its index.  
    /// \return true if successful, false otherwise.
    bool GetStats_Max(unsigned &row, unsigned &col, double &re, double &im );

    /// \brief  Computes the value (re+im*j) of the maximum element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MaxVal(double &re, double &im );

    /// \brief  Computes the value of the largest absolute column element and its row index.
    /// \return true if successful, false otherwise.
    bool GetStats_MaxAbsCol(const unsigned col, double &value, unsigned &row );

    /// \brief  Computes the value (re+im*j) of the maximum column element and its row index.  
    /// \return true if successful, false otherwise.
    bool GetStats_MaxCol(const unsigned col, double &re, double &im, unsigned &row );

    /// \brief  Computes the value (re+im*j) of the maximum column element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MaxColVal(const unsigned col, double &re, double &im );

    /// \brief  Computes the value of the largest absolute row element and its column index.
    /// \return true if successful, false otherwise.
    bool GetStats_MaxAbsRow(const unsigned row, double &value, unsigned &col );

    /// \brief  Computes the value (re+im*j) of the maximum row element and its column index.  
    /// \return true if successful, false otherwise.
    bool GetStats_MaxRow(const unsigned row, double &re, double &im, unsigned &col );

    /// \brief  Computes the value (re+im*j) of the maximum row element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MaxRowVal(const unsigned row, double &re, double &im );

    /// \brief  Computes the value of the smallest absolute element and its index.
    /// \return true if successful, false otherwise.
    bool GetStats_MinAbs(unsigned &row, unsigned &col, double &value );

    /// \brief  Computes the value (re+im*j) of the minimum element and its index.  
    /// \return true if successful, false otherwise.
    bool GetStats_Min(unsigned &row, unsigned &col, double &re, double &im );

    /// \brief  Computes the value (re+im*j) of the minimum element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MinVal(double &re, double &im );

    /// \brief  Computes the value of the smallest absolute column element and its row index.
    /// \return true if successful, false otherwise.
    bool GetStats_MinAbsCol(const unsigned col, double &value, unsigned &row );

    /// \brief  Computes the value (re+im*j) of the minimum column element and its row index.  
    /// \return true if successful, false otherwise.
    bool GetStats_MinCol(const unsigned col, double &re, double &im, unsigned &row );

    /// \brief  Computes the value (re+im*j) of the minimum column element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MinColVal(const unsigned col, double &re, double &im );

    /// \brief  Computes the value of the smallest absolute row element and its column index.
    /// \return true if successful, false otherwise.
    bool GetStats_MinAbsRow(const unsigned row, double &value, unsigned &col );

    /// \brief  Computes the value (re+im*j) of the minimum row element and its column index.  
    /// \return true if successful, false otherwise.
    bool GetStats_MinRow(const unsigned row, double &re, double &im, unsigned &col );

    /// \brief  Computes the value (re+im*j) of the minimum row element.  
    /// \return true if successful, false otherwise.
    bool GetStats_MinRowVal(const unsigned row, double &re, double &im );

    /// \brief  Computes the range of the data in the specified column. 
    /// Range = MaxVal - MinVal.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_ColRange( const unsigned col, double &re, double &im );

    /// \brief  Computes the range of the data in the specified row. 
    /// Range = MaxVal - MinVal.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_RowRange( const unsigned row, double &re, double &im );

    /// \brief  Computes the range of the data in the matrix. 
    /// Range = MaxVal - MinVal.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_Range( double &re, double &im );

    /// \brief  Computes the sum for the specified column.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnSum( const unsigned col,  double &re, double &im );

    /// \brief  Computes the sum for the specified row.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_RowSum( const unsigned row, double &re, double &im );

    /// \brief  Computes the sum for the matrix.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_Sum( double &re, double &im );

    /// \brief  Computes the sample mean for the specified column.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnMean( const unsigned col, double &re, double &im );

    /// \brief  Computes the sample mean for the specified row.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_RowMean( const unsigned row, double &re, double &im );

    /// \brief  Computes the sample mean for the matrix.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_Mean( double &re, double &im );

    /// \brief  Computes the sample standard deviation for the specified column.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnStdev( const unsigned col, double &value );

    /// \brief  Computes the sample standard deviation for the specified row.
    /// \return true if successful, false otherwise.
    bool GetStats_RowStdev( const unsigned row, double &value );

    /// \brief  Computes the sample standard deviation for the matrix.
    /// \return true if successful, false otherwise.
    bool GetStats_Stdev( double &value );

    /// \brief  Computes the sample variance for the specified column.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnVar( const unsigned col, double &value );

    /// \brief  Computes the sample variance for the specified row.
    /// \return true if successful, false otherwise.
    bool GetStats_RowVar( const unsigned row, double &value );

    /// \brief  Computes the sample variance for the matrix.
    /// \return true if successful, false otherwise.
    bool GetStats_Var( double &value );

    /// \brief  Computes the norm of the specified column.
    /// If real, norm = sqrt( sum( val*val ) ).
    /// If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnNorm( const unsigned col, double &value );

    /// \brief  Computes the norm of the specified row.
    /// If real, norm = sqrt( sum( val*val ) ).
    /// If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    /// \return true if successful, false otherwise.
    bool GetStats_RowNorm( const unsigned row, double &value );

    /// \brief  Computes the norm of the matrix.
    /// If real, norm = sqrt( sum( val*val ) ).
    /// If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    /// \return true if successful, false otherwise.
    bool GetStats_Norm( double &value );

    /// \brief  Computes the sample RMS value for the specified column.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnRMS( const unsigned col, double &value );

    /// \brief  Computes the sample RMS value for the specified row.
    /// \return true if successful, false otherwise.
    bool GetStats_RowRMS( const unsigned row, double &value );

    /// \brief  Computes the sample RMS value for the matrix.
    /// \return true if successful, false otherwise.
    bool GetStats_RMS( double &value );


    /// \brief  Computes the sample skewness value for the specified column.
    /// The skewness is the third central moment divided by the cube of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnSkewness( const unsigned col, double &re, double &im );

    /// \brief  Computes the sample skewness value for the specified row.
    /// The skewness is the third central moment divided by the cube of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_RowSkewness( const unsigned row, double &re, double &im );

    /// \brief  Computes the sample skewness value for the matrix.
    /// The skewness is the third central moment divided by the cube of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetStats_Skewness( double &re, double &im );



    /// \brief  Computes the sample kurtosis value for the specified column.
    /// The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    /// Reference: http://en.wikipedia.org/wiki/Kurtosis.
    /// Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    /// g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    /// \return true if successful, false otherwise.
    bool GetStats_ColumnKurtosis( const unsigned col, double &re, double &im );

    /// \brief  Computes the sample kurtosis value for the specified row.
    /// The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    /// Reference: http://en.wikipedia.org/wiki/Kurtosis.
    /// Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    /// g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    /// \return true if successful, false otherwise.
    bool GetStats_RowKurtosis( const unsigned row, double &re, double &im );

    /// \brief  Computes the sample kurtosis value for the matrix.
    /// The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    /// Reference: http://en.wikipedia.org/wiki/Kurtosis.
    /// Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    /// g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    /// \return true if successful, false otherwise.
    bool GetStats_Kurtosis( double &re, double &im );


  public: // Matrix Specific operations

    /// \brief  Computes the trace of M where M is a square matrix.
    /// Trace = Sum of diagonal elements.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    bool GetTrace( double &re, double &im );

    /// \brief  Computes the determinatnt of the square matrix M.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    bool GetDeterminant( double &re, double &im );



  public: // Safe operations that set a Matrix argument. Rather than Matrix as a return type.

    /// \brief  Sets the diagonal elements of the matrix into DiagonalVector as a column vector.
    /// \return true if successful, false otherwise.
    bool GetDiagonal( Matrix& DiagonalVector );

    /// \brief  Computes a moving average using N lead samples and M lagging samples
    /// for the specified column and stores it in MovAvg.
    /// \return true if successful, false otherwise.  
    bool GetColumnMovAvg( const unsigned col, const unsigned lead, const unsigned lag, Matrix &MovAvg );

    /// \brief  Computes a moving average using N lead samples and M lagging samples
    /// for the matrix and stores it in MovAvg.
    /// \return true if successful, false otherwise.  
    bool GetMovAvg( const unsigned lead, const unsigned lag, Matrix &MovAvg );

    /// \brief  Computes: InvATA = inverse( transpose(A) * A ). Assumes this matrix is A.
    /// e.g. Matrix A; Matrix InvATA; A = ...; bool result = A.GetATAInverse( InvATA );
    /// \return true if successful, false otherwise.  
    bool GetATAInverse( Matrix &InvATA );

    /// \brief  LU factorization.
    /// Performs a factorization to produce a unit lower triangular matrix, L, 
    /// an upper triangular matrix, U, and permutation matrix P so that
    /// P*X = L*U.
    /// P, L and U are copmuted correctly if IsFullRank is set to true.
    /// e.g. Matrix A; A = ...; bool isFullRank, Matrix L,U,P; bool result = A.GetLUFactorization( isFullRank, P, L, U );
    /// \return true if successful, false otherwise.  
    bool GetLUFactorization( bool &isFullRank, Matrix &P, Matrix &L, Matrix &U );



    /// \brief  Retrieve the elements of the matrix specified by the index vectors. 
    /// The index vectors must be nx1 and preferably not complex.
    ///
    /// \return true if successful, false otherwise.
    bool GetIndexedValues( Matrix& RowIndex, Matrix& ColIndex, Matrix& Result );


    /// \brief  
    //bool Find_EqualTo( const double value, const double tolerance = 1e-12, Matrix &IndexMatrix );     

    /// get the indices that are not this value
    //bool Find_NotEqualTo( const double value, const double tolerance = 1e-12 );     
   
    /// get the indices less than this value
    //bool Find_LessThan( const double value );     
   
    /// get the indices less than this value
    //bool Find_MoreThan( const double value );     
    

    

  public: // Advanced Functionality

    /// \brief  Retrieve the matrix comment string. The string
    /// will be empty if none is available. The matrix comment string
    /// is often the header line read when using ReadFromFile(). \n
    /// e.g. file.txt has:
    /// time(s)   x(m)   y(m)
    /// 1.0       20.0   30.0
    ///
    /// \code
    /// bool result;
    /// Matrix A;
    /// result = A.ReadFromFile("file.txt");
    /// // A == [1.0 20.0 30.0]
    /// std::string comment = A.GetMatrixComment();
    /// // comment == "time(s)   x(m)   y(m)"
    /// \endcode
    ///
    /// \return The matrix comment string.
    std::string GetMatrixComment();


    /// \brief  Alter the matrix so that its data is within the startTime to the startTime+duration
    /// and compensate for any rollovers in the time system (e.g. GPS time in seconds rolls over
    /// at 604800.0 s). This function assumes that time is one of the matrix columns and requires
    /// this index, the timeColumn.
    /// \return true if successful, false otherwise.
    bool TimeWindow( 
      const unsigned timeColumn, //!< The column containing time.
      const double startTime,    //!< The specified start time (inclusive).
      const double duration,     //!< The duration to include.
      const double rolloverTime  //!< The potential time at which system time rolls over.
      );

    /// \brief  Alter the matrix so that its data is within [startTime endTime].
    /// This function assumes that time is one of the matrix columns and requires
    /// this index, the timeColumn.
    /// \return true if successful, false otherwise.
    bool TimeLimit( 
      const unsigned timeColumn, //!< The column containing time
      const double startTime,    //!< The specified start time (inclusive)
      const double endTime       //!< The duration to include
      );

    /// \brief  This static function matches matrices in time with specified precision
    /// where time is a column of each matrix. This function also
    /// allows time to rollover at a specified interval.
    ///
    ///    precision 0 = match to whole number \n
    ///    precision 1 = match to nearest 0.1 \n
    ///    precision 2 = match to nearest 0.01 \n
    ///    etc. \n
    /// rolloverTime examples \n
    ///     GPS time of week (s): rolloverTime= 604800.0 \n
    ///     hours               : rolloverTime = 24.0 \n
    ///     minutes             : rolloverTime = 60.0 \n
    ///
    /// The time data must be non-decreasing but the time may rollover
    /// by the specified amount. 
    /// e.g. rolloverTime = 60.0 \n
    ///      0,1,2,3,4,...59,60,1,2,5,10,60,1,2,3... \n
    ///
    /// This function may be called by: bool result = Matrix::TimeMatch( ... );
    /// \return true if successful, false otherwise.
    static bool TimeMatch( 
      Matrix &A,                   //!< The matrix with interpolation times
      const unsigned timeColumnA,  //!< The zero based column index for matrix A
      Matrix &B,                   //!< The matrix to be interpolated
      const unsigned timeColumnB,  //!< The zero based column index for matrix B
      const unsigned precision,    //!< The rounding precision used for time matching, 0 = whole, 1 = 0.1, 2 = 0.01, etc
      const double rolloverTime    //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
      );

    /// \brief  This static function interpolates Matrix B values by the times defined 
    /// in the column in Matrix A. Time must be increasing but times can 
    /// rollover with the specified rolloverTime.
    ///
    /// This function returns A and B with the same number of rows and 
    /// time aligned time columns.
    ///
    /// This function may be called by: bool result = Matrix::Interpolate( ... );
    /// \return true if successful, false otherwise.
    ///  
    static bool Interpolate( 
      Matrix &A,                    //!< The matrix with interpolation times
      const unsigned timeColumnA,   //!< The zero based column index for matrix A
      Matrix &B,                    //!< The matrix to be interpolated
      const unsigned timeColumnB,   //!< The zero based column index for matrix B
      const double maxInterpolationInterval, //!< The largest interpolation interval allowed
      const double rolloverTime     //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
      );


  public: // Functions that return a Matrix

    /// \brief  Return the column matrix specified by the column index. Returns (nrows x 1).
    Matrix  Column(const unsigned col);

    /// \brief  Return the row matrix specified by the column index. Returns (ncols x 1).
    Matrix  Row(const unsigned row);


    /// \brief  Return the tranpose of the matrix.
    Matrix  Transpose();
    /// \brief  Return the tranpose of the matrix.
    Matrix  T(); // short version

    /// \brief  Return the diagonal of the matrix as a vector.
    Matrix  Diagonal();

    /// \brief  Return the inverse of the matrix.
    Matrix  Inverse();
    /// \brief  Return the inverse of the matrix.
    Matrix  Inv(); // short version


    /// \brief  Return the Fourier Transform of each column of the matrix. 
    /// Power of two uses FFT, otherwise fast DFT.
    Matrix  FFT();

    /// \brief  Return the inverse Fourier Transform of each column of the matrix. 
    /// Power of two uses IFFT, otherwise fast IDFT.
    Matrix  IFFT();


  public:

    /// \brief  This is a nested class that is an element of the matrix. i.e. Matrix M; M(i,j) is the element. 
    /// It is used for operator(,) access by the Matrix.
    class Element
    {
    public:

      /// \brief  The only constructor binds the reference to the deep level matrix.
      /// The row and column members are set later.
      Element(MTX& mtx);

      /// \brief  The destructor.
      virtual ~Element();

    private:
      friend class Matrix; //!< The matrix class is a friend.

      MTX& m_mtx;     //!< The reference to the deep level matrix.
      unsigned m_row; //!< The corresponding row of this element in the matrix.
      unsigned m_col; //!< The corresponding column of this element in the matrix.

    public:

      const double real(); //!< Get the real part.
      const double imag(); //!< Get the imaginary part.
      
      /// \brief  The operator overload for setting the matrix element from a double.
      const Element& operator= (double v);

      /// \brief  The operator overload for setting the matrix element from a std::complex.
      const Element& operator= (std::complex<double> v);

      /// \brief  The operator overload for setting the matrix element from another the matrix element.
      const Element& operator= (Element v);

      /// \brief  This operator allows explict conversion to a std::complex.
      operator const std::complex<double>() const;

      void operator+= (const double scalar);            //!< \brief  Add a scalar inplace. 
      void operator+= (const std::complex<double>& v);  //!< \brief  Add a complex value inplace. 
      void operator+= (const Element& v);               //!< \brief  Add a Element inplace. 
      void operator-= (const double scalar);            //!< \brief  Subtract a scalar inplace. 
      void operator-= (const std::complex<double>& v);  //!< \brief  Subtract a complex value inplace. 
      void operator-= (const Element& v);               //!< \brief  Subtract a Element inplace. 
      void operator*= (const double scalar);            //!< \brief  Multiply a scalar inplace. 
      void operator*= (const std::complex<double>& v);  //!< \brief  Multiply a complex value inplace. 
      void operator*= (const Element& v);               //!< \brief  Multiply a Element inplace. 
      void operator/= (const double scalar);            //!< \brief  Divide a scalar inplace. 
      void operator/= (const std::complex<double>& v);  //!< \brief  Divide a complex value inplace. 
      void operator/= (const Element& v);               //!< \brief  Divide a Element inplace. 
      
      /// \brief  The operator overload for adding a double to an element.
      friend const std::complex<double> operator+ (const Element& m, double scalar);

      /// \brief  The operator overload for adding an element to an element.
      friend const std::complex<double> operator+ (const Element& a, const Element& b);

      /// \brief  The operator overload for adding an element to an complex.
      friend const std::complex<double> operator+ (const Element& a, const std::complex<double>& b);

      /// \brief  The operator overload for adding an element and a double.
      friend const std::complex<double> operator+ (double scalar, const Element& m);

      /// \brief  The operator overload for adding a complex and an element
      friend const std::complex<double> operator+ (const std::complex<double>& b, const Element& a);


      /// \brief  The operator overload for subtracting a double from an element.
      friend const std::complex<double> operator- (const Element& m, double scalar);
      
      /// \brief  The operator overload for subtracting an element from an element.
      friend const std::complex<double> operator- (const Element& a, const Element& b);

      /// \brief  The operator overload for subtracting an complex from an element.
      friend const std::complex<double> operator- (const Element& a, const std::complex<double>& b);

      /// \brief  The operator overload for subtracting an element from a double.
      friend const std::complex<double> operator- (double scalar, const Element& m);

      /// \brief  The operator overload for subtracting an element from a complex.
      friend const std::complex<double> operator- (const std::complex<double>& b, const Element& a);


      /// \brief  The operator overload for multiplying an element and a double.
      friend const std::complex<double> operator* (const Element& m, double scalar);
      
      /// \brief  The operator overload for multiplying an element and an element.
      friend const std::complex<double> operator* (const Element& a, const Element& b);

      /// \brief  The operator overload for multiplying an element and a complex.
      friend const std::complex<double> operator* (const Element& a, const std::complex<double>& b);

      /// \brief  The operator overload for multiplying a double and an element.
      friend const std::complex<double> operator* (double scalar, const Element& m);
      
      /// \brief  The operator overload for multiplying a complex and an element.
      friend const std::complex<double> operator* (const std::complex<double>& b, const Element& a);


      /// \brief  The operator overload for dividing an element by a double.
      friend const std::complex<double> operator/ (const Element& m, double scalar);

      /// \brief  The operator overload for dividing an element by an element.
      friend const std::complex<double> operator/ (const Element& a, const Element& b);

      /// \brief  The operator overload for dividing an element by an complex.
      friend const std::complex<double> operator/ (const Element& a, const std::complex<double>& b);

      /// \brief  The operator overload for dividing a double by an element.
      friend const std::complex<double> operator/ (double scalar, const Element& m);

      /// \brief  The operator overload for dividing a complex by an element.
      friend const std::complex<double> operator/ (const std::complex<double>& b, const Element& a);


      /// \brief  The operator overload for testing equality between an element and a double.
      friend const bool operator== (const Element& m, double scalar);
      
      /// \brief  The operator overload for testing equality between an element and an element.
      friend const bool operator== (const Element& a, const Element& b);

      /// \brief  The operator overload for testing equality between an element and a complex.
      friend const bool operator== (const Element& a, const std::complex<double>& b);

      /// \brief  The operator overload for testing equality between a double and an element.
      friend const bool operator== (double scalar, const Element& m);
      
      /// \brief  The operator overload for testing equality between a complex and an element.
      friend const bool operator== (const std::complex<double>& b, const Element& a);

    };


  public: // Operator Overloads      

    /// \brief  Get a reference to an element in the matrix to set or get its value.
    Element& operator() (unsigned row, unsigned col);

    /// \brief  Get a reference to an element in the matrix as a column or row vector to set or get its value.
    /// This can be used to access a matrix of (col,row), col = index/nrows, row = index/ncols. 
    /// Matrix A(10); // The matrix is real with dimensions 10x1
    /// A(0) = 10.0;  // The matrix is real.
    /// stComplex cplx = {1.0,2.0};
    /// A(1) = cplx;  // The matrix is now complex with dimensions 10x1.
    Element& operator() (unsigned index);


    bool operator+= (const int    scalar) { return (*this)+=(double)scalar; } //!< add a scaler int     (shorthand notation: A += 5).
    bool operator+= (const float  scalar) { return (*this)+=(double)scalar; } //!< add a scaler float   (shorthand notation: A += 5).
    bool operator+= (const double scalar);                                    //!< add a scaler double  (shorthand notation: A += 5).
    bool operator+= (const std::complex<double> cplx);                        //!< add a scaler complex (shorthand notation: A += (5+2i)).

    bool operator-= (const int    scalar) { return (*this)-=(double)scalar; } //!< subtract a scaler int     (shorthand notation: A -= 5).
    bool operator-= (const float  scalar) { return (*this)-=(double)scalar; } //!< subtract a scaler float   (shorthand notation: A -= 5).
    bool operator-= (const double scalar);                                    //!< subtract a scaler double  (shorthand notation: A -= 5).
    bool operator-= (const std::complex<double> cplx);                        //!< subtract a scaler complex (shorthand notation: A -= (5+2i)).

    bool operator*= (const int    scalar) { return (*this)*=(double)scalar; } //!< multiply a scalar int     (shorthand notation: A *= 5).
    bool operator*= (const float  scalar) { return (*this)*=(double)scalar; } //!< multiply a scalar float   (shorthand notation: A *= 5).
    bool operator*= (const double scalar);                                    //!< multiply a scalar double  (shorthand notation: A *= 5).
    bool operator*= (const std::complex<double> cplx);                        //!< multiply a scaler complex (shorthand notation: A *= (5+2i)).

    bool operator/= (const int    scalar) { return (*this)/=(double)scalar; } //!< divide a scalar int     (shorthand notation: A /= 5).
    bool operator/= (const float  scalar) { return (*this)/=(double)scalar; } //!< divide a scalar float   (shorthand notation: A /= 5).
    bool operator/= (const double scalar);                                    //!< divide a scalar double  (shorthand notation: A /= 5).
    bool operator/= (const std::complex<double> cplx);                        //!< divide a scaler complex (shorthand notation: A /= (5+2i)).

    bool operator+= (const Matrix& mat);  //!< add a matrix      (shorthand notation: A += B).   
    bool operator-= (const Matrix& mat);  //!< subtract a matrix (shorthand notation: A -= B).

    /// \brief  The postfix ++ operator overload.
    /// Add +1.0 to all elements and returns matrix values after the increment, e.g. Matrix B = A++.
    /// Use Inplace_Increment for a boolean return for safer operation.
    friend Matrix operator++ (Matrix& mat, int);

    /// \brief  The postfix -- operator overload.
    /// Subtract 1.0 to all elements and returns matrix values after the increment, e.g. Matrix B = A--.
    /// Use Inplace_Decrement for a boolean return for safer operation.  
    friend Matrix operator-- (Matrix& mat, int);

    /// \brief  Multiply two matrices and copy the result. Result = mat1 * mat2.
    friend Matrix operator* (const Matrix& mat1, const Matrix& mat2); 

    /// \brief  Multiply two matrices and copy the result. Result = mat1 * mat2.
    friend Matrix operator* (Matrix& mat1, Matrix& mat2); 

    /// \brief  Add two matrices and copy the result. Result = mat1 + mat2.
    friend Matrix operator+ (Matrix& mat1, Matrix& mat2);
    
    /// \brief  Add two matrices and copy the result. Result = mat1 + mat2.
    friend Matrix operator+ (const Matrix& mat1, const Matrix& mat2);

    /// \brief  Subtract two matrices and copy the result. Result = mat1 - mat2.
    friend Matrix operator- (Matrix& mat1, Matrix& mat2); 
    
    /// \brief  Subtract two matrices and copy the result. Result = mat1 - mat2.
    friend Matrix operator- (const Matrix& mat1, const Matrix& mat2); 


    /// \brief  Raise all matrix elements to the power scalar.
    friend Matrix operator^ (Matrix& mat, const int scalar)     { return mat^( (double)scalar ); }

    /// \brief  Raise all matrix elements to the power scalar.
    friend Matrix operator^ (Matrix& mat, const float scalar)   { return mat^( (double)scalar ); }

    /// \brief  Raise all matrix elements to the power scalar.
    friend Matrix operator^ (Matrix& mat, const double scalar);

    /// \brief  Add to a matrix by a scalar variable: ie. A = 2.0 + B and B + 2.0 (adds to 2.0 to all elements).
    friend Matrix operator+ (const double scalar, Matrix& mat);
    friend Matrix operator+ (Matrix& mat, const int    scalar)  { return ((double)scalar) + mat; }
    friend Matrix operator+ (Matrix& mat, const float  scalar)  { return ((double)scalar) + mat; }
    friend Matrix operator+ (Matrix& mat, const double scalar)  { return scalar + mat;           }
    friend Matrix operator+ (const int    scalar, Matrix& mat)  { return ((double)scalar) + mat; }
    friend Matrix operator+ (const float  scalar, Matrix& mat)  { return ((double)scalar) + mat; }

    /// \brief  Subtract from a matrix by a scalar variable: ie. A = B - 2.0.
    friend Matrix operator- (Matrix& mat, const double scalar)  { return mat + (-1.0*scalar);         }
    friend Matrix operator- (Matrix& mat, const int    scalar)  { return mat + (-1.0*(double)scalar); }
    friend Matrix operator- (Matrix& mat, const float  scalar)  { return mat + (-1.0*(double)scalar); }

    /// \brief  Subtract a matrix from a scalar variable: ie. A = 2.0 - B == -B + 2.0
    friend Matrix operator- (const double scalar, Matrix& mat);
    friend Matrix operator- (const int    scalar, Matrix& mat)  { return mat - ((double)scalar); }
    friend Matrix operator- (const float  scalar, Matrix& mat)  { return mat - ((double)scalar); }

    /// \brief  Multiply matrix by a scalar variable: A = 2.0 * B and A = B * 2.0.
    friend Matrix operator* (const double scalar, Matrix& mat);
    friend Matrix operator* (Matrix& mat, const int    scalar)  { return (((double)scalar) * mat); }
    friend Matrix operator* (Matrix& mat, const float  scalar)  { return (((double)scalar) * mat); }
    friend Matrix operator* (Matrix& mat, const double scalar)  { return (scalar * mat);           }
    friend Matrix operator* (const int    scalar, Matrix& mat)  { return (((double)scalar) * mat); }
    friend Matrix operator* (const float  scalar, Matrix& mat)  { return (((double)scalar) * mat); }   

    /// \brief  Divide matrix by a scalar variable: A = B / 2.0.
    friend Matrix operator/ (Matrix& mat, const double scalar);
    friend Matrix operator/ (Matrix& mat, const int    scalar)  { return mat / ((double)scalar); }
    friend Matrix operator/ (Matrix& mat, const float  scalar)  { return mat / ((double)scalar); }

    /// \brief  Divide matrix into a scalar variable: A = 2.0 / B. e.g. A = [2.0 2.0; 2.0 2.0] / B, B is 2x2.
    friend Matrix operator/ (const double scalar, Matrix& mat);
    friend Matrix operator/ (const int    scalar, Matrix& mat)  { return ((double)scalar) / mat; }
    friend Matrix operator/ (const float  scalar, Matrix& mat)  { return ((double)scalar) / mat; }


    // Accessing matrix data as real only:
    // The matrix can be accessed using the square bracket operators.
    // e.g. Matrix A(2,2); double b = A[0][0]; A[0][1] = 10.0; A[0][1] = -10.0;
  public:

    /// \brief  A nested class for access only to the real part of the matrix. 
    /// It is used for operator[] access by the Matrix.    
    ///
    /// Sequential [][] operators can be used to access the matrix. The
    /// first overload of [] is at the Matrix level. This returns a copy of 
    /// a RealOnlyAccess and the second overload of [] allows access to the 
    /// required element from the matrix.
    ///
    /// For matrices that are always real. This method of accessing each element
    /// is the most efficient.
    class RealOnlyAccess
    {
    public:

      /// \brief  Set or get an element value.
      double& operator [] (const unsigned col);

      /// \brief  The matrix is a friend class.
      friend class Matrix;

      /// \brief  The only constructor.
      RealOnlyAccess( MTX& pMatrix, const unsigned row );

      /// \brief  The destructor.
      virtual ~RealOnlyAccess();

      /// \brief  The assignment operator from a double for single index operations. 
      RealOnlyAccess& operator=(const double value);

      /// \brief  The assignment operator from another RealOnlyAccess object for single index operations.
      RealOnlyAccess& operator=(RealOnlyAccess& rhs);

      /// \brief  The assignment operator from a MatrixElement object for single index operations.
      RealOnlyAccess& operator=(Element& rhs);

      /// \brief  The casting operator overload for double.
      operator const double();
      
    public: // operator overloads

      bool operator+= (const int    scalar) { return (*this)+=(double)scalar; } //!< add a scaler int     (shorthand notation: A[0] += 5).
      bool operator+= (const float  scalar) { return (*this)+=(double)scalar; } //!< add a scaler float   (shorthand notation: A[0] += 5).
      bool operator+= (const double scalar);                                    //!< add a scaler double  (shorthand notation: A[0] += 5).

      bool operator-= (const int    scalar) { return (*this)-=(double)scalar; } //!< subtract a scaler int     (shorthand notation: A[0] -= 5).
      bool operator-= (const float  scalar) { return (*this)-=(double)scalar; } //!< subtract a scaler float   (shorthand notation: A[0] -= 5).
      bool operator-= (const double scalar);                                    //!< subtract a scaler double  (shorthand notation: A[0] -= 5).

      bool operator*= (const int    scalar) { return (*this)*=(double)scalar; } //!< multiply a scalar int     (shorthand notation: A[0] *= 5).
      bool operator*= (const float  scalar) { return (*this)*=(double)scalar; } //!< multiply a scalar float   (shorthand notation: A[0] *= 5).
      bool operator*= (const double scalar);                                    //!< multiply a scalar double  (shorthand notation: A[0] *= 5).

      bool operator/= (const int    scalar) { return (*this)/=(double)scalar; } //!< divide a scalar int     (shorthand notation: A[0] /= 5).
      bool operator/= (const float  scalar) { return (*this)/=(double)scalar; } //!< divide a scalar float   (shorthand notation: A[0] /= 5).
      bool operator/= (const double scalar);                                    //!< divide a scalar double  (shorthand notation: A[0] /= 5).
      
    private:
      /// \brief  Check the specified indices. Throw an exception if they are invalid.
      /// \return true if valid, false otherwise. 
      bool IndexCheck( const unsigned row, const unsigned col );

      /// \brief  Check the specified index into the Matrix as a vector. 
      ///         Throw an exception if the index is invalid.
      /// \return true if valid, false otherwise. 
      bool IndexCheck( const unsigned index );

      MTX& m_Matrix;   //!< The reference to the source matrix.    
      unsigned m_row;  //!< The current row to access.
    };

    /// \brief  Retrieve a copy of a RealOnlyAccess object which is then used for the second [] overload.
    RealOnlyAccess operator[] (const unsigned row); 

  public: // functions for error handling.

    /// \brief  Clear the matrix from memory and handle the error message.
    void MatrixError( const char* error );

    /// \brief  Clear the matrix from memory and handle the error message.
    void MatrixError( const char* function, const char* error );

    /// \brief  A static function to handle the error message.
    static void StaticMatrixError( const char* error );

    /// \brief  A static function to handle the error message.
    static void StaticMatrixError( const char* function, const char* error );


  protected:

    /// \brief  Check the specified indices. Throw an exception if they are invalid.
    /// \return true if valid, false otherwise. return code should not be reached!
    bool IndexCheck( const unsigned row, const unsigned col );

    /// \brief  Check the specified index into the Matrix as a vector. 
    ///         Throw an exception if the index is invalid.
    /// \return true if valid, false otherwise. return code should not be reached!    
    bool IndexCheck( const unsigned index );

    /// \brief  A single element from the matrix. This is used for write access with operator().
    Element m_MatrixElement;

    /// \brief  The deep level matrix container.
    MTX m_Matrix;

    /// \brief  This indicates if the mtx core engine been initialized.
    static bool m_IsMTXInitialized; 
  };

} // end namespace Zenautics

#endif // _ZENAUTICS_MATRIX_H_


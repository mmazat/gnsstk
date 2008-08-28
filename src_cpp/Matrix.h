/**
\file     Matrix.h
\brief    The Zenautics Matrix Class
\author   Glenn D. MacGougan (GDM)
\date     2008-05-07
\version  0.05 Beta

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
#define _MATRIX_NO_EXCEPTION // removes exception handling support. \n
*/

#ifndef _ZENAUTICS_MATRIX_H_
#define _ZENAUTICS_MATRIX_H_

#include <complex> // For std::complex<double> (Standard Template Library)
#include <string>  // For std::string (Standard Template Library)
#include "cmatrix.h" // The core matrix engine is written in 'c'.

//#define _MATRIX_NO_EXCEPTION // removes exception handling support if required.


namespace Zenautics
{

#ifndef _MATRIX_NO_EXCEPTION
  /**
  \class   MatrixException
  \brief   A class for exceptions thrown by the Matrix class.  
  
  The MATRIX_USE_EXCEPTION_HANDLING define enables the use of exception 
  handling using try{} catch{}. A MatrixException is thrown. The use of 
  exception handling is very highly recommended. When it is turned off, 
  the matrix will try to output a message and then call 'exit(1)'.
  
  \code
  int main()
  { 
    try
    {
      Matrix A(2,2);
      double d = A(3,1).real(); // causes an out of bounds exception
    }
    catch( MatrixException& matrix_exception )
    {
      cout << matrix_exception << endl;
    }
    catch ( ... )
    {
      cout << "Caught unknown exception" << endl;
    }
    return 0;
  }
  \endcode
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
    /// \brief  The matrix exception string.  
    std::string m_ExceptionString;

  private:
    /// \brief  The matrix exception character string.  
    char m_msg[256];    
  };
#endif


  /**
  \class   Matrix
  \brief   The matrix/vector class. 
           Both real and complex data are inherently supported.
           One and two dimensional data.
  
  The matrix class supports advanced real and complex functionality. It is
  optimized for columnwise operations. Refer to example_main.cpp for a 
  complete example program using the Matrix.
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
    explicit Matrix(const unsigned nrows);

    /// \brief  A matrix style constructor.
    ///
    /// Matrix A(nrows,ncols); creates an nrowsxncols real 'matrix'. A real matrix is assumed.
    /// Matrix A(nrows,ncols,false); creates an nrowsxncols complex 'matrix'. A real matrix is assumed.
    Matrix(const unsigned nrows, const unsigned ncols, const bool isReal=true);

    /// \brief  The copy constructor.
    Matrix(const Matrix& mat);                          

    /// \brief  A constructor reading data from a file.
    Matrix(const char* path, bool& itWorked);

    /** \brief  A constructor initialized the matrix from a string.

    There are two general possible interpretations of the string input. \n
    
    (1) Square bracket delimited matrix. e.g. \n
    
    \code
    Matrix A = "[1 2 3; 4 5 6]"; // or 
    Matrix A = "[1, 2, 3; 4, 5, 6]";
    \endcode

    In this case '[' donates the start of a matrix and ']' denotes the end. \n
    Row vectors [1 2 3] and [4 5 6] are separated by ';'.  \n
    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A = "[1+1i 2+3j 1-2i; 4 5 6]"; // or
    Matrix A = "[1+1i, 2+3j, 1-2i; 4, 5, 6]";
    \endcode
    
    (2) Free form delimited matrix. e.g. \n

    \code
    Matrix A = "1 2 3 \\n 4 5 6 \\n";
    \endcode

    In this case, the newline delimits different rows of the matrix. (\\r\\n also works). \n
    Row vectors can still be delimited by ';' as well. \n
    
    \code
    Matrix B = "1 2 3; 4 5 6; \\n 7 8 9";
    \endcode 
    
    will set a 3x3 matrix == [1 2 3; 4 5 6; 7 8 9]. \n

    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A = "[1+1i 2+3j 1-2i\\n 4 5 6]";   // or
    Matrix A = "1+1i, 2+3j, 1-2i\\n 4, 5, 6"; // or
    Matrix A = "1+1i 2+3j 1-2i; 4, 5, 6";   
    \endcode 

    All result in A = [1+1i 2+3i 1-2i; 4 5 6]; \n
    */
    Matrix(const char* strMatrix);

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

    /**
    \brief  The assignement operator from a string matrix.   
    
    There are two general possible interpretations of the string input. \n
    
    (1) Square bracket delimited matrix. e.g. \n
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6]"; // or 
    A = "[1, 2, 3; 4, 5, 6]";
    \endcode

    In this case '[' donates the start of a matrix and ']' denotes the end. \n
    Row vectors [1 2 3] and [4 5 6] are separated by ';'.  \n
    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A;
    A = "[1+1i 2+3j 1-2i; 4 5 6]"; // or
    A = "[1+1i, 2+3j, 1-2i; 4, 5, 6]";
    \endcode
    
    (2) Free form delimited matrix. e.g. \n

    \code
    Matrix A; 
    A = "1 2 3 \\n 4 5 6 \\n";
    \endcode

    In this case, the newline delimits different rows of the matrix. (\\r\\n also works). \n
    Row vectors can still be delimited by ';' as well. \n
    
    \code
    B = "1 2 3; 4 5 6; \\n 7 8 9";
    \endcode 
    
    will set a 3x3 matrix == [1 2 3; 4 5 6; 7 8 9]. \n

    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A;
    A = "[1+1i 2+3j 1-2i\\n 4 5 6]";   // or
    A = "1+1i, 2+3j, 1-2i\\n 4, 5, 6"; // or
    A = "1+1i 2+3j 1-2i; 4, 5, 6";   
    \endcode 

    All result in A = [1+1i 2+3i 1-2i; 4 5 6]; \n
    */
    Matrix& operator=(const char* strMatrix);

  public:

    /**
    \brief  Clear the matrix memory. Set the matrix to size 0x0.
    
    \code 
    Matrix A(10,10); // A 10 x 10 matrix
    if( !A.Clear() )
      return false;
    // A is now 0x0 
    \endcode

    \return true if successul, false if error.
    */
    bool Clear();

  public: // Matrix Qualifiers

    /// \brief  Is this matrix empty?
    bool isEmpty() const;

    /// \brief  Is the matrix mat conformal for multiplication (*this * mat)?
    bool isConformal(const Matrix& mat) const;

    /// \brief  Is this matrix the same size as mat?
    bool isSameSize(const Matrix& mat) const;  

    /// \brief  Is this a square matrix?
    bool isSquare() const;

    /// Check if this matrix is stored as a complex matrix.
    bool isStoredAsComplex();

    /// Check if this a real matrix.
    bool isReal();

    /// Check if this a complex matrix.
    bool isComplex(); 

    /// Check if this is a vector. Is the matrix either nx1 or 1xn.
    bool isVector();

    unsigned GetNrCols() const;   //!< return no. of cols
    unsigned ncols() const;       //!< return no. of cols
    unsigned GetNrElems() const;  //!< return total no. of elements
    unsigned nelems() const;      //!< return total no. of elements
    unsigned GetNrRows() const;   //!< return no. of rows         
    unsigned nrows() const;       //!< return no. of rows         
    unsigned GetLength() const;   //!< return the maximum dimension either nrows or ncols whichever is greater.


    /**
    \brief  Return the real part of the matrix at this row and column.

    \code 
    Matrix A = "2+4i";
    double a = A.real(0,0); // a is 2.0
    \endcode
    */
    double real(const unsigned row, const unsigned col);

    /**
    \brief  Return the real part of the matrix at this vector index.

    \code 
    Matrix A = "[2+4i, 10-1i]";
    double a = A.real(1); // a is 10.0
    \endcode
    */
    double real(const unsigned index);

    /**
    \brief  Return the imaginary part of the matrix at this row and column.

    \code 
    Matrix B = "2+4i";
    double b = B.imag(0); // b is 4.0
    \endcode
    */    
    double imag(const unsigned row, const unsigned col);

    /**
    \brief  Return the imaginary part of the matrix at this vector index.
    
    \code 
    Matrix B = "[2+4i, 1-10i]";
    double b = B.imag(1); // b is -10.0
    \endcode
    */    
    double imag(const unsigned index);


  public: // Input Operations

    /**
    \brief  Read the matrix from an ASCII file with the path given by the 'c' style string
    (with automatric support for many delimiters, whitespace, or ',', or ';', or many others) 
    or a compressed BINARY matrix file used in the Save function.
    Complex and real data input are supported.
    A non-numeric header line can be present which will be skipped.

    \code
    Matrix A;
    Matrix B;
    Matrix C;
    bool result;

    result = A.ReadFromFile("data.txt"); // Read an ASCII numeric data file.
    result = B.ReadFromFile("data.csv"); // Read a comma delimited numeric data file. e.g. saved from EXCEL.
    result = C.ReadFromFile("data.mtx"); // Read a compressed binary matrix (MTX format).
    \endcode
    
    \return true if successful, false otherwise
    */
    bool ReadFromFile( const char *path );
    
    /**
    \brief  Read the matrix from a file given the file path as a standard string.
    
    \code
    Matrix A;
    std::string str = "data.txt";
    if( !A.ReadFromFile(str) )
      return false;
    \endcode

    \return true if successful, false otherwise.
    */
    bool ReadFromFile( std::string path );


    /**  
    \brief  A safe function for performing a copy of another matrix.
    
    \code
    Matrix A(2,2);
    A[0][0] = 1.0;
    A[0][1] = 2.0;
    A[1][0] = 3.0;
    A[1][1] = 4.0;
    Matrix B;
    if( !B.Copy(A) )
      return false;
    \endcode
    
    \return true if successful, false otherwise
    */
    bool Copy( Matrix& src );


    /**      
    \brief  A safe function for setting the matrix from a double.
    
    \code
    double d = 10.0;
    Matrix A;
    if( !A.Copy(d) )
      return false;
    \endcode
    
    \return true if successful, false otherwise
    */
    bool Copy( const double& value );


    /**      
    \brief  A safe function for setting the matrix from a std::complex<double>.
    
    \code
    std::complex<double> cplx(1.0,2.0);
    Matrix A;
    if( !A.Copy(cplx) )
      return false;
    \endcode
    
    \return true if successful, false otherwise
    */
    bool Copy( const std::complex<double>& cplx );


  public: // Output Operations

    /**
    \brief  Saves a matrix to the specified file path (a 'c' style string)
            using a proprietary compressed format.
    \code
    Matrix A;
    A = "[1,2,3; 4,5,6; 7,8,9]";
    if( !A.Save("data.mtx" ) )
      return false;
    \endcode 

    \return true if successful, false otherwise
    */
    bool Save( const char* path );

    /**
    \brief  Saves a matrix to the specified file path (a std::string)
            using a proprietary compressed format.
            
    \code
    Matrix A;
    std::string str = "data.mtx";
    A = "[1,2,3; 4,5,6; 7,8,9]";
    if( !A.Save(str) )
      return false;
    \endcode 

    \return true if successful, false otherwise
    */
    bool Save( std::string path );
    
    /**
    \brief  Print the matrix to a file with automatically determined column width 
            and the specified precision, uses "%'blank''-'autowidth.precision'g'", to 
            the 'c' style path string provided.
    \code
    A = "[1,2,3; 4,5,6; 7,8,9]";
    if( !A.Print( "data.txt", 14 ) ) // Print the matrix to data.txt
      return false;
    \endcode   

    \return true if successful, false otherwise
    */
    bool Print( const char *path, const unsigned precision = 9, bool append = false );

    /**
    \brief  Print the matrix to a file with automatically determined column width 
            and the specified precision, uses "%'blank''-'autowidth.precision'g'", to 
            the std:string path provided.
    \code
    A = "[1,2,3; 4,5,6; 7,8,9]";
    std::string str = "data.txt";
    if( !A.Print( str, 14 ) ) // Print the matrix to data.txt
      return false;
    \endcode   

    \return true if successful, false otherwise
    */
    bool Print( std::string path, const unsigned precision, bool append = false );

    /**
    \brief  Print the matrix to the standard output (stdout) with automatically 
            determined column width and the specified precision, 
            uses "%'blank''-'autowidth.precision'g'".
    \code
    Matrix A;
    A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    bool result = A.PrintStdout(6); // Print to stdout with automatic width determination.
    // results in:
    // 0123456789012345678901234567890
    //  1.123  0  2.123 -1
    //  3.123  0  4.123 -1
    \endcode
        
    \return true if successful, false otherwise
    */
    bool PrintStdout( const unsigned precision = 6 );

    /**
    \brief  Print the matrix to a buffer of maxlength with automatically determined column width 
    and the specified precision, uses "%'blank''-'autowidth.precision'g'"

    \code
    Matrix A;
    A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    char buffer[256]; 
    bool result = A.PrintToBuffer( buffer, 256, 6); // Print to a buffer with automatic width determination.
    cout << buffer << endl;
    // results in:
    // 0123456789012345678901234567890
    //  1.123  0  2.123 -1
    //  3.123  0  4.123 -1
    \endcode
     
    \return true if successful, false otherwise
    */
    bool PrintToBuffer( char* buffer, const unsigned maxlength, const unsigned precision );


    /**
    \brief  Print the matrix to a file with specifed width and precision
            PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
            to file specified with the 'c' style path string provided.
    \code
    Matrix A;
    A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    if( !A.PrintFixedWidth( "data.txt", 6, 3 ) )
      return false;
    // results in: data.txt with
    // 0123456789012345678901234567890
    //  1.123     0 2.123    -1
    //  3.123     0 4.123    -1
    \endcode

    \return true if successful, false otherwise
    */
    bool PrintFixedWidth( const char* path, const unsigned width, const unsigned precision, bool append = false );


    /**
    \brief  Print the matrix to a file with specifed width and precision
            PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
            to file specified with the std::string path string provided.
    \code
    Matrix A;
    A = "[1.123 0 2.123 -1; 3.123 0 4.123 -1]";  // Set A using string notation.
    std::string str = "data.txt";
    if( !A.PrintFixedWidth( str, 6, 3 ) )
      return false;
    // results in: data.txt with
    // 0123456789012345678901234567890
    //  1.123     0 2.123    -1
    //  3.123     0 4.123    -1
    \endcode

    \return true if successful, false otherwise
    */   
    bool PrintFixedWidth( std::string path, const unsigned width, const unsigned precision, bool append = false );

    /**
    \brief  Print the matrix to a buffer of maxlength with specifed width and precision
            PrintAutoWidth is recommended over this function, "%'blank''-'width.precision'g'"
    \code
    Matrix A;
    A = "[1.123 2.123 -1; 3.123 4.123 -1]";  // Set A using string notation.
    char buffer[256]; 
    bool result = A.PrintFixedWidthToBuffer( buffer, 256, 10, 6 ); // Print to a buffer with fixed width.
    cout << buffer << endl;
    // results in:
    // 0123456789012345678901234567890    
    //  1.123     2.123    -1
    //  3.123     4.123    -1
    \endcode
      
    \return true if successful, false otherwise
    */
    bool PrintFixedWidthToBuffer( char* buffer, const unsigned maxlength, const unsigned width, const unsigned precision );

    /**
    \brief  Print the matrix to a file path specified by the 'c' style string 
            with specifed precision and delimiter.
    \code
    Matrix A;
    A = "[1.123 2.123 -1; 3.123 4.123 -1]";  // Set A using string notation.
    if( !A.PrintDelimited( "data.csv", 5, ',' ) )
      return false;
    // results in: data.csv with
    // 0123456789012345678901234567890    
    // 1.123,2.123,-1
    // 3.123,4.123,-1
    \endcode

    \return true if successful, false otherwise
    */
    bool PrintDelimited( const char *path, const unsigned precision, const char delimiter, bool append = false );

    /**
    \brief  Print the matrix to a file path specified by the std::string 
            with specifed precision and delimiter.
    \code
    Matrix A;
    A = "[1.123 2.123 -1; 3.123 4.123 -1]";  // Set A using string notation.
    std::string str = "data.csv";
    if( !A.PrintDelimited( str, 5, ',' ) )
      return false;
    // results in: data.csv with
    // 0123456789012345678901234567890    
    // 1.123,2.123,-1
    // 3.123,4.123,-1
    \endcode

    \return true if successful, false otherwise
    */
    bool PrintDelimited( std::string path, const unsigned precision, const char delimiter, bool append = false );
    
    /**
    \brief  Print the matrix to a 'c' style string buffer of maxlength with specifed precision and delimiter.
    
    \code
    Matrix A;
    A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    char buffer[256]; 
    if( !A.PrintDelimitedToBuffer( buffer, 256, 6, ',' ) ) // Print to a buffer using comma delimiters.
      return false;
    cout << buffer << endl;
    // results in:
    // 1.123,2.123
    // 3.123,4.123
    \endcode
      
    \return true if successful, false otherwise
    */
    bool PrintDelimitedToBuffer( char *buffer, const unsigned maxlength, const unsigned precision, const char delimiter );

    /**
    \brief  Print a row to a 'c' style string buffer.
    
    \code
    Matrix A;
    A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    char buffer[256]; 
    if( !A.PrintRowToString( 1, buffer, 256, 4, 6 ) ) // Print the second row to the char buffer.
      return false;
    cout << buffer << endl;
    // results in:
    // 3.123   4.123
    \endcode
    
    \return true if successful, false otherwise
    */
    bool PrintRowToString( const unsigned row, char *buffer, const unsigned maxlength, const int width, const int precision );


  public: // Change the dimensions of the matrix

    /**  
    \brief  Remove a single column from the matrix.

    \code
    Matrix A;
    A = "[1.123 0 2.123; 3.123 0 4.123]";  // Set A using string notation.
    if( !A.RemoveColumn(1) ) // Remove the column with the zeros
      return false;
    // results in 
    // A
    // 1.123 2.123
    // 3.123 4.123
    \endcode
        
    \return true if successful, false otherwise.  
    */
    bool RemoveColumn( const unsigned col );

    /**
    \brief  Remove all the columns 'after' the column index given.

    \code
    Matrix A;
    A = "[1.123 0 2.123; 3.123 0 4.123]";  // Set A using string notation.
    if( !A.RemoveColumnsAfterIndex(0) ) // Remove the 2nd and 3rd columns, i.e. after the 0th column.
      return false;
    // results in 
    // A
    // 1.123
    // 3.123
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool RemoveColumnsAfterIndex( const unsigned col );

    /** 
    \brief  Remove the rows and columns specified by the indices in the rows[] and cols[] arrays.
    
    \code
    Matrix A(4,4);
    unsigned rows[2];
    unsigned cols[2];
    rows[0] = 0; // remove row 0
    rows[1] = 2; // remove row 2
    cols[0] = 0; // remove column 0
    cols[1] = 2; // romve column 2
    A.RemoveRowsAndColumns( 2, (unsigned int *)rows, 2, (unsigned int *)cols );
    // A is now a 2x2 matrix
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool RemoveRowsAndColumns( const unsigned nrows, const unsigned rows[], const unsigned ncols, const unsigned cols[] );

    /**
    \brief  Insert a column matrix into the matrix.
    
    \code
    Matrix A;
    Matrix B(2,2);
    A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    if( !A.InsertColumn( B, 1, 1 ) ) // Insert second column of B into the second column a A.
      return false;
    // results in:
    // A (2x3)
    // 1.123  0   2.123
    // 3.123  0   4.123
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool InsertColumn( const Matrix &src, const unsigned dst_col, const unsigned src_col );

    /** 
    \brief  Add a column to the end of the matrix.

    \code
    Matrix A;
    atrix B(2,2);
    A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    if( !A.AddColumn( B, 1 ) ) // Add second column of B to A.
      return false;
    // results in:
    // A (2x3)
    // 1.123  2.123 0
    // 3.123  4.123 0
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool AddColumn( const Matrix &src, const unsigned src_col );

    /**
    \brief  Combine two matrices with the same nrows, A becomes A|B.

    \code
    Matrix A;
    atrix B(2,2);
    A = "[1.123 2.123; 3.123 4.123]";  // Set A using string notation.
    if( !A.Concatonate( B ) ) // make A = A | B
      return false;
    // results in:
    // A (2x4)
    // 1.123  2.123 0 0
    // 3.123  4.123 0 0
    \endcode

    \return true if successful, false otherwise.  
    */
    bool Concatonate( const Matrix &src );

    /**
    \brief  Redimension the matrix, original data is saved in place, new 
            data is set to zero. The default value for ncols allows 
            redimensioning as a vector.
    \code
    Matrix A(4,4);       // A is 4x4
    A[0][0] = 1;
    A[1][1] = -1;
    if( !A.Redim(2,2) )  // A is 2x2 but data values are retained.
      return false;
    // results in:
    // A (2x2)
    // 1  0
    // 0 -1

    Matrix B(10);     // B is a vector with length 10.
    B[0] = -1;
    B[1] = 1;
    if( !B.Redim(2) ) // B is a vector with length 2 but data values are retained
      return false;
    // results in:
    // B 
    // -1
    // 1
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool Redim( const unsigned nrows, const unsigned ncols=1 );

    /**
    \brief  Resize the matrix, original data is lost, new data is set to zero.
            The default value for ncols allows resizing as a vector.

    \code
    Matrix A(4,4);       // A is 4x4
    A[0][0] = 1;
    A[1][1] = -1;
    if( !A.Resize(2,2) )  // A is 2x2 and zero.
      return false;
    // results in:
    // A (2x2)
    // 0 0
    // 0 0

    Matrix B(10);     // B is a vector with length 10.
    B[0] = -1;
    B[1] = 1;
    if( !B.Resize(2) ) // B is a vector with length 2 and is zero.
      return false;
    // results in:
    // B 
    // 0
    // 0
    \endcode
            
    \return true if successful, false otherwise.  
    */
    bool Resize( const unsigned nrows, const unsigned ncols=1 );


  public: // Setting matrix values

    /**
    \brief  Set the matrix from the static 'c' style matrix indexed by mat[i*ncols + j].

    \code
    Matrix A;
    double data[4] = {1.0,2.0,3.0,4.0};
    if( !A.SetFromStaticMatrix( data, 1, 4 ) )
      return false;
    \\ results in 
    \\ A
    \\ 1.0 2.0 3.0 4.0
    if( !A.SetFromStaticMatrix( data, 2, 2 ) )
      return false;    
    \\ results in 
    \\ A
    \\ 1.0 2.0 
    \\ 3.0 4.0    
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool SetFromStaticMatrix( const double mat[], const unsigned nrows, const unsigned ncols );

    /**
    \brief  Setting the matrix values from a string matrix.
    
    There are two general possible interpretations of the string input. \n
    
    (1) Square bracket delimited matrix. e.g. \n
    
    \code
    Matrix A;
    A.SetFromMatrixString( "[1 2 3; 4 5 6]" ); // or 
    A.SetFromMatrixString( "[1, 2, 3; 4, 5, 6]" );
    \endcode

    In this case '[' donates the start of a matrix and ']' denotes the end. \n
    Row vectors [1 2 3] and [4 5 6] are separated by ';'.  \n
    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A;
    A.SetFromMatrixString( "[1+1i 2+3j 1-2i; 4 5 6]" ); // or
    A.SetFromMatrixString( "[1+1i, 2+3j, 1-2i; 4, 5, 6]" );
    \endcode
    
    (2) Free form delimited matrix. e.g. \n

    \code
    Matrix A; 
    A.SetFromMatrixString( "1 2 3 \\n 4 5 6 \\n" );
    \endcode

    In this case, the newline delimits different rows of the matrix. (\\r\\n also works). \n
    Row vectors can still be delimited by ';' as well. \n
    
    \code
    A.SetFromMatrixString( "1 2 3; 4 5 6; \\n 7 8 9" );
    \endcode 
    
    will set a 3x3 matrix == [1 2 3; 4 5 6; 7 8 9]. \n

    Commas can delimit row vector data but are not needed. \n
    Complex input: e.g. 
    
    \code
    Matrix A;
    A.SetFromMatrixString( "[1+1i 2+3j 1-2i\\n 4 5 6]" );   // or
    A.SetFromMatrixString( "1+1i, 2+3j, 1-2i\\n 4, 5, 6" ); // or
    A.SetFromMatrixString( "1+1i 2+3j 1-2i; 4, 5, 6" );   
    \endcode 

    All result in A = [1+1i 2+3i 1-2i; 4 5 6]; \n
   
    \return true if successful, false otherwise.
    */
    bool SetFromMatrixString(const char* strMatrix);


    /**    
    \brief  Copy the src data in column col to dst matrix, resize dst if possible and necessary.
    
    \code
    Matrix A;
    A = "[1 -1; 2 -2; 3 -3]".
    Matrix B;
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.CopyColumn(0,B); // Copy the first column of A into B.
    result = B.PrintStdout();   // Print Matrix B. B = [1;2;3];
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool CopyColumn( const unsigned src_col, Matrix &dst );

    /**
    \brief  Insert a submatrix (src) into dst, starting at indices dst(row,col).
    
    \code
    Matrix A(4,4); // A 4x4 matrix of zeros.
    Matrix B(2,2); // A 2x2 matrix that we will fill with sevens.
    B.Fill(7.0);
    bool result;
    result = A.PrintStdout();           // Print Matrix A.
    result = A.InsertSubMatrix(B,1,1);  // Put B in the middle of A.
    result = A.PrintStdout();           // Print Matrix A. A = [0 0 0 0; 0 7 7 0; 0 7 7 0; 0 0 0 0].
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool InsertSubMatrix( const Matrix &src, const unsigned dst_row, const unsigned dst_col );


    /**
    \brief  Extract a submatrix (dst) from this matrix from (inclusive) 
            the rows and columns specified.
    \code 
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B;
    bool result = A.ExtractSubMatrix( B, 1, 0, 2, 2 );
    // B == [4 5 6; 7 8 9]
    \endcode

    \return true if successful, false otherwise.
    */
    bool ExtractSubMatrix( 
      Matrix &dst,             //!< The destination matrix to contain the submatrix.
      const unsigned from_row, //!< The zero-based index for the from row.
      const unsigned from_col, //!< The zero-based index for the from column.
      const unsigned to_row,   //!< The zero-based index for the to row.
      const unsigned to_col    //!< The zero-based index for the to column.
      );

    /**
    \brief  Zero the entire matrix.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.Zero();          // Set A back to zeros.
    result = A.PrintStdout();   // Print Matrix A. A = [0 0 0; 0 0 0; 0 0 0].
    \endcode
    
    \return true if successful, false otherwise.  
    */
    bool Zero();

    /**
    \brief  Zero all elements in a specified column.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.ZeroColumn(1);   // Set the second column of A back to zeros.
    result = A.PrintStdout();   // Print Matrix A. A = [1 0 3; 4 0 6; 7 0 9].
    \endcode
       
    \return true if successful, false otherwise.    
    */
    bool ZeroColumn( const unsigned col );

    /**
    \brief  Zero all elements in a specified row.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.ZeroRow(1);      // Set the second row of A back to zeros.
    result = A.PrintStdout();   // Print Matrix A. A = [1 2 3; 0 0 0; 7 8 9].
    \endcode
       
    \return true if successful, false otherwise.    
    */
    bool ZeroRow( const unsigned row );


    /**
    \brief  Efficiently swaps the contents of this matrix with matrix M.
    The contents are exhanged without the need to copy matrix data.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = "[1 2; 3 4]";        
    bool result;
    result = A.Swap(B);    
    result = A.PrintStdout();   // Print Matrix A. A = [1 2; 3 4]
    result = B.PrintStdout();   // Print Matrix B. B = [1 2 3; 4 5 6; 7 8 9]
    \endcode

    \return true if successful, false otherwise.    
    */
    bool Swap( Matrix &M );

    /**
    \brief  Fill the matrix with the given value.

    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.Fill(7);         // Fill the matrix with 7.0.
    result = A.PrintStdout();   // Print Matrix A. A = [7 7 7; 7 7 7; 7 7 7].
    \endcode

    \return true if successful, false otherwise.    
    */
    bool Fill( const double value );

    /**
    \brief  Fill the matrix column with the given value.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.FillColumn(1,7); // Fill the second column with 7.0.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 7 3; 4 7 6; 7 7 9].
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool FillColumn( const unsigned col, const double value );

    /**
    \brief  Fills the matrix row with the given value.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.FillRow(1,7);    // Fill the second row with 7.0.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 2 3; 7 7 7; 7 8 9].
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool FillRow( const unsigned row, const double value );

    /**
    \brief  Reverse the order of elements of a column.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.FlipColumn(1);   // Flip the second column.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 8 3; 4 5 6; 7 2 9].
    \endcode
       
    \return true if successful, false otherwise.    
    */
    bool FlipColumn( const unsigned col );

    /**
    \brief  Reverse the order of elements of a row.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.FlipRow(1);      // Flip the second row.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 2 3; 6 5 4; 7 8 9].
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool FlipRow( const unsigned row );

    /**
    \brief  Set the matrix to identity using the current dimensions.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();   // Print Matrix A.
    result = A.Identity();      // Set A to identity.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 0 0; 0 1 0; 0 0 1].
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Identity();

    /**   
    \brief  Set the matrix to identity using the specified dimension (nxn).
    
    \code
    Matrix A;
    bool result;
    result = A.Identity(3);     // Set A to identity, 3x3.
    cout << endl;
    result = A.PrintStdout();   // Print Matrix A. A = [1 0 0; 0 1 0; 0 0 1].
    \endcode    
    
    \return true if successful, false otherwise.        
    */
    bool Identity(const unsigned dimension);



  public: // Inplace Operations

    /**
    \brief  Transpose the matrix as an inplace operation.
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();         // Print Matrix A.
    result = A.Inplace_Transpose();   // Make A = transpose(A).
    cout << endl;
    result = A.PrintStdout();         // Print Matrix A. A = [1 4 7; 2 5 8; 3 6 9].
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Transpose();
    bool Inplace_transpose() { return this->Inplace_Transpose(); }

    /**
    \brief  Round the matrix elements to the specified presision. \n
    e.g. precision = 0    1.8    -> 2     (default)\n
    e.g. precision = 1,   1.45   -> 1.5   \n
    e.g. precision = 2    1.456  -> 1.46  \n
    e.g. precision = 3,   1.4566 -> 1.457 \n
    *
    \code
    Matrix A;
    A = "[1.09 2.08 3.07; 4.06 5.05 6.04; 7.03 8.02 9.01]";
    bool result;
    result = A.PrintStdout();     // Print Matrix A.
    result = A.Inplace_Round(1);  // Make A = round(A) to the 1st decimal place.
    cout << endl;
    result = A.PrintStdout();     // Print Matrix A. A = "[1.1 2.1 3.1; 4.1 5.1 6.0; 7.0 8.0 9.0]";
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Round( const unsigned precision = 0 );
    bool Inplace_round( const unsigned precision = 0 ) { return this->Inplace_Round( precision ); }

    /**
    \brief  Round the matrix elements to the nearest integers towards minus infinity.
    
    \code
    Matrix A;
    A = "[1.9 2.8 3.7; -4.6 -5.5 -6.4; 7.3 8.2 9.1]";
    bool result;
    result = A.PrintStdout();     // Print Matrix A.
    result = A.Inplace_Floor();   // Make A = floor(A).
    cout << endl;
    result = A.PrintStdout();     // Print Matrix A. A = "[1 2 3; -5 -6 -7; 7 8 9]";
    \endcode    
       
    \return true if successful, false otherwise.    
    */
    bool Inplace_Floor();
    bool Inplace_floor() { return this->Inplace_Floor(); }

    /**
    \brief  Round the matrix elements to the nearest integers towards infinity.
    
    \code
    Matrix A;
    A = "[1.9 2.8 3.7; -4.6 -5.5 -6.4; 7.3 8.2 9.1]";
    bool result;
    result = A.PrintStdout();     // Print Matrix A.
    result = A.Inplace_Ceil();    // Make A = ceil(A).
    cout << endl;
    result = A.PrintStdout();     // Print Matrix A. A = "[2 3 4; -4 -5 -6; 8 9 10]";
    \endcode    
           
    \return true if successful, false otherwise.    
    */
    bool Inplace_Ceil();
    bool Inplace_ceil() { return this->Inplace_Ceil(); }

    /**
    \brief  Rounds the matrix elements of X to the nearest integers towards zero.
    
    \code
    Matrix A;
    A = "[1.9 2.8 3.7; -4.6 -5.5 -6.4; 7.3 8.2 9.1]";
    bool result;
    result = A.PrintStdout();     // Print Matrix A.
    result = A.Inplace_Fix();     // Make A = fix(A).
    cout << endl;
    result = A.PrintStdout();     // Print Matrix A. A = "[1 2 3; -4 -5 -6; 7 8 9]";
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Fix();

    /** \brief  Add a scaler double (ie: M += 5).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();        // Print Matrix A.
    result = A.Inplace_AddScalar(1); // A += 1.
    cout << endl;
    result = A.PrintStdout();        // Print Matrix A. A = "[2 3 4; 5 6 7; 8 9 10]";
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_AddScalar( const double scalar );

    /**
    \brief  Subtract a scaler double (ie: M -= 5).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();             // Print Matrix A.
    result = A.Inplace_SubtractScalar(1); // A -= 1.
    cout << endl;
    result = A.PrintStdout();             // Print Matrix A. A = "[0 1 2; 3 4 5; 6 7 8]";
    \endcode    
       
    \return true if successful, false otherwise.    
    */
    bool Inplace_SubtractScalar( const double scalar );

    /**
    \brief  Multiply by scaler double (ie: M *= 5).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();              // Print Matrix A.
    result = A.Inplace_MultiplyScalar(5);  // A *= 5.
    cout << endl;
    result = A.PrintStdout();              // Print Matrix A. A = "[5 10 15; 20 25 30; 35 40 45]";
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_MultiplyScalar( const double scalar );

    /**
    \brief  Divide by scaler double (ie: M /= 5).
    
    \code
    Matrix A;
    A = "[5 10 15; 20 25 30; 35 40 45]";
    bool result;
    result = A.PrintStdout();           // Print Matrix A.
    result = A.Inplace_DivideScalar(5); // A /= 5.
    cout << endl;
    result = A.PrintStdout();           // Print Matrix A. A = "[1 2 3; 4 5 6; 7 8 9]";
    \endcode    
       
    \return true if successful, false otherwise.    
    */
    bool Inplace_DivideScalar( const double scalar );

    /**
    \brief  Raise the matrix to a power scaler double (ie: M ^= 5).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();           // Print Matrix A.
    result = A.Inplace_PowerScalar(2);  // A = A.^2. Not A*A! Each element is raised.
    cout << endl;
    result = A.PrintStdout();           // Print Matrix A. A = "[1 4 9; 16 25 36; 49 64 81]";
    \endcode    
           
    \return true if successful, false otherwise.    
    */
    bool Inplace_PowerScalar( const double scalar );

    /**
    \brief  Add a scaler double (ie: M += (4+2i)).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();           // Print Matrix A.
    std::complex<double> cplx(4.0,2.0);
    result = A.Inplace_AddScalarComplex(cplx);  // A += (4+2i).
    cout << endl;
    result = A.PrintStdout();           // Print Matrix A. A = "[5+2i 6+2i 7+2i; 8+2i 9+2i 10+2i; 11+2i 12+2i 13+2i]";
    cout << "A(0,0) = " << A(0,0).real() << "+" << A(0,0).imag() << "i " << endl;
    \endcode    
               
    * \return true if successful, false otherwise.    
    */
    bool Inplace_AddScalarComplex( const std::complex<double> cplx );

    /**
    \brief  Subtract a scaler double (ie: M -= (5+2i)).
    
    \code
    Matrix A;
    A = "[1 2 3; 4 5 6; 7 8 9]";
    bool result;
    result = A.PrintStdout();           // Print Matrix A.
    std::complex<double> cplx(5.0,2.0);
    result = A.Inplace_SubtractScalarComplex(cplx);  // A -= (5+2i).
    cout << endl;
    result = A.PrintStdout();           // Print Matrix A. A = "[-4-2i -3-2i -2-2i; -1-2i 0-2i 1-2i; 2-2i 3-2i 4-2i]";
    cout << "A(0,0) = " << A(0,0).real() << "+" << A(0,0).imag() << "i " << endl;
    \endcode    
                   
    \return true if successful, false otherwise.    
    */
    bool Inplace_SubtractScalarComplex( const std::complex<double> cplx );

    /**
    \brief  Multiply by scaler double (ie: M *= (5+2i)).

    \code
    Matrix M;
    M = "[10 20]";
    std::complex<double> cplx(5,2);
    if( !M.Inplace_MultiplyScalarComplex(cplx) )
      return false;
    // M
    // 50+20i  100+40i
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_MultiplyScalarComplex( const std::complex<double> cplx );

    /**
    \brief  Divide by scaler double (ie: M /= (5+1i)).

    \code
    Matrix M;
    M = "[10+2i 20+4i]";
    std::complex<double> cplx(5,1);
    if( !M.Inplace_DivideScalarComplex(cplx) )
      return false;
    // M
    // 2  4
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_DivideScalarComplex( const std::complex<double> cplx );

    /**
    \brief  Raise the matrix to a power scaler double (ie: M ^= (5+2i)).

    \code
    Matrix M;
    M = "[2 3]";
    std::complex<double> cplx(5,2);
    if( !M.Inplace_PowerScalarComplex(cplx) )
      return false;
    // M
    // 5.87062319178566+31.4568876931598i    -142.459949032798+196.860770397691i
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_PowerScalarComplex( const std::complex<double> cplx );

    /**
    \brief  Compute the absolute value of each element in the matrix.

    \code
    Matrix A;
    A = "[-1 -2; -3 -4]";
    if( !A.Inplace_Abs() )
      return false;
    // A
    // 1 2
    // 3 4
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Abs();

    /**
    \brief  Compute the value^2 of each element in the matrix.

    \code
    Matrix A;
    A = "[1 2; -3 -4]";
    if( !A.Inplace_Sqr() )
      return false;
    // A
    // 1 4
    // 9 16
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Sqr();

    /**
    \brief  Computes the sqrt(value) of each element in the matrix.

    \code
    Matrix A;
    A = "[1 4; 9 16]";
    if( !A.Inplace_Sqrt() )
      return false;
    // A
    // 1 2
    // 3 4
    \endcode        
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Sqrt();

    /**
    \brief  Computes the exp(value) of each element in the matrix.

    \code
    Matrix A;
    A = "[1 2; 3 4]";
    if( !A.Inplace_Exp() )
      return false;
    // A ~
    //  2.71828  7.38905
    // 20.08553 54.59815
    \endcode        
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Exp();

    /**
    \brief  Computes the natural logarithm, ln(value) of each element in the matrix.

    \code
    Matrix A;
    A = "[2.71828  7.38905; 20.08553 54.59815]";    
    if( !A.Inplace_Ln() )
      return false;
    // A ~
    // 1 2
    // 3 4
    \endcode         
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Ln();

    /**
    \brief  Add +1.0 to all elements, e.g. M++.

    \code
    Matrix A;
    A = "[1 2; 3 4]";
    if( !A.Inplace_Increment() )
      return false;
    // A 
    // 2 3
    // 4 5
    \endcode        
    
    \return true if successful, false otherwise.   
    */
    bool Inplace_Increment();

    /**
    \brief  Subtract 1.0 from all elements, e.g. M--.

    \code
    Matrix A;
    A = "[1 2; 3 4]";
    if( !A.Inplace_Decrement() )
      return false;
    // A 
    // 0 1
    // 2 3
    \endcode            
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_Decrement();

    /**
    \brief  Add matrix B to this matrix inplace. A += B, inplace.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 3 4]";
    if( !A.Inplace_Add(B) )
      return false;
    // A 
    // 2 4
    // 6 8
    \endcode            
    
    \return true if successful, false otherwise.
    */
    bool Inplace_Add( const Matrix &B );

    /**
    \brief  Subtract matrix B from this matrix inplace. A -= B, inplace.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 3 4]";
    if( !A.Inplace_Subtract(B) )
      return false;
    // A 
    // 0 0
    // 0 0
    \endcode            
    
    \return true if successful, false otherwise.
    */
    bool Inplace_Subtract( const Matrix &B );

    /**
    \brief  Pre-Multiply this matrix by B. A = B*A, inplace.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 2 1]";
    if( !A.Inplace_PreMultiply(B) )
      return false;
    // A 
    // 7 10
    // 5 8
    \endcode  
    
    \return true if successful, false otherwise.
    */
    bool Inplace_PreMultiply( const Matrix &B );

    /**
    \brief  Pre-Multiply this matrix by tranpose(B). A = tranpose(B)*A, inplace.
    No transpose occurs and hence more efficient.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[5 6; 7 8]";
    if( !A.Inplace_TranposePreMultiply(B) )
      return false;
    // A 
    // 26 38
    // 30 44
    \endcode  
    
    \return true if successful, false otherwise.
    */
    bool Inplace_TranposePreMultiply( const Matrix &B );

    /**
    \brief  Post-Multiply this matrix by B. A = A*B, inplace.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 2 1]";
    if( !A.Inplace_PostMultiply(B) )
      return false;
    // A 
    // 5 4
    // 11 10
    \endcode      
    
    \return true if successful, false otherwise.  
    */
    bool Inplace_PostMultiply( const Matrix &B );

    /**
    \brief  Post-Multiply this matrix by transpose(B). A = A*transpose(B), inplace.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[5 6; 7 8]";
    if( !A.Inplace_PostMultiplyTranspose(B) )
      return false;
    // A 
    // 17 23
    // 39 53
    \endcode      
    
    \return true if successful, false otherwise.  
    */
    bool Inplace_PostMultiplyTranspose( const Matrix &B );

    /**
    \brief  Dot multiply A .*= B, inplace. A and B must have the same dimensions.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 2 1]";
    if( !A.Inplace_DotMultiply(B) )
      return false;
    // A 
    // 1 4
    // 6 4
    \endcode          
    
    \return true if successful, false otherwise.
    */
    bool Inplace_DotMultiply( const Matrix &B );

    /**
    \brief  Dot divide A ./= B, inplace. A and B must have the same dimensions.

    \code
    Matrix A;
    Matrix B;
    A = "[1 2; 3 4]";
    B = "[1 2; 2 1]";
    if( !A.Inplace_DotDivide(B) )
      return false;
    // A 
    // 1   1
    // 1.5 4
    \endcode          
    
    \return true if successful, false otherwise.
    */
    bool Inplace_DotDivide( const Matrix &B );

    /**
    \brief  Sorts each column of the matrix in ascending order.
            If complex, sorts based on magnitude.
    \code
    Matrix A;
    A = "[1;3;2;4;6;5;7]";
    if( !A.Inplace_SortAscending() )
      return false;
    // A
    // [1;2;3;4;5;6;7]
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool Inplace_SortAscending();

    /**
    \brief  Sorts each column of M in descending order.
            If complex, sorts based on magnitude.
    \code
    Matrix A;
    A = "[1;3;2;4;6;5;7]";
    if( !A.Inplace_SortDescending() )
      return false;
    // A
    // [7;6;5;4;3;2;1]
    \endcode    
    
    \return true if successful, false otherwise.
    */
    bool Inplace_SortDescending();

    /**
    \brief  Sorts a specific column in ascending order.
            If complex, sorts based on magnitude.
    \code
    Matrix A;
    A = "[0 1;0 3;0 2;0 4;0 6;0 5;0 7]";
    if( !A.Inplace_SortColumnAscending(1) )
      return false;
    // A
    // A = "[0 1;0 2;0 3;0 4;0 5;0 6;0 7]";
    \endcode
       
    \return true if successful, false otherwise.
    */
    bool Inplace_SortColumnAscending( const unsigned col );

    /**
    \brief  Sorts a specific column in descending order.
            If complex, sorts based on magnitude.
    \code
    Matrix A;
    A = "[0 1;0 3;0 2;0 4;0 6;0 5;0 7]";
    if( !A.Inplace_SortColumnDescending(1) )
      return false;
    // A
    // A = "[0 7;0 6;0 5;0 4;0 3;0 2;0 1]";
    \endcode       
    
    \return true if successful, false otherwise.
    */
    bool Inplace_SortColumnDescending( const unsigned col );

    /**
    \brief  Sorts a specific column in ascending order and fills a  
            column vector with the sorted index. The index vector will be resized 
            if needed. If complex, sorts based on magnitude.
    \code
    Matrix A;
    Matrix I;
    A = "[0 1;0 3;0 2;0 4;0 6;0 5;0 7]";
    if( !A.Inplace_SortColumnIndexed(1, I) )
      return false;
    // A = "[0 1;0 2;0 3;0 4;0 5;0 6;0 7]";
    // I = "[0;2;1;3;5;4;6]"
    \endcode    
    
    \return true if successful, false otherwise.
    */
    bool Inplace_SortColumnIndexed( const unsigned col, Matrix &Index );

    /**
    \brief  Sorts the entire matrix by a specific column.
            If complex, sorts based on magnitude.

    \code
    Matrix A;
    Matrix I;
    A = "[0 1;2 3;1 2;3 4;5 6;4 5;6 7]";
    if( !A.Inplace_SortByColumn(0) )
      return false;
    // A = "[0 1;1 2;2 3;3 4;4 5;5 6;6 7]";
    \endcode 
    
    \return true if successful, false otherwise.
    */
    bool Inplace_SortByColumn( const unsigned col );

    /**
    \brief  Computes the inplace inverse of the matrix.
    
    Uses fast closed form solutions for:
    1x1, 2x2, 3x3
    
    Otherwise, the matrix is first tested to determine if it is a symmetric 
    positive-definite matrix. If so, Cholesky decomposition is used
    to facilitate the inversion of a lower triangular matrix. If the
    matrix is not symmetric and positive-definite robust inversion
    using gaussing elimination is attempted.
    
    If the matrix is singular, the original matrix is unchanged.

    \code 
    Matrix A;
    A = "[10 14; 14 20]";
    if( !A.Inplace_Invert() )
      return false;
    // A
    //     5  -3.5
    //  -3.5   2.5
    \endcode
    
    \return true if successful, false if empty, singular or not square.
    */
    bool Inplace_Invert();

    /**
    \brief  Perfroms an inplace inverse using Gaussian Elimination methods.

    \code 
    Matrix A;
    A = "[1 2; 3 4]";
    if( !A.Inplace_InvertRobust() )
      return false;
    // A
    //   -2     1
    //  1.5  -0.5
    \endcode
    
    \return true if successful, false if empty, singular or not square.
    */
    bool Inplace_InvertRobust();

    /**
    \brief  Compute the inplace inverse of a unit lower triangular matrix. 
    
    \code
    Matrix A;
    // A
    //    1    0    0
    //   -2    2    0
    //    4   -3   -3    
    A = "[1 0 0; -2 2 0; 4 -3 -3]";
    if( !A.Inplace_LowerTriangularInverse() )
      return false;
    // A
    //    1    0    0
    //    1  1/2    0
    // -1/3  1/2  1/3
    \endcode
    
    \return true if successful, false if empty, singular or not square.
    */
    bool Inplace_LowerTriangularInverse();

    /**
    \brief  Compute the inplace Fourier Transform of each column of the matrix.

    \code
    Matrix A;
    A = "[0; 0; 0; 0; 1; 1; 1; 1;]"; 
    if( !A.Inplace_FFT() )
     return false;
    // A
    //  4                         
    // -1+2.41421356237309i
    //  0                         
    // -1+0.414213562373095i
    //  0                         
    // -1-0.414213562373095i
    //  0                         
    // -1-2.41421356237309i
    \endcode

    endcode

    \return   true if successful, false if unable to perform the FFT.
    */
    bool Inplace_FFT();


    /**
    \brief  Compute the inplace Two-Dimensional Fourier Transform of the matrix.
    FFT2 is equivalent to transpose( FFT( transpose( FFT(each column) ) ) )

    \code
    Matrix A;
    Matrix B;
    bool result;
    result = A.Inplace_colon(1.0,1.0,32.0);
    B = A*A.Transpose(); // (32x32 square matrix)
    result = B.Inplace_FFT2();
    \endcode

    endcode

    \return   true if successful, false if unable to perform the 2D FFT.
    */
    bool Inplace_FFT2();

    /**
    \brief  Compute the inplace inverse Fourier Transform of each column of the matrix.

    \code
    Matrix A;
    A = "[4; -1+2.41421356237309i; 0; -1+0.414213562373095i; 0; -1-0.414213562373095i; 0; -1-2.41421356237309i;]"; 
    if( !A.Inplace_IFFT() )
     return false;
    // A
    // 0                         
    // 0
    // 0                         
    // 0
    // 1                         
    // 1
    // 1                         
    // 1
    \endcode

    \return   true if successful, false if unable to perform the FFT.
    */
    bool Inplace_IFFT();


    /**
    \brief  Compute the inplace inverse Fourier Transform of the matrix.
    IFFT2 is equivalent to transpose( IFFT( transpose( IFFT(each column) ) ) )

    \return   true if successful, false if unable to perform the FFT.
    */
    bool Inplace_IFFT2();


  public: // Safe operations that set the matrix. Safe in that they return a boolean.

    /**
    \brief  Add A = B+C. The result, A, is stored in this matrix. 

    \code
    Matrix A;
    Matrix B;
    Matrix C;
    B = "[1 2; 3 4]";
    C = "[-1 2; -3 4]";
    if( !A.Add( B, C ) )
      return false;
    // A
    // 0 4
    // 0 8
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool Add( const Matrix &B, const Matrix &C );

    /**
    \brief  Subtract A = B-C. The result, A, is stored in this matrix. 
    
    \code
    Matrix A;
    Matrix B;
    Matrix C;
    B = "[1 2; 3 4]";
    C = "[-1 2; -3 4]";
    if( !A.Subtract( B, C ) )
      return false;
    // A
    // 2 0
    // 6 0
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool Subtract( const Matrix &B, const Matrix &C );

    /**
    \brief  Multiply A = B*C. The result, A, is stored in this matrix. 
    
    \code
    Matrix A;
    Matrix B;
    Matrix C;
    B = "[1 2; 3 4]";
    C = "[-1 2; -3 4]";
    if( !A.Multiply( B, C ) )
      return false;
    // A
    //  -7  10
    // -15  22
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool Multiply( const Matrix &B, const Matrix &C );


    /**
    \brief  Multiply A = transpose(B)*C. The result, A, is stored in this matrix. 
    
    \code
    Matrix A;
    Matrix B;
    Matrix C;
    B = "[1 2; 3 4]";
    C = "[-1 2; -3 4]";
    if( !A.TransposeMultiply( B, C ) )
      return false;
    // A
    // -10  14
    // -14  20
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool TransposeMultiply( const Matrix &B, const Matrix &C );


    /**
    \brief  Multiply A = B*transpose(C). The result, A, is stored in this matrix. 
    
    \code
    Matrix A;
    Matrix B;
    Matrix C;
    B = "[1 2; 3 4]";
    C = "[-1 2; -3 4]";
    if( !A.MultiplyTranspose( B, C ) )
      return false;
    // A
    // 3  5
    // 5  7
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool MultiplyTranspose( const Matrix &B, const Matrix &C );
    

  public: // Matlab/Octave style functions

    /**
    \brief  Compute the absolute value of each element of the matrix inplace.
    
    \code
    Matrix A;
    A = "[-1 2 3]";
    if( !A.Inplace_abs() )
      return false;
    // A 
    // [1 2 3]
    \endcode

    \return true if successful, false otherwise.    
    */
    bool Inplace_abs();

    /**
    \brief  Compute the arc-cosine of each element of the matrix inplace.
            Complex results are obtained if elements are greater than abs(1).
            Results in radians.
    \code
    Matrix A;
    A = "[0 0.5 1]";
    if( !A.Inplace_acos() )
      return false;
    // A 
    // [pi/2 pi/3 0]
    \endcode
    
    \return true if successful, false otherwise.   
    */
    bool Inplace_acos();

    /**
    \brief  Compute the arc-cosine of each element of the matrix inplace.
            Complex results are obtained if elements are greater than abs(1).
            Results in degrees.
    \code
    Matrix A;
    A = "[0 0.5 1]";
    if( !A.Inplace_acosd() )
      return false;
    // A 
    // [90 60 0]
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_acosd();

    /**
    \brief  Compute the inverse hyperbolic cosine of each element of the matrix inplace.
            Results in radians.
    \code
    Matrix A;
    A = "[0  1.0471975511966 1.5707963267949]";
    if( !A.Inplace_acosh() )
      return false;
    // A 
    // [0 pi/3 pi/2]
    \endcode        
    
    \return true if successful, false otherwise.   
    */
    bool Inplace_acosh();

    /**
    \brief  Compute the phase angle in radians of the elements of the matrix.

    \code
    Matrix A;
    A = "[1+1i  1-1i 3+2i]";
    if( !A.Inplace_acosh() )
      return false;
    // A 
    // [pi/4 -pi/4 0.588002603547568]
    \endcode    
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_angle();

    /**
    \brief  Compute the arc-sine of each element of the matrix inplace.
            Complex results are obtained if elements are greater than abs(1).
            Results in radians.
    \code
    Matrix A;
    A = "[0  0.5 1.0]";
    if( !A.Inplace_asin() )
      return false;
    // A 
    // [0 pi/6 pi/2]
    \endcode   
    
    \return true if successful, false otherwise.    
    */    
    bool Inplace_asin();

    /**
    \brief  Compute the arc-sine of each element of the matrix inplace.
            Complex results are obtained if elements are greater than abs(1).
            Results in degrees.
    \code
    Matrix A;
    A = "[0  0.5 1.0]";
    if( !A.Inplace_asind() )
      return false;
    // A 
    // [0 30 90]
    \endcode   
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_asind();

    /**
    \brief  Compute the inverse hyperbolic sine of each element of the matrix inplace.
            Results in radians.
    \code
    Matrix A;
    A = "[0  0.521095305493747  1.1752011936438]";
    if( !A.Inplace_asinh() )
      return false;
    // A 
    // [0 0.5 1]
    \endcode   
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_asinh();

    /**
    \brief  Compute the arc-tangent of each element of the matrix inplace.
            Results in radians bounded [-pi/2, pi/2].
    \code
    Matrix A;
    A = "[0  1.73205080756888  1.63312393531954e+016]";
    if( !A.Inplace_atan() )
      return false;
    // A 
    // [0 pi/3 pi/2]
    \endcode   
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_atan();

    /**
    \brief  Compute the arc-tangent of each element of the matrix inplace.
            Results in degrees bounded [-90, 90].
    \code
    Matrix A;
    A = "[0  1.73205080756888  1.63312393531954e+016]";
    if( !A.Inplace_atand() )
      return false;
    // A 
    // [0 60 90]
    \endcode   
        
    \return true if successful, false otherwise.    
    */
    bool Inplace_atand();

    /**
    \brief  Compute the inverse hyperbolic tangent of each element of the matrix inplace.

    \code
    Matrix A;
    A = "[0  0.46211715726001  0.761594155955765]";
    if( !A.Inplace_atanh() )
      return false;
    // A 
    // [0 0.5 1]
    \endcode   
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_atanh();

    /**
    \brief  Create a column vector [start:increment:end) beginning at start
    with step size of increment until less than or equal to end. 
    Note that arguments must be real scalars. \n

    \code
    Matrix A;
    if( !A.Inplace_colon( 2, 2, 9 ) )
      return false;
    // A
    // [2; 4; 6; 8]
    if( !A.Inplace_colon( 2, -2, -9 ) )
      return false;
    // A
    // [2; 0; -2; -4; -6; -9;]    
    if( !A.Inplace_colon( -10, 0.01, 10 ) )
      return false;
    // A
    // [-10 -9.99 -9.98 ... 10]    
    \endcode
    
    \return true if successful, false otherwise.     
    */
    bool Inplace_colon( double start, double increment, double end );

    /**
    \brief  Compute the cosine of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  1.0471975511966  1.5707963267949]"; // [0 pi/3 pi/2]
    if( !A.Inplace_cos() )
      return false;
    // A
    // 1 0.5 0
    \endcode 

    \return true if successful, false otherwise.        
    */
    bool Inplace_cos();

    /**
    \brief  Compute the hyperbolic cosine of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  0.5 1]";
    if( !A.Inplace_cosh() )
      return false;
    // A
    // 1  1.12762596520638  1.54308063481524
    \endcode 
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_cosh();

    /**
    \brief  Compute the cotangent of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  1.0471975511966  1.5707963267949]"; // [0  pi/3 pi/2]
    if( !A.Inplace_cot() )
      return false;
    // A
    // Inf  0.577350269189626  0
    \endcode 
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_cot();

    /**
    \brief  Compute the hyperbolic cotangent of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  0.5  1]";
    if( !A.Inplace_coth() )
      return false;
    // A
    // Inf   2.16395341373865 1.31303528549933
    \endcode 
    
    \return true if successful, false otherwise.        
    */
    bool Inplace_coth();

    /**
    \brief  Complex conjugate. z = x+yi. conj(z) = x-yi.

    \code
    Matrix A;
    A = "[2-2i -3+2i]";
    if( !A.Inplace_conj() )
      return false;
    // A
    // 2+2i  -3-2i
    \endcode

    \return true if successful, false otherwise.        
    */
    bool Inplace_conj();


    /**
    \brief  Compute the exponential of each element of the matrix inplace. 
            If real, computes the exp(value) of each element in the matrix.
            If complex, computes exp(M) = exp(real)*(cos(imag)+i*sin(imag)).
    \code
    Matrix A;
    A = "[1 2]";
    if( !A.Inplace_exp() )
      return false;
    // A
    //  2.71828182845905  7.38905609893065
    \endcode

    \return true if successful, false otherwise.        
    */
    bool Inplace_exp();

    /**
    \brief  Create an indentity matrix with nrows and ncols.
    
    \code
    Matrix A;
    if( !A.eye(3,3) )
      return false;
    // A
    // 1 0 0 
    // 0 1 0 
    // 0 0 1
    \endcode

    \return true if successful, false otherwise.        
    */
    bool Inplace_eye( const unsigned nrows, const unsigned ncols );


     /**
    \brief  Imaginary part of the complex matrix. z = x+yi. real(z) = y.

    \code
    Matrix A;
    A = "[2-2i -3+2i]";
    if( !A.Inplace_imag() )
      return false;
    // A
    // -2  2
    \endcode

    \return true if successful, false otherwise.        
    */
    bool Inplace_imag();


    /**
    \brief  Compute the log base 2 of the elements of the matrix.
            Complex results if elements are negative. 
    \code
    Matrix A;
    A = "[2 32]";
    if( !A.Inplace_log2() )
      return false;
    // A
    // 1 5
    \endcode
    
    \return true if successful, false otherwise.     
    */
    bool Inplace_log2();

    /**
    \brief  Compute the log base 10 of the elements of the matrix.
            Complex results if elements are negative. 
    \code
    Matrix A;
    A = "[10 1000]";
    if( !A.Inplace_log10() )
      return false;
    // A
    // 1 3
    \endcode
    
    \return true if successful, false otherwise.   
    */
    bool Inplace_log10();

    /**
    \brief  Create a matrix of nrows by ncols filled with 1.0.

    \code
    Matrix A;
    if( !A.Inplace_ones(2,3) )
      return false;
    // A
    // 1 1 1
    // 1 1 1
    \endcode
    
    \return true if successful, false otherwise.     
    */
    bool Inplace_ones( const unsigned nrows, const unsigned ncols );

    
    /**
    \brief  Produce a matrix that is composed of pseudo-random numbers.
    Values are elements are uniform distribution [0,1].

    \code
    Matrix A;
    if( !A.Inplace_rand(1000,1) ) // create a 1000x1 vector with uniform distribution [0,1]
      return false;
    \endcode
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_rand( const unsigned nrows, const unsigned ncols, const unsigned seed = rand() );

    /**
    \brief  Produce a matrix that is composed of pseudo-random numbers. 
    Values are elements are standard normal distribution with mean zero, 
    variance of one and standard of deviation one. N(0,1)

    \code
    Matrix A;
    if( !A.Inplace_randn(1000,1) ) // create a 1000x1 vector with standard normal distribution N[0,1]
      return false;
    \endcode
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_randn( const unsigned nrows, const unsigned ncols, const unsigned seed = rand() );

    /**
    \brief  Real part of the complex matrix. z = x+yi. real(z) = x.

    \code
    Matrix A;
    A = "[2-2i -3+2i]";
    if( !A.Inplace_real() )
      return false;
    // A
    // 2  3
    \endcode
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_real();

    /**
    \brief  Compute the sine of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0         0.523598775598299           1.5707963267949]"; //[0 pi/6 pi/2]
    if( !A.Inplace_sin() )
      return false;
    // A
    // 0 0.5 1
    \endcode
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_sin();

    /**
    \brief  Compute the sinc of each element*pi of the matrix inplace. 
    i.e. y = sin(pi*x)./(pi*x).

    \code
    Matrix A;
    A = "[0  0.523598775598299  1.5707963267949]"; //[0 pi/6 pi/2]
    if( !A.Inplace_sinc() )
      return false;
    // A
    // 1  0.606257160324575  -0.19765087483668
    \endcode
    
    \return true if successful, false otherwise.      
    */
    bool Inplace_sinc();

    /**
    \brief  Compute the hyperbolic sine of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0 0.5 1]";
    if( !A.Inplace_sinh() )
      return false;
    // A
    // 0  0.521095305493747  1.1752011936438
    \endcode
    
    \return true if successful, false otherwise.     
    */
    bool Inplace_sinh();

    /**
    \brief  Compute the sqrt of each element of the matrix inplace.

    \code
    Matrix A;
    A = "[0 9 121]";
    if( !A.Inplace_sqrt() )
      return false;
    // A
    // 0  3  11
    \endcode
    
    \return true if successful, false otherwise.       
    */
    bool Inplace_sqrt();

    /**
    \brief  Compute the tangent of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  0.785398163397448  1.5707963267949]"; // [0 pi/4 pi/2]
    if( !A.Inplace_tan() )
      return false;
    // A
    // 0  1  1.63312393531954e+016
    \endcode
    
    \return true if successful, false otherwise.    
    */
    bool Inplace_tan();

    /**
    \brief  Compute the hyperbolic tangent of each element of the matrix inplace. This 
            function assumes radian values in the matrix.
    \code
    Matrix A;
    A = "[0  0.785398163397448  1.5707963267949]"; // [0 pi/4 pi/2]
    if( !A.Inplace_tanh() )
      return false;
    // A
    // 0  0.655794202632672  0.917152335667274
    \endcode

    \return true if successful, false otherwise.      
    */
    bool Inplace_tanh();

    /**
    \brief  Create a matrix of nrows by ncols filled with 0.0.

    \code
    Matrix A;
    if( !A.Inplace_zeros(2,3) )
      return false;
    // A
    // 0 0 0
    // 0 0 0
    \endcode
    
    \return true if successful, false otherwise.        
    */
    bool Inplace_zeros( const unsigned nrows, const unsigned ncols );


  public: // Statistics

    /**
    \brief  Computes the value of the largest absolute element and its index.

    \code
    Matrix A;
    unsigned row;
    unsigned col;
    double value;
    A = "[1 2 3 4 5]";
    if( !A.GetStats_MaxAbs( row, col, value ) )
      return false;
    // row   == 0
    // col   == 4
    // value == 5
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxAbs(unsigned &row, unsigned &col, double &value );

    /**
    \brief  Computes the value (re+im*j) of the maximum element and its index.  
            When complex the maximum absolute value is determined.
    \code
    Matrix A;
    unsigned row;
    unsigned col;
    double re;
    double im;
    A = "[1 2 3 4 5-22i]";
    if( !A.GetStats_Max( row, col, re, im ) )
      return false;
    // row   == 0
    // col   == 4
    // re    == 5
    // im    == -22
    \endcode
    
    \return true if successful, false otherwise.
    */
    bool GetStats_Max(unsigned &row, unsigned &col, double &re, double &im );

    /**
    \brief  Computes the value (re+im*j) of the maximum element.  
            When complex the maximum absolute value is determined.
    \code
    Matrix A;
    double re;
    double im;
    A = "[1 2 3 4 5-22i]";
    if( !A.GetStats_MaxVal( re, im ) )
      return false;
    // re    == 5
    // im    == -22
    \endcode

    \return true if successful, false otherwise.
    */
    bool GetStats_MaxVal(double &re, double &im );

    /**
    \brief  Computes the value of the largest absolute column element and its row index.
    
    \code
    Matrix A;
    unsigned row;
    double value;
    A = "[1 2 3; 4 -5 6]";
    if( !A.GetStats_MaxAbsCol( 1, value, row ) )
      return false;
    // value == 5
    // row   == 1
    \endcode

    \return true if successful, false otherwise.
    */
    bool GetStats_MaxAbsCol(const unsigned col, double &value, unsigned &row );

    /**
    \brief  Computes the value (re+im*j) of the maximum column element and its row index.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxCol(const unsigned col, double &re, double &im, unsigned &row );

    /**
    \brief  Computes the value (re+im*j) of the maximum column element.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxColVal(const unsigned col, double &re, double &im );

    /**
    \brief  Computes the value of the largest absolute row element and its column index.
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxAbsRow(const unsigned row, double &value, unsigned &col );

    /**
    \brief  Computes the value (re+im*j) of the maximum row element and its column index.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxRow(const unsigned row, double &re, double &im, unsigned &col );

    /**
    \brief  Computes the value (re+im*j) of the maximum row element.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MaxRowVal(const unsigned row, double &re, double &im );

    /**
    \brief  Computes the value of the smallest absolute element and its index.
    \return true if successful, false otherwise.
    */
    bool GetStats_MinAbs(unsigned &row, unsigned &col, double &value );

    /**
    \brief  Computes the value (re+im*j) of the minimum element and its index.  
    \return true if successful, false otherwise.
    */
    bool GetStats_Min(unsigned &row, unsigned &col, double &re, double &im );

    /**
    \brief  Computes the value (re+im*j) of the minimum element.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MinVal(double &re, double &im );

    /**
    \brief  Computes the value of the smallest absolute column element and its row index.
    \return true if successful, false otherwise.
    */
    bool GetStats_MinAbsCol(const unsigned col, double &value, unsigned &row );

    /**
    \brief  Computes the value (re+im*j) of the minimum column element and its row index.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MinCol(const unsigned col, double &re, double &im, unsigned &row );

    /**
    \brief  Computes the value (re+im*j) of the minimum column element.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MinColVal(const unsigned col, double &re, double &im );

    /**
    \brief  Computes the value of the smallest absolute row element and its column index.
    \return true if successful, false otherwise.
    */
    bool GetStats_MinAbsRow(const unsigned row, double &value, unsigned &col );

    /**
    \brief  Computes the value (re+im*j) of the minimum row element and its column index.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MinRow(const unsigned row, double &re, double &im, unsigned &col );

    /**
    \brief  Computes the value (re+im*j) of the minimum row element.  
    \return true if successful, false otherwise.
    */
    bool GetStats_MinRowVal(const unsigned row, double &re, double &im );

    /**
    \brief  Computes the range of the data in the specified column. 
    Range = MaxVal - MinVal.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColRange( const unsigned col, double &re, double &im );

    /**
    \brief  Computes the range of the data in the specified row. 
    Range = MaxVal - MinVal.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowRange( const unsigned row, double &re, double &im );

    /**
    \brief  Computes the range of the data in the matrix. 
    Range = MaxVal - MinVal.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_Range( double &re, double &im );

    /**
    \brief  Computes the sum for the specified column.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnSum( const unsigned col,  double &re, double &im );

    /**
    \brief  Computes the sum for the specified row.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowSum( const unsigned row, double &re, double &im );

    /**
    \brief  Computes the sum for the matrix.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_Sum( double &re, double &im );

    /**
    \brief  Computes the sample mean for the specified column.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnMean( const unsigned col, double &re, double &im );

    /**
    \brief  Computes the sample mean for the specified row.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowMean( const unsigned row, double &re, double &im );

    /**
    \brief  Computes the sample mean for the matrix.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_Mean( double &re, double &im );

    /**
    \brief  Computes the sample standard deviation for the specified column.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnStdev( const unsigned col, double &value );

    /**
    \brief  Computes the sample standard deviation for the specified row.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowStdev( const unsigned row, double &value );

    /**
    \brief  Computes the sample standard deviation for the matrix.
    \return true if successful, false otherwise.
    */
    bool GetStats_Stdev( double &value );

    /**
    \brief  Computes the sample variance for the specified column.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnVar( const unsigned col, double &value );

    /**
    \brief  Computes the sample variance for the specified row.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowVar( const unsigned row, double &value );

    /**
    \brief  Computes the sample variance for the matrix.
    \return true if successful, false otherwise.
    */
    bool GetStats_Var( double &value );

    /**
    \brief  Computes the norm of the specified column.
    If real, norm = sqrt( sum( val*val ) ).
    If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnNorm( const unsigned col, double &value );

    /**
    \brief  Computes the norm of the specified row.
    If real, norm = sqrt( sum( val*val ) ).
    If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    \return true if successful, false otherwise.
    */
    bool GetStats_RowNorm( const unsigned row, double &value );

    /**
    \brief  Computes the norm of the matrix.
    If real, norm = sqrt( sum( val*val ) ).
    If complex, norm = sqrt( sum( val*conjugate(val) ) ).
    \return true if successful, false otherwise.
    */
    bool GetStats_Norm( double &value );

    /**
    \brief  Computes the sample RMS value for the specified column.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnRMS( const unsigned col, double &value );

    /**
    \brief  Computes the sample RMS value for the specified row.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowRMS( const unsigned row, double &value );

    /**
    \brief  Computes the sample RMS value for the matrix.
    \return true if successful, false otherwise.
    */
    bool GetStats_RMS( double &value );


    /**
    \brief  Computes the sample skewness value for the specified column.
    The skewness is the third central moment divided by the cube of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_ColumnSkewness( const unsigned col, double &re, double &im );

    /**
    \brief  Computes the sample skewness value for the specified row.
    The skewness is the third central moment divided by the cube of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_RowSkewness( const unsigned row, double &re, double &im );

    /**
    \brief  Computes the sample skewness value for the matrix.
    The skewness is the third central moment divided by the cube of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    \return true if successful, false otherwise.
    */
    bool GetStats_Skewness( double &re, double &im );



    /**
    \brief  Computes the sample kurtosis value for the specified column.
    The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    Reference: http://en.wikipedia.org/wiki/Kurtosis.
    Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    
    \return true if successful, false otherwise.
    */
    // g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    bool GetStats_ColumnKurtosis( const unsigned col, double &re, double &im );

    /**
    \brief  Computes the sample kurtosis value for the specified row.
    The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    Reference: http://en.wikipedia.org/wiki/Kurtosis.
    Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    
    \return true if successful, false otherwise.
    */
    // g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    bool GetStats_RowKurtosis( const unsigned row, double &re, double &im );

    /**
    \brief  Computes the sample kurtosis value for the matrix.
    The kurtosis is the fourth central moment divided by fourth power of the standard deviation.
    If the matrix is real, only the real value, re is set, im = 0. 
    If the matrix is complex, both re and im are set.
    To adjust the computed kurtosis value for bias, subtract 3 from the real component.
    Reference: http://en.wikipedia.org/wiki/Kurtosis.
    Reference: http://mathworld.wolfram.com/Kurtosis.html (kurtosis proper is computed).
    
    \return true if successful, false otherwise.
    */
    // g_2 = \frac{m_4}{m_{2}^2} = \frac{n\,\sum_{i=1}^n (x_i - \overline{x})^4}{\left(\sum_{i=1}^n (x_i - \overline{x})^2\right)^2}
    bool GetStats_Kurtosis( double &re, double &im );


  public: // Matrix Specific operations

    /**
    /// \brief  Computes the trace of M where M is a square matrix.
    /// Trace = Sum of diagonal elements.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    /// \return true if successful, false otherwise.
    */
    bool GetTrace( double &re, double &im );

    /**
    /// \brief  Computes the determinatnt of the square matrix M.
    /// If the matrix is real, only the real value, re is set, im = 0. 
    /// If the matrix is complex, both re and im are set.
    */
    bool GetDeterminant( double &re, double &im );



  public: // Safe operations that set a Matrix argument. Rather than Matrix as a return type.

    /**
    /// \brief  Sets the diagonal elements of the matrix into DiagonalVector as a column vector.
    /// \return true if successful, false otherwise.
    */
    bool GetDiagonal( Matrix& DiagonalVector );

    /**
    /// \brief  Computes a moving average using N lead samples and M lagging samples
    /// for the specified column and stores it in MovAvg.
    /// \return true if successful, false otherwise.  
    */
    bool GetColumnMovAvg( const unsigned col, const unsigned lead, const unsigned lag, Matrix &MovAvg );

    /**
    /// \brief  Computes a moving average using N lead samples and M lagging samples
    /// for the matrix and stores it in MovAvg.
    /// \return true if successful, false otherwise.  
    */
    bool GetMovAvg( const unsigned lead, const unsigned lag, Matrix &MovAvg );

    /**
    /// \brief  Computes: InvATA = inverse( transpose(A) * A ). Assumes this matrix is A.
    /// e.g. Matrix A; Matrix InvATA; A = ...; bool result = A.GetATAInverse( InvATA );
    /// \return true if successful, false otherwise.  
    */
    bool GetATAInverse( Matrix &InvATA );

    /**
    /// \brief  LU factorization.
    /// Performs a factorization to produce a unit lower triangular matrix, L, 
    /// an upper triangular matrix, U, and permutation matrix P so that
    /// P*X = L*U.
    /// P, L and U are copmuted correctly if IsFullRank is set to true.
    /// e.g. Matrix A; A = ...; bool isFullRank, Matrix L,U,P; bool result = A.GetLUFactorization( isFullRank, P, L, U );
    /// \return true if successful, false otherwise.  
    */
    bool GetLUFactorization( bool &isFullRank, Matrix &P, Matrix &L, Matrix &U );


    /**
    \brief  Lower x Diagonal x transpose(Lower): matrix factorization.
    This method avoids using square roots and can be used for any square, full rank, symmetrical matrix .

    \code
    Matrix LDLt = "[3 6;6 16]";
    Matrix L;
    Matrix d;
    bool result = LDLt.GetLDLt( L, d );
    // L == [1 0;2 1]
    // d == [3; 4]; i.e. D == [3 0;0 4]
    \endcode

    \return true if successful, false otherwise.  
    */
    bool GetLDLt( 
      Matrix& L,  //!< A unit lower triangular matrix.
      Matrix& d,  //!< The diagonal vector from the diagonal of the D matrix.
      bool checkSymmetric = true //!< Enforce a symmetry check. Runs faster if disabled.
      );

    /**
    \brief  Upper x Diagonal x transpose(Upper): matrix factorization.
    This method avoids using square roots and can be used for any square, full rank, symmetrical matrix .

    \code
    Matrix UDUt = "[19 8;8 4]";
    Matrix U;
    Matrix d;
    bool result = UDUt.GetUDUt( U, d );
    // U == [1 0;2 1]
    // d == [3; 4]; i.e. D == [3 0;0 4]
    \endcode

    \return true if successful, false otherwise.  
    */
    bool GetUDUt( 
      Matrix& U,  //!< A unit upper triangular matrix.
      Matrix& d,  //!< The diagonal vector from the diagonal of the D matrix.
      bool checkSymmetric = true //!< Enforce a symmetry check. Runs faster if disabled.
      );

    /**
    /// \brief  Retrieve the elements of the matrix specified by the index vectors. 
    /// The index vectors must be nx1 and preferably not complex.
    ///
    /// \return true if successful, false otherwise.
    */
    bool GetIndexedValues( Matrix& RowIndex, Matrix& ColIndex, Matrix& Result );


    /**
    /// \brief  Set the elements of the matrix specified by the index vectors. 
    /// The index vectors must be nx1 and preferably not complex.
    ///
    /// \return true if successful, false otherwise.
    */
    bool SetIndexedValues( Matrix& RowIndex, Matrix& ColIndex, Matrix& SourceData );


    /*
    /// \brief  
    */
    //bool Find_EqualTo( const double value, const double tolerance = 1e-12, Matrix &IndexMatrix );     

    /**
    /// get the indices that are not this value
    */
    //bool Find_NotEqualTo( const double value, const double tolerance = 1e-12 );     
   
    /**
    /// get the indices less than this value
    */
    //bool Find_LessThan( const double value );     
   
    /**
    /// get the indices less than this value
    */
    //bool Find_MoreThan( const double value );     
    

    

  public: // Advanced Functionality

    /**
    \brief  Plot one series, X vs Y. The i'th column (x-axis) vs 
            j'th column (y-axis) of the Matrix directly to a compressed 
            (run-length-encoded) bitamp.
    \code
    bool TryPlot()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix TS; // time | sin(time)
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      TS = T;
      result = TS.Concatonate( S );
      if( !result )
        return false;
      result = TS.Plot( 0, 1 ); // makes plot.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, "test1.bmp", "A Sinusoid", "time (s)", "voltage (V)", "sinusoid", "(V)" ); 
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,                //!< The column index (0toN-1) with the x series data (if this is the same as y_col, then the index is plotted as x).
      const unsigned y_col,                //!< The column index (0toN-1) with the y series data.      
      const std::string bmpfilename = "plot.bmp", //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",        //!< The plot title.
      const std::string xlabel = "",       //!< The x-axis label.
      const std::string ylabel = "",       //!< The y-axis label.
      const std::string series_label = "", //!< The series label.
      const std::string units = "",        //!< The series data units.
      const bool isXGridOn = true,         //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,         //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,      //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5,   //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8,   //!< The plot height in cm.
      const unsigned plot_width_cm = 10    //!< The plot width in cm.
      );

    /**
    \brief  Plot two series, X vs Y1, Y2 using columns of the Matrix.
            Plots directly to a compressed (run-length-encoded) bitamp.
    \code
    bool TryPlot2()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix C; // sin(time)
      Matrix F; // time | sin(time) | cos(time)
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      C = T;
      result = C.Inplace_cos();
      if( !result )
        return false;
      F = T;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2 ); // makes plot2.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, "test2.bmp", "Two Sinusoids", "time (s)", "voltage (V)", "sine", "(V)", "cosine", "(V)" );   
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,                //!< The column index (0toN-1) with the x series data.
      const unsigned y_col_1,              //!< The column index (0toN-1) with the y_1 series data.
      const unsigned y_col_2,              //!< The column index (0toN-1) with the y_2 series data.      
      const std::string bmpfilename = "plot2.bmp", //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",           //!< The plot title.
      const std::string xlabel = "",          //!< The x-axis label.
      const std::string ylabel = "",          //!< The y-axis label.
      const std::string series_label_1 = "",  //!< The series label.
      const std::string units_1 = "",         //!< The series data units.
      const std::string series_label_2 = "",  //!< The series label.
      const std::string units_2 = "",         //!< The series data units.
      const bool isXGridOn = true,       //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,       //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,    //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5, //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8, //!< The plot height in cm.
      const unsigned plot_width_cm = 10  //!< The plot width in cm.
      );

    /**
    \brief  Plot three series, X vs Y1, Y2, Y3 using columns of the Matrix.
            Plots directly to a compressed (run-length-encoded) bitamp.
    \code
    bool TryPlot3()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix C; // sin(time)
      Matrix Sinc; // sin(time)
      Matrix F; // time | sin(time) | cos(time) | sinc(time)
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      C = T;
      result = C.Inplace_cos();
      if( !result )
        return false;
      Sinc = T;
      result = Sinc.Inplace_sinc();
      if( !result )
        return false;
      F = T;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Concatonate( Sinc );
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3 ); // makes plot3.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, "plot3test.bmp", "sin cos sinc", "time (s)", "voltage (V)", "sine", "(V)", "cosine", "(V)", "sinc", "(V)" );   
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
      const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
      const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
      const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
      const std::string bmpfilename = "plot3.bmp",     //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",           //!< The plot title.
      const std::string xlabel = "",          //!< The x-axis label.
      const std::string ylabel = "",          //!< The y-axis label.
      const std::string series_label_1 = "",  //!< The series label.
      const std::string units_1 = "",         //!< The series data units.
      const std::string series_label_2 = "",  //!< The series label.
      const std::string units_2 = "",         //!< The series data units.
      const std::string series_label_3 = "",  //!< The series label.
      const std::string units_3 = "",         //!< The series data units.      
      const bool isXGridOn = true,       //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,       //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,    //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5, //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8, //!< The plot height in cm.
      const unsigned plot_width_cm = 10  //!< The plot width in cm.
      );

    /**
    \brief  Plot four series, X vs Y1, Y2, Y3 using columns of the Matrix.
            Plots directly to a compressed (run-length-encoded) bitamp.
    \code
    bool TryPlot3()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix C; // sin(time)
      Matrix Sinc; // sin(time)
      Matrix F; // time | sin(time) | cos(time) | sinc(time) | sin(time)+1
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      C = T;
      result = C.Inplace_cos();
      if( !result )
        return false;
      Sinc = T;
      result = Sinc.Inplace_sinc();
      if( !result )
        return false;
      F = T;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Concatonate( Sinc );
      if( !result )
        return false;
      S += 1.0;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4 ); // makes plot4.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4, "plot4test.bmp", "sin cos sinc sin+1", "time (s)", "voltage (V)", "sine", "(V)", "cosine", "(V)", "sinc", "(V)", "sin+1", "(V)" );   
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
      const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
      const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
      const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
      const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.
      const std::string bmpfilename = "plot4.bmp",     //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",           //!< The plot title.
      const std::string xlabel = "",          //!< The x-axis label.
      const std::string ylabel = "",          //!< The y-axis label.
      const std::string series_label_1 = "",  //!< The series label.
      const std::string units_1 = "",         //!< The series data units.
      const std::string series_label_2 = "",  //!< The series label.
      const std::string units_2 = "",         //!< The series data units.
      const std::string series_label_3 = "",  //!< The series label.
      const std::string units_3 = "",         //!< The series data units.      
      const std::string series_label_4 = "",  //!< The series label.
      const std::string units_4 = "",         //!< The series data units.            
      const bool isXGridOn = true,       //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,       //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,    //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5, //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8, //!< The plot height in cm.
      const unsigned plot_width_cm = 10  //!< The plot width in cm.
      );

    /**
    \brief  Plot five series, X vs Y1, Y2, Y3, Y4, Y5 using columns of the Matrix.
            Plots directly to a compressed (run-length-encoded) bitamp.
    \code
    bool TryPlot3()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix C; // sin(time)
      Matrix Sinc; // sin(time)
      Matrix F; // time | sin(time) | cos(time) | sinc(time) | sin(time)+1 | cos(time)-1
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      C = T;
      result = C.Inplace_cos();
      if( !result )
        return false;
      Sinc = T;
      result = Sinc.Inplace_sinc();
      if( !result )
        return false;
      F = T;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Concatonate( Sinc );
      if( !result )
        return false;
      S += 1.0;
      result = F.Concatonate( S );
      if( !result )
        return false;
      C -= 1.0;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4, 5 ); // makes plot5.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4, 5, "plot5test.bmp", "sin cos sinc sin+1 cos-1", "time (s)", "voltage (V)", "sine", "(V)", "cosine", "(V)", "sinc", "(V)", "sin+1", "(V)", "cos-1", "(V)" );
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
      const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
      const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
      const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
      const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.
      const unsigned y_col_5,            //!< The column index (0toN-1) with the y_5 series data.
      const std::string bmpfilename = "plot5.bmp",     //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",           //!< The plot title.
      const std::string xlabel = "",          //!< The x-axis label.
      const std::string ylabel = "",          //!< The y-axis label.
      const std::string series_label_1 = "",  //!< The series label.
      const std::string units_1 = "",         //!< The series data units.
      const std::string series_label_2 = "",  //!< The series label.
      const std::string units_2 = "",         //!< The series data units.
      const std::string series_label_3 = "",  //!< The series label.
      const std::string units_3 = "",         //!< The series data units.      
      const std::string series_label_4 = "",  //!< The series label.
      const std::string units_4 = "",         //!< The series data units.            
      const std::string series_label_5 = "",  //!< The series label.
      const std::string units_5 = "",         //!< The series data units.                  
      const bool isXGridOn = true,       //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,       //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,    //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5, //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8, //!< The plot height in cm.
      const unsigned plot_width_cm = 10  //!< The plot width in cm.
      );

    /**
    \brief  Plot six series, X vs Y1, Y2, Y3, Y4, Y5, Y6 using columns of the Matrix.
            Plots directly to a compressed (run-length-encoded) bitamp.
    \code
    bool TryPlot3()
    {
      bool result;
      Matrix T; // time
      Matrix S; // sin(time)
      Matrix C; // sin(time)
      Matrix Sinc; // sin(time)
      Matrix F; // time | sin(time) | cos(time) | sinc(time) | sin(time)+1 | cos(time)-1 | sinc^2(time)
      double pi = 3.1415926535897;
      result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
      if( !result )
        return false;
      S = T;
      result = S.Inplace_sin();
      if( !result )
        return false;
      C = T;
      result = C.Inplace_cos();
      if( !result )
        return false;
      Sinc = T;
      result = Sinc.Inplace_sinc();
      if( !result )
        return false;
      F = T;
      result = F.Concatonate( S );
      if( !result )
        return false;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = F.Concatonate( Sinc );
      if( !result )
        return false;
      S += 1.0;
      result = F.Concatonate( S );
      if( !result )
        return false;
      C -= 1.0;
      result = F.Concatonate( C );
      if( !result )
        return false;
      result = Sinc.Inplace_Sqr();
      if( !result )
        return false;
      result = F.Concatonate( Sinc );
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4, 5, 6 ); // makes plot6.bmp
      if( !result )
        return false;
      result = F.Plot( 0, 1, 2, 3, 4, 5, 6, "plot6test.bmp", "sin cos sinc sin+1 cos-1 sinc^2", "time (s)", "voltage (V)", "sine", "(V)", "cosine", "(V)", "sinc", "(V)", "sin+1", "(V)", "cos-1", "(V)", "sinc^2", "(V)" );
      if( !result )
        return false;
      return true;
    }
    \endcode
    \return true if successful, false otherwise.
    */
    bool Plot( 
      const unsigned x_col,              //!< The column index (0toN-1) with the x series data.
      const unsigned y_col_1,            //!< The column index (0toN-1) with the y_1 series data.
      const unsigned y_col_2,            //!< The column index (0toN-1) with the y_2 series data.
      const unsigned y_col_3,            //!< The column index (0toN-1) with the y_3 series data.      
      const unsigned y_col_4,            //!< The column index (0toN-1) with the y_4 series data.
      const unsigned y_col_5,            //!< The column index (0toN-1) with the y_5 series data.
      const unsigned y_col_6,            //!< The column index (0toN-1) with the y_5 series data.
      const std::string bmpfilename = "plot6.bmp",     //!< The file name (or full path name) of the output bitmap file.
      const std::string title = "",           //!< The plot title.
      const std::string xlabel = "",          //!< The x-axis label.
      const std::string ylabel = "",          //!< The y-axis label.
      const std::string series_label_1 = "",  //!< The series label.
      const std::string units_1 = "",         //!< The series data units.
      const std::string series_label_2 = "",  //!< The series label.
      const std::string units_2 = "",         //!< The series data units.
      const std::string series_label_3 = "",  //!< The series label.
      const std::string units_3 = "",         //!< The series data units.      
      const std::string series_label_4 = "",  //!< The series label.
      const std::string units_4 = "",         //!< The series data units.            
      const std::string series_label_5 = "",  //!< The series label.
      const std::string units_5 = "",         //!< The series data units.                  
      const std::string series_label_6 = "",  //!< The series label.
      const std::string units_6 = "",         //!< The series data units.                        
      const bool isXGridOn = true,       //!< A boolean to indicate if the x grid lines are on.
      const bool isYGridOn = true,       //!< A boolean to indicate if the y grid lines are on.  
      const bool includeStats = true,    //!< A boolean to indicate if statistics info should be included on the plot.  
      const unsigned precisionStats = 5, //!< The number of significant digits in the statistics.
      const unsigned plot_height_cm = 8, //!< The plot height in cm.
      const unsigned plot_width_cm = 10  //!< The plot width in cm.
      );


    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y,                         //!< The series must be [Nx1] or [1xN].
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
      );  

    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_1,    //!< The series label.
      const std::string units_1,           //!< The series units.
      Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
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
      );  

    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_1,    //!< The series label.
      const std::string units_1,           //!< The series units.
      Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_2,    //!< The series label.
      const std::string units_2,           //!< The series units.
      Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
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
      );

    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_1,    //!< The series label.
      const std::string units_1,           //!< The series units.
      Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_2,    //!< The series label.
      const std::string units_2,           //!< The series units.
      Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_3,    //!< The series label.
      const std::string units_3,           //!< The series units.
      Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
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
      );

    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_1,    //!< The series label.
      const std::string units_1,           //!< The series units.
      Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_2,    //!< The series label.
      const std::string units_2,           //!< The series units.
      Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_3,    //!< The series label.
      const std::string units_3,           //!< The series units.
      Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_4,    //!< The series label.
      const std::string units_4,           //!< The series units.
      Matrix &X_5,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_5,                         //!< The series must be [Nx1] or [1xN].
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
      );

    friend bool Plot(
      const std::string bmpfilename,     //!< The file name (or full path name) of the output bitmap file.
      const std::string title,           //!< The plot title.
      const std::string xlabel,          //!< The x-axis label.
      const std::string ylabel,          //!< The y-axis label.
      Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_1,    //!< The series label.
      const std::string units_1,           //!< The series units.
      Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_2,    //!< The series label.
      const std::string units_2,           //!< The series units.
      Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_3,    //!< The series label.
      const std::string units_3,           //!< The series units.
      Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_4,    //!< The series label.
      const std::string units_4,           //!< The series units.
      Matrix &X_5,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_5,                         //!< The series must be [Nx1] or [1xN].
      const std::string series_label_5,    //!< The series label.
      const std::string units_5,           //!< The series units.
      Matrix &X_6,                         //!< The series must be [Nx1] or [1xN].
      Matrix &Y_6,                         //!< The series must be [Nx1] or [1xN].
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
      );



    /**
    \brief  Retrieve the matrix comment string. The string
    will be empty if none is available. The matrix comment string
    is often the header line read when using ReadFromFile(). \n
    e.g. file.txt has:
    time(s)   x(m)   y(m)
    1.0       20.0   30.0
    
    \code
    bool result;
    Matrix A;
    result = A.ReadFromFile("file.txt");
    // A == [1.0 20.0 30.0]
    std::string comment = A.GetMatrixComment();
    // comment == "time(s)   x(m)   y(m)"
    \endcode
    
    \return The matrix comment string.
    */
    std::string GetMatrixComment();


    /**
    \brief  Alter the matrix so that its data is within the startTime to the startTime+duration
            and compensate for any rollovers in the time system (e.g. GPS time in seconds rolls over
            at 604800.0 s). This function assumes that time is one of the matrix columns and requires
            this index, the timeColumn.

    \return true if successful, false otherwise.
    */
    bool TimeWindow( 
      const unsigned timeColumn, //!< The column containing time.
      const double startTime,    //!< The specified start time (inclusive).
      const double duration,     //!< The duration to include.
      const double rolloverTime  //!< The potential time at which system time rolls over.
      );

    /**
    \brief  Alter the matrix so that its data is within [startTime endTime].
            This function assumes that time is one of the matrix columns and requires
            this index, the timeColumn.

    \return true if successful, false otherwise.
    */
    bool TimeLimit( 
      const unsigned timeColumn, //!< The column containing time
      const double startTime,    //!< The specified start time (inclusive)
      const double endTime       //!< The duration to include
      );

    /**
    \brief  This static function matches matrices in time with specified precision
    where time is a column of each matrix. This function also
    allows time to rollover at a specified interval.
    
    precision 0 = match to whole number \n
    precision 1 = match to nearest 0.1 \n
    precision 2 = match to nearest 0.01 \n
    etc. \n

    rolloverTime examples \n
    GPS time of week (s): rolloverTime= 604800.0 \n
    hours               : rolloverTime = 24.0 \n
    minutes             : rolloverTime = 60.0 \n
    
    The time data must be non-decreasing but the time may rollover
    by the specified amount. 
    e.g. rolloverTime = 60.0 \n
         0,1,2,3,4,...59,60,1,2,5,10,60,1,2,3... \n
    
    This function may be called by: bool result = Matrix::TimeMatch( ... );

    \return true if successful, false otherwise.
    */
    static bool TimeMatch( 
      Matrix &A,                   //!< The matrix with interpolation times
      const unsigned timeColumnA,  //!< The zero based column index for matrix A
      Matrix &B,                   //!< The matrix to be interpolated
      const unsigned timeColumnB,  //!< The zero based column index for matrix B
      const unsigned precision,    //!< The rounding precision used for time matching, 0 = whole, 1 = 0.1, 2 = 0.01, etc
      const double rolloverTime    //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
      );

    /**
    \brief  This static function interpolates Matrix B values by the times defined 
            in the column in Matrix A. Time must be increasing but times can 
            rollover with the specified rolloverTime.
    
    This function returns A and B with the same number of rows and 
    time aligned time columns.
    
    This function may be called by: bool result = Matrix::Interpolate( ... );

    \return true if successful, false otherwise.
    */
    static bool Interpolate( 
      Matrix &A,                    //!< The matrix with interpolation times
      const unsigned timeColumnA,   //!< The zero based column index for matrix A
      Matrix &B,                    //!< The matrix to be interpolated
      const unsigned timeColumnB,   //!< The zero based column index for matrix B
      const double maxInterpolationInterval, //!< The largest interpolation interval allowed
      const double rolloverTime     //!< The rollover time, e.g. 60 s for minute based timing, 0.0 means rollovers not allowed
      );


  public: // Functions that return a Matrix

    /**
    \brief  Return the column matrix specified by the column index. Returns (nrows x 1).

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.Column(1);
    // B == [2; 5; 8]
    \endcode
    */
    Matrix  Column(const unsigned col);

    /**
    \brief  Return the row matrix specified by the column index. Returns (ncols x 1).

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.Row(1);
    // B == [4 5 6]
    \endcode    
    */
    Matrix  Row(const unsigned row);

    /**
    \brief  Return the tranpose of the matrix.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.Transpose();
    // B == "[1 4 7; 2 5 8; 3 6 9]";
    \endcode        
    */
    Matrix  Transpose();

    /**
    \brief  Return the tranpose of the matrix.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.T();
    // B == "[1 4 7; 2 5 8; 3 6 9]";
    \endcode 
    */
    Matrix  T(); // short version

    /**
    \brief  Return the diagonal of the matrix as a vector.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.Diagonal();
    // B == "[1; 5; 9]";
    \endcode     
    */
    Matrix  Diagonal();

    /**
    \brief  Return the inverse of the matrix.

    \code
    Matrix A = "[1 0 1; -2 1 3; 4 -1 -6]";
    Matrix B = A.Inverse();
    // B == "[0.6 0.2 0.2; 0 2 1; 0.4 -0.2 -0.2]";
    \endcode
    */
    Matrix  Inverse();
    
    /**
    \brief  Return the inverse of the matrix.

    \code
    Matrix A = "[1 0 1; -2 1 3; 4 -1 -6]";
    Matrix B = A.Inv();
    // B == "[0.6 0.2 0.2; 0 2 1; 0.4 -0.2 -0.2]";
    \endcode    
    */
    Matrix  Inv(); // short version


    /**
    \brief  Return the Fourier Transform of each column of the matrix. 
            Power of two uses FFT, otherwise fast DFT.
    */
    Matrix  FFT();

    /**
    \brief  Return the inverse Fourier Transform of each column of the matrix. 
            Power of two uses IFFT, otherwise fast IDFT.
    */
    Matrix  IFFT();


    /**
    \brief  Return the Two Dimensional Fourier Transform of the matrix.
    */
    Matrix  FFT2();

    /**
    \brief  Return the Two Dimensional Inverse Fourier Transform of the matrix.
    */
    Matrix  IFFT2();

    /**
    \brief  Return the real part of the matrix            

    \code
    Matrix A = "[1-1i 2-2i 3-3i; 4-4i 5-5i 6-6i; 7-7i 8-8i 9-9i]";
    Matrix B = A.Real();
    // B == "[1 2 3; 4 5 6; 7 8 9]";
    \endcode        
    */
    Matrix  Real();

    /**
    \brief  Return the imag part of the matrix            

    \code
    Matrix A = "[1-1i 2-2i 3-3i; 4-4i 5-5i 6-6i; 7-7i 8-8i 9-9i]";
    Matrix B = A.Imag();
    // B == "[-1 -2 -3; -4 -5 -6; -7 -8 -9]";
    \endcode            
    */
    Matrix  Imag();

    /**
    \brief  Return the complex conjugate of the matrix            

    \code
    Matrix A = "[1-1i 2-2i 3-3i; 4-4i 5-5i 6-6i; 7-7i 8-8i 9-9i]";
    Matrix B = A.conj();
    // B == "[1+1i 2+2i 3+3i; 4+4i 5+5i 6+6i; 7+7i 8+8i 9+9i]";
    \endcode    
    */
    Matrix  conj();


    /** 
    \brief  Returns the matrix plus Identity.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.AddIdentity();
    // B == "[2 2 3; 4 6 6; 7 8 10]";
    \endcode    
    */
    Matrix AddIdentity();

    /** 
    \brief  Returns the matrix minus Identity.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.MinusIdentity();
    // B == "[0 2 3; 4 4 6; 7 8 8]";
    \endcode    
    */
    Matrix MinusIdentity();

    /** 
    \brief  Returns Identity minus the matrix.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.IdentityMinusMe();
    // B == "[0 -2 -3; -4 -4 -6; -7 -8 -8]";
    \endcode    
    */
    Matrix IdentityMinusMe();

    /** 
    \brief  Returns the matrix * -1. This is more efficient than A *= -1.

    \code
    Matrix A = "[1 2 3; 4 5 6; 7 8 9]";
    Matrix B = A.Negate();
    // B == "[-1 -2 -3; -4 -5 -6; -7 -8 -9]";
    \endcode    
    */
    Matrix Negate();

    /** 
    \brief  Sets the matrix as the NxN hilbert matrix. H_ij = 1.0 / (i+j-1.0) for i=1:N, j=1:N.

    \code
    Matrix H;
    bool result;
    result = H.Hilbert(3);
    // H == "[1 1/2 1/3; 1/2 1/3 1/4; 1/3 1/4 1/5]";
    \endcode    
    */
    bool Hilbert( const unsigned N );

    /**
    \brief  Return the square root of each element in the matrix.

    \code
    Matrix A = "[-1 4 9;16 25 36;49 64 81]";
    Matrix B = A.Sqrt();
    // B == "[0+1i 2 3;4 5 6; 7 8 9]";
    \endcode        
    */
    Matrix  Sqrt();

    /**
    \brief  Return the exponent of each element in the matrix.
    */
    Matrix  Exp();

    /**
    \brief  Return the logarithm of each element in the matrix.
    */
    Matrix  Ln();

    /**
    \brief  Return the cosine of each element in the matrix.
    */
    Matrix  cos();

    /**
    \brief  Return the arc-cosine of each element in the matrix.
    */
    Matrix  acos();

    /**
    \brief  Return the sine of each element in the matrix.
    */
    Matrix  sin();

    /**
    \brief  Return the arc-sine of each element in the matrix.
    */
    Matrix  asin();

    /**
    \brief  Return the tangent of each element in the matrix.
    */
    Matrix  tan();

    /**
    \brief  Return the arc-tangent of each element in the matrix.
    */
    Matrix  atan();

    /**
    \brief  Return the hyperbolic cosine of each element in the matrix.
    */
    Matrix  cosh();

    /**
    \brief  Return the inverse hyperbolic cosine of each element in the matrix.
    */
    Matrix  acosh();

    /**
    \brief  Return the hyperbolic sine of each element in the matrix.
    */
    Matrix  sinh();

    /**
    \brief  Return the inverse hyperbolic sine of each element in the matrix.
    */
    Matrix  asinh();

    /**
    \brief  Return the hyperbolic tangent of each element in the matrix.
    */
    Matrix  tanh();

    /**
    \brief  Return the inverse hyperbolic tangent of each element in the matrix.
    */
    Matrix  atanh();

    /**
    \brief  Return the cotangent of each element in the matrix.
    */
    Matrix  cot();

    /**
    \brief  Return the hyperbolic cotangent of each element in the matrix.
    */
    Matrix  coth();

    /**
    \brief  Return the absolute value (magnitude if complex) of each element in the matrix.
    */
    Matrix  abs();

    /**
    \brief  Return the phase angle in radians of the elements in the matrix.
    If M is a real matrix, Phase is a zero matrix.
    If M is a complex matrix, Phase is a real matrix = atan2(im,re).
    */
    Matrix  angle();

    /**
    \brief  Return a matrix with all elements in raised to the power X^(power_re + power_im*i).
    */
    Matrix  pow(const double power_re, const double power_im = 0.0);

    /**
    \brief  Return a matrix with elements rounded to the specified precision.\n
    e.g. precision = 0    1.8    -> 2     \n
    e.g. precision = 1,   1.45   -> 1.5   \n
    e.g. precision = 2    1.456  -> 1.46  \n
    e.g. precision = 3,   1.4566 -> 1.457 \n
    precision has a maximum of 32. After which no rounding occurs.
    */
    Matrix  round(const unsigned precision);

    /**
    \brief  Return a matrix with elements rounded to the nearest integers towards minus infinity.
    */
    Matrix  floor();

    /**
    \brief  Return a matrix with elements rounded to the nearest integers towards infinity.
    */
    Matrix  ceil();

    /**
    \brief  Return a matrix with elements rounded to the nearest integers towards zero.
    */
    Matrix  fix();

    /**
    \brief  Return a matrix with all elements inverted (1/x).
    */
    Matrix  dotInvert();

    /**
    \brief  Return a matrix with each element subtracted from 1.0. i.e. 1-X.
    */
    Matrix  oneMinusMe();

    /**
    \brief  Return the matrix that has each element multiplied by each element of B.
    This matrix must be the same dimensions as B unless B is a scalar.
    */
    Matrix  DotMultiply(const Matrix& B);


  public:

    /**
    \brief  This is a nested class that is an element of the matrix. i.e. Matrix M; M(i,j) is the element. 
            It is used for operator(,) access by the Matrix.
    */
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



    /**
  \brief  Plot a single X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */
  bool Plot(
    const std::string bmpfilename,        //!< The file name (or full path name) of the output bitmap file.
    const std::string title,              //!< The plot title.
    const std::string xlabel,             //!< The x-axis label.
    const std::string ylabel,             //!< The y-axis label.
    Matrix &X,                            //!< The series must be [Nx1] or [1xN].
    Matrix &Y,                            //!< The series must be [Nx1] or [1xN].
    const std::string series_label,       //!< The series label.
    const std::string units,              //!< The series units.
    const bool isConnected = true,        //!< Are the data points connected.
    const MTX_enumColor color = MTX_BLUE, //!< The color of the data points/line.
    const bool isXGridOn = true,          //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,          //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,       //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,    //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,    //!< The plot height in cm.
    const unsigned plot_width_cm = 10     //!< The plot width in cm.
    );

  /**
  \brief  Plot two X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot2()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)", T, S+1.0, "sine+1", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */  
  bool Plot(
    const std::string bmpfilename,          //!< The file name (or full path name) of the output bitmap file.
    const std::string title,                //!< The plot title.
    const std::string xlabel,               //!< The x-axis label.
    const std::string ylabel,               //!< The y-axis label.
    Matrix &X_1,                            //!< The series must be [Nx1] or [1xN].
    Matrix &Y_1,                            //!< The series must be [Nx1] or [1xN].
    const std::string series_label_1,       //!< The series label.
    const std::string units_1,              //!< The series units.
    Matrix &X_2,                            //!< The series must be [Nx1] or [1xN].
    Matrix &Y_2,                            //!< The series must be [Nx1] or [1xN].
    const std::string series_label_2,       //!< The series label.
    const std::string units_2,              //!< The series units.
    const bool isConnected_1 = true,        //!< Are the data points connected.
    const MTX_enumColor color_1 = MTX_BLUE, //!< The color of the data points/line.
    const bool isConnected_2 = true,        //!< Are the data points connected.
    const MTX_enumColor color_2 = MTX_LIMEGREEN,  //!< The color of the data points/line.    
    const bool isXGridOn = true,            //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,            //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,         //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,      //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,      //!< The plot height in cm.
    const unsigned plot_width_cm = 10       //!< The plot width in cm.
    );  

  /**
  \brief  Plot three X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot3()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)", T, S+1.0, "sine+1", "(V)", T, S+2.0, "sine+2", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */  
  bool Plot(
    const std::string bmpfilename,       //!< The file name (or full path name) of the output bitmap file.
    const std::string title,             //!< The plot title.
    const std::string xlabel,            //!< The x-axis label.
    const std::string ylabel,            //!< The y-axis label.
    Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    const bool isConnected_1 = true,     //!< Are the data points connected.
    const MTX_enumColor color_1 = MTX_BLUE,  //!< The color of the data points/line.
    const bool isConnected_2 = true,     //!< Are the data points connected.
    const MTX_enumColor color_2 = MTX_LIMEGREEN,  //!< The color of the data points/line.    
    const bool isConnected_3 = true,     //!< Are the data points connected.
    const MTX_enumColor color_3 = MTX_RED,  //!< The color of the data points/line.        
    const bool isXGridOn = true,         //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,         //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,      //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,   //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,   //!< The plot height in cm.
    const unsigned plot_width_cm = 10    //!< The plot width in cm.
    );

  /**
  \brief  Plot four X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot4()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)", T, S+1.0, "sine+1", "(V)", T, S+2.0, "sine+2", "(V)", T, S+3.0, "sine+3", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */  
  bool Plot(
    const std::string bmpfilename,       //!< The file name (or full path name) of the output bitmap file.
    const std::string title,             //!< The plot title.
    const std::string xlabel,            //!< The x-axis label.
    const std::string ylabel,            //!< The y-axis label.
    Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    const bool isConnected_1 = true,     //!< Are the data points connected.
    const MTX_enumColor color_1 = MTX_BLUE,  //!< The color of the data points/line.
    const bool isConnected_2 = true,     //!< Are the data points connected.
    const MTX_enumColor color_2 = MTX_LIMEGREEN,  //!< The color of the data points/line.    
    const bool isConnected_3 = true,     //!< Are the data points connected.
    const MTX_enumColor color_3 = MTX_RED,  //!< The color of the data points/line.        
    const bool isConnected_4 = true,     //!< Are the data points connected.
    const MTX_enumColor color_4 = MTX_PURPLE,  //!< The color of the data points/line.            
    const bool isXGridOn = true,         //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,         //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,      //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,   //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,   //!< The plot height in cm.
    const unsigned plot_width_cm = 10    //!< The plot width in cm.
    );

  /**
  \brief  Plot five X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot5()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)", T, S+1.0, "sine+1", "(V)", T, S+2.0, "sine+2", "(V)", T, S+3.0, "sine+3", "(V)", T, S+4.0, "sine+4", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */  
  bool Plot(
    const std::string bmpfilename,       //!< The file name (or full path name) of the output bitmap file.
    const std::string title,             //!< The plot title.
    const std::string xlabel,            //!< The x-axis label.
    const std::string ylabel,            //!< The y-axis label.
    Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    Matrix &X_5,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_5,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_5,    //!< The series label.
    const std::string units_5,           //!< The series units.
    const bool isConnected_1 = true,     //!< Are the data points connected.
    const MTX_enumColor color_1 = MTX_BLUE,  //!< The color of the data points/line.
    const bool isConnected_2 = true,     //!< Are the data points connected.
    const MTX_enumColor color_2 = MTX_LIMEGREEN,  //!< The color of the data points/line.    
    const bool isConnected_3 = true,     //!< Are the data points connected.
    const MTX_enumColor color_3 = MTX_RED,  //!< The color of the data points/line.        
    const bool isConnected_4 = true,     //!< Are the data points connected.
    const MTX_enumColor color_4 = MTX_PURPLE,  //!< The color of the data points/line.        
    const bool isConnected_5 = true,     //!< Are the data points connected.
    const MTX_enumColor color_5 = MTX_ORANGE,  //!< The color of the data points/line.        
    const bool isXGridOn = true,         //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,         //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,      //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,   //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,   //!< The plot height in cm.
    const unsigned plot_width_cm = 10    //!< The plot width in cm.
    );

  /**
  \brief  Plot six X vs Y series directly to a compressed (run-length-encoded) bitmap file.  
  
  \code
  bool TryPlot6()
  {
    Matrix T;
    Matrix S;
    bool result;
    double pi = 3.1415926535897;
    result = T.Inplace_colon( -2*pi, 0.01, 2*pi );
    if( !result )
      return false;
    S = T;
    result = S.Inplace_sin();  
    if( !result )
      return false;
    result = Plot( "sine.bmp", "Testing Plot", "time (s)", "voltage (V)", T, S, "sine", "(V)", T, S+1.0, "sine+1", "(V)", T, S+2.0, "sine+2", "(V)", T, S+3.0, "sine+3", "(V)", T, S+4.0, "sine+4", "(V)", T, S+5.0, "sine+5", "(V)" );
    if( !result )
      return false;
    return true;
  }
  \endcode

  \return true if successful, false otherwise.
  */  
  bool Plot(
    const std::string bmpfilename,       //!< The file name (or full path name) of the output bitmap file.
    const std::string title,             //!< The plot title.
    const std::string xlabel,            //!< The x-axis label.
    const std::string ylabel,            //!< The y-axis label.
    Matrix &X_1,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_1,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_1,    //!< The series label.
    const std::string units_1,           //!< The series units.
    Matrix &X_2,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_2,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_2,    //!< The series label.
    const std::string units_2,           //!< The series units.
    Matrix &X_3,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_3,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_3,    //!< The series label.
    const std::string units_3,           //!< The series units.
    Matrix &X_4,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_4,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_4,    //!< The series label.
    const std::string units_4,           //!< The series units.
    Matrix &X_5,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_5,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_5,    //!< The series label.
    const std::string units_5,           //!< The series units.
    Matrix &X_6,                         //!< The series must be [Nx1] or [1xN].
    Matrix &Y_6,                         //!< The series must be [Nx1] or [1xN].
    const std::string series_label_6,    //!< The series label.
    const std::string units_6,           //!< The series units.
    const bool isConnected_1 = true,     //!< Are the data points connected.
    const MTX_enumColor color_1 = MTX_BLUE,  //!< The color of the data points/line.
    const bool isConnected_2 = true,     //!< Are the data points connected.
    const MTX_enumColor color_2 = MTX_LIMEGREEN,  //!< The color of the data points/line.    
    const bool isConnected_3 = true,     //!< Are the data points connected.
    const MTX_enumColor color_3 = MTX_RED,  //!< The color of the data points/line.        
    const bool isConnected_4 = true,     //!< Are the data points connected.
    const MTX_enumColor color_4 = MTX_PURPLE,  //!< The color of the data points/line.        
    const bool isConnected_5 = true,     //!< Are the data points connected.
    const MTX_enumColor color_5 = MTX_ORANGE,  //!< The color of the data points/line.        
    const bool isConnected_6 = true,     //!< Are the data points connected.
    const MTX_enumColor color_6 = MTX_GREEN,  //!< The color of the data points/line.        
    const bool isXGridOn = true,         //!< A boolean to indicate if the x grid lines are on.
    const bool isYGridOn = true,         //!< A boolean to indicate if the y grid lines are on.  
    const bool includeStats = true,      //!< A boolean to indicate if statistics info should be included on the plot.  
    const unsigned precisionStats = 5,   //!< The number of significant digits in the statistics.
    const unsigned plot_height_cm = 8,   //!< The plot height in cm.
    const unsigned plot_width_cm = 10    //!< The plot width in cm.
    );

} // end namespace Zenautics

#endif // _ZENAUTICS_MATRIX_H_


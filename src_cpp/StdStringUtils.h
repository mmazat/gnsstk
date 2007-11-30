/**
\file     StdStringUtils.h
\brief    Additional functions for std::string.

Useful string functions and emulating some function calls 
used by MFC CString. 

[1]. Microsoft CString MSDN page
http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vcmfc98/html/_mfc_cstring_class_members.asp    

\author   Glenn D. MacGougan (GDM)
\since    2006-12-20
\date     2007-01-09

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

#ifndef _STDSTRINGUTILES_H_ 
#define _STDSTRINGUTILES_H_ 

#include <string>

namespace StdStringUtils
{
  /// \brief    Extract the string from the source string until one of the 
  ///           characters in charSet is found.
  ///
  /// \code 
  /// std::string str( "$test,10,20,30*44" );
  /// std::string dst;
  /// bool result;
  /// result = StdStringUtils::SpanExcluding( str, ",*", dst );
  /// ASSERT( dst == "$test" );
  /// \endcode
  ///
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  void SpanExcluding(const std::string &src, const std::string charSet, std::string &dst);    

  /// \brief    Extract the string from the source string until one of the 
  ///           characters in charSet is found. Determine the string position of the 
  ///           end of the string extracted. ie. To aid in the start position of the 
  ///           next search.
  ///           
  ///
  /// \code 
  /// std::string str( "$test,10,20,30*44" );
  /// std::string dst;
  /// std::string::size_type p;
  /// bool result;
  /// result = StdStringUtils::SpanExcluding( str, ",*", dst, p );
  /// ASSERT( dst == "$test" );
  /// ASSERT( p == 5 );
  /// \endcode
  ///
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  void SpanExcluding(const std::string &src, const std::string charSet, std::string &dst, std::string::size_type& pos);

  void SpanNotExcluding(const std::string &src, const std::string charSet, std::string &dst, std::string::size_type& pos);

  /// \brief    Conversion to uppercase.
  /// \author   GDM
  void MakeUpper(std::string &str);

  /// \brief    Conversion to lowercase
  /// \author   GDM
  void MakeLower(std::string &str);

  /// \brief    Remove whitespace starting from right edge.
  /// \author   GDM
  void TrimRight(std::string &str);

  /// \brief    Remove whitespace starting from left side.    
  /// \author   GDM
  void TrimLeft(std::string &str);

  /// \brief    Remove whitespace from the left and right.
  /// \author   GDM
  /// \return   A copy of the trimmed string.
  void TrimLeftAndRight(std::string &str);

  /// \brief    Get a double from the string.
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  bool GetDouble( const std::string &src, double &dValue );

  /// \brief    Get a float from the string.
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  bool GetFloat( const std::string &src, float &fValue );

  /// \brief    Get an int from the string.
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  bool GetInt( const std::string &src, int &iValue );

  /// \brief    Get an unsigned from the string.
  /// \author   GDM
  /// \return   true if successful, false otherwise.
  bool GetUnsignedInt( const std::string &src, unsigned &uiValue );


  /// \brief    Extract a substring from a string delimited by a specific delimiter
  ///           using a 0-based index.
  ///
  /// e.g.
  /// \code
  /// std::string A = "$abcde,11,22,33";
  /// std::string B;
  /// ExtractField( A, 0, ',', B ); // B == "$abcde" 
  /// A = "$abcde 11 22 33";
  /// ExtractField( A, 2, 'w', B ); // B == "22" 
  /// \endcode
  ///    
  /// \author   GDM
  /// \return  true if successful, false if error.
  bool ExtractField( const std::string &str, const unsigned index, const char delimiter, std::string &field );


  /// \brief  Extract a substring from a string delimited by a specific delimiter
  ///         and alter the original string so that the substring and delimiter
  ///         are removed.
  /// e.g.
  /// \code
  /// std::string A = "$abcde,11,22,33";
  /// std::string B;
  /// InplaceExtractField(A, ',' B); // B == "$abcde", A == "11,22,33"
  /// InplaceExtractField(A, ',' B); // B == "11", A == "22,33"
  /// InplaceExtractField(A, ',' B); // B == "22", A == "33"
  /// InplaceExtractField(A, ',' B); // B == "33", A == ""
  /// InplaceExtractField(A, ',' B); // B == "", A == ""
  /// \endcode    
  ///
  /// \author   GDM
  /// \return  true if successful, false if error.
  bool ExtractFieldInplace( std::string &str, const char delimiter, std::string &field );    


  /// \brief    This function assumes the string is a filename like 
  ///           "c:\folder\data.gps". 
  ///           BaseName == "c:\folder\data", is the extracted string.
  /// \author   GDM
  void GetBaseName( const std::string str, std::string &BaseName );

  /// \brief    Assuming this string is a path. Directory is set to the 
  ///           folder containing the file.
  /// e.g. For:
  /// "c:\folder\subfolder\subsubfolder\data.txt", 
  /// "c:\folder\subfolder\subsubfolder" is extracted.
  /// \author   GDM
  void GetDirectoryOfThisStringPath( const std::string str, std::string &Directory );

  /// \brief    Assuming this string is a path. 
  ///           FileName is set (without the file path).
  /// e.g. For:
  /// "c:\folder\subfolder\subsubfolder\data.txt", 
  /// "data.txt" is extracted.
  /// \author   GDM
  void GetFileNameFromThisStringPath( const std::string str, std::string &FileName );

} // end of namespace StdStringUtils 

#endif // _STDSTRINGUTILES_H_ 


/**
\file     StdStringUtils.cpp
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

#include <algorithm>
#include "StdStringUtils.h"


//#define _CRT_SECURE_NO_DEPRECATE
#ifndef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

namespace StdStringUtils
{
  void SpanExcluding(const std::string &src, const std::string charSet, std::string &dst)
  {
    std::string::size_type p = 0;
    if( src.empty() )
      return;

    dst = src;
    p = src.find_first_of( charSet );
    if( p != std::string::npos )
      dst.erase(p);
  }

  void SpanExcluding(const std::string &src, const std::string charSet, std::string &dst, std::string::size_type& pos)
  {
    int index;
    std::string::size_type p = 0;
    if( src.empty() )
      return;

    dst = src;
    p = src.find_first_of( charSet );
    if( p != std::string::npos )
    {
      dst.erase(p);
      pos = src.find_first_not_of( charSet, p );
      index = static_cast<int>(pos-1);
      if( index < 0 )
      {
        pos = 0;
      }
      else
      {
        pos = pos-1;
      }
    }
    else
    {
      pos = std::string::npos;
    }
  }


  void SpanNotExcluding(const std::string &src, const std::string charSet, std::string &dst, std::string::size_type& pos)
  {
    std::string::size_type p = 0;
    if( src.empty() )
      return;

    dst = src;
    p = src.find_first_not_of( charSet );
    if( p != std::string::npos )
    {
      dst.erase(p);
      pos = p;
    }
    else
    {
      pos = std::string::npos;
    }
  }

  void MakeUpper(std::string &str)
  {
    std::transform(str.begin(),str.end(),str.begin(),toupper);
  }

  void MakeLower(std::string &str)
  {
    std::transform(str.begin(),str.end(),str.begin(),tolower);
  }

  void TrimRight(std::string &str) 
  {
    std::string::size_type p = 0;
    std::string whitespace = " \t\n\f\r";
    p = str.find_last_not_of(whitespace);
    if( p != std::string::npos )
    {
      if( p != str.length()-1 ) // check not at the end of the string
        str.erase(p+1);
    }
    else
    {
      str.erase();
    }
  }

  void TrimLeft(std::string &str) 
  {
    std::string::size_type p = 0;
    std::string whitespace = " \t\n\f\r";
    p = str.find_first_not_of(whitespace);
    if( p != std::string::npos )
    {
      if( p != 0 )
        str = str.substr( p );
    }      
    else
    {
      str.erase();
    }
  }

  void TrimLeftAndRight(std::string &str) 
  {
    TrimLeft(str);
    TrimRight(str);
  }

  bool GetDouble( const std::string &str, double &value )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sscanf_s( str.c_str(), "%lf", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#else    
    if( sscanf( str.c_str(), "%lf", &value) == 1 )
    { 
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#endif
  }
  

  bool GetFloat( const std::string &str, float &value )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sscanf_s( str.c_str(), "%f", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#else    
    if( sscanf( str.c_str(), "%f", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#endif
  }

  bool GetInt( const std::string &str, int &value )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sscanf_s( str.c_str(), "%i", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#else    
    if( sscanf( str.c_str(), "%i", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#endif
  }

  bool GetUnsignedInt( const std::string &str, unsigned &value )
  {
#ifndef _CRT_SECURE_NO_DEPRECATE
    if( sscanf_s( str.c_str(), "%u", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#else    
    if( sscanf( str.c_str(), "%u", &value) == 1 )
    {
      return true;
    }
    else
    {
      value = 0;
      return false;
    }
#endif
  }
  
  bool ExtractField( const std::string &str, const unsigned index, const char delimiter, std::string &field )
  {
    unsigned i = 0;
    std::string::size_type scount = 0;
    std::string DelimiterStr;
    std::string ExtractStr;

    if( delimiter == 'w' )
    {
      DelimiterStr = " \t\r\n\f";
    }
    else
    {
      DelimiterStr = delimiter;
    }
    ExtractStr = str;

    for( i = 0; i <= index; i++ )
    {
      if( scount >= str.length() )
      {
        ExtractStr.erase();
        field.erase();
        break;
      }
      else
      {
        ExtractStr = str.substr( scount );
      }

      SpanExcluding( ExtractStr, DelimiterStr, field );
      scount += field.length() + 1; // plus one for the delimiter
      if( field.empty() && ExtractStr.empty() )
        break; // no more data      
    }
    if( !ExtractStr.empty() )
    {
      return true;
    }
    else
    {
      field.erase();
      return false;
    }
  }

  bool ExtractFieldInplace( std::string &str, const char delimiter, std::string &field )
  {
    std::string DelimiterStr;

    if( delimiter == 'w' )
    {
      DelimiterStr = " \t\r\n\f";
    }
    else
    {
      DelimiterStr = delimiter;
    }
    if( str.empty() )
    {
      field.erase();
      return false;
    }
    SpanExcluding( str, DelimiterStr, field );      
    if( field.length() < str.length() )
      str = str.substr( field.length() + 1 );
    else
      str.erase();

    if( delimiter == 'w' )
    {
      TrimLeft(str);
    }
    return true;
  }

  void GetBaseName( const std::string str, std::string &BaseName )
  {
    std::string::size_type p = 0;
    p = str.find_last_of( "." );
    if( p == std::string::npos )      
    {
      BaseName = str;
    }
    else
    {
      BaseName = str.substr(0,p);
    }
  }

  void GetDirectoryOfThisStringPath( const std::string str, std::string &Directory )
  {
    std::string::size_type p = 0;
    p = str.find_last_of( "\\//" );
    if( p == std::string::npos )
    {
      Directory.erase();
    }
    else
    {
      if( p+1 < str.length() )
        Directory = str.substr( 0, p+1 );
      else
        Directory = str;
    }
  }

  void GetFileNameFromThisStringPath( const std::string str, std::string &FileName )
  {
    std::string::size_type p = 0;
    p = str.find_last_of( "\\//" );
    if( p == std::string::npos )
    {
      FileName = str;
    }
    else
    {
      if( p+1 < str.length() )
        FileName = str.substr(p+1);
      else
        FileName = str;
    }
  }

} // end of namespace StdStringUtils 


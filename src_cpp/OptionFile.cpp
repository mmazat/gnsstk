/**
\file    OptionFile.cpp
\brief   A class for handling option files. 
         ';' delimits a comment to follow. \n
         The general format is: \n
         field, comment = value ; comment \n
         e.g. \n
         ; A data value will follow this comment \n
         DataValue, (some comment about it) = 88 ; another comment here \n

\author  Glenn D. MacGougan (GDM)
\date    2007-11-28
\since   2006-12-07

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

#include <stdio.h>
#include <ctype.h>
#include "OptionFile.h"
#include "StdStringUtils.h"

#define OPTIONFILE_MAX_LINES (8192)
#define OPTIONFILE_MAX_LINE_BUFFER (8192)

//#define _CRT_SECURE_NO_DEPRECATE
#ifndef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

OptionFile::~OptionFile()
{
  if( m_Options != NULL )
    delete [] m_Options;
  m_nrOptions = 0;
  m_OptionFilePathUsed.erase();
}


OptionFile::OptionFile()
: m_Options(NULL),
  m_nrOptions(0)
{     
}

bool OptionFile::ReadOptionFile( const std::string OptionFilePath )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned k = 0;
  unsigned n = 0;
  unsigned numLines = 0;
  unsigned length = 0;
  std::string::size_type p = 0;
  std::string::size_type equalsIndex = 0;
  bool result = false;
  FILE *fid = NULL;
  char lineBuffer[OPTIONFILE_MAX_LINE_BUFFER];
  char lineBufferRevised[OPTIONFILE_MAX_LINE_BUFFER];
  char* fgetsBuffer = NULL;
  std::string DataLine;
  std::string TmpStr;
  std::string FieldAndComment;
  std::string Value;

  // test that the path is set
  if( OptionFilePath.empty() )
  {
    return false;
  }

  // test if the file exists
  if( DoesFileExist( OptionFilePath ) == false )
  {
    return false;
  }

  // open the input file
#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, OptionFilePath.c_str(), "r" ) != 0 )
  {
    return false;
  }
#else
  fid = fopen( OptionFilePath.c_str(), "r" );
#endif
  if( fid == NULL )
  {
    return false;
  }

  m_OptionFilePathUsed = OptionFilePath;

  // count the number of lines in the file up to a maximum line count
  for( i = 0; i < OPTIONFILE_MAX_LINES; i++ )
  {
    fgetsBuffer = fgets( lineBuffer, OPTIONFILE_MAX_LINE_BUFFER, fid );
    if( fgetsBuffer == NULL )
    {
      break;
    }
  }
  numLines = i;

  // allocate enough option structs to hold all the option data
  m_Options = new stOption[numLines];
  if( m_Options == NULL )
  {
    return false;
  }

  // read in the option file information
  rewind( fid );
  for( i = 0; i < numLines; i++ )
  {
    // initialize strings
    m_Options[j].Field.erase();
    m_Options[j].Comment.erase();
    m_Options[j].Value.erase();
    m_Options[j].PostValueComment.erase();

    fgetsBuffer = fgets( lineBuffer, OPTIONFILE_MAX_LINE_BUFFER, fid );
    if( fgetsBuffer == NULL )
    {
      break;
    }

    // Search and replace "?;" with "? ;" when ? is not whitespace
    n = 0;
    length = (unsigned)strlen( lineBuffer );
    lineBufferRevised[n] = lineBuffer[0];
    n++;
    for( k = 1; k < length; k++ )
    {
      if( lineBuffer[k] == ';' )
      {
        if( isspace(lineBuffer[k-1]) )
        {
          lineBufferRevised[n] = lineBuffer[k];
          n++;
        }
        else
        {
          // add a space
          lineBufferRevised[n] = ' ';
          n++;
          lineBufferRevised[n] = lineBuffer[k];
          n++;
        }
      }
      else
      {
        lineBufferRevised[n] = lineBuffer[k];
        n++;
      }
    }
    lineBufferRevised[n] = '\0';

    DataLine = lineBufferRevised;
    StdStringUtils::TrimLeftAndRight( DataLine ); // get rid of whitespace left and right

    // check for empty line
    if( DataLine.empty() )
    {
      continue;
    }

    // is this line a pure comment line
    if( DataLine[0] == ';' )
    {
      continue; // then just skip this line
    }

    // check that this line contains
    // <field> = <value> // at a minimum
    equalsIndex = DataLine.rfind( "=" );
    if( equalsIndex == std::string::npos )
    {
      continue; // invalid line
    }

    // check that there is a data <field>
    FieldAndComment = DataLine.substr( 0, equalsIndex );
    if( equalsIndex+1 < DataLine.length() )
      DataLine = DataLine.substr( equalsIndex+1 );
    else
      DataLine.erase();

    StdStringUtils::TrimLeftAndRight( DataLine );
    StdStringUtils::TrimLeftAndRight( FieldAndComment );
    if( FieldAndComment.empty() ) 
    {
      continue; // no <field>, invalid line
    }

    // check if there is a comment in the Field component
    p = FieldAndComment.find( "," );
    if( p != std::string::npos )
    {
      // get the data <field>
      result = StdStringUtils::ExtractFieldInplace( FieldAndComment, ',', m_Options[j].Field );
      if( result == false )
      {
        continue; // invalid line
      }
      StdStringUtils::TrimLeftAndRight( m_Options[j].Field );
      if( m_Options[j].Field.empty() )
      {
        continue; // invalid line (only a comment present)
      }

      m_Options[j].Comment = FieldAndComment;
      StdStringUtils::TrimLeftAndRight( m_Options[j].Comment ); // comment can be empty
    }
    else
    {
      m_Options[j].Field = FieldAndComment;
      m_Options[j].Comment.erase();
    }

    // get the value string with a post value comment if any
    Value.erase();
    Value = DataLine;

    if( Value.empty() )
    {
      m_Options[j].Value = Value;
      m_Options[j].PostValueComment.erase();
      j++;
      continue;
    }

    // check if there is a comment
    p = Value.find( ";" );
    if( p != std::string::npos )
    {
      result = StdStringUtils::ExtractFieldInplace( Value, ';', m_Options[j].Value );
      StdStringUtils::TrimLeftAndRight( m_Options[j].Value );
      m_Options[j].PostValueComment = Value;
      StdStringUtils::TrimLeftAndRight( m_Options[j].PostValueComment );
      j++;
    }
    else
    {
      m_Options[j].Value = Value;
      m_Options[j].PostValueComment.erase();
      j++;
    }
  }

  fclose( fid );

  m_nrOptions = j;

  return true;
}

bool OptionFile::DoesFileExist( const std::string& OptionFilePath )
{
  FILE *fid = NULL;

#ifndef _CRT_SECURE_NO_DEPRECATE
  if( fopen_s( &fid, OptionFilePath.c_str(), "r" ) != 0 )
  {
    return false;
  }
#else
  fid = fopen( OptionFilePath.c_str(), "r" );
#endif
  if( fid == NULL )
  {
    return false;
  }
  else
  {
    fclose( fid );
    return true;
  }
}

bool OptionFile::GetValue( const std::string Field, std::string &Value )
{
  std::string Comment;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetValue( const std::string Field, double &value )
{
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetDouble( Value, value ) )
      return true;
    else
      return false;
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetValue( const std::string Field, float  &value )
{
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetFloat( Value, value ) )
      return true;
    else
      return false;
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetValue( const std::string Field, short  &value )
{
  int tmp = 0;
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetInt( Value, tmp ) )
    {
      value = (short) tmp;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetValue( const std::string Field, unsigned short &value )
{
  unsigned int tmp = 0;
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetUnsignedInt( Value, tmp ) )
    {
      value = (unsigned short) tmp;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}


bool OptionFile::GetValue( const std::string Field, int &value )
{
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetInt( Value, value ) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetValue( const std::string Field, unsigned int &value )
{
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::GetUnsignedInt( Value, value ) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}


bool OptionFile::GetValue( const std::string Field, bool &value )
{
  int iTmp = 0;
  char ch = 0;
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    ch = Value[0];
    if( isdigit( ch ) )
    {
      if( StdStringUtils::GetInt( Value, iTmp ) )
      {
        if( iTmp == 0 )
        {
          value = false;
        }
        else
        {
          value = true;
        }
      }
      else
      {
        value = false;
        return false;
      }
    }
    else
    {
      ch = toupper( ch );
      if( ch == 'Y' )
      {
        if( Value.length() > 1 )
        {
          StdStringUtils::MakeUpper( Value );
          if( Value == "YES" )
          {
            value = true;
          }
          else
          {
            value = false;
          }
        }
        else
        {
          value = true;
        }
      }
      else
      {
        value = false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}


bool OptionFile::GetValueArray( 
  const std::string Field,      //!< The field identifier
  int *intArray,           //!< The pointer to the integer array.
  const unsigned maxItems, //!< The maximum number of elements in the array.
  unsigned &nrItems        //!< The number of valid items read into the array.
  )
{
  unsigned i = 0;
  unsigned n = 0; // number of valid items read
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  std::string SingleInt;
  std::string::size_type p = 0;

  if( intArray == NULL )
    return false;

  nrItems = 0;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    for( i = 0; i < maxItems; i++ )
    {
      if( Value.empty() )
        break;

      // check for ',' as a delimiter
      p = Value.find( "," );
      if( p != std::string::npos )
      {
        if( StdStringUtils::ExtractFieldInplace( Value, ',', SingleInt ) == false )
          return false;
      }
      else
      {
        if( StdStringUtils::ExtractFieldInplace( Value, 'w', SingleInt ) == false )
          return false;
      }

      if( SingleInt.empty() )
        break;

      if( StdStringUtils::GetInt( SingleInt, intArray[i] ) == false )
        return false;

      n++;
    }
  }
  else
  {
    return false;
  }

  nrItems = n;

  return true;
}


bool OptionFile::GetValueArray( 
  const std::string Field, //!< The field identifier
  double *dArray,          //!< The pointer to the integer array.
  const unsigned maxItems, //!< The maximum number of elements in the array.
  unsigned &nrItems        //!< The number of valid items read into the array.
  )
{
  unsigned i = 0;
  unsigned n = 0; // number of valid items read
  std::string Comment;
  std::string Value;
  std::string PostValueComment;
  std::string SingleDouble;
  std::string::size_type p = 0;

  if( dArray == NULL )
    return false;

  nrItems = 0;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    for( i = 0; i < maxItems; i++ )
    {
      if( Value.empty() )
        break;

      // check for ',' as a delimiter
      p = Value.find( "," );
      if( p != std::string::npos )
      {
        if( StdStringUtils::ExtractFieldInplace( Value, ',', SingleDouble ) == false )
          return false;
      }
      else
      {
        if( StdStringUtils::ExtractFieldInplace( Value, 'w', SingleDouble ) == false )
          return false;
      }

      if( SingleDouble.empty() )
        break;

      if( StdStringUtils::GetDouble( SingleDouble, dArray[i] ) == false )
        return false;

      n++;
    }
  }
  else
  {
    return false;
  }

  nrItems = n;

  return true;
}



bool OptionFile::GetComment( const std::string Field, std::string &Comment )
{
  std::string Value;
  std::string PostValueComment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool OptionFile::GetPostValueComment( const std::string Field, std::string &PostValueComment )
{
  std::string Value;
  std::string Comment;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool OptionFile::GetDMSValue( const std::string Field, double &value )
{
  std::string Value;
  std::string Comment;
  std::string PostValueComment;
  std::string TmpStr;
  int degrees = 0;
  int minutes = 0;
  double seconds = 0;
  if( FindField( Field, Comment, Value, PostValueComment ) )
  {
    if( StdStringUtils::ExtractFieldInplace( Value, 'w', TmpStr ) == false )
      return false;
    if( StdStringUtils::GetInt( TmpStr, degrees ) == false )
      return false;
    if( StdStringUtils::ExtractFieldInplace( Value, 'w', TmpStr ) == false )
      return false;
    if( StdStringUtils::GetInt( TmpStr, minutes ) == false )
      return false;
    if( minutes < 0 )
      return false;

    if( StdStringUtils::GetDouble( Value, seconds ) == false )    
      return false;

    if( degrees < 0 )
      value = degrees - minutes/60.0 - seconds/3600.0;
    else
      value = degrees + minutes/60.0 + seconds/3600.0;

    return true;
  }
  else
  {
    return false;
  }
}



bool OptionFile::FindField( 
  const std::string &Field, 
  std::string &Comment,
  std::string &Value,
  std::string &PostValueComment )
{
  unsigned i = 0;
  std::string UpperCaseFieldToFind;
  std::string UpperCaseField;

  // initial string to empty
  Comment.erase();
  Value.erase();
  PostValueComment.erase();

  // convert the search string to upper case and trim it
  UpperCaseFieldToFind = Field;
  StdStringUtils::TrimLeftAndRight( UpperCaseFieldToFind );

  if( UpperCaseFieldToFind.empty() )
    return false;

  StdStringUtils::MakeUpper( UpperCaseFieldToFind );

  // search the stored fields
  for( i = 0; i < m_nrOptions; i++ )
  {
    UpperCaseField = m_Options[i].Field;
    StdStringUtils::MakeUpper( UpperCaseField );

    if( UpperCaseFieldToFind == UpperCaseField )
    {
      Comment = m_Options[i].Comment;
      Value = m_Options[i].Value;
      PostValueComment = m_Options[i].PostValueComment;
      return true;
    }
  }

  return false;
}



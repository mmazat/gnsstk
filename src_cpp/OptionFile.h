/**
\file    OptionFile.h 
\brief   A class for handling option files. \n
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

#ifndef _OPTIONFILE_H_
#define _OPTIONFILE_H_

#include <string> // using std::string

///  \brief   A class for handling option files. 
///           ';' delimits a comment to follow. \n
///           The general format is: \n
///           field, comment = value ; comment \n
///
class OptionFile
{
protected: // Encapsulated Types

  typedef struct
  {
    std::string Field;
    std::string Comment;
    std::string Value;
    std::string PostValueComment;
  } stOption;


public: // Lifecycle

  /// \brief    OptionFile destructor
  virtual ~OptionFile(); 

  /// \brief    The default constructor.
  OptionFile();

  /// \brief    The disabled copy constructor.
  OptionFile( OptionFile& rhs ) {} ;


public: 

  /// \brief    Reads in the options
  /// \return   true if successful, false otherwise
  bool ReadOptionFile( const std::string OptionFilePath );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.
  bool GetValue( const std::string Field, std::string &Value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, double &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, float  &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.
  bool GetValue( const std::string Field, short  &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, unsigned short &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, int &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, unsigned int &value );

  /// \brief    Get data from a field
  /// \return   true if successful, false otherwise.    
  bool GetValue( const std::string Field, bool &value );

  /// \brief    Get an array of integers up to a maximum number of items.
  /// \return   true if successful, false otherwise.    
  bool GetValueArray( 
    const std::string Field,      //!< The field identifier
    int *intArray,           //!< The pointer to the integer array.
    const unsigned maxItems, //!< The maximum number of elements in the array.
    unsigned &nrItems        //!< The number of valid items read into the array.
    );

  /// \brief    Get an array of double up to a maximum number of items.
  /// \return   true if successful, false otherwise.    
  bool GetValueArray( 
    const std::string Field,      //!< The field identifier
    double *dArray,          //!< The pointer to the integer array.
    const unsigned maxItems, //!< The maximum number of elements in the array.
    unsigned &nrItems        //!< The number of valid items read into the array.
    );

  /// \brief    Get a double value in degress from a Degree Minutes Seconds value
  bool GetDMSValue( const std::string Field, double &dms );

  /// \brief    Get the comment for the field specified.
  /// \return   true if successful, false otherwise.    
  bool GetComment( const std::string Field, std::string &Comment );

  /// \breif    Get the comment for the field specified present after the <value> 
  ///           field. i.e. <field>, <comment> = <valud> ; <post-value-comment>    
  /// \return   true if successful, false otherwise.        
  bool GetPostValueComment( const std::string Field, std::string &PostValueComment );

  /// \brief    Get the number of options available.
  /// \returns  The number of options.
  unsigned GetNumberOfOptions() { return m_nrOptions; }

public: // Assignment

  /// \brief    Disabled operator= overload
  const OptionFile& operator=( const OptionFile& src ) {};


protected: // Internal Implementation

  /// \return   true if file exists
  bool DoesFileExist( const std::string& OptionFilePath );

  /// \return   true if the find was successful, and the data was set
  bool FindField( 
    const std::string &Field, 
    std::string &Comment,
    std::string &Value,
    std::string &PostValueComment );

protected: // Member Variables

  /// The array of options.
  stOption *m_Options;

  /// The number of valid options.
  unsigned m_nrOptions;

  /// The path to the option file.
  std::string m_OptionFilePathUsed;

};


#endif // _OPTIONFILE_H_

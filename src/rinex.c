/**
\file    rinex.h
\brief   GNSS core 'c' function library: RINEX related functions.
\author  Glenn D. MacGougan (GDM)
\date    2007-12-03
\since   2007-12-02

\b REFERENCES \n
- 

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
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include "rinex.h"

#define RINEX_LINEBUF_SIZE (8192)


//!< A static function to trim the whitespace from the left and right of a C string.
static BOOL RINEX_trim_left_right(
  char* str,            //!< (input/output) The input C string.
  unsigned max_length,  //!< (input) The maximum length of the input string.
  unsigned *str_length  //!< (output) The length of the string after trimming the whitespace.
  );

//!< A static function to get the header lines indicated by the record descriptor.
static BOOL RINEX_get_header_lines(
  const char* header,              //!<  (input) The full RINEX header buffer.
  const unsigned header_size,      //!<  (input) The size of the valid data in the RINEX header buffer.
  const char* record_desciptor,    //!<  (input) The record descriptor. e.g. "RINEX VERSION / TYPE"
  char* lines_buffer,              //!< (output) The output buffer. 
  const unsigned max_lines_buffer, //!<  (input) The maximum size of the output buffer.
  unsigned* nr_lines               //!< (output) The number of lines read that correspond to the record descriptor.
  );


//!< A static function to erase a substring from a string.
static BOOL RINEX_erase(
  char *erase_me, //!< (input) Erase this string from the input string.
  char* str       //!< (input/output) The input C string.  
  ); 

//!< A static function to decode the "# / TYPES OF OBSERV" part of the RINEX OBS header.
BOOL RINEX_GetObservationTypes(
  const char* header_buffer,
  const unsigned header_buffer_size,
  RINEX_structDecodedHeader* header
  );

//!< A static function to decode the time, epoch flag, and array of satellite ids present.
// static
BOOL RINEX_GetObservationEpoch(
  char* line_buffer,                //!< (input/output) The line buffer containing the RINEX time, epoch flag, number of satellites in current epoch, and array of satellite ids. Potentially followed by the receiver clock offset (seconds, optional) 
  const unsigned line_buffer_size,  //!< (input) The size of the line buffer [bytes].
  RINEX_TIME *epoch,                //!< (output) The RINEX epoch.
  RINEX_enumEpochFlag *epoch_flag   //!< (output) The epoch flag.
  );




BOOL RINEX_trim_left_right(
  char* str,            //!< (input) The input C string.
  unsigned max_length,  //!< (input) The maximum length of the input string.
  unsigned *str_length  //!< (output) The length of the string after trimming the whitespace.
  )
{
  int i = 0;
  int j = 0;
  int start = 0;
  size_t length = 0;
  if( str == NULL )
    return FALSE;
  if( str_length == NULL )
    return FALSE;

  // Remove leading whitesapce
  length = strlen( str );
  if( length > max_length )
    return FALSE;
  if( length == 0 )
    return TRUE;

  for( i = 0; i < (int)length; i++ )
  {
    if( isspace(str[i]) )
      continue;
    else
      break;
  }
  start = i;
  i = 0;
  for( j = start; j < (int)length; j++ )
  {
    str[i] = str[j];
    i++;
  }
  str[i] = '\0';

  // Remove trailing whitespace.
  length = strlen( str );
  for( i = (int)(length-1); i > 0; i-- )
  {
    if( isspace(str[i]) )
    {
      str[i] = '\0';
    }      
    else
    {
      break;
    }
  }

  length = strlen( str );
  *str_length = (unsigned)length;
  
  return TRUE;
}


BOOL RINEX_get_header_lines(
  const char* header_buffer,       //!<  (input) The full RINEX header buffer.
  const unsigned header_size,      //!<  (input) The size of the valid data in the RINEX header buffer.
  const char* record_desciptor,    //!<  (input) The record descriptor. e.g. "RINEX VERSION / TYPE"
  char* lines_buffer,              //!< (output) The output buffer. 
  const unsigned max_lines_buffer, //!<  (input) The maximum size of the output buffer.
  unsigned* nr_lines               //!< (output) The number of lines read that correspond to the record descriptor.
  )
{
  char *strptr = NULL;
  int i = 0;
  int j = 0;
  int count = 0;
  int offset = 0;
  int scount = 0;
  size_t length_record_desciptor = 0;

  if( header_buffer == NULL )
    return FALSE;
  if( header_size == 0 )
    return FALSE;
  if( record_desciptor == NULL )
    return FALSE;
  if( lines_buffer == NULL )
    return FALSE;
  if( nr_lines == NULL )
    return FALSE;

  *nr_lines = 0;

  length_record_desciptor = strlen( record_desciptor );
  if( length_record_desciptor == 0 )
    return FALSE;

  strptr = strstr( header_buffer+offset, record_desciptor );
  if( strptr == NULL )
    return FALSE; // No line found with this descriptor.

  while(strptr != NULL)
  { 
    // Determine the index into the buffer where the line with that record descriptor begins.
    i = (int)(strptr - header_buffer);

    for( j = i; j > 0; j-- )
    {
      if( header_buffer[j] == '\n' || header_buffer[j] == '\r' )
      {
        j++;
        break;
      }
    }    
    count = i-j + (int)length_record_desciptor;

    if( count < 0 )
      return FALSE;

    if( count+scount+1 >= (int)max_lines_buffer )
      return FALSE;

    strncpy( lines_buffer+scount, header_buffer+j, count );
    lines_buffer[scount+count] = '\n';
    lines_buffer[scount+count+1] = '\0';
    scount += count+1; // +1 to include the \n

    *nr_lines += 1;

    offset = i+1;
    if( offset >= (int)header_size )
      break;

    strptr = strstr( header_buffer+offset, record_desciptor );
    if( strptr == NULL )
      break;    
  };
  return TRUE;
}


BOOL RINEX_erase(
  char *erase_me, //!< (input) Erase this string from the input string.
  char* str       //!< (input/output) The input C string.
  )
{
  int i = 0;
  int j = 0;
  char *strptr = NULL;
  size_t len = 0;
  size_t len_erase_me = 0;

  if( erase_me == NULL )
    return FALSE;
  if( str == NULL )
    return FALSE;
  
  len_erase_me = strlen( erase_me );
  if( len_erase_me == 0 )
    return TRUE;

  len = strlen(str);
  if( len == 0 )
    return TRUE;

  if( len_erase_me > len )
    return TRUE;

  strptr = strstr( str, erase_me );
  
  while( strptr != NULL )
  {
    j = (int)(strptr - str);   // start of the string to be erased
    i = j + (int)len_erase_me; // end of the string to be erased

    for( i; i < (int)len; i++ )
    {
      str[j] = str[i];
      if( str[j] == '\0' )
        break;
      j++;
    }
    str[j] = '\0';
    len = strlen(str);  

    strptr = strstr( str, erase_me );
  }
  return TRUE;
}


// static 
BOOL RINEX_GetObservationTypes(
  const char* header_buffer,
  const unsigned header_buffer_size,
  RINEX_structDecodedHeader* header
  )
{
  char lines_buffer[RINEX_LINEBUF_SIZE];
  unsigned nr_lines = 0;
  BOOL result;
  char *pch = NULL;
  unsigned count = 0;
  char token[128];
  unsigned len=0;
  BOOL isFirst = TRUE;
  
  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "# / TYPES OF OBSERV",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE;  
  // strip the record description from the string
  result = RINEX_erase( "# / TYPES OF OBSERV", lines_buffer );
  if( result == FALSE )
    return FALSE;
  
  // Determine the number of observation types.
  if( sscanf( lines_buffer, "%d", &(header->nr_obs_types) ) != 1 )
    return FALSE;
  
  // Clean up the string a little.
  result = RINEX_trim_left_right( lines_buffer, RINEX_LINEBUF_SIZE, &len );
  if( result == FALSE )
    return FALSE;

  // Tokenize the string.
  pch = strtok( lines_buffer, " \t\r\n\f" );
  while( pch != NULL && count < header->nr_obs_types )
  {
    if( !isFirst )
    {
      strcpy( token, pch );
      result = RINEX_trim_left_right( token, 128, &len );
      if( result == FALSE )
        return FALSE;
      if( strlen(token) > 0 )
      {
        if( strcmp( token, "L1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_L1;
        else if( strcmp( token, "L2" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_L2;
        else if( strcmp( token, "C1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_C1;
        else if( strcmp( token, "P1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_P1;
        else if( strcmp( token, "P2" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_P2;
        else if( strcmp( token, "D1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_D1;
        else if( strcmp( token, "D2" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_D2;
        else if( strcmp( token, "T1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_T1;
        else if( strcmp( token, "T2" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_T2;
        else if( strcmp( token, "S1" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_S1;
        else if( strcmp( token, "S2" ) == 0 )
          header->obs_types[count] =  RINEX_OBS_TYPE_S2;
        else
          header->obs_types[count] =  RINEX_OBS_TYPE_UNKNOWN;

        count++;
      }
    }
    pch = strtok (NULL, " ,.-");    
    isFirst = FALSE;
  }

  if( count != header->nr_obs_types )
    return FALSE;

  return TRUE;
}


// static
BOOL RINEX_GetObservationEpoch(
  char* line_buffer,                //!< (input/output) The line buffer containing the RINEX time, epoch flag, number of satellites in current epoch, and array of satellite ids. Potentially followed by the receiver clock offset (seconds, optional) 
  const unsigned line_buffer_size,  //!< (input) The size of the line buffer [bytes].
  RINEX_TIME *epoch,                //!< (output) The RINEX epoch.
  RINEX_enumEpochFlag *epoch_flag   //!< (output) The epoch flag.
  )
{
  char *pch = NULL;
  unsigned count = 0;
  int itmp = 0;

  if( line_buffer == NULL )
    return FALSE;
  if( epoch == NULL )
    return FALSE;
  if( epoch_flag == FALSE )
    return FALSE;
  if( line_buffer_size == 0 )
    return FALSE;

  // Tokenize the input line buffer.
  pch = strtok( line_buffer, " \t\r\n\f" );
  while( pch != NULL && count < 7 )
  {
    switch( count )
    {
    case 0:
      {
        if( sscanf( pch, "%d", &(epoch->year) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 1: 
      {
        if( sscanf( pch, "%d", &(epoch->month) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 2: 
      {
        if( sscanf( pch, "%d", &(epoch->day) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 3:
      {
        if( sscanf( pch, "%d", &(epoch->hour) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 4:
      {
        if( sscanf( pch, "%d", &(epoch->minute) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 5: 
      {
        if( sscanf( pch, "%f", &(epoch->seconds) ) != 1 ) 
          return FALSE; 
        break;
      }
    case 6:
      {
        if( sscanf( pch, "%d", &(itmp) ) != 1 ) 
          return FALSE; 
        *epoch_flag = (RINEX_enumEpochFlag)itmp;
        break;
      }
    default:
      {
        break;
      }
    }   
    
    pch = strtok( NULL, " \t\r\n\f" );
    count++;
  }

  return TRUE;

}




BOOL RINEX_GetHeader( 
  const char* filepath,           //!< Path to the RINEX file.
  char* buffer,                   //!< (input/output) A character buffer in which to place the RINEX header.
  const unsigned buffer_max_size, //!< (input)  The maximum size of the buffer [bytes]. This value should be large enough to hold the entire header, (8192 to 16384).
  unsigned *buffer_size,          //!< (output) The length of the header data placed in the buffer [bytes].
  double *version,                //!< (output) The RINEX version number. e.g. 1.0, 2.0, 2.2, 3.0, etc.
  RINEX_enumFileType *file_type   //!< (output) The RINEX file type. 
  )
{
  FILE* fid = NULL;                 // A file pointer for the RINEX file.
  char linebuf[1024];               // A container for one line of the header.
  char *strptr = NULL;              // A pointer to a string.
  BOOL end_of_header_found = FALSE; // A boolean to indicate if the end of header was found.
  char type_char;
  
  size_t line_length = 0; // The length of one line.
  unsigned scount = 0;      // A counter/index used to compose the header buffer.

  fid = fopen( filepath, "r" );
  if( fid == NULL )
    return FALSE;

  // The first line of the file must be the RINEX VERSION / TYPE
  if( fgets( linebuf, 1024, fid ) == NULL )
    return FALSE;
  strptr = strstr( linebuf, "RINEX VERSION / TYPE" );
  if( strptr == NULL )
    return FALSE;

  // Add the first line to the buffer
  line_length = strlen( linebuf );
  if( scount+line_length >= buffer_max_size )    
    return FALSE;    
  scount += sprintf( buffer+scount, "%s", linebuf );

  // Extract the RINEX version and type.
  if( sscanf( linebuf, "%lf %c", version, &type_char ) != 2 )
    return FALSE;
  *file_type = (RINEX_enumFileType)type_char;

  do
  {
    if( fgets( linebuf, 1024, fid ) == NULL )
      break;

    if( strstr( linebuf, "END OF HEADER" ) != NULL )
    {
      end_of_header_found = TRUE;
    }

    // Add the line of the buffer.
    line_length = strlen( linebuf );
    if( scount+line_length >= buffer_max_size )   
      return FALSE;    
    scount += sprintf( buffer+scount, "%s", linebuf );

  }while( !end_of_header_found );

  if( end_of_header_found )
  {
    *buffer_size = scount;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}



BOOL RINEX_DecodeHeader_ObservationFile(
  const char* header_buffer,         //!< (input) The character buffer containing the RINEX header.
  const unsigned header_buffer_size, //!< (input) The size of the character buffer containing the RINEX header [bytes]. Not the maximum size, the size of the valid data in the buffer.
  RINEX_structDecodedHeader* header  //!< (output) The decoded header data.
  )
{
  BOOL result = FALSE;
  char lines_buffer[RINEX_LINEBUF_SIZE];
  unsigned nr_lines = 0;
  char rinex_type_char = 0;
  char time_system_str[128];
  unsigned len = 0;
  
  if( header_buffer == NULL )
    return FALSE;
  if( header_buffer_size == 0 )
    return FALSE;
  if( header == NULL )
    return FALSE;

  memset( header, 0, sizeof(RINEX_structDecodedHeader) );

  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "RINEX VERSION / TYPE",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE;
  if( nr_lines != 1 )
    return FALSE;
  if( sscanf( lines_buffer, "%lf %c", &(header->version), &rinex_type_char ) != 2 )
    return FALSE;
  header->type = (RINEX_enumFileType)rinex_type_char;


  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "MARKER NAME",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE;
  if( nr_lines != 1 )
    return FALSE;
  if( sscanf( lines_buffer, "%s", &(header->marker_name) ) != 1 )
    return FALSE;

  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "APPROX POSITION XYZ",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == TRUE )
  {    
    if( nr_lines != 1 )
      return FALSE;
    if( sscanf( lines_buffer, "%lf %lf %lf", &(header->x), &(header->y), &(header->z) ) != 3 )
      return FALSE;
  }

  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "ANTENNA: DELTA H/E/N",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE;
  if( nr_lines != 1 )
    return FALSE;
  if( sscanf( lines_buffer, "%lf %lf %lf", &(header->antenna_delta_h), &(header->antenna_ecc_e), &(header->antenna_ecc_n) ) != 3 )
    return FALSE;

  
  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "WAVELENGTH FACT L1/2",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE;  
  if( nr_lines == 1 )
  {
    // Only default values specified.
    if( sscanf( lines_buffer, "%d %d", &(header->default_wavefactor_L1), &(header->default_wavefactor_L2) ) != 2 )
      return FALSE;
  }
  else
  {
    // First read the default values specified.
    if( sscanf( lines_buffer, "%d %d", &(header->default_wavefactor_L1), &(header->default_wavefactor_L2) ) != 2 )
      return FALSE;

    //GDM_TODO deal with multiline WAVELENTH FACT L1/2
  }

  result = RINEX_GetObservationTypes( header_buffer, header_buffer_size, header );
  if( result == FALSE )
    return FALSE;  


  result = RINEX_get_header_lines(
    header_buffer,
    header_buffer_size,
    "TIME OF FIRST OBS",
    lines_buffer,
    RINEX_LINEBUF_SIZE,
    &nr_lines
    );
  if( result == FALSE )
    return FALSE; 

  if( sscanf( lines_buffer, "%d %d %d %d %d %f %s", 
    &(header->time_of_first_obs.year),
    &(header->time_of_first_obs.month),
    &(header->time_of_first_obs.day),
    &(header->time_of_first_obs.hour),
    &(header->time_of_first_obs.minute),
    &(header->time_of_first_obs.seconds),
    time_system_str ) != 7 )
  {
    return FALSE;
  }
  result = RINEX_trim_left_right( time_system_str, 128, &len );
  if( result == FALSE )
    return FALSE; 
  if( strcmp( time_system_str, "TIME") == 0  ) // no string present, defaults to GPS
    header->time_of_first_obs.time_system = RINEX_TIME_SYSTEM_GPS;
  else if( strcmp( time_system_str, "GPS" ) == 0 )
    header->time_of_first_obs.time_system = RINEX_TIME_SYSTEM_GPS;
  else if( strcmp( time_system_str, "GLO" ) == 0 )
    header->time_of_first_obs.time_system = RINEX_TIME_SYSTEM_GLO;
  else
    header->time_of_first_obs.time_system = RINEX_TIME_SYSTEM_UNKNOWN;




  

  
  return TRUE;
}




BOOL RINEX_GetNextObservationSet(
  FILE* fid,                               //!< (input) An open (not NULL) file pointer to the RINEX data.
  RINEX_structDecodedHeader* RINEX_header, //!< (input/output) The decoded RINEX header information. The wavelength markers can change as data is decoded.
  BOOL *wasEndOfFileReached,               //!< Has the end of the file been reached (output).
  BOOL *wasObservationFound,               //!< Was a valid observation found (output).
  unsigned *filePosition,                  //!< The file position for the start of the message found (output).  
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structObservation* obsArray, //!< A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  )
{
  char linebuf[RINEX_LINEBUF_SIZE];
  unsigned len = 0;
  BOOL result;
  RINEX_TIME epoch;
  RINEX_enumEpochFlag epoch_flag;

  // Check the input.
  if( fid == NULL )
    return FALSE;
  if( header == NULL )
    return FALSE;
  if( wasEndOfFileReached == NULL )
    return FALSE;
  if( wasObservationFound == NULL )
    return FALSE;
  if( filePosition == NULL )
    return FALSE;
  if( header == NULL )
    return FALSE;
  if( obsArray == NULL )
    return FALSE;
  if( nrObs == NULL )
    return FALSE;

  *wasObservationFound = FALSE;
  *wasEndOfFileReached = FALSE; 


  // The epoch's tme system type is the same as the header's time of first obs.
  epoch.time_system = RINEX_header->time_of_first_obs.time_system;

  // Read in the RINEX time
  do
  {
    if( fgets( linebuf, RINEX_LINEBUF_SIZE, fid ) == NULL )
    {
      if( feof(fid) )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }
      else
      {
        return FALSE;
      }
    }
    result = RINEX_trim_left_right( linebuf, RINEX_LINEBUF_SIZE, &len );
    if( result == FALSE )
      return FALSE;
  
  }while( len == 0 );

  result = RINEX_GetObservationEpoch(
    linebuf,
    len,
    &epoch,
    &epoch_flag
    );



  return TRUE;
}
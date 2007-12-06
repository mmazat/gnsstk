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
#include "time_conversion.h"
#include "constants.h"

#define RINEX_LINEBUF_SIZE (8192)


/// A static function to trim the whitespace from the left and right of a C string.
static BOOL RINEX_trim_left_right(
  char* str,            //!< (input/output) The input C string.
  unsigned max_length,  //!< (input) The maximum length of the input string.
  unsigned *str_length  //!< (output) The length of the string after trimming the whitespace.
  );

/// A static function to get the header lines indicated by the record descriptor.
static BOOL RINEX_get_header_lines(
  const char* header,              //!<  (input) The full RINEX header buffer.
  const unsigned header_size,      //!<  (input) The size of the valid data in the RINEX header buffer.
  const char* record_desciptor,    //!<  (input) The record descriptor. e.g. "RINEX VERSION / TYPE"
  char* lines_buffer,              //!< (output) The output buffer. 
  const unsigned max_lines_buffer, //!<  (input) The maximum size of the output buffer.
  unsigned* nr_lines               //!< (output) The number of lines read that correspond to the record descriptor.
  );

/// A static function to erase a substring from a string.
static BOOL RINEX_erase(
  char *erase_me, //!< (input) Erase this string from the input string.
  char* str       //!< (input/output) The input C string.  
  ); 
 
/// A static function to decode the "# / TYPES OF OBSERV" part of the RINEX OBS header.
static BOOL RINEX_GetObservationTypes(
  const char* header_buffer,
  const unsigned header_buffer_size,
  RINEX_structDecodedHeader* header
  );


/// \brief  A static function to interpret special records 
/// (embedded Header records within the observation data).
/// This function is called when the epoch flag is greater than 1.
static BOOL RINEX_DealWithSpecialRecords(
  FILE* fid,                               //!< (input) An open (not NULL) file pointer to the RINEX data.
  RINEX_structDecodedHeader* RINEX_header, //!< (input/output) The decoded RINEX header information. The wavelength markers can change as data is decoded.
  BOOL *wasEndOfFileReached,               //!< Has the end of the file been reached (output).
  unsigned *filePosition,                  //!< The file position for the start of the message found (output).  
  const unsigned nr_special_records        //!< The number of special records.
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


BOOL RINEX_DealWithSpecialRecords(
  FILE* fid,                               //!< (input) An open (not NULL) file pointer to the RINEX data.
  RINEX_structDecodedHeader* RINEX_header, //!< (input/output) The decoded RINEX header information. The wavelength markers can change as data is decoded.
  BOOL *wasEndOfFileReached,               //!< Has the end of the file been reached (output).
  unsigned *filePosition,                  //!< The file position for the start of the message found (output).  
  const unsigned nr_special_records        //!< The number of special records.
  )
{
  char line_buffer[RINEX_LINEBUF_SIZE];
  BOOL result = FALSE;
  size_t length = 0;
  unsigned i = 0;

  if( fid == NULL )
    return FALSE;
  if( RINEX_header == NULL )
    return FALSE;
  if( wasEndOfFileReached == NULL )
    return FALSE;
  if( filePosition == NULL )
    return FALSE;

  // check nothing to do.
  if( nr_special_records == 0 )
    return TRUE;

  for( i = 0; i < nr_special_records; i++ )
  {

    if( fgets( line_buffer, RINEX_LINEBUF_SIZE, fid ) == NULL )
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
    *filePosition = ftell(fid);

    if( strstr(line_buffer, "COMMENT") != NULL )
    {
      // This line is a comment. Ignore and continue.
    }
    else if( strstr(line_buffer, "WAVELENGTH FACT L1/2") != NULL )
    {
      // The wavelength factors have changed for some satellites.

      // GDM todo deal with these changes
    }
    else if( strstr(line_buffer, "MARKER NAME") != NULL )
    {
      // The marker name has changed.
      result = RINEX_erase("MARKER NAME", line_buffer);
      if( result == FALSE )
        return FALSE;
      result = RINEX_trim_left_right(line_buffer, RINEX_LINEBUF_SIZE, &length );
      if( result == FALSE )
        return FALSE;
      if( length < 64 )
      {
        strcpy(RINEX_header->marker_name, line_buffer);
      }
      else
      {
        return FALSE;
      }
    }
    else if( strstr(line_buffer, "MARKER NUMBER") != NULL )
    {
      // ignore for now
    }
    else if( strstr(line_buffer, "ANTENNA: DELTA H/E/N") != NULL )
    {
      if( sscanf( line_buffer, "%lf %lf %lf", 
        &(RINEX_header->antenna_delta_h), 
        &(RINEX_header->antenna_ecc_e), 
        &(RINEX_header->antenna_ecc_n) ) != 3 )
      {
        return FALSE;
      }
    }
    else if( strstr(line_buffer, "APPROX POSITION XYZ") != NULL )
    {
      if( sscanf( line_buffer, "%lf %lf %lf", 
        &(RINEX_header->x), 
        &(RINEX_header->y), 
        &(RINEX_header->z) ) != 3 )
      {
        return FALSE;
      }
    }
    else 
    {
      // The rest not handled yet.
    }
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
  char line_buffer[1024];               // A container for one line of the header.
  char *strptr = NULL;              // A pointer to a string.
  BOOL end_of_header_found = FALSE; // A boolean to indicate if the end of header was found.
  char type_char;
  
  size_t line_length = 0; // The length of one line.
  unsigned scount = 0;      // A counter/index used to compose the header buffer.

  fid = fopen( filepath, "r" );
  if( fid == NULL )
    return FALSE;

  // The first line of the file must be the RINEX VERSION / TYPE
  if( fgets( line_buffer, 1024, fid ) == NULL )
    return FALSE;
  strptr = strstr( line_buffer, "RINEX VERSION / TYPE" );
  if( strptr == NULL )
    return FALSE;

  // Add the first line to the buffer
  line_length = strlen( line_buffer );
  if( scount+line_length >= buffer_max_size )    
    return FALSE;    
  scount += sprintf( buffer+scount, "%s", line_buffer );

  // Extract the RINEX version and type.
  if( sscanf( line_buffer, "%lf %c", version, &type_char ) != 2 )
    return FALSE;
  *file_type = (RINEX_enumFileType)type_char;

  do
  {
    if( fgets( line_buffer, 1024, fid ) == NULL )
      break;

    if( strstr( line_buffer, "END OF HEADER" ) != NULL )
    {
      end_of_header_found = TRUE;
    }

    // Add the line of the buffer.
    line_length = strlen( line_buffer );
    if( scount+line_length >= buffer_max_size )   
      return FALSE;    
    scount += sprintf( buffer+scount, "%s", line_buffer );

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
  result = RINEX_erase("MARKER NAME", lines_buffer);
  if( result == FALSE )
    return FALSE;
  result = RINEX_trim_left_right(lines_buffer, RINEX_LINEBUF_SIZE, &len );
  if( result == FALSE )
    return FALSE;
  if( len < 64 )
  {
    strcpy(header->marker_name, lines_buffer);
  }
  else
  {
    return FALSE;
  }

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
  GNSS_structMeasurement* obsArray,        //!< A pointer to a user provided array of GNSS_structMeasurement (input/output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  )
{
  char line_buffer[RINEX_LINEBUF_SIZE]; // A character buffer to hold a line from the RINEX file.
  size_t length = 0;                // A string length.
  RINEX_TIME epoch;                 // The RINEX time.
  RINEX_enumEpochFlag epoch_flag;   // A RINEX epoch flag.
  char *pch = NULL;                 // A string pointer used in tokenizing a C string.
  unsigned count = 0;               // A counter.
  int itmp = 0;                     // A temporary integer.
  int i = 0;                        // A counter.
  int j = 0;                        // A counter.
  int obsArray_index = 0;           // A counter.
  char numstr[64];                  // A string to hold a number.
  unsigned nr_obs = 0;              // The number of satellite observations indicated for this epoch.
  double darray[64];                // A array of doubles.
  BOOL isL1data_present = FALSE;
  BOOL isL2data_present = FALSE;
  BOOL isEpochValidToDecode = FALSE;
  int nr_special_records = 0;
  
  double tow = 0; // A time of week (0-604399.99999) [s].
  unsigned short week = 0; // The GPS week (0-1024+) [weeks].
  
  RINEX_enumSatelliteSystemType next_sat_type = RINEX_SATELLITE_SYSTEM_UNKNOWN;
  
  typedef struct 
  {
    RINEX_enumSatelliteSystemType type;
    unsigned id;
  } struct_id_type;
  struct_id_type sats[64]; // An array to hold the satellite id and system type.

  typedef struct
  {
    char str[128];
    size_t length;
    BOOL isValid;
  } struct_token;
  struct_token token[64]; // An array of string tokens.
  unsigned nr_tokens = 0; // The number of valid tokens.
  
  typedef struct
  {
    unsigned char loss_of_lock_indicator;
    unsigned char signal_strength;
    double value;
  } struct_obs;

  struct_obs obs[64];


  BOOL result;                       

  numstr[0] = '\0';
  numstr[1] = '\0';
  numstr[2] = '\0';
  numstr[3] = '\0';

  // Check the input.
  if( fid == NULL )
    return FALSE;
  if( wasEndOfFileReached == NULL )
    return FALSE;
  if( wasObservationFound == NULL )
    return FALSE;
  if( filePosition == NULL )
    return FALSE;
  if( obsArray == NULL )
    return FALSE;
  if( nrObs == NULL )
    return FALSE;
  if( RINEX_header->type != RINEX_FILE_TYPE_OBS )
    return FALSE;

  *wasObservationFound = FALSE;
  *wasEndOfFileReached = FALSE; 

  // Set the token array to zero.
  memset( token, 0, sizeof(struct_token)*64 );
  
  // The epoch's tme system type is the same as the header's time of first obs.
  epoch.time_system = RINEX_header->time_of_first_obs.time_system;

  if( epoch.time_system != RINEX_TIME_SYSTEM_GPS )
  {
    // Not supported for now!
    return FALSE;
  }

  // Read in the RINEX epoch, epoch flag, and satellite ids string.
  do
  {
    do // advance over empty lines if any
    {
      if( fgets( line_buffer, RINEX_LINEBUF_SIZE, fid ) == NULL )
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
      result = RINEX_trim_left_right( line_buffer, RINEX_LINEBUF_SIZE, &length );
      if( result == FALSE )
        return FALSE;
    }while( length == 0 );
  
    if( length == 0 )
      return FALSE;

    // To make life easier later:
    // Search and replace "G ", "R ", "T ", "S " with
    //                    "G_", "R_", "T_", "S_"
    // These are the satellite ids.
    for( i = 0; i < (int)(length-1); i++ )
    {
      if( line_buffer[i] == 'G' || line_buffer[i] == 'R' || line_buffer[i] == 'T' || line_buffer[i] == 'S' )
      {
        if( line_buffer[i+1] == ' ' )
        {
          line_buffer[i+1] = '_';
        }
      }
    }
  
    // Tokenize the input line buffer.
    pch = strtok( line_buffer, " \t\r\n\f" );  
    nr_tokens = 0;
    while( pch != NULL && count < 64 )
    {
      token[nr_tokens].length = strlen(pch);
      if( token[nr_tokens].length < 128 )
      {
        strcpy( token[nr_tokens].str, pch );
        token[nr_tokens].isValid = TRUE;
      }

      pch = strtok( NULL, " \t\r\n\f" );
      nr_tokens++;
    }
    if( nr_tokens >= 64 )
      return FALSE;

    // For events without significant epoch the epoch fields can be left blank.
    // Thus, there are a few cases: 
    // (a) No epoch information,  2 integers (an event flag and the nr_special_records to follow 0-999).
    // (b) All the epoch information + (a) (year month day hour minute seconds epoch_flag nr_special_records)
    // (c) All the epoch information, epoch flag, number of satellites, the satellite list, and optionally the rx clock offset.
    if( nr_tokens == 2 )
    {
      // This can only indicate a epoch flag with a number of special records to follow.
      // Read in the epoch flag and the number of special records to follow.
      if( sscanf( token[0].str, "%d", &itmp ) != 1 )
        return FALSE;
      epoch_flag = (RINEX_enumEpochFlag)itmp;        

      if( sscanf( token[1].str, "%d", &nr_special_records ) != 1 )
        return FALSE;

      // Deal with special records if any
      result = RINEX_DealWithSpecialRecords(
        fid,
        RINEX_header, 
        wasEndOfFileReached,
        filePosition,
        nr_special_records 
        );

      continue;
    }
    else 
    {
      // The number of tokens must now contain
      // year month day hour minute seconds epoch_flag --other stuff--,
      // --other stuff-- depending on the epoch flag
      if( nr_tokens < 8 )
        return FALSE;  

      // Exract the epoch information from the tokenized line buffer.
      i = 0;
      if( sscanf( token[i].str, "%d", &(epoch.year) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%d", &(epoch.month) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%d", &(epoch.day) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%d", &(epoch.hour) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%d", &(epoch.minute) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%f", &(epoch.seconds) ) != 1 ) 
        return FALSE; 
      i++;
      if( sscanf( token[i].str, "%d", &(itmp) ) != 1 ) 
        return FALSE; 
      i++;
      epoch_flag = (RINEX_enumEpochFlag)itmp;

      if( epoch_flag > 1 )
      {
        if( sscanf( token[i].str, "%d", &nr_special_records ) != 1 )
          return FALSE;

        // Deal with special records if any
        result = RINEX_DealWithSpecialRecords(
          fid,
          RINEX_header, 
          wasEndOfFileReached,
          filePosition,
          nr_special_records 
          );
        continue;
      }
      else
      {
        isEpochValidToDecode = TRUE;
      }
    }

  }while( !isEpochValidToDecode );

  if( token[7].length == 0 )
    return FALSE;
  
  // The eighth token contains the number of satellites and there id's
  for( i = 0; i < (int)token[7].length; i++ )
  {
    // Satellite can be denoted by the following letters (RINEX_v_ 2.1)
    // 'G': GPS
    // 'R': GLONASS
    // 'S': Geostationary signal payload
    // 'T': NNSS Transit
    //
    // e.g. string here is 5G_8G12G13R_8S20 means 5 satellite observations, 
    // with GPS PRN's 8, 12, 14, GLONASS id 7, and SBAS id 20 (by the way: making up the id's here).
    if( token[7].str[i] == '_' )
    {
      continue;
    }  
    if( token[7].str[i] == '-' || token[7].str[i] == '+' || token[7].str[i] == '.' || token[7].str[i] == 'E' || token[7].str[i] == 'e' )
    {
      // Any float numbers should not be present on this line.
      return FALSE;
    }

    if( isdigit( token[7].str[i] ) )
    {
      numstr[j] = token[7].str[i];
      j++;
    }
    else
    {
      if( token[7].str[i] != RINEX_SATELLITE_SYSTEM_GPS && 
        token[7].str[i] != RINEX_SATELLITE_SYSTEM_GLO && 
        token[7].str[i] != RINEX_SATELLITE_SYSTEM_GEO && 
        token[7].str[i] != RINEX_SATELLITE_SYSTEM_NSS )
      {
        return FALSE;
      }

      numstr[j] = '\0';
      j = 0;
      // A number always precedes a non-number here. Decode the number.
      if( sscanf( numstr, "%d", &itmp ) != 1 )
        return FALSE;
      if( count == 0 )
      {
        // This is the number of observations        
        nr_obs = itmp;
        if( nr_obs >= 64 )
        {
          return FALSE; // a very unlikely error condition.
        }
        if( nr_obs > maxNrObs )
        {
          return FALSE;
        }
      }
      else
      {
        // This is a satellite id of type (current value of) next_sat_type.
        sats[count-1].id = itmp;
        sats[count-1].type = next_sat_type;
      }
      count++;
      if( count > nr_obs+1 )
      {
        return FALSE; // a very unlikely error condition.
      }
      next_sat_type = (RINEX_enumSatelliteSystemType)token[7].str[i];
    }
  }
  if( count == 0 )
  {
    return FALSE;
  }

  // The last satellite id must still be interpreted.
  numstr[j] = '\0';
  j = 0;
  if( sscanf( numstr, "%d", &itmp ) != 1 )
    return FALSE;
  sats[count-1].id = itmp;
  sats[count-1].type = next_sat_type;
  count++;

  if( count > nr_obs+1 )
  {
    // Error in number of satellite ids read compared to nr_obs.
    return FALSE;
  }

  if( count != nr_obs+1 )
  {
    if( nr_obs <= 12 )
    {
      return FALSE;
    }
    else
    {
      // Get the next line from the file.
      if( fgets(line_buffer, RINEX_LINEBUF_SIZE, fid) == NULL )
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

      length = strlen(line_buffer);
      if( length == 0 )
        return FALSE;

      if( RINEX_trim_left_right( line_buffer, RINEX_LINEBUF_SIZE, &length ) == FALSE )
        return FALSE;

      for( i = 0; i < (int)length; i++ )
      {
        if( isspace(line_buffer[i]) )
        {
          continue;
        }  
        if( line_buffer[i] == '-' || line_buffer[i] == '+' || line_buffer[i] == '.' || line_buffer[i] == 'E' || line_buffer[i] == 'e' )
        {
          // Any float numbers should not be present on this line.
          return FALSE;
        }

        if( !isdigit( line_buffer[i] ) ) // In this case the satellite system type letter is first.
        {
          if( line_buffer[i] != RINEX_SATELLITE_SYSTEM_GPS && 
            line_buffer[i] != RINEX_SATELLITE_SYSTEM_GLO && 
            line_buffer[i] != RINEX_SATELLITE_SYSTEM_GEO && 
            line_buffer[i] != RINEX_SATELLITE_SYSTEM_NSS &&
            line_buffer[i] != RINEX_SATELLITE_SYSTEM_MIXED )
          {
            return FALSE;
          }

          if( j != 0 )
          {
            numstr[j] = '\0';
            j = 0;
            if( sscanf( numstr, "%d", &itmp ) != 1 )
            {
              return FALSE;
            }
            sats[count-1].id = itmp;
            sats[count-1].type = next_sat_type;

            count++;
            if( count > nr_obs+1 )
            {
              return FALSE; // a very unlikely error condition.
            }
          }          
          next_sat_type = (RINEX_enumSatelliteSystemType)line_buffer[i];
        }
        else
        {
          numstr[j] = line_buffer[i];
          j++;
        }
      }
    }
    // The last satellite id must still be interpreted.
    numstr[j] = '\0';
    j = 0;
    if( sscanf( numstr, "%d", &itmp ) != 1 )
    {
      return FALSE;
    }
    sats[count-1].id = itmp;
    sats[count-1].type = next_sat_type;
    count++;

    if( count != nr_obs+1 )
    {
      return FALSE;
    }
  }

  if( RINEX_header->nr_obs_types >= 64 )    
    return FALSE; // A very unlikely condition.



  // TIME: The time of the measurement is the receiver time of the received signals.
  // It is identical for the phase and range measurements and is identical for
  // all satellites observed at that epoch. It is expressed in GPS time (not
  // Universal Time). 
  //
  // It is stored in UTC style (year, month, day, etc) BUT is receiver time.

  if( epoch.year >= 80 && epoch.year < 2000 )
  {
    epoch.year += 1900;
  }
  else if( epoch.year > 0 && epoch.year < 79 )
  {
    epoch.year += 2000;
  }
  else
  {
    return FALSE;
  }
  TIMECONV_GetGPSTimeFromUTCTime(
    epoch.year,
    epoch.month,
    epoch.day,
    epoch.hour,
    epoch.minute,
    epoch.seconds,
    &week,
    &tow 
    );

  
  // Set measurement data default to 0.
  memset( &(obsArray[0]), 0, sizeof(GNSS_structMeasurement) );

  // Get the next line from the file.
  if( fgets(line_buffer, RINEX_LINEBUF_SIZE, fid) == NULL )
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

  switch( RINEX_header->nr_obs_types )
  {
  case 0: 
    {
      break;
    }
  case 1:
    {
      count = sscanf( line_buffer, "%13lf%c%c", &darray[0], &loss_of_lock_indicator[0], &signal_strength[0] );
      break;
    }
  case 2:
    {
      count = sscanf( line_buffer, "%13lf%c%c%13lf%c%c", 
        &darray[0], 
        &loss_of_lock_indicator[0], 
        &signal_strength[0],
        &darray[1], 
        &loss_of_lock_indicator[1], 
        &signal_strength[1] 
      );
      break;
    }
  case 3:
    {
      count = sscanf( line_buffer, "%13lf%c%c%13lf%c%c%13lf%c%c", 
        &darray[0], 
        &loss_of_lock_indicator[0], 
        &signal_strength[0],
        &darray[1], 
        &loss_of_lock_indicator[1], 
        &signal_strength[1],
        &darray[2], 
        &loss_of_lock_indicator[2], 
        &signal_strength[2]       
      );
      break;
    }
  case 4:
    {
      count = sscanf( line_buffer, "%13lf%c%c%13lf%c%c%13lf%c%c%13lf%c%c", 
        &darray[0], 
        &loss_of_lock_indicator[0], 
        &signal_strength[0] ,
        &darray[1], 
        &loss_of_lock_indicator[1], 
        &signal_strength[1],
        &darray[2], 
        &loss_of_lock_indicator[2], 
        &signal_strength[2],
        &darray[3], 
        &loss_of_lock_indicator[3], 
        &signal_strength[3] 
      );
      break;
    }
    case 5:
    {
      count = sscanf( line_buffer, "%13lf%c%c%13lf%c%c%13lf%c%c%13lf%c%c", 
        &darray[0], 
        &loss_of_lock_indicator[0], 
        &signal_strength[0] ,
        &darray[1], 
        &loss_of_lock_indicator[1], 
        &signal_strength[1],
        &darray[2], 
        &loss_of_lock_indicator[2], 
        &signal_strength[2],
        &darray[3], 
        &loss_of_lock_indicator[3], 
        &signal_strength[3] 
      );
      break;
    }
  default:
    {
      break;
    }
  }

  
  obsArray_index = 0;
  for( i = 0; i < (int)nr_obs; i++ )
  {
    j = 0;
    count = 0;
    pch = NULL;  

    // The number and types of observations are indicated in the RINEX header.
    // Read the observations into a double array.
    // The observations can be split across multiple lines.
    while( count < RINEX_header->nr_obs_types )
    {
      // Get the next line from the file.
      if( fgets(line_buffer, RINEX_LINEBUF_SIZE, fid) == NULL )
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

      

      

      length = strlen(line_buffer);
      if( length == 0 )
        return FALSE;

      if( RINEX_trim_left_right( line_buffer, RINEX_LINEBUF_SIZE, &length ) == FALSE )
        return FALSE;

      // Tokenize the input line.
      pch = strtok( line_buffer, " \t\r\n\f" );
      while( pch != NULL && count < RINEX_header->nr_obs_types )
      {
        if( sscanf( pch, "%lf", &(darray[count]) ) != 1 )
        {
          return FALSE;
        }
        count++;
        if( count > RINEX_header->nr_obs_types >= 64 )
        {
          return FALSE;
        }
        
        pch = strtok( NULL, " \t\r\n\f" );
      }
    }
    if( count != RINEX_header->nr_obs_types )
      return FALSE;




    // The GNSS observation array is channel based. 
    // We must look for matching observation sets to place within the channel based container.
    // e.g. L1, P1, C1, D1 and S1

    // first look for L1, P1, C1, D1 and S1
    switch( sats[i].type )
    {
    case RINEX_SATELLITE_SYSTEM_GPS:
      {
        obsArray[obsArray_index].system = GNSS_GPS;
        obsArray[obsArray_index].id = (unsigned short)sats[i].id;
        break;
      }
    case RINEX_SATELLITE_SYSTEM_GLO:
      {
        obsArray[obsArray_index].system = GNSS_GLONASS;
        obsArray[obsArray_index].id = (unsigned short)sats[i].id; // GLONASS slot number.
        break;
      }
    case RINEX_SATELLITE_SYSTEM_GEO:
      {
        obsArray[obsArray_index].system = GNSS_WAAS;    
        obsArray[obsArray_index].id = (unsigned short)(sats[i].id + 100);
        break;
      }
    case RINEX_SATELLITE_SYSTEM_NSS:
      {
        continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
      }
    case RINEX_SATELLITE_SYSTEM_MIXED:
      {
        continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
      }
    default:
      {
        continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
      }
    }

    // Set the time.
    obsArray[obsArray_index].tow  =  tow;
    obsArray[obsArray_index].week = week;

    // The channel index is simply the order of the data in this case.
    obsArray[obsArray_index].channel = (unsigned short)obsArray_index;

    for( j = 0; j < (int)(RINEX_header->nr_obs_types); j++ )
    {
      switch(RINEX_header->obs_types[j])
      {
      case RINEX_OBS_TYPE_L1:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL1; 

          obsArray[obsArray_index].adr = darray[j]; // cycles

          // Set the validity flags
          obsArray[obsArray_index].flags.isActive       = TRUE;
          obsArray[obsArray_index].flags.isCodeLocked   = TRUE;
          obsArray[obsArray_index].flags.isPhaseLocked  = TRUE;
          obsArray[obsArray_index].flags.isParityValid  = TRUE; // Assume valid. No half cycle slips (invalid parity changes to valid partiy causes 1/2 cycle jump).
          obsArray[obsArray_index].flags.isAdrValid     = TRUE;
          obsArray[obsArray_index].flags.isAutoAssigned = TRUE; // Assumed.
          isL1data_present = TRUE;
          break;  
        }
      case RINEX_OBS_TYPE_C1:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL1;  
          obsArray[obsArray_index].codeType = GNSS_CACode; 

          obsArray[obsArray_index].psr = darray[j]; // m

          // The observation time convention is 'transmit' time.
          obsArray[obsArray_index].tow  =  tow - obsArray[obsArray_index].psr/LIGHTSPEED;

          // Set the validity flags
          obsArray[obsArray_index].flags.isActive       = TRUE;
          obsArray[obsArray_index].flags.isCodeLocked   = TRUE;
          obsArray[obsArray_index].flags.isPsrValid     = TRUE;
          obsArray[obsArray_index].flags.isAutoAssigned = TRUE; // Assumed.
          isL1data_present = TRUE;
          break;
        }
      case RINEX_OBS_TYPE_P1:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL1;  
          obsArray[obsArray_index].codeType = GNSS_PCode; 

          obsArray[obsArray_index].psr = darray[j]; // m

          // The observation time convention is 'tranmsit' time.
          obsArray[obsArray_index].tow  =  tow - obsArray[obsArray_index].psr/LIGHTSPEED;

          // Set the validity flags
          obsArray[obsArray_index].flags.isActive       = TRUE;
          obsArray[obsArray_index].flags.isCodeLocked   = TRUE;
          obsArray[obsArray_index].flags.isPsrValid     = TRUE;
          obsArray[obsArray_index].flags.isAutoAssigned = TRUE; // Assumed.
          isL1data_present = TRUE;
          break;
        }
      case RINEX_OBS_TYPE_D1:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL1; 

          obsArray[obsArray_index].doppler = (float)darray[j]; // m

          // Set the validity flags
          obsArray[obsArray_index].flags.isDopplerValid;
          isL1data_present = TRUE;
          break;
        }
      case RINEX_OBS_TYPE_S1:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL1; 

          // GDM_TODO - A receiver dependant look up table is needed here to convert to 
          // Carrier to noise density ratio values in dB-Hz.
          obsArray[obsArray_index].cno = (float)darray[j]; // [receiver dependant!]

          // Set the validity flags
          obsArray[obsArray_index].flags.isDopplerValid = TRUE;
          isL1data_present = TRUE;
          break;
        }
      default:
        {
          break;
        }
      }
    }
    if( isL1data_present )
    {
      obsArray_index++;
      
      // Set measurement data default to 0.
      memset( &(obsArray[obsArray_index]), 0, sizeof(GNSS_structMeasurement) );

      // There may or may not be L2 measurements. The next measurement will
      // default to the same satellite just in case but will be overwritten
      // if the measurement are only L1.
      switch( sats[i].type )
      {
      case RINEX_SATELLITE_SYSTEM_GPS:
        {
          obsArray[obsArray_index].system = GNSS_GPS;
          obsArray[obsArray_index].id = (unsigned short)sats[i].id;
          break;
        }
      case RINEX_SATELLITE_SYSTEM_GLO:
        {
          obsArray[obsArray_index].system = GNSS_GLONASS;
          obsArray[obsArray_index].id = (unsigned short)sats[i].id; // GLONASS slot number.
          break;
        }
      case RINEX_SATELLITE_SYSTEM_GEO:
        {
          obsArray[obsArray_index].system = GNSS_WAAS;    
          obsArray[obsArray_index].id = (unsigned short)(sats[i].id + 100);
          break;
        }
      case RINEX_SATELLITE_SYSTEM_NSS:
        {
          continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
        }
      case RINEX_SATELLITE_SYSTEM_MIXED:
        {
          continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
        }
      default:
        {
          continue; break; // Not supported. Ignore the data from this source. Continue to outer for loop.
        }
      }

      // Set the time.
      obsArray[obsArray_index].tow  =  tow;
      obsArray[obsArray_index].week = week;

      // The channel index is simply the order of the data in this case.
      obsArray[obsArray_index].channel = (unsigned short)obsArray_index;
    }


    // Look for L2, P2, D2 and S2
    for( j = 0; j < (int)(RINEX_header->nr_obs_types); j++ )
    {
      switch(RINEX_header->obs_types[j])
      {

      case RINEX_OBS_TYPE_L2:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL2; 

          obsArray[obsArray_index].adr = darray[j]; // cycles

          // Set the validity flags
          obsArray[obsArray_index].flags.isActive       = TRUE;
          obsArray[obsArray_index].flags.isCodeLocked   = TRUE;
          obsArray[obsArray_index].flags.isPhaseLocked  = TRUE;
          obsArray[obsArray_index].flags.isParityValid  = TRUE; // Assume valid. No half cycle slips (invalid parity changes to valid partiy causes 1/2 cycle jump).
          obsArray[obsArray_index].flags.isAdrValid     = TRUE;
          obsArray[obsArray_index].flags.isAutoAssigned = TRUE; // Assumed.
          isL2data_present = TRUE;
          break;  
        }
      case RINEX_OBS_TYPE_P2:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL2;  
          obsArray[obsArray_index].codeType = GNSS_PCode; 

          obsArray[obsArray_index].psr = darray[j]; // m

          // The observation time convention is 'tranmsit' time.
          obsArray[obsArray_index].tow  =  tow - obsArray[obsArray_index].psr/LIGHTSPEED;

          // Set the validity flags
          obsArray[obsArray_index].flags.isActive       = TRUE;
          obsArray[obsArray_index].flags.isCodeLocked   = TRUE;
          obsArray[obsArray_index].flags.isPsrValid     = TRUE;
          obsArray[obsArray_index].flags.isAutoAssigned = TRUE; // Assumed.
          isL2data_present = TRUE;
          break;
        }
      case RINEX_OBS_TYPE_D2:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL2; 

          obsArray[obsArray_index].doppler = (float)darray[j]; // m

          // Set the validity flags
          obsArray[obsArray_index].flags.isDopplerValid;
          isL2data_present = TRUE;
          break;
        }
      case RINEX_OBS_TYPE_S2:
        {
          obsArray[obsArray_index].freqType = GNSS_GPSL2; 

          // GDM_TODO - A receiver dependant look up table is needed here to convert to 
          // Carrier to noise density ratio values in dB-Hz.
          obsArray[obsArray_index].cno = (float)darray[j]; // [receiver dependant!]

          // Set the validity flags
          obsArray[obsArray_index].flags.isDopplerValid = TRUE;
          isL2data_present = TRUE;
          break;
        }
      default:
        {
          break;
        }
      }
    }

    if( isL2data_present )
    {
      obsArray_index++;
      // Set measurement data default to 0.
      memset( &(obsArray[obsArray_index]), 0, sizeof(GNSS_structMeasurement) );
    }

    // Note that T1 and T2 measurements are not supported.
  }

  *nrObs = obsArray_index;
  *wasObservationFound = TRUE;
  
  return TRUE;
}


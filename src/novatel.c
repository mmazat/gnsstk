/**
\file    novatel.c
\brief   GNSS core 'c' function library: decoding/encoding NovAtel data.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2006-08-04

\b REFERENCES \n
- NovAtel OEM4 Command and Log Reference. www.novatel.com.

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
#include "novatel.h"
#include "gps.h"
#include "constants.h"

/// \brief   Calculate the CRC32 for a Novatel binary message. crcData != NULL.
/// \return  The CRC32 value.
static unsigned NOVATEL_CalculateCRC32(                                   
  unsigned char *crcData,          //!< A pointer to the data buffer used in the computation of the crc.
  const unsigned short dataLength  //!< The length of the data buffer [bytes].  
  );   


/// \brief   Calculates the CRC32 for a NOVATEL OEM4 message.
/// \return  TRUE if successful (isCRCValid = TRUE or FALSE), FALSE otherwise, i.e. error condition.
static BOOL NOVATEL_CheckCRC32( 
  unsigned char *message,             //!< A pointer to the message buffer beginning with the sync bytes.
  const unsigned short messageLength, //!< The length of the message.
  const unsigned messageCRC,          //!< The received crc in the message.
  BOOL *isCRCValid                    //!< Is the CRC valid? Does it match the calculated value.
  );


static const unsigned NOVATEL_CRC32_Table[256] =
{
  0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L, 0x706af48fL,
  0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
  0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L, 0x1db71064L, 0x6ab020f2L,
  0xf3b97148L, 0x84be41deL, 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
  0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
  0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
  0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL, 0x35b5a8faL, 0x42b2986cL,
  0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
  0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L,
  0xcfba9599L, 0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
  0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L, 0x01db7106L,
  0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
  0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL,
  0x91646c97L, 0xe6635c01L, 0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
  0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
  0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
  0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L, 0x4adfa541L, 0x3dd895d7L,
  0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
  0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL,
  0xbe0b1010L, 0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
  0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L, 0x2eb40d81L,
  0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
  0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L, 0xe3630b12L, 0x94643b84L,
  0x0d6d6a3eL, 0x7a6a5aa8L, 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
  0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
  0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
  0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L, 0xd6d6a3e8L, 0xa1d1937eL,
  0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
  0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L,
  0x316e8eefL, 0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
  0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL, 0xb2bd0b28L,
  0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
  0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL, 0x9c0906a9L, 0xeb0e363fL,
  0x72076785L, 0x05005713L, 0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
  0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
  0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
  0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL, 0x8f659effL, 0xf862ae69L,
  0x616bffd3L, 0x166ccf45L, 0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
  0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL,
  0x40df0b66L, 0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
  0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L, 0xcdd70693L,
  0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
  0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};


// static 
unsigned NOVATEL_CalculateCRC32(                                   
  unsigned char *crcData,          //!< A pointer to the data buffer used in the computation of the crc.
  const unsigned short dataLength  //!< The length of the data buffer [bytes].  
  )   
{
  unsigned i;
  unsigned crc = 0;  
  for( i = 0; i < dataLength; i++ )   
    crc = NOVATEL_CRC32_Table[(crc ^ crcData[i]) & 0xFF] ^ (crc >> 8);
  return crc;
}


// static
BOOL NOVATEL_CheckCRC32( 
  unsigned char *message,             //!< A pointer to the message buffer beginning with the sync bytes.
  const unsigned short messageLength, //!< The length of the message [bytes].
  const unsigned messageCRC,          //!< The received crc in the message.
  BOOL *isCRCValid                    //!< Is the CRC valid? Does it match the calculated value.
  )
{
  unsigned crc = 0; // The calculated crc.

  if( message == NULL )
    return FALSE;
  
  // Calculate the crc.
  crc = NOVATEL_CalculateCRC32( message, messageLength-4 );
  
  *isCRCValid = FALSE;

  if( messageCRC != crc )
    *isCRCValid = FALSE;
  else
    *isCRCValid = TRUE;

  return TRUE;
}


BOOL NOVATELOEM4_FindNextMessageInFile(
  FILE *fid,                       //!< A file pointer to an open file (input).
  unsigned char *message,          //!< A message buffer in which to place the message found (input/output).
  const unsigned maxMessageLength, //!< The maximum size of the message buffer (input).
  BOOL *wasEndOfFileReached,       //!< Has the end of the file been reached (output).
  BOOL *wasMessageFound,           //!< Was a valid message found (output).
  unsigned *filePosition,          //!< The file position for the start of the message found (output).
  unsigned short *messageLength,   //!< The length of the entire message found and stored in the message buffer (output).
  unsigned short *messageID,       //!< The message ID of the message found.
  unsigned *numberBadCRC           //!< The number of bad crc values found. (crc fails or mistaken messages).  
  )
{
  unsigned i = 0;                  // The index into the message buffer.
  unsigned char sync[3];           // 0xAA 0x44 0x12
  unsigned char headerLength = 0;  // The length of the message header [bytes].
  unsigned byteCount = 0;          // This indicates the number of bytes read by fread().
  unsigned short dataLength = 0;   // The length of the data portion of the message. i.e. *messageLength - headerLength - CRCLength.
  unsigned short msgLength = 0;    // The entire length of the current message being examined.
  int fpos = 0;                    // A signed file position (needs sign for error checking).
  unsigned messageCRC = 0;         // The message CRC value.
  BOOL startSearch = TRUE;         // A boolean to indicate that the sync search is occurring.
  BOOL isCRCValid = FALSE;         // A boolean to indicate if the CRC is valid. Does it match the calculated value.


  // Initialize the output parameters.
  *wasEndOfFileReached = FALSE;
  *wasMessageFound = 0;
  *filePosition    = 0;
  *messageLength   = 0;
  *messageID       = 0;
  *numberBadCRC    = 0;
  
  // Ensure that the file pointer is valid.
  if( fid == NULL )
    return FALSE;

  // Ensure that the maximum message length is appropriate.
  if( maxMessageLength < 32 ) // at least 3 sync bytes plus 25 header bytes plus the 4 byte CRC.
  {
    return FALSE;
  }
  
  // Ensure that the file pointer is not at the end of the file.
  if( feof(fid) )
  {
    *wasEndOfFileReached = TRUE;
    return TRUE;
  }
  
  while( !(*wasEndOfFileReached) && !(*wasMessageFound) )
  {
    if( startSearch )
    {
      // Get the first two sync bytes.
      sync[0] = (unsigned char)fgetc( fid );
      if( feof( fid ) )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }

      sync[1] = (unsigned char)fgetc( fid );
      if( feof( fid ) )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }  

      startSearch = FALSE;
    }

    // Get the last sync byte.
    sync[2] = (unsigned char)fgetc( fid );
    if( feof( fid ) )
    {
      *wasEndOfFileReached = TRUE;
      return TRUE;
    }

    // Check the full sync.
    if( sync[0] == 0xAA && sync[1] == 0x44 && sync[2] == 0x12 )
    {
      // Found a potential message.
      i = 0;
      message[i] = sync[0]; i++;
      message[i] = sync[1]; i++;
      message[i] = sync[2]; i++;

      // Determine the file position of the start of the message
      fpos = ftell(fid);
      if( fpos < 2 ) // check less than 2 because we are at least 3 bytes into the file
      {
        return FALSE;
      }
      *filePosition = fpos - 3; // The start of the message sync.

      // Get the header length.
      headerLength = (unsigned char)fgetc( fid );
      if( feof( fid ) )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }
      message[i] = headerLength;
      i++;

      // Get rest of the header (maximum of 255 bytes).
      byteCount = (int)fread( &(message[i]), sizeof(unsigned char), headerLength-4, fid );
      if( byteCount+4 != headerLength )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }

      // Determine the 2 byte message ID.
      *messageID  = message[i]; 
      i++;
      *messageID |= message[i] << 8;      
      i++;

      // Skip the message type and the port address.
      i += 2;

      // Determine the 2 byte data length.
      dataLength = message[i];
      i++;
      dataLength |= message[i] << 8;
      i++;

      // Check that the entire message length will fit in the message buffer.
      msgLength = headerLength + dataLength + 4; // plus 4 for the CRC.
      if( msgLength > maxMessageLength )
      {
        // Perhaps the data length was bad.
        // Start searching again after the sync that was found.
        // Set the file position to the last of the three sync bytes.
        if( fseek( fid, fpos, SEEK_SET ) != 0 )
        {
          // The seek failed.
          return FALSE;
        }
        startSearch = TRUE;
        continue;
      }

      // Get the data part of the message.
      // advance i to the start of the data
      i = headerLength;
      byteCount = (int)fread( &(message[i]), sizeof(unsigned char), dataLength, fid );
      if( byteCount != dataLength )
      {
        // Either a message has an invalid length or a message was cut off.
        // We do not want to read over any valid message so start the search again.
        // Start searching again after the sync that was found.
        // Set the file position to the last of the three sync bytes.
        if( fseek( fid, fpos, SEEK_SET ) != 0 )
        {
          // The seek failed.
          return FALSE;
        }
        startSearch = TRUE;
        continue;
      }
      i += dataLength;

      // Get the CRC.
      byteCount = (int)fread( &(message[i]), sizeof(unsigned char), 4, fid );
      if( byteCount != 4 )
      {
        *wasEndOfFileReached = TRUE;
        return TRUE;
      }
      messageCRC  = message[i];
      i++;
      messageCRC |= message[i] << 8;
      i++;
      messageCRC |= message[i] << 16;
      i++;
      messageCRC |= message[i] << 24;
      i++;
      
      // Compare the received message CRC with the calculated CRC.
      if( !NOVATEL_CheckCRC32( message, msgLength, messageCRC, &isCRCValid ) )
      {
        return FALSE;
      }
      if( !isCRCValid )
      {
        *numberBadCRC += 1;

        // Start searching again after the sync that was found.
        // Set the file position to the last of the three sync bytes.
        if( fseek( fid, fpos, SEEK_SET ) != 0 )
        {
          // The seek failed.
          return FALSE;
        }
        startSearch = TRUE;
        continue;
      }

      *wasMessageFound = TRUE;
      *messageLength = msgLength;
      break;
    }
    else
    {
      // Shift the first two bytes of the sync.
      sync[0] = sync[1];
      sync[1] = sync[2];
    }
  }
  return TRUE;
}


BOOL NOVATELOEM4_DecodeBinaryMessageHeader(
  const unsigned char *message,            //!< The message buffer containing a complete NOVATEL OEM4 binary message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader *header   //!< A pointer to a NovAtel OEM4 header information struct (output).
  )
{
  unsigned u32 = 0;
  NOVATELOEM4_structRxStatusBitField* ptrRxStatusBitField = NULL;

  // sanity checks
  if( message == NULL )
    return FALSE;
  if( header == NULL )
    return FALSE;

  if( messageLength < 4 ) // at least the sync bytes and the header length
    return FALSE;

  
  // Get the header length;
  header->headerLength = message[3];
  if( messageLength < header->headerLength )
    return FALSE;
  
  header->messageID  = message[4];
  header->messageID |= message[5] << 8;

  header->messageType = message[6];

  header->portAddress = message[7];
  
  header->dataLength  = message[8];
  header->dataLength |= message[9] << 8;

  header->sequenceNr  = message[10];
  header->sequenceNr |= message[11] << 8;

  header->idleTime = message[12];
  
  header->eTimeStatus = (NOVATELOEM4_enumTimeStatus)message[13];

  header->gpsWeek  = message[14];
  header->gpsWeek |= message[15] << 8;

  header->gpsMilliSeconds  = message[16];
  header->gpsMilliSeconds |= message[17] << 8;
  header->gpsMilliSeconds |= message[18] << 16;
  header->gpsMilliSeconds |= message[19] << 24;

  u32  = message[20];
  u32 |= message[21] << 8;
  u32 |= message[22] << 16;
  u32 |= message[23] << 24;
  ptrRxStatusBitField = (NOVATELOEM4_structRxStatusBitField *)&u32;  
  header->receiverStatus = *ptrRxStatusBitField;

  header->reserved  = message[24];
  header->reserved |= message[25] << 8;

  header->receiverVersion  = message[26];
  header->receiverVersion |= message[27] << 8;

  return TRUE;
}


BOOL NOVATELOEM4_DecodeRANGECMPB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structObservation* obsArray, //!< A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  )
{
  unsigned i = 0;                 // A counter.
  unsigned index = 0;             // An index into the message buffer.
  unsigned char headerLength = 0; // The length of the header only as indicated in the header.
  unsigned short dataLength = 0;  // The length of the data part of the message as indicated in the header.
  unsigned short msgLength = 0;   // The computed length of the entire message.
  unsigned short testLength = 0;  // The expected length of the data given the number of observations.
  
  // Perform sanity checks.
  if( message == NULL )
    return FALSE;
  if( obsArray == NULL )
    return FALSE;
  if( message[0] != 0xAA || message[1] != 0x44 || message[2] != 0x12 ) // sync must be present
    return FALSE;

  // Decode the binary header.
  if( !NOVATELOEM4_DecodeBinaryMessageHeader( message, messageLength, header ) )
    return FALSE;

  // Get the header length.
  headerLength = message[3];

  // Get the data length.
  dataLength = message[8];
  dataLength |= message[9] << 8;

  // Check that the data length + header length + 4 (crc) == messageLength provided.
  msgLength = dataLength + headerLength + 4;
  if( msgLength != messageLength )
    return FALSE;

  // Get the number of observations.
  index = headerLength;
  *nrObs = message[index]; 
  index++;
  *nrObs |= message[index] << 8;
  index++;
  *nrObs |= message[index] << 16;
  index++;
  *nrObs |= message[index] << 24;

  // Check that there is enough room in the array provided.
  if( maxNrObs < *nrObs )
    return FALSE;

  // Check that the number of observations fits the data length given.
  testLength = (unsigned short)(*nrObs*24 + 4);
  if( testLength != dataLength )
   return FALSE;

  // Decode the data.
  index = headerLength+4;
  for( i = 0; i < *nrObs; i++ )
  {
    long long int rangecmp_psr       = 0;  // Pseudorange
    long rangecmp_adr                = 0;  // Carrier Phase
    double adr_rolls                 = 0;  // ADR Roll Over
    long rangecmp_doppler_freq       = 0;  // Doppler Frequency 
    unsigned char rangecmp_stdev_psr = 0;  // Standard Deviation of Pseudorange
    unsigned char rangecmp_stdev_adr = 0;  // Standard Deviation of Carrier Phase
    unsigned long rangecmp_lock_time = 0;  // Lock Time
    unsigned long rangecmp_cno       = 0;  // Carier to Noise Density
    double dtmp = 0;

    // Tracking Status  (32 bits 0 - 31)   
    obsArray[i].rawTrackingStatus  = message[index];       index++;
    obsArray[i].rawTrackingStatus |= message[index] <<  8; index++;
    obsArray[i].rawTrackingStatus |= message[index] << 16; index++;
    obsArray[i].rawTrackingStatus |= message[index] << 24; index++;

    if( !NOVATELOEM4_DecodeTrackingStatus( obsArray[i].rawTrackingStatus, &obsArray[i].trackingStatus ) )    
      return FALSE;    
  
    // Doppler Frequency (28 bits 32-59)
    rangecmp_doppler_freq |=  message[index];        index++;
    rangecmp_doppler_freq |=  message[index] << 8;   index++;
    rangecmp_doppler_freq |=  message[index] << 16;  index++;
    rangecmp_doppler_freq |= (message[index] & 0x0F) << 24;  
    // check negative value.
    if( (rangecmp_doppler_freq >> 27) > 0 )
    {    
      rangecmp_doppler_freq |= 0xF0000000;      
    }
    dtmp = (double)(rangecmp_doppler_freq);
    dtmp /= 256.0;
    obsArray[i].doppler = (float)dtmp;

    // Pseudorange (36 bits 60-95)
    rangecmp_psr    |= (message[index]&0xF0)>>4; index++;
    rangecmp_psr    |=  message[index]<<4;       index++;
    rangecmp_psr    |=  message[index]<<12;      index++;
    rangecmp_psr    |=  message[index]<<20;      index++;
    rangecmp_psr    |= (long long int)(message[index])<<28; index++;    
    // check negative value.
    if( ((long long int)(rangecmp_psr)>>35) > 0 )
    {
      // The checkneg variable was added to avoid compiler warnings with g++ (v4.1.2).
      unsigned long int checkneg = 0xFFFFFFF0;
      checkneg *= (1<<31);
      checkneg *= 2; // this is 0xFFFF FFF0 0000 0000
      rangecmp_psr |= checkneg;
    }
    obsArray[i].psr  = (double)rangecmp_psr;
    obsArray[i].psr /= 128.0; 

    // Carrier Phase (32 bits 96-127)
    rangecmp_adr   = message[index];       index++;
    rangecmp_adr  |= message[index] << 8;  index++;
    rangecmp_adr  |= message[index] << 16; index++;
    rangecmp_adr  |= message[index] << 24; index++;
    obsArray[i].adr  = (double) rangecmp_adr;
    obsArray[i].adr /= 256.0; 

    // Carrier Phase Roll over Calculations
    if (obsArray[i].trackingStatus.eFrequency == NOVATELOEM4_L1)
      adr_rolls =(obsArray[i].psr / GPS_WAVELENGTHL1  + obsArray[i].adr) / 8388608.0;
    else
      adr_rolls =(obsArray[i].psr / GPS_WAVELENGTHL2  + obsArray[i].adr) / 8388608.0;

    if ( adr_rolls <= 0)
      adr_rolls = adr_rolls - 0.5;
    else
      adr_rolls = adr_rolls + 0.5;

    obsArray[i].adr = obsArray[i].adr  - (8388608.0 * (int)adr_rolls);

    // Pseudorange Standard Deviation (4 bits 128-131)
    rangecmp_stdev_psr = ((message[index] & 0x0F)); 

    // Refer to page 352 OEMV Family Firmware Version 3.100 Reference Manual Rev 3
    // Refer to page 143 OEM4 Family Firmware Version 1.000 Command and Log Reference Manual Rev 4
    switch(rangecmp_stdev_psr)
    { 
    case 0  : obsArray[i].psrstd = 0.050f; break;
    case 1  : obsArray[i].psrstd = 0.075f; break;
    case 2  : obsArray[i].psrstd = 0.113f; break;
    case 3  : obsArray[i].psrstd = 0.169f; break;
    case 4  : obsArray[i].psrstd = 0.253f; break;
    case 5  : obsArray[i].psrstd = 0.380f; break;
    case 6  : obsArray[i].psrstd = 0.570f; break;
    case 7  : obsArray[i].psrstd = 0.854f; break;
    case 8  : obsArray[i].psrstd = 1.281f; break;
    case 9  : obsArray[i].psrstd = 2.375f; break;
    case 10 : obsArray[i].psrstd = 4.750f; break;
    case 11 : obsArray[i].psrstd = 9.500f; break;
    case 12 : obsArray[i].psrstd = 19.000f; break;
    case 13 : obsArray[i].psrstd = 38.000f; break;
    case 14 : obsArray[i].psrstd = 76.000f; break;
    case 15 : obsArray[i].psrstd = 152.000f; break;
    };

    // Carrier Phase Standard Deviation (4 bits 132-135)
    rangecmp_stdev_adr =  message[index] >> 4 ; 
    index++;
    obsArray[i].adrstd = (float)(rangecmp_stdev_adr+1)/512;

    // Prn Number (8 bits 136-143)
    obsArray[i].prn = (unsigned short)message[index]; 
    index++;

    // Lock Time  (21 bits 144-164)
    rangecmp_lock_time   |= message[index];      index++;
    rangecmp_lock_time   |= message[index] << 8; index++;
    rangecmp_lock_time   |= ((message[index] & 0x1F)) << 16;  

    obsArray[i].locktime  = (float)(rangecmp_lock_time);
    obsArray[i].locktime /= 32.0;

    // Carrier to Noise Density ( 5 bits 165-169)
    rangecmp_cno         = (message[index] & 0xE0 ) >> 5; index++;  
    rangecmp_cno        |= (message[index] & 0x03 ) << 3;  

    obsArray[i].cno = (float)rangecmp_cno + 20;

    // Reserved (22 bits 170-191)
    obsArray[i].reserved  = (message[index] & 0xFC )>> 5; index ++ ;
    obsArray[i].reserved |= message[index] << 8;          index ++ ;
    obsArray[i].reserved |= message[index] << 16;         index ++ ;
  }

  return TRUE;
}

BOOL NOVATELOEM4_DecodeRANGEB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structObservation* obsArray, //!< A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  )
{
  unsigned i = 0;                 // A counter.
  unsigned j = 0;                 // A counter.
  unsigned index = 0;             // An index into the message buffer.
  unsigned char headerLength = 0; // The length of the header only as indicated in the header.
  unsigned short dataLength = 0;  // The length of the data part of the message as indicated in the header.
  unsigned short msgLength = 0;   // The computed length of the entire message.
  unsigned short testLength = 0;  // The expected length of the data given the number of observations.
  unsigned char dbytes[8];        // Enough bytes to store a double.
  unsigned char fbytes[4];        // Enough bytes to store a float.
  double *dptr;                   // A pointer to a double.
  float *fptr;                    // A pointer to a float.
  
  // Perform sanity checks.
  if( message == NULL )
    return FALSE;
  if( obsArray == NULL )
    return FALSE;
  if( message[0] != 0xAA || message[1] != 0x44 || message[2] != 0x12 ) // sync must be present
    return FALSE;

  // Decode the binary header.
  if( !NOVATELOEM4_DecodeBinaryMessageHeader( message, messageLength, header ) )
    return FALSE;

  // Get the header length.
  headerLength = message[3];

  // Get the data length.
  dataLength = message[8];
  dataLength |= message[9] << 8;

  // Check that the data length + header length + 4 (crc) == messageLength provided.
  msgLength = dataLength + headerLength + 4;
  if( msgLength != messageLength )
    return FALSE;

  // Get the number of observations.
  index = headerLength;
  *nrObs = message[index]; 
  index++;
  *nrObs |= message[index] << 8;
  index++;
  *nrObs |= message[index] << 16;
  index++;
  *nrObs |= message[index] << 24;

  // Check that there is enough room in the array provided.
  if( maxNrObs < *nrObs )
    return FALSE;

  // Check that the number of observations fits the data length given.
  testLength = (unsigned short)(*nrObs*44 + 4);
  if( testLength != dataLength )
    return FALSE;

  // Decode the data.
  index = headerLength+4;
  for( i = 0; i < *nrObs; i++ )
  {
    obsArray[i].prn       = message[index];      index++;
    obsArray[i].prn      |= message[index] << 8; index++;
    obsArray[i].reserved  = message[index];      index++;
    obsArray[i].reserved |= message[index] << 8; index++;
    
    for( j=0; j<8; j++ ){ dbytes[j] = message[index]; index++; }
    dptr = (double*)&dbytes;
    obsArray[i].psr = *dptr;

    for( j=0; j<4; j++ ){ fbytes[j] = message[index]; index++; }
    fptr = (float*)&fbytes;
    obsArray[i].psrstd = *fptr;

    for( j=0; j<8; j++ ){ dbytes[j] = message[index]; index++; }
    dptr = (double*)&dbytes;
    obsArray[i].adr = *dptr;

    for( j=0; j<4; j++ ){ fbytes[j] = message[index]; index++; }
    fptr = (float*)&fbytes;
    obsArray[i].adrstd = *fptr;

    for( j=0; j<4; j++ ){ fbytes[j] = message[index]; index++; }
    fptr = (float*)&fbytes;
    obsArray[i].doppler = *fptr;

    for( j=0; j<4; j++ ){ fbytes[j] = message[index]; index++; }
    fptr = (float*)&fbytes;
    obsArray[i].cno = *fptr;

    for( j=0; j<4; j++ ){ fbytes[j] = message[index]; index++; }
    fptr = (float*)&fbytes;
    obsArray[i].locktime = *fptr;

    obsArray[i].rawTrackingStatus  = message[index];       index++;
    obsArray[i].rawTrackingStatus |= message[index] <<  8; index++;
    obsArray[i].rawTrackingStatus |= message[index] << 16; index++;
    obsArray[i].rawTrackingStatus |= message[index] << 24; index++;

    if( !NOVATELOEM4_DecodeTrackingStatus( obsArray[i].rawTrackingStatus, &obsArray[i].trackingStatus ) )    
      return FALSE;    
  }
 
  return TRUE;
}


BOOL NOVATELOEM4_DecodeTrackingStatus(
  const unsigned rawTrackingStatus,                 //!< The raw 32 bit tracking status value.
  NOVATELOEM4_structTrackingStatus *trackingStatus  //!< The decoded tracking status information.
  )
{
  // local typedef
  typedef struct
  {
    unsigned trackingState          :5;
    unsigned channelNumber          :5;
    unsigned phaseLockFlag          :1;
    unsigned parityKnownFlag        :1;
    unsigned codeLockedFlag         :1;
    unsigned correlatorSpacing      :3;
    unsigned satelliteSystem        :3;
    unsigned reserved1              :1;
    unsigned grouping               :1;
    unsigned frequency              :2;
    unsigned codeType               :3;
    unsigned forwardErrorCorrection :1;
    unsigned primaryL1Channel       :1;
    unsigned halfCycleFlag          :1;
    unsigned reserved3              :1;
    unsigned prnLockFlag            :1;
    unsigned channelAssignment      :1;
  } NOVATELOEM4_channelStatusBitField; // 0-31 bits

  NOVATELOEM4_channelStatusBitField bitField;
  NOVATELOEM4_channelStatusBitField *ptrBitField;

  if( trackingStatus == NULL )
    return FALSE;

  ptrBitField = (NOVATELOEM4_channelStatusBitField *)&rawTrackingStatus;
  bitField = *ptrBitField;

  trackingStatus->eTrackingState     = (NOVATELOEM4_enumTrackingState)bitField.trackingState;
  trackingStatus->channelNumber      = bitField.channelNumber;
  trackingStatus->isPhaseLocked      = bitField.phaseLockFlag;
  trackingStatus->isParityKnown      = bitField.parityKnownFlag;
  trackingStatus->isCodeLocked       = bitField.codeLockedFlag;
  trackingStatus->eCorrelatorSpacing = (NOVATELOEM4_enumCorrelatorSpacing)bitField.correlatorSpacing;
  trackingStatus->eSatelliteSystem   = (NOVATELOEM4_enumSatelliteSystem)bitField.satelliteSystem;
  trackingStatus->isGrouped          = bitField.grouping;
  trackingStatus->eFrequency         = (NOVATELOEM4_enumFrequency)bitField.frequency;
  trackingStatus->eCodeType          = (NOVATELOEM4_enumCodeType)bitField.codeType;
  trackingStatus->isFECEnabled       = bitField.forwardErrorCorrection;
  trackingStatus->isPrimaryL1Channel = bitField.primaryL1Channel;
  trackingStatus->isHalfCycleAdded   = bitField.halfCycleFlag;
  trackingStatus->isForcedAssignment = bitField.channelAssignment;

  return TRUE;
}


BOOL NOVATELOEM4_DecodeRAWEPHEMB(
  const unsigned char *message,           //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,     //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header, //!< A pointer to a NovAtel OEM4 header information struct (output).
  unsigned *prn,                          //!< The satellite PRN number.
  unsigned *reference_week,               //!< The reference GPS week (0-1024+) [weeks].
  unsigned *reference_time,               //!< The reference GPS time of week (0-604800) [s].
  unsigned       *tow,                    //!< The time of week in subframe1, the time of the leading bit edge of subframe 2 [s]
  unsigned short *iodc,                   //!< 10 bit issue of data (clock), 8 LSB bits will match the iode                  []    
  unsigned char  *iode,                   //!< 8 bit  issue of data (ephemeris)                                              []
  unsigned       *toe,                    //!< reference time ephemeris (0-604800)                                           [s]
  unsigned       *toc,                    //!< reference time (clock)   (0-604800)                                           [s]      
  unsigned short *week,                   //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]    
  unsigned char  *health,                 //!< 6 bit health parameter, 0 if healthy, unhealth othersize                      [0=healthy]    
  unsigned char  *alert_flag,             //!< 1 = URA may be worse than indicated                                           [0,1]
  unsigned char  *anti_spoof,             //!< anti-spoof flag from 0=off, 1=on                                              [0,1]    
  unsigned char  *code_on_L2,             //!< 0=reserved, 1=P code on L2, 2=C/A on L2                                       [0,1,2]
  unsigned char  *ura,                    //!< User Range Accuracy lookup code, 0 is excellent, 15 is use at own risk        [0-15], see p. 83 GPSICD200C
  unsigned char  *L2_P_data_flag,         //!< flag indicating if P is on L2 1=true                                          [0,1]
  unsigned char  *fit_interval_flag,      //!< fit interval flag (four hour interval or longer) 0=4 fours, 1=greater         [0,1]
  unsigned short *age_of_data_offset,     //!< age of data offset                                                            [s]
  double *tgd,     //!< group delay                                                                   [s]
  double *af2,     //!< polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
  double *af1,     //!< polynomial clock correction coefficient (clock drift)                         [s/s]
  double *af0,     //!< polynomial clock correction coefficient (clock bias)                          [s]    
  double *m0,      //!< mean anomaly at reference time                                                [rad]
  double *delta_n, //!< mean motion difference from computed value                                    [rad/s]
  double *ecc,     //!< eccentricity                                                                  []
  double *sqrta,   //!< square root of the semi-major axis                                            [m^(1/2)]
  double *omega0,  //!< longitude of ascending node of orbit plane at weekly epoch                    [rad]
  double *i0,      //!< inclination angle at reference time                                           [rad]
  double *w,       //!< argument of perigee                                                           [rad]
  double *omegadot,//!< rate of right ascension                                                       [rad/s]
  double *idot,    //!< rate of inclination angle                                                     [rad/s]
  double *cuc,     //!< amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
  double *cus,     //!< amplitude of the sine harmonic correction term to the argument of latitude    [rad]
  double *crc,     //!< amplitude of the cosine harmonic correction term to the orbit radius          [m]
  double *crs,     //!< amplitude of the sine harmonic correction term to the orbit radius            [m]
  double *cic,     //!< amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
  double *cis      //!< amplitude of the sine harmonic correction term to the angle of inclination    [rad]
  )
{
  unsigned short msgLength; // The computed message length [bytes].
  unsigned index = 0;       // An index in to the message.
  BOOL result = FALSE;

  // Perform sanity checks.
  if( message == NULL )
    return FALSE;
  if( message[0] != 0xAA || message[1] != 0x44 || message[2] != 0x12 ) // sync must be present
    return FALSE;

  // Decode the binary header.
  if( !NOVATELOEM4_DecodeBinaryMessageHeader( message, messageLength, header ) )
    return FALSE;

  // Check that the length of the message matches the header information.
  msgLength = header->headerLength + header->dataLength + 4;
  if( msgLength != messageLength )
    return FALSE;

  index = header->headerLength;
  *prn  = message[index];       index++;
  *prn |= message[index] << 8;  index++;
  *prn |= message[index] << 16; index++;
  *prn |= message[index] << 24; index++;

  *reference_week  = message[index];       index++;
  *reference_week |= message[index] << 8;  index++;
  *reference_week |= message[index] << 16; index++;
  *reference_week |= message[index] << 24; index++;

  *reference_time  = message[index];       index++;
  *reference_time |= message[index] << 8;  index++;
  *reference_time |= message[index] << 16; index++;
  *reference_time |= message[index] << 24; index++;

  result = GPS_DecodeRawGPSEphemeris(
    &message[index],
    &message[index+30],
    &message[index+60],    
    (unsigned short)(*prn),
    tow,
    iodc,
    iode,  
    toe,   
    toc,   
    week,  
    health,
    alert_flag,
    anti_spoof,
    code_on_L2,
    ura,       
    L2_P_data_flag,
    fit_interval_flag,
    age_of_data_offset, 
    tgd,                
    af2,                
    af1,                
    af0,                
    m0,                
    delta_n,           
    ecc,               
    sqrta,             
    omega0,            
    i0,                
    w,                 
    omegadot,          
    idot,              
    cuc,               
    cus,               
    crc,               
    crs,               
    cic,               
    cis                
    );
  if( !result )
    return FALSE;

  return TRUE;
}

/*
paaudiotest
  Test for paaudioseries.
  Jack Lee, CVTE CRI, Guangzhou, China
  lizhongjie@cvte.com
  
  CHANGES:
  0.1 initial release
*/

#include <windows.h>
#include "stdafx.h"

#include "paaudioseries.h"


typedef int (*APR)( int, float *, int, int, int, int, float *, int, int, int, paReturnFun, int );
typedef int (*ADI)( int, char *, int );


typedef struct
{
    HINSTANCE hInstance_;

    APR paudioPlayRec;
    ADI paudioDeviceInfo;
} PaAudioSeriesEntryPoints;


// memory fucntion
static void * paAllocateMemory( long size )
{
    void * result = GlobalAlloc( GPTR, size );

    return result;
}

static void paFreeMemory( void * block )
{
    if( block != NULL )
        GlobalFree( block );
}

int _tmain(int argc, _TCHAR* argv[])
{
    const char * dllName = "paaudioseries.dll";
    const char * funNameAPR = "audioPlayRec";
    const char * funNameADI = "audioDeviceInfo";

    PaAudioSeriesEntryPoints paAudioSeriesEntryPoints = { NULL, NULL, NULL };

    paAudioSeriesEntryPoints.hInstance_ = LoadLibraryA( dllName );
    if( paAudioSeriesEntryPoints.hInstance_ != NULL )
    {
        paAudioSeriesEntryPoints.paudioPlayRec = (APR)GetProcAddress( paAudioSeriesEntryPoints.hInstance_, funNameAPR );
        paAudioSeriesEntryPoints.paudioDeviceInfo = (ADI)GetProcAddress( paAudioSeriesEntryPoints.hInstance_, funNameADI );
    }

    if( paAudioSeriesEntryPoints.paudioDeviceInfo != NULL )
    {
        char buf[1024*16];
        int ret;

        memset( buf, 0, 1024*16 );
        ret = paAudioSeriesEntryPoints.paudioDeviceInfo( -1, buf, 1024*16 );
        if( ret == paSuccess )
        {
            printf( "\nAudio Device List:\n" );
            printf( buf );
        }

        memset( buf, 0, 1024*16 );
        ret = paAudioSeriesEntryPoints.paudioDeviceInfo( 12, buf, 1024*16 );
        if( ret == paSuccess )
        {
            printf( "\nAudio Device 12 Information:\n" );
            printf( buf );
        }
    }

    if( paAudioSeriesEntryPoints.paudioPlayRec != NULL )
    {
        float * precBuf;

        precBuf = (float *)paAllocateMemory( sizeof(float) * 44100 * 5 * 2 );
        if( precBuf != NULL )
        {
            int ret;

            printf( "\nStart Recording 5 seconds.\n" );
            ret = paAudioSeriesEntryPoints.paudioPlayRec( -1, NULL, 0, 0, 0, 2, precBuf, 44100 * 5, 0, 1, NULL, 44100 );
            printf( "\nEnd Recording.\n" );
            if( ret == paSuccess )
            {
                printf( "\nRecording Success.\n" );
            }
        }
    }

    if( paAudioSeriesEntryPoints.hInstance_ != NULL )
    {
        paAudioSeriesEntryPoints.paudioPlayRec = NULL;
        paAudioSeriesEntryPoints.paudioDeviceInfo = NULL;

        FreeLibrary( paAudioSeriesEntryPoints.hInstance_ );
        paAudioSeriesEntryPoints.hInstance_ = NULL;
    }

	return 0;
}


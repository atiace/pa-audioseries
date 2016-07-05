/*
paaudioseries
  A function for playback and recording the audio buffer on multi-channel hardware.
  Jack Lee, CVTE CRI, Guangzhou, China
  lizhongjie@cvte.com
  
  paCallback is the callback function for portaudio
  
  audioPlayRec open the portaudio stream
  
  CHANGES:
  0.1 initial release
*/

#include <windows.h>
#include <stdio.h>

#include "portaudio.h"


// local data types and define
typedef float				SAMPLE;			// format for portaudio - float = 32 bit
#define SAMPLE_FORMAT		paFloat32
#define FRAMES_PER_BUFFER	256

#define PLAYREC_NONE		(0x00)
#define PLAYREC_PLAY		(0x01)
#define PLAYREC_REC			(0x02)
#define PLAYREC_PLAYREC		(PLAYREC_PLAY|PLAYREC_REC)

typedef enum PaStatusCode
{
    paSuccess = 0,

    paBadParameters = -10000,
    paDriverError
} PaStatusCode;

// local struct for holding the audio buffer and info about it. Received by the paCallback
typedef struct
{
    int recMode;			// what we're doing - playback, recording, or both

    SAMPLE * playBuffer;	// playback buffer
    int playBufLen;			// playback buffer length
    int playBufPos;			// playback current pos of buffer
    int playFirstChannel;	// first playback channel
    int playChannels;		// playback channels
    int playDevChannels;	// playback device output channels

    SAMPLE * recBuffer;		// recording buffer
    int recBufLen;			// recording buffer length
    int recBufWritePos;		// recording current write pos of buffer
    int recBufReadPos;		// recording current read pos of buffer
    int recFirstChannel;	// first recording channel
    int recChannels;		// recording channels
    int recDevChannels;		// recording device input channels
} paAudioData;

typedef bool (*paReturnFun)( SAMPLE *, int, int );


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

// ring buffer position operate
static int nextWritePos( paAudioData * paudioData )
{
    int nextPos = paudioData->recBufWritePos + 1;
    if( nextPos >= paudioData->recBufLen )
        nextPos -= paudioData->recBufLen;		// ring buffer
    if( nextPos == paudioData->recBufReadPos )	// recording buffer is full
        return -1;
    paudioData->recBufWritePos = nextPos;
    return 0;
}

static int nextReadPos( paAudioData * paudioData )
{
    if( paudioData->recBufReadPos != paudioData->recBufWritePos )
    {
        paudioData->recBufReadPos += 1;
        if( paudioData->recBufReadPos >= paudioData->recBufLen )
            paudioData->recBufReadPos -= paudioData->recBufLen;
    }
    if( paudioData->recBufReadPos == paudioData->recBufWritePos )	// recording buffer is empty
        return -1;
    return 0;
}

static int getRecDataLen( paAudioData * paudio )
{
    if( paudio->recBufWritePos >= paudio->recBufReadPos )
        return paudio->recBufWritePos - paudio->recBufReadPos;
    else
        return paudio->recBufLen + paudio->recBufWritePos - paudio->recBufReadPos;
}

// This routine will be called by the PortAudio engine when audio is needed.
// It may be called at interrupt level on some machines so don't do anything
// that could mess up the system like calling malloc() or free().
static int paCallback( const void * inputBuffer, void * outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo * timeInfo,
                        PaStreamCallbackFlags statusFlags,
                        void * userData )
{
    paAudioData * paudioData = (paAudioData *)userData;

    SAMPLE * pIn = (SAMPLE *)inputBuffer;
    SAMPLE * pOut = (SAMPLE *)outputBuffer;

    (void) timeInfo;	// Prevent unused variable warnings.
    (void) statusFlags;

    if( paudioData->recMode & PLAYREC_PLAY && pOut != NULL )
    {
        // frame
        for( unsigned long index = 0; index < framesPerBuffer; index++ )
        {
            // channel
            for( int channel = 0; channel < paudioData->playDevChannels; channel++ )
            {
                if( paudioData->playFirstChannel <= channel && channel < paudioData->playFirstChannel + paudioData->playChannels )	// in the used channel range
                {
                    if( paudioData->playBufPos < paudioData->playBufLen )
                    {
                        *(pOut + paudioData->playDevChannels * index + channel) = *(paudioData->playBuffer + paudioData->playChannels * paudioData->playBufPos + (channel - paudioData->playFirstChannel));
                    }
                    else
                        *(pOut + paudioData->playDevChannels * index + channel) = 0;	// over buffer
                }
                else
                    *(pOut + paudioData->playDevChannels * index + channel) = 0;		// unused channel
            }
            paudioData->playBufPos++;
        }

        if( paudioData->playBufPos >= paudioData->playBufLen )	// playback buffer is empty
            paudioData->recMode &= ~PLAYREC_PLAY;
    }

    if( paudioData->recMode & PLAYREC_REC && pIn != NULL )
    {
        // frame
        for( unsigned long index = 0; index < framesPerBuffer; index++ )
        {
            // channel
            for( int channel = paudioData->recFirstChannel; channel < paudioData->recFirstChannel + paudioData->recChannels; channel++ )
            {
                if( channel < paudioData->recDevChannels )	// in the device input channel range
                {
                    *(paudioData->recBuffer + paudioData->recChannels * paudioData->recBufWritePos + (channel - paudioData->recFirstChannel)) = *(pIn + paudioData->recDevChannels * index + channel);
                }
                else
                    *(paudioData->recBuffer + paudioData->recChannels * paudioData->recBufWritePos + (channel - paudioData->recFirstChannel)) = 0;	// over the device input channel
            }

            if( nextWritePos( paudioData ) == -1 )	// buffer is full
            {
                paudioData->recMode &= ~PLAYREC_REC;
                break;
            }
        }
    }

    if( paudioData->recMode == PLAYREC_NONE )
        return paComplete;

    return paContinue;
}

int audioPlayRec( int playdevID, SAMPLE * playbuffer, int playbuflen, int playbuffirstchannel, int playbuflastchannel, int recdevID, SAMPLE * recbuffer, int recbuflen, int recbuffirstchannel, int recbuflastchannel, paReturnFun recCallback, int samplerate )
{
    paAudioData audio;
    SAMPLE * precRingArray = NULL;

    audio.recMode = PLAYREC_NONE;

    audio.playBuffer = NULL;
    audio.playBufLen = 0;
    audio.playBufPos = 0;
    audio.playFirstChannel = 0;
    audio.playChannels = 0;
    audio.playDevChannels = 0;

    audio.recBuffer = NULL;
    audio.recBufLen = 0;
    audio.recBufWritePos = 0;
    audio.recBufReadPos = 0;
    audio.recFirstChannel = 0;
    audio.recChannels = 0;
    audio.recDevChannels = 0;

    // playback
    if( playdevID != paNoDevice && playbuffer != NULL && playbuflen > 0 && playbuffirstchannel >= 0 && playbuflastchannel >= playbuffirstchannel )
    {
        int playChannels = playbuflastchannel - playbuffirstchannel + 1;

        audio.recMode |= PLAYREC_PLAY;

        audio.playBuffer = playbuffer;
        audio.playBufLen = playbuflen;
        audio.playFirstChannel = playbuffirstchannel;
        audio.playChannels = playChannels;
    }

	// recording
    if( recdevID != paNoDevice && playbuffirstchannel >= 0 && recbuflastchannel >= recbuffirstchannel )
    {
        int recChannels = recbuflastchannel - recbuffirstchannel + 1;

        if( recbuffer != NULL && recbuflen > 0 )			// recording one times
        {
            audio.recMode |= PLAYREC_REC;

            audio.recBuffer = recbuffer;
            audio.recBufLen = recbuflen;
            audio.recFirstChannel = recbuffirstchannel;
            audio.recChannels = recChannels;
        }
        else if( recCallback != NULL )						// recording series
        {
            int recBufferLength = samplerate * 2;			// 2 seconds ring buffer

            precRingArray = (SAMPLE *)paAllocateMemory( sizeof(SAMPLE) * recBufferLength * recChannels );
            if( precRingArray != NULL )
            {
                audio.recMode |= PLAYREC_REC;

                audio.recBuffer = precRingArray;
                audio.recBufLen = recBufferLength;
                audio.recFirstChannel = recbuffirstchannel;
                audio.recChannels = recChannels;
            }
        }
    }

    if( audio.recMode == PLAYREC_NONE )
    {
        return paBadParameters;
    }

    PaStreamParameters inputParameters, outputParameters;
    PaStreamParameters *pinputParameters, *poutputParameters;
    PaStream *stream;
    PaError err;

    err = Pa_Initialize();
    if( err != paNoError )
        goto error;

    if( audio.recMode & PLAYREC_PLAY )
    {
        poutputParameters = &outputParameters;

        outputParameters.device = playdevID;
        outputParameters.channelCount = Pa_GetDeviceInfo( outputParameters.device )->maxOutputChannels;
        outputParameters.sampleFormat = SAMPLE_FORMAT;
        outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
        outputParameters.hostApiSpecificStreamInfo = NULL;

        audio.playDevChannels = outputParameters.channelCount;
    }
    else
        poutputParameters = NULL;

    if( audio.recMode & PLAYREC_REC )
    {
        pinputParameters = &inputParameters;

        inputParameters.device = recdevID;
        inputParameters.channelCount = Pa_GetDeviceInfo( inputParameters.device )->maxInputChannels;
        inputParameters.sampleFormat = SAMPLE_FORMAT;
        inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
        inputParameters.hostApiSpecificStreamInfo = NULL;

        audio.recDevChannels = inputParameters.channelCount;
    }
    else
        pinputParameters = NULL;

    err = Pa_OpenStream( &stream,
                        pinputParameters,
                        poutputParameters,
                        samplerate,
                        FRAMES_PER_BUFFER,
                        paNoFlag,
                        paCallback,
                        &audio );
    if( err != paNoError )
        goto error;

    err = Pa_StartStream( stream );
    if( err != paNoError )
        goto error;

    while( ( err = Pa_IsStreamActive( stream ) ) == 1 )
    {
        if( recCallback != NULL )
        {
            bool stop = false;
            SAMPLE * pdataArray = (SAMPLE *)paAllocateMemory( sizeof(SAMPLE) * FRAMES_PER_BUFFER * audio.recChannels );

            if( pdataArray != NULL )
            {
                while( getRecDataLen( &audio ) >= FRAMES_PER_BUFFER )	// recording data is more than FRAMES_PER_BUFFER
                {
                    for( int index = 0; index < FRAMES_PER_BUFFER; index++ )	// frame
                    {
                        for( int channel = 0; channel < audio.recChannels; channel++ )	// channel
                        {
                            *(pdataArray + audio.recChannels * index + channel) = *(audio.recBuffer + audio.recChannels * audio.recBufReadPos + channel);
                        }
                        if( nextReadPos( &audio ) == -1 )
                            break;
                    }

                    if( recCallback( pdataArray, FRAMES_PER_BUFFER, audio.recChannels ) == paComplete )
                    {
                        stop = true;
                        break;
                    }
                }
            }
            else
            {
                stop = true;
            }

            if( pdataArray != NULL )
                paFreeMemory( pdataArray );

            if( stop )	break;	// recording stop

            Pa_Sleep( (FRAMES_PER_BUFFER * 1000) / samplerate );
        }
        else
            Pa_Sleep( 50 );
    }
    if( err < 0 )
        goto error;

    err = Pa_StopStream( stream );
    if( err != paNoError )
        goto error;

    err = Pa_CloseStream( stream );
    if( err != paNoError )
        goto error;

    Pa_Terminate();

    if( precRingArray != NULL )
        paFreeMemory( precRingArray );

    return paSuccess;

error:
    Pa_Terminate();

    if( precRingArray != NULL )
        paFreeMemory( precRingArray );

    return paDriverError;
}


/* below is for device list */

// local data types and define
#define NUM_PER_DEVICE		256
#define NUM_FOR_DEVICE		4096

#ifdef _MSC_VER
#define snprintf	_snprintf
#define vsnprintf	_vsnprintf
#endif

static int audioDeviceSupportedSampleRates( PaStreamParameters * inputParameters, PaStreamParameters * outputParameters, char * pBuf, int bufLen )
{
    static double standardSampleRates[] = {
        8000.0, 9600.0, 11025.0, 12000.0, 16000.0, 22050.0, 24000.0,
        32000.0, 44100.0, 48000.0, 88200.0, 96000.0, 192000.0, -1	// negative terminated list
    };
    static PaSampleFormat sampleFormats[] = { paInt16, paInt24, paInt32, paFloat32, 0 };
    static char sampleFormatNames[][10] = { {"Int16"}, {"Int24"}, {"Int32"}, {"Float32"}, {""} };
    PaError err;
    int strLen = 0;

    if( inputParameters != NULL || outputParameters != NULL )
    {
        for( int format = 0; sampleFormats[format] > 0; format++ )
        {
            strLen += snprintf( &pBuf[strLen], bufLen - strLen, "Format: %s", sampleFormatNames[format] );

            if( inputParameters != NULL)
                inputParameters->sampleFormat = sampleFormats[format];
            if( outputParameters != NULL)
                outputParameters->sampleFormat = sampleFormats[format];

            int count = 0;
            for( int sample = 0; standardSampleRates[sample] > 0; sample++ )
            {
                err = Pa_IsFormatSupported( inputParameters, outputParameters, standardSampleRates[sample] );
                if( err == paFormatIsSupported )
                {
                    if( count++ == 0)
                        strLen += snprintf( &pBuf[strLen], bufLen - strLen, "\n\t%8.2f", standardSampleRates[sample] );
                    else
                        strLen += snprintf( &pBuf[strLen], bufLen - strLen, ", %8.2f", standardSampleRates[sample] );
                }
            }

            if( !count )
                strLen += snprintf( &pBuf[strLen], bufLen - strLen, "\n\tNone\n" );
            else
                strLen += snprintf( &pBuf[strLen], bufLen - strLen, "\n" );
		}
    }

    return strLen;
}

static int audioDeviceDetailInfo( int deviceID, char * pBuf, int bufLen )
{
    const PaDeviceInfo * pdi;
    int strLen = 0;

    pdi = Pa_GetDeviceInfo( deviceID );

    if( pdi != NULL )
    {
        strLen += snprintf( &pBuf[strLen], bufLen - strLen,
                            "Device %d name: %s\n"
                            "Host API name: %s\n"
                            "Max input channels: %d, Max output channels: %d\n"
                            "Default input latency:  %8.4f(low) - %8.4f(high)\n"
                            "Default output latency: %8.4f(low) - %8.4f(high)\n"
                            "Default sample rate: %8.2f\n",
                            deviceID, pdi->name,
                            Pa_GetHostApiInfo( pdi->hostApi )->name,
                            pdi->maxInputChannels, pdi->maxOutputChannels,
                            pdi->defaultLowInputLatency, pdi->defaultHighInputLatency,
                            pdi->defaultLowOutputLatency, pdi->defaultHighOutputLatency,
                            pdi->defaultSampleRate );

        if( true )	// Print supported standard sample rates
        {
            PaStreamParameters inputParameters, outputParameters;

            inputParameters.device = deviceID;
            inputParameters.channelCount = pdi->maxInputChannels;
            inputParameters.sampleFormat = paInt16;					// default only
            inputParameters.suggestedLatency = 0;
            inputParameters.hostApiSpecificStreamInfo = NULL;

            outputParameters.device = deviceID;
            outputParameters.channelCount = pdi->maxOutputChannels;
            outputParameters.sampleFormat = paInt16;				// default only
            outputParameters.suggestedLatency = 0;
            outputParameters.hostApiSpecificStreamInfo = NULL;

            if( inputParameters.channelCount > 0 )
            {
                strLen += snprintf( &pBuf[strLen], bufLen - strLen, "Supported standard sample rates input.\n" );
                strLen += audioDeviceSupportedSampleRates( &inputParameters, NULL, &pBuf[strLen], bufLen - strLen );
            }

            if( outputParameters.channelCount > 0 )
            {
                strLen += snprintf( &pBuf[strLen], bufLen - strLen, "Supported standard sample rates output.\n" );
                strLen += audioDeviceSupportedSampleRates( NULL, &outputParameters, &pBuf[strLen], bufLen - strLen );
            }

            if( inputParameters.channelCount > 0 && outputParameters.channelCount > 0 )
            {
                strLen += snprintf( &pBuf[strLen], bufLen - strLen, "Supported standard sample rates input/output.\n" );
                strLen += audioDeviceSupportedSampleRates( &inputParameters, &outputParameters, &pBuf[strLen], bufLen - strLen );
            }
        }
    }

    return strLen;
}

static int audioDeviceListInfo( int deviceID, char * pBuf, int bufLen )
{
    const PaDeviceInfo * pdi;
    int strLen = 0;

    pdi = Pa_GetDeviceInfo( deviceID );

    if( pdi != NULL )
    {
        strLen += snprintf( &pBuf[strLen], bufLen - strLen,
                            "%02d. %s \t- Protocol: %s, Max input channels: %d, Max output channels: %d\n",
                            deviceID, pdi->name, Pa_GetHostApiInfo( pdi->hostApi )->name, pdi->maxInputChannels, pdi->maxOutputChannels );
    }

    return strLen;
}

int audioDeviceInfo( int deviceID, char * pBuf, int length )
{
    PaError err;
    int numDevices;
    char * pReturn = NULL;
    int strLen = 0;

    err = Pa_Initialize();
    if( err != paNoError )
        goto error;

    numDevices = Pa_GetDeviceCount();
    if( numDevices < 0 )
        goto error;

    if( deviceID == -1 )	// list device
    {
        int bufLen = NUM_PER_DEVICE * numDevices;

        pReturn = (char *)paAllocateMemory( sizeof(char) * bufLen );
        memset( pReturn, 0, sizeof(char) * bufLen );

        for( int i = 0; i < numDevices; i++ )
        {
            strLen += audioDeviceListInfo( i, &pReturn[strLen], bufLen - strLen );
        }
    }
    else if( deviceID >= 0 && deviceID < numDevices )	// device informaation
    {
        int bufLen = NUM_PER_DEVICE * numDevices;

        pReturn = (char *)paAllocateMemory( sizeof(char) * bufLen );
        memset( pReturn, 0, sizeof(char) * bufLen );

        strLen += audioDeviceDetailInfo( deviceID, &pReturn[strLen], bufLen - strLen );
    }
    else
        goto error;

    Pa_Terminate();

    memcpy( pBuf, pReturn, strLen > length ? length : strLen );

    if( pReturn != NULL )
        paFreeMemory( pReturn );

    return paSuccess;

error:
    Pa_Terminate();

    if( pReturn != NULL )
        paFreeMemory( pReturn );

    return paDriverError;
}


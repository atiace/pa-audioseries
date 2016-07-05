/*
paaudioseries
  A matlab function for playback and recording the audio buffer on multi-channel hardware.
  Jack Lee, CVTE CRI, Guangzhou, China
  lizhongjie@cvte.com
  
  mexFunction is the entry point from Matlab - parses arguments and converts input
  
  paCallback is the callback function for portaudio
  
  audioPlayRec open the portaudio stream
  
  CHANGES:
  0.1 initial release
*/

#include <math.h>
#include <string.h>

#include "mex.h"
#include "portaudio.h"


#define messagePrintf( x )		mexPrintf x 
#define errorPrintf( x )		mexPrintf x 
#define debugPrintf( x )


////////// local data types
typedef float				SAMPLE;			// format for portaudio - float = 32 bit
#define SAMPLE_FORMAT		paFloat32
#define SAMPLE_CLASSID		mxSINGLE_CLASS	// mxINT8_CLASS/mxUINT8_CLASS/mxINT16_CLASS/mxUINT16_CLASS/mxINT32_CLASS/mxUINT32_CLASS/mxINT64_CLASS/mxUINT64_CLASS/mxSINGLE_CLASS/mxDOUBLE_CLASS
#define FRAMES_PER_BUFFER	256

#define PLAYREC_NONE		(0x00)
#define PLAYREC_PLAY		(0x01)
#define PLAYREC_REC			(0x02)
#define PLAYREC_PLAYREC		(PLAYREC_PLAY|PLAYREC_REC)

// local struct for holding the audio buffer and info about it. Received by the paCallback
typedef struct
{
    int recMode;			// what we're doing - playback, recording, or both

    SAMPLE *playBuffer;		// playback buffer
    int playBufLen;			// playback buffer length
    int playBufPos;			// playback current pos of buffer
    int playFirstChannel;	// first playback channel
    int playChannels;		// playback channels
    int playDevChannels;	// playback device output channels

    SAMPLE *recBuffer;		// recording buffer
    int recBufLen;			// recording buffer length
    int recBufWritePos;		// recording current write pos of buffer
    int recBufReadPos;		// recording current read pos of buffer
    int recFirstChannel;	// first recording channel
    int recChannels;		// recording channels
    int recDevChannels;		// recording device input channels
}
paAudioData;

////////// function prototypes
void convDoubleToSAMPLE( double * srcBuf, SAMPLE * desBuf, int bufLen );
int audioPlayRec( int playdevID, SAMPLE * playbuffer, int playbuflen, int playbuffirstchannel, int playbuflastchannel, int recdevID, SAMPLE * recbuffer, int recbuflen, int recbuffirstchannel, int recbuflastchannel, char * recCallbackName, int samplerate );
void printDevicesInfo( void );

// entry point from Matlab.
// parse arguments, convert buffer to SAMPLE, then call audioPlayRec with parsed args.
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    double * doubleptr = NULL;
    bool playback = false, recording = false, seriesRecording = false;
    int playDevId = -1, recDevId = -1, sampleRate = -1, recSamples = -1, recFirstChannel = -1, recLastChannel = -1;
    SAMPLE * playBuffer = NULL;
    SAMPLE * recBuffer = NULL;
    char * recCallbackName = NULL;
    int playBufRows = -1, playBufCols = -1, recBufRows = -1, recBufCols = -1;

    // Check for proper number of arguments
    if( nrhs == 0 )
    {
        printDevicesInfo();
        return;
    }
    else if( nrhs != 7 )
        goto error;

    // get playback device
    if( mxIsChar( prhs[0] ) )
        goto error;
    doubleptr = mxGetPr( prhs[0] );
    if( doubleptr != NULL )
        playDevId = (int)*doubleptr;
    if( playDevId >= 0 )
        playback = true;

    // get recording device
    if( mxIsChar( prhs[3] ) )
        goto error;
    doubleptr = mxGetPr( prhs[3] );
    if( doubleptr != NULL )
        recDevId = (int)*doubleptr;
    if( recDevId >= 0 )
        recording = true;

    if( !playback && !recording )
    {
        errorPrintf( ( "ERROR: playDevID < 0 and recDevID < 0. Nothing to be done.\n" ) );
        goto error;
    }

    // get the playback buffer
    if( playback )
    {
        if( mxIsComplex( prhs[1] ) )
        {
            errorPrintf( ( "ERROR: playBuffer must be noncomplex.\n" ) );
            goto error;
        }
        playBufRows = (int)mxGetM( prhs[1] );	// get rows of the playback buffer		- samples
        playBufCols = (int)mxGetN( prhs[1] );	// get columns of the playback buffer	- channels
        if( mxIsSingle( prhs[1] ) )
        {
            SAMPLE * bufptr = (SAMPLE *)mxGetData( prhs[1] );
            if( bufptr == NULL )
            {
                errorPrintf( ( "ERROR: playBuffer is NULL.\n" ) );
                goto error;
            }
            playBuffer = bufptr;
        }
        else if( mxIsDouble( prhs[1] ) )
        {
            double * bufptr = mxGetPr( prhs[1] );
            if( bufptr == NULL )
            {
                errorPrintf( ( "ERROR: playBuffer is NULL.\n" ) );
                goto error;
            }
            mxArray * pplayArray = mxCreateNumericMatrix( playBufRows, playBufCols, SAMPLE_CLASSID, mxREAL );
            SAMPLE * desBuf = (SAMPLE *)mxGetData( pplayArray );
            if( desBuf == NULL )
            {
                errorPrintf( ( "ERROR: playBuffer is count not created.\n" ) );
                goto error;
            }
            convDoubleToSAMPLE( bufptr, desBuf, playBufRows * playBufCols );
            playBuffer = desBuf;
        }
        else
        {
            errorPrintf( ( "ERROR: playBuffer is an invalid data type.\n" ) );
            goto error;
        }
    }

    // get samplerate
    if( mxIsChar( prhs[2] ) )
        goto error;
    doubleptr = mxGetPr( prhs[2] );
    if( doubleptr != NULL )
        sampleRate = (int)*doubleptr;
    if( sampleRate <= 0 )
    {
        errorPrintf( ("ERROR: sampleRate is error.\n") );
        goto error;
    }

    // get recording parameters
    if( recording )
    {
        if( mxIsChar( prhs[4] ) )
        {
            // get recording callback function name
            seriesRecording = true;
            recCallbackName = mxArrayToString( prhs[4] );
            if( recCallbackName == NULL )
            {
                errorPrintf( ("ERROR: recCallbackName is error.\n") );
                goto error;
            }
        }
        else
        {
            // get recording samples
            doubleptr = mxGetPr( prhs[4] );
            if( doubleptr != NULL )
                recSamples = (int)*doubleptr;
            if( recSamples <= 0 )
            {
                errorPrintf( ("ERROR: recSamples is error.\n") );
                goto error;
            }
        }

        // get number of recording channels
        if( mxIsChar( prhs[5] ) )
            goto error;
        doubleptr = mxGetPr( prhs[5] );
        if( doubleptr != NULL )
            recFirstChannel = (int)*doubleptr;
        if( recFirstChannel < 0 )
        {
            errorPrintf( ("ERROR: recFirstChannel is error.\n") );
            goto error;
        }
        if( mxIsChar( prhs[6] ) )
            goto error;
        doubleptr = mxGetPr( prhs[6] );
        if( doubleptr != NULL )
            recLastChannel = (int)*doubleptr;
        if( recLastChannel < 0 )
        {
            errorPrintf( ("ERROR: recLastChannel is error.\n") );
            goto error;
        }
        else if( recLastChannel < recFirstChannel )
            recLastChannel = recFirstChannel;

        if( !seriesRecording )
        {
            // calc others parameters
            recBufRows = recSamples;
            recBufCols = recLastChannel - recFirstChannel + 1;
            ////// allocate memory for the output matrix
            plhs[0] = mxCreateNumericMatrix( recBufRows, recBufCols, SAMPLE_CLASSID, mxREAL );
            recBuffer = (SAMPLE *)mxGetData( plhs[0] );
        }
    }

    if( !playback )
    {
        playDevId = paNoDevice;
        playBufRows = playBufCols = 0;
    }

    if( !recording )
    {
        recDevId = paNoDevice;
        recBufRows = recBufCols = 0;
    }
    else if( seriesRecording )
    {
        recBufRows = recBufCols = 0;
    }

#if 0	// just for debug only
    messagePrintf( ( "\nplaydevID\t: %d"
                    "\nplaybuffer\t: 0x%X"
                    "\nplaybuflen\t: %d"
                    "\nplaybuffirstchannel\t: %d"
                    "\nplaybuflastchannel\t: %d"
                    "\nrecdevID\t: %d"
                    "\nrecbuffer\t: 0x%X"
                    "\nrecbuflen\t: %d"
                    "\nrecbuffirstchannel\t: %d"
                    "\nrecbuflastchannel\t: %d"
                    "\nrecCallbackName\t: %s"
                    "\nsamplerate\t: %d\n",
                    playDevId, playBuffer, playBufRows * playBufCols, 0, playBufCols - 1 - 0,
                    recDevId, recBuffer, recBufRows * recBufCols, recFirstChannel, recLastChannel, recCallbackName, sampleRate ) );
#else
    audioPlayRec(playDevId, playBuffer, playBufRows * playBufCols, 0, playBufCols - 1 - 0,
                    recDevId, recBuffer, recBufRows * recBufCols, recFirstChannel, recLastChannel, recCallbackName, sampleRate);
#endif

    return;

error:

    messagePrintf( ( "\nUsage: \n"
                    "\t1. recBuffer = paaudioseries( playDevID, playBuffer, sampleRate, recDevID, recSamples, recFirstChannel, recLastChannel );\n"
                    "\t2. paaudioseries( playDevID, playBuffer, sampleRate, recDevID, recCallbackName, recFirstChannel, recLastChannel );\n"
                    "\t3. paaudioseries( );\n"
                    "\t- playDevID	is the playback device ID, list by paaudioseries( ).\n"
                    "\t- playBuffer is the playback audio data buffer, MATLAB float matrix array.\n"
                    "\t- sampleRate is the playback and recording sampling rate.\n"
                    "\t- recDevID	is the recording device ID, list by paaudioseries( ).\n"
                    "\t- recSamples is the recording length, sampleRate x time.\n"
                    "\t- recCallbackName is the series recording data callback function name.\n"
                    "\t- recFirstChannel is the recording start channel, start by 0.\n"
                    "\t- recLastChannel  is the recording end channel, start by recFirstChannel.\n" ) );

    return;
}

void convDoubleToSAMPLE( double * srcBuf, SAMPLE * desBuf, int bufLen )
{
    for( int i = 0; i < bufLen; i++ )
        desBuf[i] = (SAMPLE)srcBuf[i];
}

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
                        *(pOut + paudioData->playDevChannels * index + channel) = *(paudioData->playBuffer + paudioData->playBufLen * (channel - paudioData->playFirstChannel) + paudioData->playBufPos);
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
                    *(paudioData->recBuffer + paudioData->recBufLen * (channel - paudioData->recFirstChannel) + paudioData->recBufWritePos) = *(pIn + paudioData->recDevChannels * index + channel);
                }
                else
                    *(paudioData->recBuffer + paudioData->recBufLen * (channel - paudioData->recFirstChannel) + paudioData->recBufWritePos) = 0;	// over the device input channel
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

int audioPlayRec( int playdevID, SAMPLE * playbuffer, int playbuflen, int playbuffirstchannel, int playbuflastchannel, int recdevID, SAMPLE * recbuffer, int recbuflen, int recbuffirstchannel, int recbuflastchannel, char * recCallbackName, int samplerate )
{
    paAudioData audio;
    mxArray * precRingArray = NULL;

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

    if( playdevID != paNoDevice && playbuffer != NULL && playbuflen > 0 && playbuffirstchannel >= 0 && playbuflastchannel >= playbuffirstchannel )
    {
        int playChannels = playbuflastchannel - playbuffirstchannel + 1;

        audio.recMode |= PLAYREC_PLAY;

        audio.playBuffer = playbuffer;
        audio.playBufLen = playbuflen / playChannels;		// just only one channel samples
        audio.playFirstChannel = playbuffirstchannel;
        audio.playChannels = playChannels;
    }

    if( recdevID != paNoDevice && playbuffirstchannel >= 0 && recbuflastchannel >= recbuffirstchannel )
    {
        int recChannels = recbuflastchannel - recbuffirstchannel + 1;

        if( recbuffer != NULL && recbuflen > 0 )
        {
            audio.recMode |= PLAYREC_REC;

            audio.recBuffer = recbuffer;
            audio.recBufLen = recbuflen / recChannels;		// just only one channel samples
            audio.recFirstChannel = recbuffirstchannel;
            audio.recChannels = recChannels;
        }
        else if( recCallbackName != NULL )
        {
            int recBufferLength = samplerate * 2;			// 2 seconds ring buffer

            precRingArray = mxCreateNumericMatrix( recBufferLength, recChannels, SAMPLE_CLASSID, mxREAL );
            SAMPLE * recBuf = (SAMPLE *)mxGetData( precRingArray );
            if( recBuf != NULL )
            {
                audio.recMode |= PLAYREC_REC;

                audio.recBuffer = recBuf;
                audio.recBufLen = recBufferLength;			// just only one channel samples
                audio.recFirstChannel = recbuffirstchannel;
                audio.recChannels = recChannels;
            }
        }
    }

    if( audio.recMode == PLAYREC_NONE )
    {
        errorPrintf( ( "Playback and Recording parameters is wrong!\n" ) );
        return paNoError;
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
        if( recCallbackName != NULL )
        {
            bool stop = false;

            while( getRecDataLen( &audio ) >= FRAMES_PER_BUFFER )	// recording data is more than FRAMES_PER_BUFFER
            {
                mxArray * pdataArray = mxCreateNumericMatrix( FRAMES_PER_BUFFER, audio.recChannels, SAMPLE_CLASSID, mxREAL );
                mxArray * presultArray = NULL;

                SAMPLE * dataBuf = (SAMPLE *)mxGetData( pdataArray );
                if( dataBuf != NULL )
                {
                    for( int index = 0; index < FRAMES_PER_BUFFER; index++ )	// frame
                    {
                        for( int channel = 0; channel < audio.recChannels; channel++ )	// channel
                        {
                            *(dataBuf + FRAMES_PER_BUFFER * channel + index) = *(audio.recBuffer + audio.recBufLen * channel + audio.recBufReadPos);
                        }
                        if( nextReadPos( &audio ) == -1 )
                            break;
                    }

                    mexCallMATLAB( 1, &presultArray, 1, &pdataArray, recCallbackName );

                    if( !mxIsDouble( presultArray ) || mxGetPr( presultArray ) == NULL || ((int)*mxGetPr( presultArray )) != 0 )	// return 0 should be continue, others should be stop
                        stop = true;
                }

                if( presultArray != NULL )
                    mxDestroyArray( presultArray );
                if( pdataArray != NULL )
                    mxDestroyArray( pdataArray );

                if( stop )	break;	// recording stop
            }

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
        mxDestroyArray( precRingArray );

    return err;

error:
    Pa_Terminate();

    errorPrintf( ( "Error number: %d\n", err ) );
    errorPrintf( ( "Error message: %s\n", Pa_GetErrorText(err) ) );

    if( precRingArray != NULL )
        mxDestroyArray( precRingArray );

    return err;
}

// print the supported standard sample rates to stdout
static void printSupportedStandardSampleRates( PaStreamParameters *inputParameters, PaStreamParameters *outputParameters )
{
    static double standardSampleRates[] = {
        8000.0, 9600.0, 11025.0, 12000.0, 16000.0, 22050.0, 24000.0,
        32000.0, 44100.0, 48000.0, 88200.0, 96000.0, 192000.0, -1	// negative terminated list
    };
    static PaSampleFormat sampleFormats[] = { /*paInt16, paInt24, paInt32,*/ paFloat32, 0 };
    static char sampleFormatNames[][10] = { /*{"Int16"}, {"Int24"}, {"Int32"},*/ {"Float32"} };
    PaError err;

    if( inputParameters == NULL && outputParameters == NULL )
    {
        errorPrintf( ( "ERROR: inputParameters is NULL & outputParameters is NULL.\n" ) );
        return;
    }

    for( int format = 0; sampleFormats[format] > 0; format++ )
    {
        messagePrintf( ( "Format: %s", sampleFormatNames[format] ) );

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
                    messagePrintf( ( "\n\t%8.2f", standardSampleRates[sample] ) );
                else
                    messagePrintf( ( ", %8.2f", standardSampleRates[sample] ) );
            }
        }

        if( !count )
            messagePrintf( ( "\n\tNone\n" ) );
        else
            messagePrintf( ( "\n" ) );
    }
}

// print the audio device to stdout.
static void printDeviceInfo( int device )
{
    const PaDeviceInfo *pdi;
    bool defaultDisplayed = false;

    pdi = Pa_GetDeviceInfo( device );

    if( pdi != NULL )
    {
        messagePrintf( ( "\n" ) );

        if( device == Pa_GetDefaultInputDevice() )
        {
            messagePrintf( ( "[ Default Input" ) );
            defaultDisplayed = true;
        }
        else if( device == Pa_GetHostApiInfo( pdi->hostApi )->defaultInputDevice )
        {
            const PaHostApiInfo *phai = Pa_GetHostApiInfo( pdi->hostApi );
            messagePrintf( ( "[ Default %s Input", phai->name ) );
            defaultDisplayed = true;
        }

        if( device == Pa_GetDefaultOutputDevice() )
        {
            messagePrintf( ( ( defaultDisplayed ? "," : "[") ) );
            messagePrintf( ( " Default Output" ) );
            defaultDisplayed = true;
        }
        else if( device == Pa_GetHostApiInfo( pdi->hostApi )->defaultOutputDevice )
        {
            const PaHostApiInfo *phai = Pa_GetHostApiInfo( pdi->hostApi );
            messagePrintf( ( ( defaultDisplayed ? "," : "[") ) );
            messagePrintf( ( " Default %s Output", phai->name ) );
            defaultDisplayed = true;
        }

        if( defaultDisplayed )
            messagePrintf( ( " ]\n" ) );

        messagePrintf( ( ""
            "Device %d name: %s\n"
            "Host API name: %s\n"
            "Max input channels: %d, Max output channels: %d\n"
            "Default input latency:  %8.4f(low) - %8.4f(high)\n"
            "Default output latency: %8.4f(low) - %8.4f(high)\n"
            "Default sample rate: %8.2f\n",
            device, pdi->name,
            Pa_GetHostApiInfo( pdi->hostApi )->name,
            pdi->maxInputChannels, pdi->maxOutputChannels,
            pdi->defaultLowInputLatency, pdi->defaultHighInputLatency,
            pdi->defaultLowOutputLatency, pdi->defaultHighOutputLatency,
            pdi->defaultSampleRate ) );

        if( true )	// Print supported standard sample rates
        {
            PaStreamParameters inputParameters, outputParameters;

            inputParameters.device = device;
            inputParameters.channelCount = pdi->maxInputChannels;
            inputParameters.sampleFormat = paInt16;					// default only
            inputParameters.suggestedLatency = 0;
            inputParameters.hostApiSpecificStreamInfo = NULL;

            outputParameters.device = device;
            outputParameters.channelCount = pdi->maxOutputChannels;
            outputParameters.sampleFormat = paInt16;				// default only
            outputParameters.suggestedLatency = 0;
            outputParameters.hostApiSpecificStreamInfo = NULL;

            if( inputParameters.channelCount > 0 )
            {
                messagePrintf( ( "Supported standard sample rates input.\n" ) );
                printSupportedStandardSampleRates( &inputParameters, NULL );
            }

            if( outputParameters.channelCount > 0 )
            {
                messagePrintf( ( "Supported standard sample rates output.\n" ) );
                printSupportedStandardSampleRates( NULL, &outputParameters );
            }

            if( inputParameters.channelCount > 0 && outputParameters.channelCount > 0 )
            {
                messagePrintf( ( "Supported standard sample rates input/output.\n" ) );
                printSupportedStandardSampleRates( &inputParameters, &outputParameters );
            }
        }
    }
}

// print info about the all audio devices to stdout.
void printDevicesInfo( void )
{
    PaError err;
    int numDevices;

    messagePrintf( ( "Printing audio devices information...\n" ) );

    err = Pa_Initialize();
    if( err != paNoError )
    {
        errorPrintf( ( "ERROR: Pa_Initialize returned 0x%x.\n", err ) );
        goto error;
    }

    messagePrintf( ( "PortAudio version: 0X%08X\n", Pa_GetVersion() ) );
    messagePrintf( ( "Version text: '%s'\n", Pa_GetVersionText() ) );

    numDevices = Pa_GetDeviceCount();
    if( numDevices < 0 )
    {
        errorPrintf( ( "ERROR: Pa_GetDeviceCount returned 0x%x.\n", numDevices ) );
        goto error;
    }

    messagePrintf( ( "Number of devices = %d\n", numDevices ) );

    for( int i = 0; i < numDevices; i++ )
        printDeviceInfo( i );

    Pa_Terminate();
    return;

error:
    Pa_Terminate();
    errorPrintf( ( "Error number: %d\n", err ) );
    errorPrintf( ( "Error message: %s\n", Pa_GetErrorText(err) ) );
}


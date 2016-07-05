/*
paaudioseries
  A function for playback and recording the audio buffer on multi-channel hardware.
  Jack Lee, CVTE CRI, Guangzhou, China
  lizhongjie@cvte.com
  
  CHANGES:
  0.1 initial release
*/

typedef enum PaStatusCode
{
    paSuccess = 0,

    paBadParameters = -10000,
    paDriverError
} PaStatusCode;

typedef enum PaReturnResult
{
    paContinue=0,
    paComplete=1
} PaReturnResult;

/**************************************************************************
 * callback function for series recording to return a packet
 * parameters:
 * float *  - the packet date buffer point
 * int      - recording samples of a channel
 * int      - recording channels
 * return:
 * bool     - PaReturnResult
**************************************************************************/
typedef bool (*paReturnFun)( float *, int, int );

/**************************************************************************
 * playback, recording or both multi channel with multi protocol audio device
 * recbuffer and recbuflen is used for fixed length recording, it should be set to NULL and 0 for series recording.
 * recCallback is used for series recording, it should be set to NULL for fixed length recording.
 * parameters:
 * int playdevID            - playback device ID
 * float * playbuffer       - playback buffer
 * int playbuflen           - playback samples of a channel
 * int playbuffirstchannel  - playback start channel number of the device
 * int playbuflastchannel   - playback end channel number of the device
 * int recdevID             - recording device ID
 * float * recbuffer        - recording buffer
 * int recbuflen            - recording samples of a channel
 * int recbuffirstchannel   - recording start channel number of the device
 * int recbuflastchannel    - recording end channel number of the device
 * paReturnFun recCallback  - recording callback funcrion point for series recording
 * int samplerate           - playback and recording sample rate
 * return:
 * int                      - status return, 0 is success.
**************************************************************************/
int audioPlayRec( int playdevID, float * playbuffer, int playbuflen, int playbuffirstchannel, int playbuflastchannel, int recdevID, float * recbuffer, int recbuflen, int recbuffirstchannel, int recbuflastchannel, paReturnFun recCallback, int samplerate );

/**************************************************************************
 * get the audio device list or the device detail information
 * parameters:
 * int deviceID             - device ID, -1 is used for get device list, others is used for get the device detail information
 * char * pBuf              - information buffer
 * int length               - buffer size
 * return:
 * int                      - status return, 0 is success.
**************************************************************************/
int audioDeviceInfo( int deviceID, char * pBuf, int length );


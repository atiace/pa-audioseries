# pa-audioseries

----------

Play and record multi-channel audio with multi-protocol audio device.

This project is a modification of pa-wavplay by Matt Frear and posted on both Source Forge and the Mathworks File Exchange.

The original code:

- played multi-channel audio from Matlab using ASIO, DirectSound, and Windows Audio.
- worked with 32-bit installations of Matlab only on Windows.

This update extends the original to:

- play multi-channel audio from Matlab using Windows MME, DirectSound, ASIO, WASAPI, and Windows WDMKS.
- works with both 32-bit and 64-bit installations of Matlab on Windows.
- support series recording for Matlab signal process and analysis.

The code is built upon the open source PortAudio API and ASIO SDK.

Requirements: Windows Vista or later.

The code directory is below,

>pa-audioseries

>|- asiosdk\_v2\_3

>|- Matlab

>|- paaudioseries

>|- portaudio\_v19\_2014

Matlab - project for Matlab.

paaudioseries - project for standard C\C++ dll.

portaudio\_v19\_2014 - port audio v19 2014.01.30 release edition.

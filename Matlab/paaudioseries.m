function y = paaudioseries(varargin)
%  paaudioseries: simultaneous playback & recording of multichannel sound
%  
%    Usage:
%      1. recBuffer = paaudioseries( playDevID, playBuffer, sampleRate, recDevID, recSamples, recFirstChannel, recLastChannel )
%      2. paaudioseries( playDevID, playBuffer, sampleRate, recDevID, recCallbackName, recFirstChannel, recLastChannel )
%      3. paaudioseries( )
%  
%    paaudioseries is a tool for playback and recording multi-channel audio through your multi device.
%  
%    - playDevID        the playback device ID, list by paaudioseries( ).
%    - playBuffer       the playback audio buffer, MATLAB float/double matrix array.
%    - sampleRate	    the sampling frequency
%    - recDevID         the recording device ID.
%    - recSamples       the recording length, sampleRate * time.
%    - recCallbackName  the callback function name for series recording.
%    - recFirstChannel  the recording start channel, start by 0.
%    - recLastChannel   the recording end channel.
%  
%    Copyright 2016 CVTE Corp.
%    lizhongjie@cvte.com from CVTE CRI.




% [EOF] paaudioseries.m
function y = audioSeriesCallback( recordMatrix )
%  audioSeriesCallback process the recording data packet
%  
%    recordMatrix  - the recording data matrix array.
%    y             - return 0 is continue, 1 is complete.

global recBuf

recBuf = [recBuf; recordMatrix];

[m, n] = size( recBuf );

disp( size( recBuf ) );

if( n > 0 && m <= 44100 * 5 )
    y = 0;
else
    y = 1;

end


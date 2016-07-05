function audioSeriesTest( )
%  audioSeriesTest sample for how to used the paaudioseries function
%  

global recBuf

recBuf = [];

paaudioseries(-1, 0, 44100, 12, 'audioSeriesCallback', 0, 7);

plot( recBuf );

end


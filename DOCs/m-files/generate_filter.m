function generate_filter(f,m,fs)
    b = fir2(63,f,m);
    [H,w] = freqz(b,1,256);
    f = w/pi*fs/2;
    subplot(2,2,1)
    plot(f,abs())
end
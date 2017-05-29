function y = muestrearSin10Hz(Fs)
    fs = 10000;
    Ts = 1/fs;
    t = 0:Ts:1;
    x = sin(2*pi*10*t+0.1);
    y = zeros(1,length(1:int32(fs/Fs):length(x)));
    Y = zeros(size(x));
    j = 1;
    for k=1:int32(fs/Fs):length(x)
        y(j) = x(k);
        Y(k) = x(k);
        j = j+1;
    end
    plot(t,x);
    hold on
    %tnew = Ts.*[0:(fs/Fs):length(x)-1];
    if length(0:(fs/Fs)*Ts:1)<length(y)
        stem(0:(fs/Fs)*Ts:1+Ts,y,'red');
    elseif length(0:(fs/Fs)*Ts:1)>length(y)
        stem(0:(fs/Fs)*Ts:1-Ts,y,'red');
    else
        stem(0:(fs/Fs)*Ts:1,y,'red');
    end
    hold off
    grid on
end
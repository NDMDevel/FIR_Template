function [num,den] = generate_filter(f,m,fs)
    b = fir2(63,f,m);
    [H,w] = freqz(b,1,256);
    f = w/pi*fs/2;
    
    num = b;    % numerador, coincide con h[n] del filtro
    den = 1;    % denominador, siempre 1 porque es FIR
    
    subplot(2,1,1)
    plot(f,abs(H))
    ylabel('Magnitude [Gain]');
    xlabel('Freq [Hz]');
    grid on

    subplot(2,1,2);
    plot(f,unwrap(angle(H)))
    ylabel('Pahse');
    xlabel('Freq [Hz]');
    grid on
    generate_q15_coefs(b);
end
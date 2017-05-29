function [x_q15,h_q15] = q15_test(x,h)
    x_q15 = dec2q15(x,'hex');
    x_q15 = x_q15(:);

    h_q15 = dec2q15(h,'hex');
    h_q15 = h_q15(:);

    
end

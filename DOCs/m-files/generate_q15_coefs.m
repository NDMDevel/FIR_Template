function generate_q15_coefs(h)
    for n=1:length(h)
        fprintf('h[%d] = 0x%s;\n',n-1,dec2q15(h(n),'hex'));
    end
end
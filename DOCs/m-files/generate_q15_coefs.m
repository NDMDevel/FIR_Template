function generate_q15_coefs(h)
    f = fopen('generate_q15_coefs.txt','w');
    for n=1:length(h)
        fprintf(f,'h[%d] = 0x%s;\n',n-1,dec2q15(h(n),'hex'));
    end
    fclose(f);
end
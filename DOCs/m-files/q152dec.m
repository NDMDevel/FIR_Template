%%%%%%%%%%% author: Szymon Leja 2008-01-11

function [y]=q152dec(x,form)

if (form == 'hex')
    x=hex2dec(x);
    x=dec2bin(x,16);
end;

option = {'mode' , 'roundmode', 'overflowmode', 'format'}; 
value   = {'fixed', 'ceil'     , 'saturate'    ,  [16 15]}; 
q = quantizer( option, value );
y = bin2num(q,x);

%%%%%%%%%% or that    
% y=(str2num(x(1)))*(-1);
% for i=2:16
%     y=y+((str2num(x(i)))*2^(-i+1));
% end;


        
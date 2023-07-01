function [pcData2] = adversaryDemo(pcData, advinfo)
% pcData is the original scan of UAV
% pose is the position information of UAV [x, y, theta]
% advinfo is the information of adversary
% including the range and angle [r1 r2 alpha1 alpha2] alpha in degree
r1 = advinfo(1); r2 = advinfo(2); alpha1 = advinfo(3); alpha2 = advinfo(4);
pidata_test = linspace(pi,-pi,1084);
pidata_test = pidata_test';
pidata_test = pidata_test(1:end-1,1);
pidata_test(1,1) = -pidata_test(1,1);

pcangle = pidata_test;
pidata_test(1,1) = -pidata_test(1,1);
pidata_test = [pidata_test;-pi];

num = ceil((alpha2-alpha1)/(360/1084));

label1 = 0; label2 = 0;
for a = 1:length(pidata_test)-1
    if pidata_test(a+1)<(alpha1/180*pi) && (alpha1/180*pi)<=pidata_test(a)
        label1 = a;
    end
end

for b = 1:length(pidata_test)-1
    if pidata_test(b+1)<(alpha2/180*pi) && (alpha2/180*pi)<=pidata_test(b)
        label2 = b;
    end
end
% disp('label1 is:')
% disp(label1)
% disp('label2 is:')
% disp(label2)
pcData_new = pcData.Ranges;
for i = label2: label1
    advdata= r1 + (r2-r1)*rand(1);
    if pcData_new(i,1) >= advdata
        pcData_new(i,1) = advdata;
    end
    if isnan(pcData_new(i,1))
        pcData_new(i,1) = advdata;
    end
end
pcData2 = lidarScan(pcData_new,pcangle);





end
function pcData = pcEst(pose,mapmatrix,maxlidarrange)
% x,y,theta should be the real position
% load mapdata.mat mapmatrix
x = pose(:,1); y = pose(:,2); theta = pose(:,3);
pidata_test = linspace(pi,-pi,1084);
pidata_test = pidata_test';
pidata_test = pidata_test(1:end-1,1);
pidata_test(1,1) = -pidata_test(1,1);

pc = lidarScan(mapmatrix);

% figure(1)
% 
% 
% plot(pc.Cartesian(:,1), pc.Cartesian(:,2),'.','MarkerSize',3,'color','m')


new_pc = transformScan(pc, [-x -y 0]);
new_pc = transformScan(new_pc, [0 0 -theta]);
new_pc = new_pc.removeInvalidData;
range = new_pc.Ranges;
angle = new_pc.Angles;
cart = new_pc.Cartesian;
num = length(range);

pcangle = pidata_test;
pcrange = maxlidarrange*ones(length(pidata_test),1);

pidata_test(1,1) = -pidata_test(1,1);
pidata_test = [pidata_test;-pi];

for i = 1:num   % num = 77463
    
    for j = 1:length(pidata_test)-1
        if pidata_test(j+1)<angle(i) && angle(i)<=pidata_test(j)
            if range(i) <= pcrange(j)
            pcrange(j) = range(i);
            end
        end
    end
end

for i = 1: length(pcrange)
    if pcrange(i) == maxlidarrange
        pcrange(i) = NaN;
    end
end

pcData = lidarScan(pcrange,pcangle);

% % 
% figure(3)
% plot(pcData.Cartesian(:,1),pcData.Cartesian(:,2),'.','MarkerSize',3,'color','m')
% % 
% figure(3)
% plot(new_pc.Cartesian(:,1),new_pc.Cartesian(:,2),'.','MarkerSize',3,'color','m')



end
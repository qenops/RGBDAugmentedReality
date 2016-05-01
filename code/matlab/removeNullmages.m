%path2Dir='F:/arkhalid/Google Drive/ToMapWork/Courses/4th Semester/Virtual Worlds/Project/Sequences/Radial Circular Walk';
path2Dir = '/playpen/tracknet/LivingRoom04';
listInc = [];
imgposes = dlmread('PosesColumnMajor.txt');
for i = 2:size(imgposes,1)
    strColor = sprintf('%s/Color/img_0_%04d.jpg',path2Dir,imgposes(i,1));
    strDepth = sprintf('%s/Depth/img_0_%04d.jpg',path2Dir,imgposes(i,1));
    img = imread(strColor); 
    imgD = imread(strDepth); 
    if((mean(img(:)) <1) ||(mean(imgD(:)) <1))
    delete (strColor);
    delete (strDepth);
    else
        listInc = [listInc,i];
    end
    
end
imgposes =imgposes(listInc,:);
dlmwrite('PosesColumnMajorCorrected.txt',imgposes);
% how to write length(imgposes) as first line of file??
dlmwrite('poses.txt',imgposes);
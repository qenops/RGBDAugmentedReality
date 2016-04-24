function SaveImgPoses( poses,filename )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

opmat = [];
for i = 1:size(poses,3)
    pose =  poses(:,:,i);
    pose = pose(:)';
    opmat = [opmat;i-1,pose];
end

dlmwrite(filename,opmat,' ');
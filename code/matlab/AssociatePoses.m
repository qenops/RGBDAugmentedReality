
clear all
load tcam2H
H2tcam = inv(X);
%%
path2Dir='F:/arkhalid/Google Drive/ToMapWork/Courses/4th Semester/Virtual Worlds/Project/Sequences/Radial Circular Walk';
track = readTSVFile(path2Dir);
trackData = track{1,1};
poseTimings = trackData(:,1);
imgTimingData = track{1,2};
imgTimings = imgTimingData(:,2);
D = pdist2(imgTimings,poseTimings);
imgPosesHiBall = zeros(4,4,length(imgTimings));
errors = zeros(length(imgTimings),1);
for i = 1:size(D,1)
    [M,I] = min(D(i,:));
    imgPosesHiBall(:,:,i) = H2tcam * reshape(trackData(I,2:17),[4,4])';
    errors(i) =I;
end

SaveImgPoses( imgPosesHiBall,'PosesColumnMajor.txt' );
%%

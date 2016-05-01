
clear all
load tcam2H
H2tcam = inv(X);
H2tcam = eye(4);
%%
%path2Dir='F:/arkhalid/Google Drive/ToMapWork/Courses/4th Semester/Virtual Worlds/Project/Sequences/Radial Circular Walk';
path2Dir = '/playpen/tracknet/LivingRoom04';
track = readTSVFile(path2Dir);
trackData = track{1,1};
poseTimings = trackData(:,1);
imgTimingData = dlmread(sprintf('%s/%s',path2Dir,'timeStamps.txt'));
%imgTimingData = track{1,2};
imgTimings = imgTimingData(:,2);
D = pdist2(imgTimings,poseTimings);
imgPosesHiBall = zeros(4,4,length(imgTimings));
errors = zeros(length(imgTimings),1);
for i = 1:size(D,1)
    [M,I] = min(D(i,:));
    imgPosesHiBall(:,:,i) = H2tcam * reshape(trackData(I,2:17),[4,4])';
    errors(i) =I;
end

SaveImgPoses( imgPosesHiBall,sprintf('%s/%s',path2Dir,'PosesColumnMajor.txt') );
%%

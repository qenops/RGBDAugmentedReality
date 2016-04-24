%calibrate hi-ball wrt cluster cameras
%whats needed: 1.hiball tracker data, 2. multiple no motion video capture of a calibration pattern
%by the camera which is at origin of the cluster and the respective hiball
%track .tsv files named in the same order as the videos ,ie vid1 - 1.tsv,
%vid2 -2.tsv and so on

%this code contains mainly of implementation of the paper Robot sensor calibration: Solving
%AX=XB on the Euclidean group by F.C.Park and B.J.Martin
%%clc
close all

clear

%--------------------------------------------------------------------------
%!!!!!!!  checkerboard in this exp is 30mm by 30mm!!!!!!!!!!!!!!!!!!!!!
% Generate world coordinates of the corners of the squares
squareSize = 30;  % in units of 'mm'

% initGuess=true;%???????????????????????????????????????????????????????
%--------------------------------------------------------------------------

%%
path2Dir='F:/arkhalid/Google Drive/ToMapWork/Courses/4th Semester/Virtual Worlds/Project/Sequence 3 Calib Current';
numImages = 6;
poses = zeros(4,4,numImages);
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
    imgPosesHiBall(:,:,i) = reshape(trackData(I,2:17),[4,4])';
    errors(i) =I;
end
 [sortedErrors,sortedIndices] = sort(errors);
 p = [636,1175,927,637,382,738];
 for i = 1:numImages
     path = sprintf('%s/Color/img_0_%04d.jpg',path2Dir,p(i));
     images(:,:,:,i) = imread(path);
     poses(:,:,i) = imgPosesHiBall(:,:,p(i));
 end
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints( images);
% imageFileNames = imageFileNames(imagesUsed);
poses = poses(:,:,imagesUsed);
%assert(size(imagePoints,3)==size(track,2),'checker board not found in 1 or more images');

worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
% if(initGuess)
%     cameraParams = estimateCameraParameters(imagePoints, worldPoints,'IntitialIntrinsicMatrix',guess.stereoParams.CameraParameters1.IntrinsicMatrix,'IntialRadialDistortion',guess.stereoParams.CameraParameters1.RadialDistortion);
% else
cameraParams = estimateCameraParameters(imagePoints, worldPoints);
% end
% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'patternCentric');
 
%%
for i=1:cameraParams.NumPatterns
 calibPoses(:,:,i)=[cameraParams.RotationMatrices(:,:,i)', cameraParams.TranslationVectors(i,:)';0,0,0,1;];
end


 
%%


for i=1:cameraParams.NumPatterns
    T_ceil2H(:,:,i) = [poses(1:3,1:3,i)' , -poses(1:3,1:3,i)'*poses(1:3,4,i);0,0,0,1];
%   Tpat2cam(:,:,i) = [calibPoses(1:3,1:3,i)' , -calibPoses(1:3,1:3,i)'*calibPoses(1:3,4,i);0,0,0,1];

%   T_ceil2H(:,:,i) =  poses(:,:,i) ;
  Tpat2cam(:,:,i) = calibPoses(:,:,i);

end

for i=1:cameraParams.NumPatterns-1
 A(:,:,i) = (T_ceil2H(:,:,i+1)/(T_ceil2H(:,:,i)));
 
 B(:,:,i) = (Tpat2cam(:,:,i+1)/(Tpat2cam(:,:,i)));
 
 logA(:,:,i)=logm(A(1:3,1:3,i));
 logB(:,:,i)=logm(B(1:3,1:3,i));
 
alpha(:,i) = [-logA(2,3,i)+logA(3,2,i) , logA(1,3,i)-logA(3,1,i), -logA(1,2,i)+logA(2,1,i);]'/2;

beta(:,i) = [-logB(2,3,i)+logB(3,2,i) ,  logB(1,3,i)-logB(3,1,i), -logB(1,2,i)+logB(2,1,i);]'/2;

end

M=zeros(3,3);
for i=1:cameraParams.NumPatterns-1
    
   M=M+ beta(:,i) *alpha(:,i)';
end

  [Q,lambda]=eig(M'*M);
lamdaVals=diag(lambda);
sqrtLambaVals=sqrt(lamdaVals);
  R= Q*  diag(1./sqrtLambaVals)/Q *M';


assert(abs((det(R))-1)<0.001,'invalid rotation matrix, det!=1');

C=[]; d=[];
for i=1:cameraParams.NumPatterns-1
    C = [C;eye(3)-A(1:3,1:3,i);];
    d = [d;A(1:3,4,i)-R*B(1:3,4,i);];
end

% t =  (C'*C)\(C'*d)
t = C\d
 
 camCenter=-R'*t
 

 X=[R,t;0,0,0,1;];
 
 for i=1:cameraParams.NumPatterns-1
 residual(:,:,i)=A(:,:,i)*X-X*B(:,:,i);
 
 end
  save(sprintf('%s/tcam2H.mat',path2Dir),'X','residual');
 %%
 
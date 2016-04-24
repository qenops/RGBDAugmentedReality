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
path2Dir='D:\akash\summer15\recording\hiball\individual\hf3';

track = readTSVFile(path2Dir);

% if(initGuess)
%     guess=load('D:\akash\summer15\recording\stereoCalib\acb1acb2\out\stereoParams.mat');
% end
 

list= dir(sprintf('%s/*.MP4',path2Dir));
% assert(size(list,1)==1,'found no or multiple *.MP4 files');
     
assert(size(list,1)==size(track,2))
for i=1:size(list,1)
 v(i)=VideoReader(sprintf('%s/%s',path2Dir,list(i).name));

 images(:,:,:,i)= read(v(i),round(v(i).NumberOfFrames/2));
end

 
 poses =parseTracks(track);
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints( images);
% imageFileNames = imageFileNames(imagesUsed);

assert(size(imagePoints,3)==size(track,2),'checker board not found in 1 or more images');

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
for i=1:size(track,2)
 calibPoses(:,:,i)=[cameraParams.RotationMatrices(:,:,i)', cameraParams.TranslationVectors(i,:)';0,0,0,1;];
end


 
%%


for i=1:size(track,2)
    T_ceil2H(:,:,i) = [poses(1:3,1:3,i)' , -poses(1:3,1:3,i)'*poses(1:3,4,i);0,0,0,1];
%   Tpat2cam(:,:,i) = [calibPoses(1:3,1:3,i)' , -calibPoses(1:3,1:3,i)'*calibPoses(1:3,4,i);0,0,0,1];

%   T_ceil2H(:,:,i) =  poses(:,:,i) ;
  Tpat2cam(:,:,i) = calibPoses(:,:,i);

end

for i=1:size(track,2)-1
 A(:,:,i) = (T_ceil2H(:,:,i+1)/(T_ceil2H(:,:,i)));
 
 B(:,:,i) = (Tpat2cam(:,:,i+1)/(Tpat2cam(:,:,i)));
 
 logA(:,:,i)=logm(A(1:3,1:3,i));
 logB(:,:,i)=logm(B(1:3,1:3,i));
 
alpha(:,i) = [-logA(2,3,i)+logA(3,2,i) , logA(1,3,i)-logA(3,1,i), -logA(1,2,i)+logA(2,1,i);]'/2;

beta(:,i) = [-logB(2,3,i)+logB(3,2,i) ,  logB(1,3,i)-logB(3,1,i), -logB(1,2,i)+logB(2,1,i);]'/2;

end

M=zeros(3,3);
for i=1:size(track,2)-1
    
   M=M+ beta(:,i) *alpha(:,i)';
end

  [Q,lambda]=eig(M'*M);
lamdaVals=diag(lambda);
sqrtLambaVals=sqrt(lamdaVals);
  R= Q*  diag(1./sqrtLambaVals)/Q *M';


assert(abs((det(R))-1)<0.001,'invalid rotation matrix, det!=1');

C=[]; d=[];
for i=1:size(track,2)-1
    C = [C;eye(3)-A(1:3,1:3,i);];
    d = [d;A(1:3,4,i)-R*B(1:3,4,i);];
end

% t =  (C'*C)\(C'*d)
t = C\d
 
 camCenter=-R'*t
 

 X=[R,t;0,0,0,1;];
 
 for i=1:size(track,2)-1
 residual(:,:,i)=A(:,:,i)*X-X*B(:,:,i);
 
 end
  save(sprintf('%s/tcam2H.mat',path2Dir),'X','residual');
 
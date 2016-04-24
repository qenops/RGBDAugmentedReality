function [data] = readTSVFile(path2Dir)
%search for tsv file, if found multiple , give error
% assert(size(list,1)==1,'multiple or no .tsv files found in path');

list = dir(sprintf('%s/*.tsv',path2Dir));
for i=1:size(list,1)
fName=sprintf('%s/%s',path2Dir,list(i).name);
% fid =open(fName);
% fgetline(fid);
% close(fid);
% data =csvread(sprintf('%s/%s',path2Dir,list(1).name));


data{i}  = textread(fName,'','delimiter', ' ','headerlines',1);

  

end
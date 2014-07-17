function I = read3D(directory)

% Get list of all tif files in this directory
% DIR returns as a structure array.  You will need to use () and . to get
% the file names.
imagefiles = dir([directory '/*.tif']);      
nfiles = length(imagefiles);    % Number of files found
images = {};
for ii=1:nfiles
   currentfilename = imagefiles(ii).name;
   currentimage = imread([directory, '/' , currentfilename]);
   images{ii} = im2double(currentimage);
end

I = cat(3, images{:});
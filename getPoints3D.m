function pts = getPoints3D(I, pointsPerSlice, viewX, viewY, viewZ, interpScale)
% lets you select points from a 3D image, slice by slice
% only the part of the image defined by I(viewX, viewY, viewZ) is shown.
pts = zeros(0, pointsPerSlice, 3);

[~,~,numImages] = size(I);
scrsz = get(0,'ScreenSize');
figure('Position',[1 scrsz(4) scrsz(3)/2 scrsz(4)]);
ptsCount = 1;

for i = viewZ
    subplot('position', [0 0 1 1]), img = I(:, : ,i);
    
    set(gcf,'numbertitle','off','name',sprintf('Slice #%i', i));
    imshow(rescale2D(img(viewX, viewY), interpScale), 'Border', 'tight');
    % read pts from user
    [x,y] = getpts;
    if (length(x) == 0)
        continue;
    else
        if (length(x) ~= pointsPerSlice)
            print('you did not enter the right number of points. Please try again');
        else % correct coordinates (with (i,j) being the point in the center of the (i,j)th pixel)
            realY =  (y - 0.5) / interpScale +viewY(1) - 0.5;
            realX = (x - 0.5) / interpScale +viewX(1) - 0.5;

            slicePoints = cat(3, realX', realY', ones(1 , pointsPerSlice) * i);
            pts(ptsCount, 1:pointsPerSlice, 1:3) = slicePoints;
            ptsCount = ptsCount + 1;
        end
    end
    
end

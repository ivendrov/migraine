function fimage = graphcut(I, exponent)
% computes a segmentation of I using graph cuts
% exponent controls the degree to which the area of the boundary matters
% - a high exponent will pick the smallest gradients regardless of boundary
% area, whereas a low exponent will try to find the smallest possible
% boundary area

src = zeros(size(I));

if (ndims(I) == 3)
    [height, width, numImages] = size(I);
    %src(round(height/2),round(width/2),1:end) = Inf;
    for i = 1:numImages
        mat = zeros(height, width);
        mat(regiongrow(I(:,:,i))) = Inf;
        src(:,:,i) = mat;
    end
else
    [height, width] = size(I);
    middleRow = round(height/2);
    middleCol = round(width/2);
    src(middleRow-5:middleRow+5, middleCol-5:middleRow+5) = Inf;
end
fprintf('Done region growing');
sink = zeros(size(I));
sink(I > 0.55) = Inf;
sink(I < 0.15) = Inf;

[fimage] = graphcut_mex(I, src,sink, exponent);

function V = grow3D(I)
[~,~,numImages] = size(I);


V = zeros(size(I));
for i = 1:numImages
    V(:,:,i) = regiongrow(I(:,:,i));
end


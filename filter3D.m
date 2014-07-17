function F = filter3D(I, strength)
% 0.1 is a good filter strength

[~,~,numImages] = size(I);

F = zeros(size(I));


for i = 1:numImages
    F(:,:,i) = NLMF(I(:,:,i), struct('filterstrength', strength));   
end
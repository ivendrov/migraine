function displaySlice(V, rarity)
% displays slices of the 3D image in matrix form
% starting from slice #start, and skipping all
% but every (rarity)th image.
[~,~,numImages] = size(V);
% print matrix of images
n = floor (sqrt(numImages / rarity));
figure,
for i = 1:n
    for j = 1:n
        imgIndex = ((i-1)*n+j);
        subplot(n, n, imgIndex), imshow(V(:,:,imgIndex*rarity)), title(sprintf('%i', imgIndex*rarity));
    end
end

end


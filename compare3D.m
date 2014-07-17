function compare3D(I, BW, indices)
% compares the given image and 3D segmentation by displaying slices
% specified by indices
rows = 2;
cols = ceil(double(length(indices)) / rows);
figure,
for i = 1:length(indices)
    subplot(2, cols, i),
    hold on,
    index = indices(i);
    title(sprintf('%i', index));
    imshow(I(:,:,index), 'InitialMagnification', 300),    
    B = bwboundaries(BW(:,:,index));
    for j = 1:length(B)
        boundary = B{j};
        row = boundary(:,1);
        col = boundary(:,2);
        plot(col,row, 'Color', 'yellow');
    end
    hold off
end

    




end


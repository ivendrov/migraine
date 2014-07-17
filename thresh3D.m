function bw = thresh3D(image)
bw = zeros(size(image));

% find indices that are a priori foreground
indices = image + gradmag(image) < 0.5;
bw(indices) = image(indices);

end
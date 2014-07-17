function O = rescale3D(I, scale)
%rescale scales the given 3D image by a given factor,
% using spline interpolation

[h, w, d] = size(I);

% get the new index vectors 
[X,Y,Z] = meshgrid(linspace(1, h, h*scale), linspace(1, w, w*scale), linspace(1, d, d*scale));

% interpolate
O = interp3(I, X, Y, Z, 'spline');

end


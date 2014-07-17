function O = rescale2D(I, scale)
%rescale scales the given 2D image by a given factor,
% using spline interpolation

[h, w] = size(I);

% get the new index vectors 
[X,Y] = meshgrid(linspace(1, h, h*scale), linspace(1, w, w*scale));

% interpolate
O = interp2(I, X, Y, 'spline');

end


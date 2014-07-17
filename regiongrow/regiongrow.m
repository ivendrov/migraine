function fimage = regiongrow(I, varargin)
%
% Performing mean_shift operation on image
%
% Usage:
%   [output] = edison_wrapper(I, ...)
% 
% Inputs:
%   I - original image 
%
% Allowed parameters:
%   'hi' - hard uphill constraint
%   'lo' - soft uphill constraint
%   'sumlo' - sum of consecutive 'lo' constraints allowable
%
% Outputs:
%   result - binary image describing the resulting segmentation
%
hi = 0.0660;
lo =  0.0040; 
sumlo =  0.0120;



%fim = im2single(rgbim);
try 
    p = struct(varargin{:});
catch
    error('edison_wrapper:parse_inputs','Cannot parse arguments');
end

if ~isfield(p, 'hi')
    p.hi = hi;
end
if ~isfield(p, 'lo')
    p.lo = lo;
end
if ~isfield(p, 'sumlo')
    p.sumlo = sumlo;
end



[fimage] = regiongrow_mex(I, p);

vals = I(fimage);
threshold = mean(vals) + 0.5 * std(vals);
%fimage = imreconstruct(fimage, fimage | (I < threshold));

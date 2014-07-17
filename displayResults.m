load bw
load bwBefore
figure, isosurface(bw, 0)
set(gcf,'numbertitle','off','name','After')
figure, isosurface(bwBefore,0)
set(gcf,'numbertitle','off','name','Before')
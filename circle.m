function h = circle(x,y,r,varargin)

switch nargin
    case 4
        c = varargin{1};
    otherwise
        c = [1,0,0];
        
end 

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
%%plot(xunit, yunit,'r');
h = fill(xunit,yunit,c);
hold off
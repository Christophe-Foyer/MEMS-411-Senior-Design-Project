%% Christophe Foyer 2017

% Foyer, Christophe; Rangwala, Adam; and Nana, Deep, "Water Lenses for Low-Cost Concentrator Photovoltaics" (2017). Mechanical Engineering Design Project Class.

% This function tries to find the petzval curvature

function [x,y,xc,yc,R] = Func_Petzval(start_angle, end_angle, num, ray_num)

x = [];
y = [];

length = 1;
tol = 0.09;
length_mult = 1.1;
point_num = 9;
maxLoops = 5000;
volume = 0.05;
stretch_coef = 0.7;

%% solve using:
%using simualtion
%[lens_fun] = Simulate_water(length, tol, length_mult, point_num, maxLoops);

%using x^2 approximation
syms x
lens_fun = symfun((x/stretch_coef)^2,x);

f1 = figure('Name','Lens Simulation');

x = [];
y = [];
for i = linspace(start_angle, end_angle, num)
    disp(['Finding focal point at ' i ' degrees'])
    [focal_x,focal_y] = Func_Optics(ray_num, i, length, volume, 1.2, f1, lens_fun);
    x = [x, focal_x]; %#ok<AGROW>
    y = [y, focal_y]; %#ok<AGROW>
    
end

x = [x -x]; %mirror focal points
y = [y y]; %same here

%plot circle fit
[xc,yc,R] = circfit(x,y);

th = linspace(0,2*pi,200)';
xe = R*cos(th)+xc;
ye = R*sin(th)+yc;

hold on;
fit1 = plot(x,y,'o',[xe;xe(1)],[ye;ye(1)],'r-.');

%plot ploy2 fit
coefs = polyfit(x,y,2);
fit2 = plot(-1:0.01:1,polyval(coefs,-1:0.01:1),'g--');

%legend
legend([fit1(2),fit2],{'circle fit','poly2 fit'});
end
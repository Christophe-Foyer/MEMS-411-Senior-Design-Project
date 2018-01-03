%% Christophe Foyer 2017
% Finite element water trough simulation tool

%% ABOUT

% This program makes the assumption the through is constantly filled to the
% brim, this means it can't really be used to simulate an entire trough but
% can still be used to validate shape assumptions for the lens portion of
% this project.

% Disclaimer: this does not solve for the transient response of the system.

function [lens_fun] = Simulate_water(length, disp_tol, length_mult, point_num, maxLoops)

%% Init

%clear

%% Settings

Mod_E_Membrane = 1.5E9; %Just make it large, it shouldn't deform too much
m_per_l = 2; % mass per unit length per unit width
thickness = 0.0001; % meters
rho = 999; %density of the water
grav_const = 9.81; % m/s^2
%point_num = 29; % odd numbers works much better stability-wise
%disp_tol = 0.005; % when to stop the simulation (im meters)
%length_mult = 1.2; % slop added to distance between sides
dt = 0.002; % trial and error for stability
%length = 1; % distance between sides
%maxLoops = 5000; % maximum loops before returning results

%% Initialize points

x = linspace(-length/2,length/2,point_num);

%start with a curve
syms x_sym
func = symfun((1.5/length*x_sym)^4,x_sym);
y = linspace(0,0,point_num);
for i = 1:point_num
    y(i) = func((x(i)))-func((-0.5*length));
end

%% move point proportionally to force (assume linearity)

delta_disp_avg = disp_tol*2; %initialize it to a value that works
step = 1; %init

figure();

while (delta_disp_avg >= disp_tol) && (step <= maxLoops)
    %plot
    hold on;
    xlim([-length/2 length/2])
    ylim([-length/length_mult^3 0.1]) %length mult used to scale graph
    plot(x(step,:),y(step,:),'k');
    fill(x(step,:),y(step,:),'b','FaceAlpha',.3,'EdgeAlpha',.3); %fill with blue
    scatter(x(step,:),y(step,:),[],-y(step,:))
    frames(step) = getframe(gcf);
    clf;
    %increment step
    step = step+1;

    %import current coordinates
    x_curr = x(end, :);
    y_curr = y(end, :);
    %init new ones (will overwrite)
    x_next = x_curr;
    y_next = y_curr;
    %calculate force on each point (fixed ends)
    for point = 2:(point_num-1)
        %calculate force vector from previous point
        dist_prev = sqrt((x_curr(point)-x_curr(point-1))^2 + (y_curr(point)-y_curr(point-1))^2);
        if dist_prev > length_mult*length/point_num
            slope = (y_curr(point)-y_curr(point-1))/(x_curr(point)-x_curr(point-1));
            F1_x = -abs(cos(atan(slope))*sign(slope)*(dist_prev-length_mult*length/point_num)*thickness*Mod_E_Membrane);
            if x_curr(point) < x_curr(point-1)
                F1_x = -F1_x;
            end
            F1_y = -abs(sin(atan(slope))*sign(slope)*(dist_prev-length_mult*length/point_num)*thickness*Mod_E_Membrane);
            if y_curr(point) < y_curr(point-1)
                F1_y = -F1_y;
            end
        else
            F1_x = 0;
            F1_y = 0;
        end
        %calculate force vector from next point
        dist_next = sqrt((x_curr(point)-x_curr(point+1))^2 + (y_curr(point)-y_curr(point+1))^2);
        if dist_next > length_mult*length/point_num
            slope = (y_curr(point)-y_curr(point+1))/(x_curr(point)-x_curr(point+1));
            F2_x = abs(cos(atan(slope))*sign(slope)*(dist_next-length_mult*length/point_num)*thickness*Mod_E_Membrane);
            if x_curr(point) > x_curr(point+1)
                F2_x = -F2_x;
            end
            F2_y = abs(sin(atan(slope))*sign(slope)*(dist_next-length_mult*length/point_num)*thickness*Mod_E_Membrane);
            if y_curr(point) > y_curr(point+1)
                F2_y = -F2_y;
            end
        else
            F2_x = 0;
            F2_y = 0;
        end
        %calculate force vector from water
        hydro_p = -y_curr(point)*rho*grav_const;
        F_buoy = (hydro_p)*length_mult*length/point_num; %not accurate when untensioned but that's fine
            %find avg slope
        slope_orth = -1/(1/2*((y_curr(point)-y_curr(point-1))/(x_curr(point)-x_curr(point-1))+(y_curr(point)-y_curr(point+1))/(x_curr(point)-x_curr(point+1))));
        F_buoy_x = -cos(atan(slope_orth))*F_buoy*sign(slope_orth);
        F_buoy_y = -sin(atan(slope_orth))*F_buoy*sign(slope_orth);
        %calculate membrane weight
        F_grav_x = 0;
        F_grav_y = -m_per_l*grav_const;
        %calculate new position
        x_next(point) = x_curr(point)+((F1_x+F2_x+F_buoy_x+F_grav_x)/m_per_l*(dt^2)/2);
        y_next(point) = y_curr(point)+((F1_y+F2_y+F_buoy_y+F_grav_y)/m_per_l*(dt^2)/2);
    end
    x = [x; x_next];
    y = [y; y_next];
    %delta_disp_avg = disp_tol*2; %debugging
    delta_disp_avg = sum((((x(step,:)-x(step-1,:)).^2 + (y(step,:)-y(step-1,:)).^2)).^(1/2))/(point_num-2)/dt %#ok<NOPTS>
end

delete(findall(0,'Type','figure'))

%plot fit

coefs1 = polyfit(x(end,:),y(end,:),2);
coefs2 = polyfit(x(end,:),y(end,:),4);
coefs3 = polyfit(x(end,:),y(end,:),6);

figure('Name','Final positions and fit lines');
hold on;
plot(x(end,:),y(end,:),'*');
fit1=plot(-length/2:0.01:length/2,polyval(coefs1,-length/2:0.01:length/2),'r--');
fit2=plot(-length/2:0.01:length/2,polyval(coefs2,-length/2:0.01:length/2),'g--');
fit3=plot(-length/2:0.01:length/2,polyval(coefs3,-length/2:0.01:length/2),'b--');
legend([fit1,fit2,fit3],{'poly2 fit','poly4 fit','poly6 fit'});

%remove odd coefs (test inconclusive)
% coefs = [];
% disp(coefs3)
% disp(coefs3(5))
% for i = 1:6
%     if mod(i,2) == 0
%         coefs = [coefs, coefs3(i)]; %#ok<AGROW>
%     else
%         coefs = [coefs, 0]; %#ok<AGROW>
%     end
% end
disp('Coeficients:') %looks like odd numbers are negligeable and are making solve fail?
disp(coefs2)
syms x
lens_fun = symfun(poly2sym(coefs2,x),x);

% %play movie
% figure('Name','FEA Animation');
% xlim([0 length])
% ylim([-length/length_mult^3 0.1])
% movie(gcf,frames,1000,100);

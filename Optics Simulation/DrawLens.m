%% Christophe Foyer 2017
%% This script draws the lens and tries to find the focal point

%You can change the shape of the lens by changing the waterline_fun and the
%lens_fun functions.

%% Setup
clear
format SHORTENG
figure('Name','Lens Simulation');
drawnow;
%profile on

%% Variables
volume = 0.3; %m^2, not really volume since this is 2d %calculate the water line from volume
membrane_length = 2; %m %calculate the x_range from length
stretch_coef = 1.2; %non-dimensional multiplier thing
ray_num = 3; %how many rays are drawn

%% Optics variables
angle_of_incidence = 20; %degrees (breaks the code at EXACTLY 0 deg...)
refr_index_fluid = 1.33;
refr_index_air = 1.00029;

%% Script

% figure setup
hold on
ylim([-2 1]) %constant y range
axis equal

% convert variables
angle_from_horizontal = (90-angle_of_incidence)/360*2*pi;

%% Membrane shape
syms x
lens_fun = symfun((x/stretch_coef)^2,x); %make sure it's symetrical and starts at 0
syms y
F = solve(x==lens_fun(y),y);

% solve for membrane range
disp('solving for membrane range')
syms B
x_range = double(solve(int(sqrt(1+diff(lens_fun)^2),0, B)==membrane_length/2,B));

% plot membrane
fplot(lens_fun, [-x_range, x_range],'k')
drawnow;

% find water level from volume
syms WL
waterlevel = solve(int(abs(F(1)),0,WL)==volume/2, WL);

% solve for waterlevel intercepts
disp('solving for waterlevel intercepts')
waterlevel_fun = symfun(waterlevel,x);
S = double(solve(waterlevel_fun==lens_fun,x));
water_range = [S(1),S(2)];

% check for water overflow
if waterlevel > lens_fun(x_range)
    error('Water level too high.')
end

% plot water line
plot(water_range,ones(size(water_range))*waterlevel,'b')
drawnow;

% set light ray entry points
ray_entry = linspace(S(1),S(2),ray_num+2);
ray_entry = ray_entry(2:end-1);

%% Find entering rays function
disp('finding entering ray symbolic functions')
ray_funs_entering = sym('x', length(ray_entry));
for i = 1:length(ray_entry)
    syms x
    ray_function = symfun((x-ray_entry(i))*tan(angle_from_horizontal)+waterlevel,x);
    ray_funs_entering(i) = ray_function;
end

% plot entering light rays
for i = 1:length(ray_entry)
    fplot(ray_funs_entering(i), [ray_entry(i), x_range],'y')
    drawnow;
end

%% Find refracted rays
disp('finding refracted ray (first refraction) symbolic functions')
ray_funs_refr1 = sym('x', length(ray_entry));
ray_exit_1 = zeros(1,length(ray_entry));
ray_angles_1 = zeros(1,length(ray_entry));
for i = 1:length(ray_entry)
    %define domain
    domain = [-x_range, ray_entry(i)];
    % get angle of incident ray
    syms x
    angle = func_angle(ray_funs_entering(i), waterlevel_fun, domain, x);
    % find outgoing angle
    refr_angle = asin(sin(pi/2+angle)*refr_index_air/refr_index_fluid);
    angle_out = -angle_from_horizontal-angle+(pi/2-refr_angle);
    % add symbolic function to vector
    ray_funs_refr1(i) = symfun((x-ray_entry(i))*tan(angle_out)+waterlevel,x);
    % solve for intercept
    Sol = double(solve(ray_funs_refr1(i)==lens_fun,x));
    point = domain(1)-1; %make it outside the domain
    for j=1:length(Sol)
        if domain(1)<=Sol(j) && domain(2)>=Sol(j) && Sol(j)>point
            point = Sol(j);
        end
    end
    % throw error if no points are found
    if point == domain(1)-1
       error('No intercepts inside domain')
    end
    % add to vector
    ray_exit_1(i) = point;
    ray_angles_1(i) = angle_out;
    % plot ray
    fplot(ray_funs_refr1(i),[point,ray_entry(i)],'y')
    drawnow;
end

%% Find refracted rays 2 (probably has issues)
disp('finding refracted ray (second refraction) symbolic functions')
ray_funs_refr2 = sym('x', length(ray_entry));
for i = 1:length(ray_entry)
    %define domain
    domain = [-x_range, ray_exit_1(i)];
    % get angle of incident ray
    syms x
    angle = func_angle(ray_funs_refr1(i), lens_fun, domain, x);
    %find angle of tangeant
    tang_lens = diff(lens_fun);
    %find lens angle from horizontal
    angle_lens = func_angle(tang_lens,symfun(0*x,x), [-Inf,Inf], x);
    % find outgoing angle
    refr_angle = asin(sin(pi/2-angle)*refr_index_fluid/refr_index_air);
    angle_out = pi/2+ray_angles_1(i)+angle-refr_angle;
    % add symbolic function to vector
    tang_ray_1 = diff(ray_funs_refr1(i));
    %calculate ray functions
    ray_funs_refr2(i) = symfun((x-ray_exit_1(i))*tan(angle_out)+lens_fun(ray_exit_1(i)),x);

    tang_ray_2 = diff(ray_funs_refr2(i));
    try
        if tang_ray_2 > 0
            draw_domain = [-x_range,ray_exit_1(i)];
        else
            draw_domain = [ray_exit_1(i),x_range];
        end
        % plot ray
        fplot(ray_funs_refr2(i),draw_domain,'y')
        drawnow;
    catch
        disp('Error drawing ray, skipping...');
    end
end

%% Plot Focal Point Approximation 

disp('locating focal point (approx.); works better at low angles, includes outliers')
%Find ray intercepts
x_Points = zeros(length(ray_entry));
y_Points = zeros(length(ray_entry));
for i = 1:length(ray_entry)
    for j = 1:length(ray_entry)
        if ray_funs_refr2(i) ~= ray_funs_refr2(j)
            syms x
            x_Points(i,j) = double(solve(ray_funs_refr2(i)==ray_funs_refr2(j), x));
            y_Points(i,j) = double(subs(ray_funs_refr2(i),x,x_Points(i,j)));
        end
    end
end

%find distances
dist_mat = ((x_Points).^2+(y_Points).^2).^(1/2);

%remove outliers
%outlier_mat = abs(isoutlier(dist_mat)-1); %only works in matlab 2017
outlier_mat = abs(zeros(length(ray_entry))-1);

%update to remove outliers
x_Points = x_Points.*outlier_mat;
y_Points = y_Points.*outlier_mat;

%find average
focal_x = sum(sum(x_Points))/(sum(sum(outlier_mat))-length(ray_entry));
focal_y = sum(sum(y_Points))/(sum(sum(outlier_mat))-length(ray_entry));

%plot the point
plot(focal_x,focal_y,'rx')
drawnow;

%for debugging
%profile viewer
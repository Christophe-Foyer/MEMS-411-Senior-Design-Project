function [] = checkLens()

%variables
volume = 0.1; %m^2, not really volume since this is 2d %calculate the water line from volume
membrane_length = 1; %m %calculate the x_range from length
stretch_coef = 0.5;

%optics variables
angle_of_incidence = 30; %degrees
refr_index_fluid = 1.33;
refr_index_air = 1.00029;
refl_coef = 1; %how much light is refracted?
ray_num = 5; %how many rays are drawn

%convert variables
angle_of_incidence = (90-angle_of_incidence)/360*2*pi

%Membrane Shape
syms x
f = symfun((x/stretch_coef)^2,x); %make sure it's symetrical and starts at 0
syms y
F = solve(x==f(y),y);

%Find water level from volume
syms WL
waterlevel = solve(int(abs(F(1)),0,WL)==volume/2, WL);

%solve for waterlevel intercepts
const_f = symfun(waterlevel,x);
S = solve(const_f==f,x);
syms B
x_range = double(solve(int(sqrt(1+diff(f)^2),0, B)==membrane_length/2,B));
water_range = [S(1),S(2)];

%check for water overflow
if waterlevel > f(x_range)
    error('Water level too high.')
end

%lightpaths
%calculate light paths
rays = linspace(S(1),S(2),ray_num+2);
rays = rays(2:end-1);

%% plots
hold on
ylim([-1 1])
xlim([-x_range, x_range])

%plot waterline refraction points
plot(rays,ones(size(rays))*waterlevel,'yo')
%plot entering light rays
tan_1 = sin(angle_of_incidence)/cos(angle_of_incidence);
for i = rays
    plot(linspace(i,x_range+i,10),linspace(waterlevel/tan_1,x_range+waterlevel/tan_1,10).*tan_1,'y--')
%plot refracted rays
refr_angle = asin(sin(angle_of_incidence)*refr_index_air/refr_index_fluid)
tan_2 = sin(refr_angle)/cos(refr_angle);
ref_points = [];
tang_in = [];
for j = rays
    %find intercept with membrane
    syms x
    ray2 = symfun((x-j)*(tan_2)+waterlevel,x);
    ray2_prime = diff(ray2);
    %plot lines
    ref_point = double(solve(f==ray2,x));
    for num = 1:length(ref_point)-1
        if ref_point(num) > S(1) && ref_point(num) < S(2)
            ref_point = ref_point(num);
        end
    end
    ref_points = [ref_points, ref_point]; %#ok<AGROW>
    tang_in = [tang_in, ray2_prime(ref_point)]; %#ok<AGROW>
    if double(j) < ref_point
        fplot(ray2, [double(j), ref_point],'y--')
    else
        fplot(ray2, [ref_point, double(j)],'y--')
    end
end
%plot membrane refraction points
plot(ref_points,f(ref_points),'yo')
%plot outbound rays
for k = 1:length(ref_points)
    f2 = diff(f);
    tang_out = f2(ref_points(k));
    tang = tang_in(k);
    theta=atan(abs(tang_out-tang)/abs(1+tang_out*tang));
    tan_3 = sin(angle_of_incidence+refr_angle+theta)/cos(angle_of_incidence+refr_angle+theta);
    ray3 = symfun((x-ref_points(k))*(tan_3)+f(ref_points(k)),x);
    fplot(ray3,'y--')
end
%plot membrane
fplot(f, [-x_range, x_range],'k')
%plot waterline
plot(water_range,ones(size(water_range))*waterlevel,'b')
end

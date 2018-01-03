%% Christophe Foyer 2017
%% angle calculation function

function angle = func_angle(ray_fun, lens_fun, domain, symvar)

%assuming negative slope rays (could check later)

%find intercept points
Sol = double(solve(ray_fun==lens_fun, symvar)); %symvar symbolic variable (usually x)

%remove points outside domain and keep largest one (valid assumption?)
point = domain(1)-1; %set outside domain
for i=1:length(Sol)
    if domain(1)<=Sol(i) && domain(2)>=Sol(i) && Sol(i)>point
        point = Sol(i);
    end
end

%Throw error if no points are found
if point == domain(1)-1
   error('No intercepts inside domain')
end

%find differentials
diff_ray = diff(ray_fun);
diff_lens = diff(lens_fun);

%find tangeants
tang_ray = diff_ray; %only works for straight lines (aka light rays here)
tang_lens = diff_lens(point);

%calculate angle (do we want absolute values)
%angle=atan((tang_ray-tang_lens)/(1+tang_ray*tang_lens));

%calculate angles (no absolute values
angle = -(atan(tang_ray)-atan(tang_lens));

end
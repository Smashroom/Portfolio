function visual_problem(x)
global optimParams;
%delete(gca);
R = optimParams.radius;
[c1_x c1_y] = gen_circle(R(1),[x(3) x(4)]);
[c2_x c2_y] = gen_circle(R(2),[x(5) x(6)]);
[c3_x c3_y] = gen_circle(R(3),[x(7) x(8)]);
[c4_x c4_y] = gen_circle(R(4),[x(9) x(10)]);

if isfield(optimParams,'r')
    delete(optimParams.r);
    delete([optimParams.c1,optimParams.c2,optimParams.c3,optimParams.c4]);
end
hold on;
optimParams.r = rectangle('Position',[0 0 x(1) x(2)]);
optimParams.c1 = plot(c1_x,c1_y);
optimParams.c2 = plot(c2_x,c2_y);
optimParams.c3 = plot(c3_x,c3_y);
optimParams.c4 = plot(c4_x,c4_y);
hold off;
grid on;
axis equal;

end

function [x,y] = gen_circle(radius,center)

theta = 0 : 0.01 : 2*pi;

x = radius * cos(theta) + center(1);
y = radius * sin(theta) + center(2);

end
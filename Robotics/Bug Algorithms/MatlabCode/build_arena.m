%
% build_arena( n )
%  Interactively builds an arena with n obstacles based on user
%  input. The resulting arena data is placed in the global variable
%  arena_map.
%
%  Inputs:  n: number of obstacles to input
%  Outputs: None
%

function build_arena(n)

global arena_map arena_limits qstart qgoal;

arena_map = [];
arena_limits = [0 10 0 10];
qstart = [5 3];
qgoal = [5 9];
clf;
draw_arena();

for i=1:n
  disp(sprintf('Please select all the vertices of obstacle %i',i));
  disp(sprintf('Press ENTER when done.',i));
  [x,y] = ginput;
  arena_map{i} =[x y];
  draw_arena();
  arena_map{i}
end

end

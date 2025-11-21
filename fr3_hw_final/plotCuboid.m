function h = plotCuboid(pos, sz, color, alpha)
% plotCuboid
%
%   Plots a 3D cuboid (box) with a given position, size, color, and alpha.
%
%   Inputs:
%     pos   - [x, y, z] of the minimum corner
%     sz    - [width_x, width_y, width_z] size of the box
%     color - MATLAB color spec (e.g., 'r', [1 0 0])
%     alpha - Transparency value (0.0 to 1.0)
%
%   Output:
%     h     - Handle to the patch object
%

% Define the 8 vertices
x = pos(1) + [0 sz(1) sz(1) 0 0 sz(1) sz(1) 0];
y = pos(2) + [0 0 sz(2) sz(2) 0 0 sz(2) sz(2)];
z = pos(3) + [0 0 0 0 sz(3) sz(3) sz(3) sz(3)];

% Combine into a vertices matrix
verts = [x' y' z'];

% Define the 6 faces using vertex indices
faces = [
    1 2 3 4; % Bottom
    5 6 7 8; % Top
    1 2 6 5; % Front
    3 4 8 7; % Back
    1 4 8 5; % Left
    2 3 7 6  % Right
];

% Draw the patch
h = patch('Vertices', verts, 'Faces', faces, ...
          'FaceColor', color, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', 'none');

end


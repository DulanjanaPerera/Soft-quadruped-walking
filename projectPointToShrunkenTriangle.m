function xSafe = projectPointToShrunkenTriangle(x, V, margin)
% projectPointToShrunkenTriangle
%
% Projects a 2D point x into a support triangle shrunken inward by margin.
%
% Inputs:
%   x      : 2x1 point
%   V      : 2x3 triangle vertices, each column is one stance foot [x;y]
%   margin : inward buffer distance from each edge
%
% Output:
%   xSafe  : 2x1 projected safe point

x = x(:);

if size(V,1) ~= 2 || size(V,2) ~= 3
    error('V must be 2x3.');
end

% Ensure counter-clockwise ordering
V = orderTriangleCCW(V);

% Build shrunken triangle
[Vsh, valid] = shrinkTriangle(V, margin);

% If margin is too large, fall back to centroid
if ~valid
    xSafe = mean(V,2);
    return;
end

% If already inside shrunken triangle, return x
if isInsideConvexPolygon(x, Vsh)
    xSafe = x;
    return;
end

% Otherwise project to nearest point on shrunken triangle boundary
xSafe = nearestPointOnPolygonBoundary(x, Vsh);

end

%%
function Vccw = orderTriangleCCW(V)
% Orders triangle vertices counter-clockwise.

c = mean(V,2);
ang = atan2(V(2,:) - c(2), V(1,:) - c(1));
[~,idx] = sort(ang);
Vccw = V(:,idx);

% Ensure positive signed area
area2 = signedArea2D(Vccw);
if area2 < 0
    Vccw = Vccw(:,[1 3 2]);
end
end

%%
function A = signedArea2D(V)
% Signed polygon area for 2xN vertices.

N = size(V,2);
A = 0;

for i = 1:N
    j = i + 1;
    if j > N
        j = 1;
    end

    A = A + V(1,i)*V(2,j) - V(1,j)*V(2,i);
end

A = 0.5*A;
end

%%
function [Vsh, valid] = shrinkTriangle(V, margin)
% Shrinks a CCW triangle inward by offsetting each edge by margin.

valid = true;
Vsh = zeros(2,3);

% Edge inward normals and offset constants
n = zeros(2,3);
c = zeros(1,3);

for i = 1:3
    j = i + 1;
    if j > 3
        j = 1;
    end

    vi = V(:,i);
    vj = V(:,j);

    e = vj - vi;
    le = norm(e);

    if le < 1e-12
        valid = false;
        return;
    end

    % For CCW polygon, inward normal is left normal
    n(:,i) = [-e(2); e(1)] / le;

    % Offset line inward by margin:
    % n_i' * x = n_i' * vi + margin
    c(i) = n(:,i).' * vi + margin;
end

% New vertex i is intersection of previous offset edge and current offset edge
for i = 1:3
    prev = i - 1;
    if prev < 1
        prev = 3;
    end

    Aline = [n(:,prev).'; n(:,i).'];
    bline = [c(prev); c(i)];

    if abs(det(Aline)) < 1e-12
        valid = false;
        return;
    end

    Vsh(:,i) = Aline \ bline;
end

% Check if shrunken triangle is still valid
if signedArea2D(Vsh) <= 1e-12
    valid = false;
end

end

%%
function inside = isInsideConvexPolygon(x, V)
% Checks if point x is inside a CCW convex polygon.

N = size(V,2);
tol = 1e-10;
inside = true;

for i = 1:N
    j = i + 1;
    if j > N
        j = 1;
    end

    vi = V(:,i);
    vj = V(:,j);
    e = vj - vi;

    % Inward normal for CCW polygon
    nIn = [-e(2); e(1)] / norm(e);

    if nIn.'*(x - vi) < -tol
        inside = false;
        return;
    end
end

end

%%
function xNearest = nearestPointOnPolygonBoundary(x, V)
% Finds nearest point on boundary of convex polygon.

N = size(V,2);

bestDist = inf;
xNearest = V(:,1);

for i = 1:N
    j = i + 1;
    if j > N
        j = 1;
    end

    a = V(:,i);
    b = V(:,j);

    candidate = nearestPointOnSegment(x, a, b);
    d = norm(x - candidate);

    if d < bestDist
        bestDist = d;
        xNearest = candidate;
    end
end

end

%%
function p = nearestPointOnSegment(x, a, b)
% Projects point x onto line segment ab.

ab = b - a;
den = ab.'*ab;

if den < 1e-12
    p = a;
    return;
end

t = ((x - a).' * ab) / den;
t = max(0, min(1, t));

p = a + t*ab;

end
function config = length2config(l, r)

theta = atan2(l(2) - l(1), sqrt(0.3e1) * (l(1) + l(2)));
phi = 0.2e1 / 0.3e1 / r * sqrt((3 * l(1) ^ 2 + 3 * l(1) * l(2) + 3 * l(2) ^ 2));
phi = wrapToPi(phi);
config = [theta; phi];
end
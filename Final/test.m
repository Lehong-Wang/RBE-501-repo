
m = 0.5;
r = 0.05;
h = 0.15;

Ixx = m*(3*r^2 + h^2) / 12
Iyy = Ixx
Izz = m * r^2 / 2

Ib = [Ixx 0 0; 0 Iyy 0; 0 0 Izz]


% -cross([0 1 0], [0 0 5])

% -cross([0 1 0], [0 0 5])

% -cross([1 0 0], [0 0 5])
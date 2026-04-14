% Solución Sistema de Ecuaciones 2x2
syms w u r l Wr Wl

eq1 = (r/l)*(Wr - Wl) == w;
eq2 = (r/2)*(Wr + Wl) == u;

sol = solve([eq1, eq2], [Wr, Wl]);

Wr_sol = sol.Wr
Wl_sol = sol.Wl
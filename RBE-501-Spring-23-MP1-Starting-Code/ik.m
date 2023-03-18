


% function q = ik(S, M, currentQ, targetPose)


% q = 0
% end




load target_poses.mat

targetT = T(:,:,1)

syms Uhx Uhy Uhz Vhx Vhy Vhz Whx Why Whz Phx Phy Phz

eq = [Uhx Vhx Whx Phx;
Uhy Vhy Why Phy;
Uhz Vhz Whz Phz;
0 0 0 1] == targetT
sol = solve(eq, [Uhx Uhy Uhz Vhx Vhy Vhz Whx Why Whz Phx Phy Phz])
sol(1)
[Uhx Uhy Uhz Vhx Vhy Vhz Whx Why Whz Phx Phy Phz] = deal(sol.Uhx, sol.Uhy, sol.Uhz, sol.Vhx, sol.Vhy, sol.Vhz, sol.Whx, sol.Why, sol.Whz, sol.Phx, sol.Phy, sol.Phz)


L0 = 0.352
L1 = 0.36
L2 = 0.07
L3 = 0.254
L4 = 0.126
L5 = 0.065
L6 = L2 + L3 + L4 + L5


s1 = Phy - Why * (L5+L6)
c1 = Phx - Whx * (L5+L6)
th1 = atan2(s1, c1) - pi/2
th1D = rad2deg(double(th1))

th3 = atan2




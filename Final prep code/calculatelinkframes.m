function [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(ur5)
  Mj1 = tdh(ur5.theta(1), ur5.d(1), ur5.a(1), ur5.alpha(1));
  Mj2 = Mj1 * tdh(ur5.theta(2), ur5.d(2), ur5.a(2), ur5.alpha(2));
  Mj3 = Mj2 * tdh(ur5.theta(3), ur5.d(3), ur5.a(3), ur5.alpha(3));
  Mj4 = Mj3 * tdh(ur5.theta(4), ur5.d(4), ur5.a(4), ur5.alpha(4));
  Mj5 = Mj4 * tdh(ur5.theta(5), ur5.d(5), ur5.a(5), ur5.alpha(5));
  Mj6 = Mj5 * tdh(ur5.theta(6), ur5.d(6), ur5.a(6), ur5.alpha(6));
  
  M1 = Mj1 * [eye(3) [0, -0.02561, 0.00193]'; 0 0 0 1];
  M2 = Mj2 * [eye(3) [0.2125, 0, 0.11336]'; 0 0 0 1];
  M3 = Mj3 * [eye(3) [0.15, 0.0, 0.0265]'; 0 0 0 1];
  M4 = Mj4 * [eye(3) [0, -0.0018, 0.01634]'; 0 0 0 1];
  M5 = Mj5 * [eye(3) [0, 0.0018, 0.01634]'; 0 0 0 1];
  M6 = Mj6 * [eye(3) 	[0, 0, -0.001159]'; 0 0 0 1];
  
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = eye(4);
end


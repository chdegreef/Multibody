function jdqd = cons_jdqd(data)
l_arm = 0.5;
h_arm = 0.25;

q = data.q;
qd = data.qd;
jdqd = zeros(2,1);
jdqd(1) = l_arm*cos(q(2))*qd(2)*qd(2) - l_arm*cos(q(3))*qd(3)*qd(3) + h_arm*sin(q(3)+q(4))*(qd(3)+qd(4))^2;
jdqd(2) = l_arm*sin(q(2))*qd(2)*qd(2) - l_arm*sin(q(3))*qd(3)*qd(3) - h_arm*cos(q(3)+q(4))*(qd(3)+qd(4))^2;

end
function jdqd = cons_jdqd(data)
l = 0.5;
h = 0.25;

q = data.q;
qd = data.qd;
jdqd = zeros(2,1);
jdqd(1) = l*cos(q(2))*qd(2)*qd(2) - l*cos(q(3))*qd(3)*qd(3) + h*sin(q(3)+q(4))*(qd(3)+qd(4))^2;
jdqd(2) = l*sin(q(2))*qd(2)*qd(2) - l*sin(q(3))*qd(3)*qd(3) - h*cos(q(3)+q(4))*(qd(3)+qd(4))^2;

end
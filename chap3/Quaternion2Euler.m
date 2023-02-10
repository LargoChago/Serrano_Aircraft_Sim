function [phi, theta, psi]= Quaternion2Euler(e0,e1,e2,e3)
phi= arctan((2*((e0*e1)+(e2*e3)))/(1-(2*((e1^2)+(e2^2)))));
theta= (-pi/2)+2*arctan(( (1+(2*((e0*e2)-(e1*e3)))) / (1-(2*((e0*e2)-(e1*e3))))    )^.5 );
psi= arctan( (2*((e0*e3) + (e1*e2))) / (1- (2*((e2^2)+(e3^2)))));
end

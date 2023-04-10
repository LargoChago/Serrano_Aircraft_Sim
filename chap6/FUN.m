m= 1;
b= 10;
k= 20;
F= 1;

s= tf('s')
P = 1 / (m*s^2 + b*s + k)



Kp = 300;
Kd= 10;
Pl = Kp/ (m*s^2 + b*s + (k+Kp)) ;
C  = pid(Kp);
T  = feedback(C*P,1);
P2 = (Kp +Kd*s) / (m*s^2 + (b+Kd)*s + (k+Kp))

t= 0:.01:2;

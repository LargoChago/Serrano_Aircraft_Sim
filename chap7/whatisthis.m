v= zeros([1,100]) 
for t= 2:100 
v(t) = exp((-1/16000))*v(t-1)  + normrnd(0,.4) 
End
plot(v) 

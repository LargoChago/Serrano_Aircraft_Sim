eps = .001 ;
J = zeros(3,3) ;
X = [10;-.05;.8] ;
-5<=alpha<= 20 ;
-1<=delta_e<=1 ;
0<= delta_t<=1 ;


f(x)= [ delta; X]

for i =1:3
    x_eps=X
    x_eps[i]= x_eps+eps
    f_at_x_eps = f(x_eps)
    df_dx = (f_at_x_eps - f_at_x)/ eps
    J[:,i] = df_dx

    fmincon

    norm(z)= funx, a=[], b=[], aeq=[], beq=[], lowerbounds, upperbounds  lb=[] a
clear
%
mav_view = MAV_viewer();
t= 0;
dt= .01;

state = msg_state();

mav_view.update(state);
view(90,0)

state.q =1;
C_m_delta_e = 1;
C_m_theta = 1;
C_m_q = -1;
while(1)
    if t > 1.0 && t <= 1.1
        delta_e = 10;
    else
        delta_e = 0;
    end
    m = C_m_delta_e * delta_e + C_m_theta * state.theta + C_m_q * state.q;
state.theta = state.theta + state.q *dt
mav_view.update(state);
view(90,0)
t = t + dt;
end
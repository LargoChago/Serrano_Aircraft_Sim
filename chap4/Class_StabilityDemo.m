clear
%
mav_view = MAV_viewer();
t= 0;
dt= .01;

state = msg_state();

mav_view.update(state);
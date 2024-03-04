function xout= uavdynamicsvelocity(xin)
x1=xin(1);  % UAV x position
y1=xin(2); % UAV y position
v1x=xin(3); % UAV x velocity
v1y=xin(4); % UAV y velocity
% Reference position (waypoint) for UAV
x1ref=xin(5);
y1ref=xin(6);
% velocity reference for obstacle avoidance
v1xref=xin(7);
v1yref=xin(8);
vflag=xin(9); % vflag=1 means near obstacle
dt=0.1;
Va=20.0;

% If no obstacles nearby the vflag=0
if(vflag<1)    
    v1xref=Va*(x1ref-x1)/sqrt((x1ref-x1)^2+(y1ref-y1)^2);
    v1yref=Va*(y1ref-y1)/sqrt((x1ref-x1)^2+(y1ref-y1)^2);
end
% Dynamics propogation
v1xdot=-8*v1x+8*v1xref;
v1x=v1x+v1xdot*dt;
v1ydot=-8*v1y+8*v1yref;
v1y=v1y+v1ydot*dt;
x1dot=v1x;
x1=x1+x1dot*dt;
y1dot=v1y;
y1=y1+y1dot*dt;
xout=[x1,y1,v1x,v1y]; % update position and velocity
end
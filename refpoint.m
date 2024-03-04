
function newpoint=refpoint(currentpoint)
x1ref=currentpoint(1);
y1ref=currentpoint(2);
x1=currentpoint(3);
y1=currentpoint(4);
SwarmCen=[currentpoint(5) currentpoint(6)];
rswarm=currentpoint(7);
% SwarmCenref=[currentpoint(8) currentpoint(9)];
Posref=[x1ref y1ref];
thetaP2=atan2(y1ref,x1ref);
firstpos=[x1 y1];
thetaP1=atan2(y1,x1);
Posdis=norm(Posref-firstpos);
Disref=norm(Posref-SwarmCen);
Dis=norm(firstpos-SwarmCen);
diffP2=([x1ref y1ref] - SwarmCen);
angP2 = atan2(diffP2(2),diffP2(1));
if(Disref>rswarm-0.1*rswarm)
    x1ref=x1ref-(Disref-0.7*rswarm)*cos(angP2);
    y1ref=y1ref-(Disref-0.7*rswarm)*sin(angP2);    
end
newpoint=[x1ref, y1ref];
end

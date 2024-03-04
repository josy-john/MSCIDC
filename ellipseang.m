% Area covered by ellipse between two angles
function theta2ref=ellipseang(ar,br,theta1,n)

% theta2ref=reference angular position for UAV with current angular position theta2

% theta1= angular position of suceeding neighbour

% n= total number of UAVs in formation

% ar= major axis of elliptical fire

% br=minor axis of elliptical fire

arear=pi*ar*br/n;  % desired area between neighbouring UAVs

if(theta1>=0 && theta1<=0.5*pi)
  
        theta1a=0.5*pi;
        area1a=0.5*ar*br*(atan2(ar*tan(theta1a),br)-atan2(ar*tan(theta1),br));
        if(arear>area1a)
        arear1=arear-area1a;
        theta2ref=0.5*pi+atan2(abs((ar/br)*tan((2*abs(arear1)/(br*ar))+atan2(br*tan(0*pi),ar))),1);
        else
        theta2ref=atan2(((br/ar)*tan((2*abs(arear)/(ar*br))+atan2(ar*tan(theta1),br))),1);
        end
   
   
end

if(theta1>0.5*pi && theta1<=pi)
  
        theta1a=theta1-0.5*pi;
       
        area1a=0.5*br*ar*(atan2(br*tan(theta1a),ar)-atan2(br*tan(0),ar));
        area2a=0.25*pi*ar*br-area1a;
        if(arear>area2a)
        arear1=arear-area2a;
        theta2ref=-pi+atan2(abs((br/ar)*tan((2*abs(arear1)/(ar*br))+atan2(ar*tan(0),br))),1);
        else
        theta2ref=0.5*pi+atan2(abs((ar/br)*tan((2*abs(arear)/(br*ar))+atan2(br*tan(theta1a),ar))),1);
        end
end


if(theta1>=-pi && theta1<-0.5*pi)
  
        theta1a=theta1+pi;
        area1a=0.5*ar*br*(atan2(ar*tan(theta1a),br)-atan2(ar*tan(0),br));
        area2a=0.25*pi*ar*br-area1a;
        if(arear>area2a)
        arear1=arear-area2a;
        theta2ref=-0.5*pi+atan2(abs((ar/br)*tan((2*abs(arear1)/(ar*br))+atan2(br*tan(0),ar))),1);
        else
        theta2ref=-pi+atan2(abs((br/ar)*tan((2*abs(arear)/(ar*br))+atan2(ar*tan(theta1a),br))),1);
        end
end


if(theta1>=-0.5*pi && theta1<0)
  
        theta1a=-theta1;
        area1a=0.5*ar*br*(atan2(ar*tan(theta1a),br)-atan2(ar*tan(0),br));
        area2a=area1a;
        if(arear>area2a)
        arear1=arear-area2a;
        theta2ref=atan2(abs((br/ar)*tan((2*abs(arear1)/(br*ar))+atan2(ar*tan(0),br))),1);
        else
        
        theta2ref=-0.5*pi+atan2(abs((ar/br)*tan((2*abs(arear)/(br*ar))+atan2(br*tan(0.5*pi-theta1a),ar))),1);
        end
end

 if(arear>=0.5*pi*ar*br)
    theta2ref=theta1+pi;
 end
 
end
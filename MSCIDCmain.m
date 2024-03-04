%% MultiSwarm Cooperative Information-driven search and Divide and Conquer mitigation control(MSCIDC)%%
% This software contains MultiSwarm Cooperative Information-driven search
% and Divide and Conquer mitigation control(MSCIDC) framework for 
% multi-swarm systems, to efficiently search and neutralize
% dynamic target (forest fire) in an unknown/uncertain environment.
%%
% The script implements work reported in
%J. John, K. Harikumar, J. Senthilnath and S. Sundaram, 
% "An Efficient Approach With Dynamic Multiswarm of UAVs for Forest Firefighting," 
% in IEEE Transactions on Systems, Man, and Cybernetics: Systems, doi: 10.1109/TSMC.2024.3352660.

clear all
close all
global sense i j counter sw ns nswarm nfire distout lsense x1 y1 r1 xmax ymax xmin ymin rswarm dt r ldsense vx1 vy1 SwarmCen

% All position units are in meters (m), velocity units are in m/s, time
% units are in seconds.

tend=60*60*20;
td=0.1;
mc=100;
totalareamc=zeros(mc,1);
targetconfirmmc=zeros(mc,tend/td);
totaltarformmc=zeros(mc,tend/td);
totaltarmitigatedmc=zeros(mc,tend/td);
totaltarremainmc=zeros(mc,tend/td);
uavmitigatemc=zeros(mc,tend/td);
uavsearchmc=zeros(mc,tend/td);
quenchswarmmc=zeros(mc,tend/td);
searchswarmmc=zeros(mc,tend/td);
for mc=1:100
    nuav=15;
    nfire=5;
    nswarm=7;
    ns=[3 2 2 2 2 2 2];
    
    %nuav/nswarm*ones(1,nswarm);
    
    
    %------------------------------------------------
    tover=0; % Flag to check all fire over.
    
    dt=1; % step length multiplier
    td=0.1; % simulation sampling time
    
    rcomm=20; % inter UAV comm radius
    xmin=rcomm;ymin=rcomm;   % boundary settings
    xmax=10000-rcomm;
    ymax=10000-rcomm;
    sendev=100;% fire sensor standard deviation in m
    rswarm=800;
    
    r1=0.8*rswarm;  % levy length (m)
    r=r1/10; % browninan length (m)
    
    
    w=5;%5–30 tha
    W=40;%15–40tha
    H=18600;
    FDI=50;
    Lf=4;
    I=259.833*Lf^2.174;
    rt=I/(H*W*0.1);%1tha=0.1kg/m2
    eta=0.7;
    Lv=2640;
    h=0.077;
    Tg=1090;
    Tfuel=588;
    mcr=0.0129;
    tau=1;
    phi=1;
    D=10;
    qE=((0.27*I)*tau*phi/((2*Lf)+D))+h*(Tg-Tfuel);
    CF=mcr+qE/(eta*Lv);
    Ld=1;
    WR=CF*Ld*D;
    Va=20;
    deltaA=1.5/CF;
    La=4.3196;
    nstar=pi*La^2/deltaA;
    deltaR=sqrt(nstar*deltaA/pi);
    xfire=[200,300,400,800,900]*10; % fire center X coordinate
    yfire=[600,900,300,200,800]*10;  % fire center Y coordina
    
    rf=[25	10  20  10  5]*10;
    bf=[5	5   0   0   0]*10;
    
    
    fireend=zeros(1,nfire);
    %different fire locations analysis
%     nfire =10
%     xfire =[1000        2000        3000        4000        5000        6000        7000        7500        8000        9000];
%     yfire =[1000        6000        9000        2000        5000        8000        6000        2000        3000        9000];
%     nfire=15
%     xfire =[1000        1200        2000        2500        3000        4000        4000        5000        6000        6000        7000        7500        8000        8000        9000];
%     yfire =[1000        8700        6000        3500        9000        2000        7000        5000        4000        8000        6000        2000        3000        8000        9000];
%     rf=ones(1,nfire)*5;
%     bf=ones(1,nfire)*40; 


    lowthreshold=0.1; % probability lower limit to detect fire
    
    
    firstmemberdetector=zeros(nfire,nswarm);
    firstleader=zeros(1,nfire);
    ffseen=zeros(nfire,nuav);
    waystart=zeros(nfire,nuav);
    errwaystart=100*ones(nfire,nuav);
    
    
    
    
    
    mitigate=zeros(nfire,nuav);
    tseen=zeros(nfire,nuav);
    tref=zeros(nfire,nuav);
    
    timestamp=zeros(1,nfire);
    timedetect=zeros(1,nfire);
    targetdetect=zeros(1,nfire);
    timetoquench=zeros(nfire,nuav);
    quenchtime=zeros(1,nuav);
    searchtime=zeros(1,nuav);
    
    targetfind=zeros(nfire,nswarm);
    swarmdet=zeros(1,nfire);
    farea=[];
    fire=[];
    targetconfirm=[];
    totaltarform=[];
    totaltarmitigated=[];
    uavmitigate=[];
    uavsearch=[];
    totaltarremain=[];
    quenchswarm=[];
    searchswarm=[];
    tim=[];
    lastcounter=0;
    
    oppflag=zeros(1,nuav);
    countopp=zeros(1,nuav);
    fflag=ones(1,nfire);
    fdetect=ones(1,nfire);
    firedetectswarm=zeros(nfire,nswarm);
    quenchflag=zeros(nfire,nswarm);
    firequenchflag=zeros(1,nfire);
    
    angerr=zeros(nfire,nuav);
   
    uavorder=zeros(nfire,nuav);
    quenchuav=zeros(nfire,nuav);
    orientref=zeros(nfire,nuav+1);
    angref_mod=zeros(nfire,nuav+1);
    
    mergeflag=zeros(nfire,nswarm);

    maxval_s=[];
    lsense=[];
    sense=[];
    anglev=[];
    angddb=[];
    angbro=[];
    maxlev=[];
    maxddb=[];
    maxbro=[];
    
    leader=zeros(1,nuav);
    follower=zeros(1,nuav);
    
    
    graphmat=eye(nuav);
    uavseen=zeros(1,nuav);
    
    
    switchcount=0.0;
    store=zeros(1,nuav);
    
    targetconfirm=zeros(1,tend/td);
    totaltarform=zeros(1,tend/td);
    totaltarmitigated=zeros(1,tend/td);
    totaltarremain=zeros(1,tend/td);
    uavmitigate=zeros(1,tend/td);
    uavsearch=zeros(1,tend/td);
    quenchswarm=zeros(1,tend/td);
    searchswarm=zeros(1,tend/td);
    
    %coverage evaluation
    grid_size=50;
    xg = grid_size/2:grid_size:(xmax+rcomm-grid_size/2);
    yg = grid_size/2:grid_size:(ymax+rcomm-grid_size/2);
    [Xg,Yg] = meshgrid(xg,yg);
    [gr, gc]=size(Xg);
    grid=zeros(size(Xg));
    
    
    
    %%swarm initialisation
    
    for n=1:nswarm
        if n==1
            xc1=xmax*rand;
            yc1=ymax*rand;
            
            rn1=rand(1,ns(n));
            angle =2*pi*rand(1,ns(n));
            x1(1:ns(n))=rn1*(rswarm-0.1*rswarm).*cos(angle)+xc1;% initial x position of UAVs
            y1(1:ns(n))=rn1*(rswarm-0.1*rswarm).*sin(angle)+yc1; % initial y position of UAVs
        else
            xc1=xmax*rand;
            
            yc1=ymax*rand;
            
            rn1=rand(1,ns(n));
            angle =2*pi*rand(1,ns(n));
            x1(sum(ns(1:n-1))+1:sum(ns(1:n)))=rn1*(rswarm-0.1*rswarm).*cos(angle)+xc1;% initial x position of UAVs
            y1(sum(ns(1:n-1))+1:sum(ns(1:n)))=rn1*(rswarm-0.1*rswarm).*sin(angle)+yc1; % initial y position of UAVs
        end
    end
    
    % changing initial points if the generated point is outside search region
    for i=1:nuav
        while x1(i)>xmax|| x1(i)<xmin||y1(i)>ymax||y1(i)<ymin
            for n=1:nswarm
                if n==1
                    xc1=xmax*rand;
                    yc1=ymax*rand;
                    rn1=rand(1,ns(n));
                    angle =2*pi*rand(1,ns(n));
                    x1(1:ns(n))=rn1*(rswarm-0.1*rswarm).*cos(angle)+xc1;% initial x position of UAVs
                    y1(1:ns(n))=rn1*(rswarm-0.1*rswarm).*sin(angle)+yc1; % initial y position of UAVs
                else
                    xc1=(xmax)*rand;
                    yc1=(ymax)*rand;
                    rn1=rand(1,ns(n));
                    angle =2*pi*rand(1,ns(n));
                    x1(sum(ns(1:n-1))+1:sum(ns(1:n)))=rn1*(rswarm-0.1*rswarm).*cos(angle)+xc1;% initial x position of UAVs
                    y1(sum(ns(1:n-1))+1:sum(ns(1:n)))=rn1*(rswarm-0.1*rswarm).*sin(angle)+yc1; % initial y position of UAVs
                end
            end
        end
    end
    weight=ones(1,nuav);
    Sposi=[x1' y1'];
    for s=1:nswarm
        if s==1
            SwarmCen(s,:)=weight(1,1:ns(s))*Sposi(1:ns(s),:)/ns(s);
        else
            SwarmCen(s,:)=weight(1,sum(ns(1:s-1))+1:sum(ns(1:s)))*Sposi(sum(ns(1:s-1))+1:sum(ns(1:s)),:)/ns(s);
        end
    end
    
    
    
    for jj=1:1
        
        vx1= zeros(nuav,1);
        vy1= zeros(nuav,1);
        
        % initial ref positions
        for i=1:nuav
            z=levy(1,2,1);
            x1ref(i)=x1(i)+(dt*r1*z(1)/sqrt(z*z'));
            y1ref(i)=y1(i)+(dt*r1*z(2)/sqrt(z*z'));
            
            
            
            if((x1ref(i)>xmax)||(x1ref(i)<xmin) )
                x1ref(i)=x1(i)-(dt*r1*z(1)/sqrt(z*z'));
            end
            if((y1ref(i)>ymax) ||(y1ref(i)<ymin))
                y1ref(i)=y1(i)-(dt*r1*z(2)/sqrt(z*z'));
            end
            for s=1:nswarm
                if i<=sum(ns(1:s))
                    sw=s;
                    break;
                end
            end
            currentpoint=[x1ref(i) y1ref(i) x1(i) y1(i) SwarmCen(sw,1) SwarmCen(sw,2) rswarm];
            newpoint=refpoint(currentpoint);
            x1ref(i)=newpoint(1);
            y1ref(i)=newpoint(2);
            
            
            dist_way(i)=sqrt((x1ref(i)-x1(i))^2+(y1ref(i)-y1(i))^2);
        end
        
        waycount(i)=1;
        
        for counter=1:(tend/td)
            
            Sposi=[x1' y1'];
            for s=1:nswarm
                if s==1
                    SwarmCen(s,:)=weight(1,1:ns(s))*Sposi(1:ns(s),:)/ns(s);
                else
                    SwarmCen(s,:)=weight(1,sum(ns(1:s-1))+1:sum(ns(1:s)))*Sposi(sum(ns(1:s-1))+1:sum(ns(1:s)),:)/ns(s);
                end
            end
            
            sumfireall=sum(rf(1:nfire));
            
            
            if(sumfireall<0.5 && tover<1)
                tfinish(mc)=tim(counter-1);
                tover=1;
            end
            
            %setting nominal angular velocity of UAVs for formation
            for f=1:nfire
                if(rf(f)>100)
                    speed(f)=1;
                elseif(rf(f)<50)
                    speed(f)=0.5;
                else
                    speed(f)=0.5+(0.5/50)*(rf(f)-50);
                end
            end
            
            
            tim(counter)=counter*td;
            
            
            for i=1:nuav
                for s=1:nswarm
                    if i<=sum(ns(1:s))
                        sw=s;
                        break;
                    end
                end
                % Checking whether near to fire location
                for f=1:nfire                    
                    if rf(f)~=0
                        distout(f,:)=sqrt((x1-xfire(f)).*(x1-xfire(f))+(y1-yfire(f)).*(y1-yfire(f)));
                        distoutd(f,:)=exp(-0.5*(distout(f,:)-rf(f)-bf(f)).*(distout(f,:)-rf(f)-bf(f))/(sendev*sendev));  % sensor gaussian model
                    else
                        distout(f,:)=10^10*ones(1,nuav);
                        distoutd(f,:)=zeros(1,nuav);
                    end
                end
                
                
                if sw==1
                    maxprob(sw)=max(max(distoutd(:,1:ns(sw)),[],2));
                    [mindist,mindistfire(sw)]=min(min(distout(:,1:ns(sw)),[],2));
                    if rf(mindistfire(sw))~=0
                        lsense(1:ns(sw),counter)=(1./(ones(1,ns(sw))+distout(mindistfire(sw),1:ns(sw))-rf(mindistfire(sw))-bf(mindistfire(sw))));
                    else
                        lsense(1:ns(sw),counter)=zeros(1,ns(sw));
                    end
                else
                    maxprob(sw)=max(max(distoutd(:,sum(ns(1:sw-1))+1:sum(ns(1:sw))),[],2));
                    [mindist,mindistfire(sw)]=min(min(distout(:,sum(ns(1:sw-1))+1:sum(ns(1:sw))),[],2));
                    if rf(mindistfire(sw))~=0
                        lsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter)=(1./(ones(1,ns(sw))+distout(mindistfire(sw),sum(ns(1:sw-1))+1:sum(ns(1:sw)))-rf(mindistfire(sw))-bf(mindistfire(sw))));
                    else
                        lsense(1:ns(sw),counter)=zeros(1,ns(sw));
                    end
                end
                
                
                
                if sw==1 || nfire==1
                    if counter==1
                        [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(1:i,counter));
                    else
                        maxval_s(sw,counter)= max(lsense(1:ns(sw),counter));
                    end
                else
                    if counter==1
                        [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(sum(ns(1:sw-1))+1:i,counter));
                    else
                        maxval_s(sw,counter)= max(lsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter));
                    end
                    maxinfouav(sw)=maxinfouav(sw)+sum(ns(1:sw-1));
                end
                if maxprob(sw)>=lowthreshold
                    sense(sw,counter)=maxval_s(sw,counter);
                elseif maxprob(sw)<lowthreshold
                    sense(sw,counter)=0;
                end
                if(counter>1)
                    dsense(sw,counter)=sense(sw,counter)-sense(sw,counter-1);
                    
                    ldsense(:,counter)=lsense(:,counter)-lsense(:,counter-1);
                else
                    dsense(sw,counter)=0;
                    ldsense(i,counter)=0;
                end
                
                
                for s1=1:nswarm
                    for s2=1:nswarm
                        if s1~=s2
                            distswarm(s1,s2)=abs(norm([SwarmCen(s1,1)-SwarmCen(s2,1),SwarmCen(s1,2)-SwarmCen(s2,2)],2)-2*rswarm);
                        else
                            distswarm(s1,s2)=0;
                        end
                    end
                end
                dist_way(i)=sqrt((x1ref(i)-x1(i))^2+(y1ref(i)-y1(i))^2);
                
                if (dist_way(i)<=r/10)&& oppflag(i)==0
                    for s=1:nswarm
                        if i<=sum(ns(1:s))
                            sw=s;
                            break;
                        end
                    end
                    if nnz(ffseen(:,i))==0
                        waycount(i)=waycount(i)+1;
                        if(sense(sw,counter)==0)
                            
                            if sw==1 || nfire==1
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(1:i,counter));
                                else
                                    [maxval,maxinfouav(sw)] = max(ldsense(1:ns(sw),counter));
                                    maxval_s(sw,counter)= max(lsense(1:ns(sw),counter));
                                end
                            else
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(sum(ns(1:sw-1))+1:i,counter));
                                else
                                    [maxval,maxinfouav(sw)] = max(ldsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter));
                                    maxval_s(sw,counter)= max(lsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter));
                                end
                                maxinfouav(sw)=maxinfouav(sw)+sum(ns(1:sw-1));
                            end
                            
                            maxinfodir(sw)=atan2(vy1(maxinfouav(sw)),vx1(maxinfouav(sw)));
                            
                            diff_angle(sw)=(pi/2)*(1/(1+exp(-10*maxval_s(sw,counter))));

                            up_lim=maxinfodir(sw)+diff_angle(sw);
                            low_lim=maxinfodir(sw)-diff_angle(sw);
                            levy_ang=(up_lim-low_lim)*rand+low_lim;
                            z=abs(levy(1,1,1.0));  % Levy distribution
                            
                            x1ref(i)=x1(maxinfouav(sw))+(dt*r1*z*cos(levy_ang)/sqrt(z*z'));
                            y1ref(i)=y1(maxinfouav(sw))+(dt*r1*z*sin(levy_ang)/sqrt(z*z'));
                            
                            if((x1ref(i)>xmax)||(x1ref(i)<xmin))
                                x1ref(i)=x1(maxinfouav(sw))-(dt*r1*z*cos(levy_ang)/sqrt(z*z'));
                            end
                            if((y1ref(i)>ymax) ||(y1ref(i)<ymin))
                                y1ref(i)=y1(maxinfouav(sw))-(dt*r1*z*sin(levy_ang)/sqrt(z*z'));
                            end
                            currentpoint=[x1ref(i) y1ref(i) x1(i) y1(i) SwarmCen(sw,1) SwarmCen(sw,2) rswarm];
                            newpoint=refpoint(currentpoint);
                            x1ref(i)=newpoint(1);
                            y1ref(i)=newpoint(2);
                            
                        else
                            
                            magxy(i)=dt*r;
                            c_b=randn(1);
                            if sw==1 || nfire==1
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(1:i,counter));
                                else
                                    [maxval,maxinfouav(sw)] = max(ldsense(1:ns(sw),counter));
                                    maxval_s(sw,counter)= max(lsense(1:ns(sw),counter));
                                end
                            else
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(sum(ns(1:sw-1))+1:i,counter));
                                else
                                    [maxval,maxinfouav(sw)] = max(ldsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter));
                                    maxval_s(sw,counter)= max(lsense(sum(ns(1:sw-1))+1:sum(ns(1:sw)),counter));
                                end
                                maxinfouav(sw)=maxinfouav(sw)+sum(ns(1:sw-1));
                            end
                            maxinfodir(sw)=atan2(vy1(maxinfouav(sw)),vx1(maxinfouav(sw)));
                            
                            % normal distribution for Browninan motion
                            
                            diff_angle(sw)=(pi/2)*(1/(1+exp(-10*maxval_s(sw,counter))));
                            
                            B_up_lim=maxinfodir(sw)+diff_angle(sw);
                            B_low_lim=maxinfodir(sw)-diff_angle(sw);
                            brown_ang=(B_up_lim-B_low_lim)*rand+B_low_lim;
                            
                            
                            x1ref(i)=x1(maxinfouav(sw))+(magxy(i)*c_b*cos(brown_ang)/sqrt(c_b*c_b'));
                            y1ref(i)=y1(maxinfouav(sw))+(magxy(i)*c_b*sin(brown_ang)/sqrt(c_b*c_b'));
                            
                            % Modifying waypoints if generated outside the boundary
                            
                            if(x1ref(i)<xmin)
                                
                                x1ref(i)=x1(maxinfouav(sw))-(magxy(i)*c_b*cos(brown_ang)/sqrt(c_b*c_b'));
                            end
                            if(x1ref(i)>xmax)
                                
                                x1ref(i)=x1(maxinfouav(sw))-(magxy(i)*c_b*cos(brown_ang)/sqrt(c_b*c_b'));
                            end
                            
                            if(y1ref(i)<ymin)
                                
                                y1ref(i)=y1(maxinfouav(sw))-(magxy(i)*c_b*sin(brown_ang)/sqrt(c_b*c_b'));
                            end
                            if(y1ref(i)>ymax)
                                
                                y1ref(i)=y1(maxinfouav(sw))-(magxy(i)*c_b*sin(brown_ang)/sqrt(c_b*c_b'));
                            end
                            
                            currentpoint=[x1ref(i) y1ref(i) x1(i) y1(i) SwarmCen(sw,1) SwarmCen(sw,2) rswarm];
                            newpoint=refpoint(currentpoint);
                            x1ref(i)=newpoint(1);
                            y1ref(i)=newpoint(2);
                        end
                    end
                end
                
                dist_way(i)=sqrt((x1ref(i)-x1(i))^2+(y1ref(i)-y1(i))^2);
                
                
                vx1refold=0.0;
                vy1refold=0.0;
                vx1ref(i)=0;
                vy1ref(i)=0;
                vflag(i)=0;
                vx1refsold(i)=0;
                vy1refsold(i)=0;
                
                
                
                if(x1(i)<(xmin+rcomm))
                    vx1ref(i)=0.0;
                end
                
                if(y1(i)<(ymin+rcomm))
                    vy1ref(i)=0.0;
                end
                
                if(x1(i)>(xmax-rcomm))
                    vx1ref(i)=0.0;
                end
                
                if(x1(i)>(xmax-rcomm))
                    vy1ref(i)=0.0;
                end
                
                
                
                % %----- fire starts ---------------- for f=1:nfire
                for f=1:nfire
                    for s=1:nswarm
                        if i<=sum(ns(1:s))
                            sw=s;
                            break;
                        end
                    end
                    if ((distoutd(f,i)>0.5)||ffseen(f,i)==1)&& oppflag(i)==0
                        if (rf(f)>0.5)
                            firequenchflag(f)=1;
                            if(ffseen(f,i)==0)
                                tseen(f,i)=counter;
                                ffseen(f,i)=1;
                                uavorder(f,i)=sum(ffseen(f,:));
                                
                                if timedetect(f)==0
                                    timedetect(f)=tim(counter);
                                    targetdetect(f)=1;
                                end
                            end
                            
                            if nnz(ffseen(f,:))==1
                                swarmdet(f)=sw;
                                firstleader(f)=i;
                                fdetect(f)=0;
                            end
                            
                            
                            targetfind(f,sw)=targetfind(f,sw)+1;
                            if(targetfind(f,sw)==1)
                                firedetectswarm(f,sw)=1;
                                firstmemberdetector(f,sw)=i;
                                swarmorder(f,sw)=sum(firedetectswarm(f,:));
                            end
                            
                            rstar(f)=sqrt(sum(ffseen(f,:))*deltaA/pi);
                            
                            if(sw==swarmdet(f))||((rf(f)>25*10||nnz(fdetect)<2)&&(nnz(firedetectswarm(f,:))<=(nswarm/2)))
                                mergeflag(f,sw)=1;
                            else
                                if  oppflag(i)==0 && swarmorder(f,sw)==max(swarmorder(f,:))
                                    mergeflag(f,sw)=0;
                                    firedetectswarm(f,sw)=0;
                                    targetfind(f,sw)=0;
                                    swarmorder(f,sw)=0;
                                    if sw==1
                                        for j=1:ns(sw)
                                            ffseen(f,j)=0;
                                            tseen(f,j)=0;
                                            uavorder(f,j)=0;
                                            oppflag(j)=1;
                                            oppfire(j)=f;
                                            
                                        end
                                    else
                                        for j=sum(ns(1:sw-1))+1:sum(ns(1:sw))
                                            ffseen(f,j)=0;
                                            tseen(f,j)=0;
                                            uavorder(f,j)=0;
                                            oppflag(j)=1;
                                            oppfire(j)=f;
                                            
                                        end
                                    end
                                    search_refpoint=searchopp;
                                    x1ref(i)=search_refpoint(1);
                                    y1ref(i)=search_refpoint(2);
                                    dist_opp(i)=search_refpoint(3);
                                    countopp(i)=1;
                                end
                            end
                            if mergeflag(f,sw)==1
                                mit_swarm(f,counter)=sum(firedetectswarm(f,:));
                                if sw==1
                                    for j=1:ns(sw)
                                        if j~=firstmemberdetector(f,sw)&&ffseen(f,j)==0
                                            ffseen(f,j)=1;
                                            tseen(f,j)=tseen(f,firstmemberdetector(f,sw));
                                            uavorder(f,j)=sum(ffseen(f,:));
                                        end
                                        orientref(f,:)=zeros(1,nuav+1);
                                        angref_mod(f,:)=zeros(1,nuav+1);
                                        for jj=2:sum(ffseen(f,:))
                                            orientref(f,jj)=ellipseang(rf(f)+bf(f),rf(f),orientref(f,jj-1),sum(ffseen(f,:)));
                                            if orientref(f,jj)<0
                                                angref_mod(f,jj)=orientref(f,jj)+2*pi;
                                            else
                                                angref_mod(f,jj)=orientref(f,jj);
                                            end
                                        end
                                        angref_mod(f,sum(ffseen(f,:))+1)=2*pi;
                                        x1ref(j)=xfire(f)+(rf(f)+bf(f))*cos(angref_mod(f,uavorder(f,j)));
                                        y1ref(j)=yfire(f)+rf(f)*sin(angref_mod(f,uavorder(f,j)));
                                        
                                    end
                                else
                                    for j=sum(ns(1:sw-1))+1:sum(ns(1:sw))
                                        if j~=firstmemberdetector(f,sw)&& ffseen(f,j)==0
                                            ffseen(f,j)=1;
                                            tseen(f,j)=tseen(f,firstmemberdetector(f,sw));
                                            uavorder(f,j)=sum(ffseen(f,:));
                                        end
                                        orientref(f,:)=zeros(1,nuav+1);
                                        angref_mod(f,:)=zeros(1,nuav+1);
                                        for jj=2:sum(ffseen(f,:))
                                            orientref(f,jj)=ellipseang(rf(f)+bf(f),rf(f),orientref(f,jj-1),sum(ffseen(f,:)));
                                            if orientref(f,jj)<0
                                                angref_mod(f,jj)=orientref(f,jj)+2*pi;
                                            else
                                                angref_mod(f,jj)=orientref(f,jj);
                                            end
                                        end
                                        angref_mod(f,sum(ffseen(f,:))+1)=2*pi;
                                        x1ref(j)=xfire(f)+(rf(f)+bf(f))*cos(angref_mod(f,uavorder(f,j)));
                                        y1ref(j)=yfire(f)+rf(f)*sin(angref_mod(f,uavorder(f,j)));
                                        
                                    end
                                end
                                
                                
                                errwaystart(f,i)=sqrt((x1(i)-x1ref(i))*(x1(i)-x1ref(i))+(y1(i)-y1ref(i))*(y1(i)-y1ref(i)));
                                if(errwaystart(f,i)<25)
                                    if(waystart(f,i)==0)
                                        tref(f,i)=counter;
                                        quenchflag(f,sw)=1;
                                        quenchuav(f,i)=1;
                                    end
                                    if (waystart(f,firstmemberdetector(f,swarmdet(f)))==0)
                                        rfq(f)=rf(f);
                                        bfq(f)=bf(f);
                                    end
                                    waystart(f,i)=1;
                                end
                                
                                arear(f)=pi*(rf(f)+bf(f))*(rf(f))/sum(ffseen(f,:));
                                if(waystart(f,i)>0)
                                    
                                    if(counter<=tref(f,i))
                                        mtheta(f,i)=angref_mod(f,uavorder(f,i));
                                        thetaref(f,i)=angref_mod(f,uavorder(f,i));
                                        direc(f,i)=1;
                                    else
                                        if abs(mit_swarm(f,counter)-mit_swarm(f,counter-1))>0
                                            thetaref(f,i)=angref_mod(f,uavorder(f,i));
                                            direc(f,i)=sign(angref_mod(f,uavorder(f,i))-mtheta(f,i));
                                        end
                                        if direc(f,i)==1
                                            thetaref(f,i)=thetaref(f,i)+td*4*speed(f)/rf(f);
                                            if angref_mod(f,uavorder(f,i)+1)-thetaref(f,i)<2/rf(f)
                                                direc(f,i)=-1;
                                            end
                                        else
                                            thetaref(f,i)=thetaref(f,i)-td*4*speed(f)/rf(f);
                                            if thetaref(f,i)-angref_mod(f,uavorder(f,i))<2/rf(f)
                                                direc(f,i)=1;
                                            end
                                        end
                                        angerr(f,i)=mtheta(f,i)-thetaref(f,i);
                                        omega(f,i)=(4*speed(f)/rf(f)-0.2*(angerr(f,i)));
                                        mtheta(f,i)=mtheta(f,i)+td*omega(f,i);
                                        
                                    end
                                    
                                    x1ref(i)=xfire(f)+(rf(f)+bf(f))*cos(mtheta(f,i));
                                    y1ref(i)=yfire(f)+rf(f)*sin(mtheta(f,i));
                                    vflag(i)=0;
                                    
                                end
                            end
                        end
                    end
                    
                    if (distoutd(f,i)>0.01&&swarmdet(f)~=0&&sw~=swarmdet(f))&&((rf(f)<25*10&&nnz(fdetect)>=2)||(nnz(firedetectswarm(f,:))>(nswarm/2)))
                        if  oppflag(i)==0
                            mergeflag(f,sw)=0;
                            firedetectswarm(f,sw)=0;
                            targetfind(f,sw)=0;
                            swarmorder(f,sw)=0;
                            if sw==1
                                for j=1:ns(sw)
                                    ffseen(f,j)=0;
                                    tseen(f,j)=0;
                                    uavorder(f,j)=0;
                                    oppflag(j)=1;
                                    oppfire(j)=f;
                                    search_refpoint=searchopp;
                                    x1ref(j)=search_refpoint(1);
                                    y1ref(j)=search_refpoint(2);
                                    dist_opp(j)=search_refpoint(3);
                                    countopp(j)=1;
                                    
                                end
                            else
                                for j=sum(ns(1:sw-1))+1:sum(ns(1:sw))
                                    ffseen(f,j)=0;
                                    tseen(f,j)=0;
                                    uavorder(f,j)=0;
                                    oppflag(j)=1;
                                    oppfire(j)=f;
                                    search_refpoint=searchopp;
                                    x1ref(j)=search_refpoint(1);
                                    y1ref(j)=search_refpoint(2);
                                    dist_opp(j)=search_refpoint(3);
                                    countopp(j)=1;
                                    
                                end
                            end
                        end
                    end
                    
                end
                
                if  oppflag(i)==1
                    if countopp(i)>5 || rf(oppfire(i))==0
                        if sw==1
                            for j=1:ns(sw)
                                oppflag(j)=0;
                                countopp(j)=0;
                            end
                        else
                            for j=sum(ns(1:sw-1))+1:sum(ns(1:sw))
                                oppflag(j)=0;
                                countopp(j)=0;
                            end
                        end
                    end
                    dist_opp(i)=sqrt((x1ref(i)-x1(i))^2+(y1ref(i)-y1(i))^2);
                    if rf(oppfire(i))>0 && dist_opp(i)<=r/2
                        if sw==1
                            for j=1:ns(sw)
                                search_refpoint=searchopp;
                                x1ref(j)=search_refpoint(1);
                                y1ref(j)=search_refpoint(2);
                                dist_opp(j)=search_refpoint(3);
                                countopp(j)=countopp(j)+1;
                            end
                        else
                            for j=sum(ns(1:sw-1))+1:sum(ns(1:sw))
                                search_refpoint=searchopp;
                                x1ref(j)=search_refpoint(1);
                                y1ref(j)=search_refpoint(2);
                                dist_opp(j)=search_refpoint(3);
                                countopp(j)=countopp(j)+1;
                            end
                        end
                        
                    end
                end
                
                
                % UAV dynamics
                
                xin1=[x1(i),y1(i),vx1(i),vy1(i),x1ref(i),y1ref(i),15*vx1ref(i),15*vy1ref(i),vflag(i)];
                xout1=uavdynamicsvelocity(xin1);
                x1(i)=xout1(1);
                y1(i)=xout1(2);
                vx1(i)=xout1(3);
                vy1(i)=xout1(4);
                distg=sqrt((x1(i)-Xg).*(x1(i)-Xg)+(y1(i)-Yg).*(y1(i)-Yg));
                probg=exp(-0.5*(distg).*(distg)/(sendev*sendev));
                grid_n=find(probg>0.5);
                grid(grid_n)=1;
            end
            
            
            Sposi=[x1' y1'];
            for s=1:nswarm
                if s==1
                    SwarmCen(s,:)=weight(1,1:ns(s))*Sposi(1:ns(s),:)/ns(s);
                else
                    SwarmCen(s,:)=weight(1,sum(ns(1:s-1))+1:sum(ns(1:s)))*Sposi(sum(ns(1:s-1))+1:sum(ns(1:s)),:)/ns(s);
                    
                end
            end
            for i=1:nuav
                for s=1:nswarm
                    if i<=sum(ns(1:s))
                        sw=s;
                        break;
                    end
                end
                
            end
            
            % fire model%---------------------------------------------------------
            for f=1:nfire
                if(rf(f)>0.5)
                    for i=1:nuav
                        if(ffseen(f,i)==1)&&(waystart(f,i)>0)
                            test(f)=deltaA;
                            mitigate(f,i)=test(f)*(counter-tref(f,i)+1)*td;
                        end
                    end
                end
                
                if(rf(f)>0.5)
                    if(sum(ffseen(f,:))>0)&& nnz(tref(f,:))>0
                        firearea(f)=pi*((rfq(f))*(rfq(f)+bfq(f)))-sum(mitigate(f,:));
                        rf(f)=sqrt((firearea(f)/pi)+0.25*bf(f)*bf(f))-0.5*bf(f);
                        
                    else
                        rf(f)=rf(f)+rt*td;
                        
                    end
                else
                    rf(f)=0.0;
                    bf(f)=0;
                end
                
                if(rf(f)<0.5)
                    rf(f)=0.0;
                    ffseen(f,:)=zeros(1,nuav);
                    quenchflag(f,:)=zeros(1,nswarm);
                    
                end
                if (rf(f)==0)&&(fflag(f)==1)
                    timestamp(f)=tim(counter);
                    fflag(f)=0;
                    firequenchflag(f)=0;
                    for i=1:nuav
                        if tref(f,i)~=0
                            timetoquench(f,i)=(timestamp(f)-tref(f,i)*td);
                        end
                    end
                end
            end
            
            
            %----------------------------------------------% fire area
            for f=1:nfire
                fire(f,counter)=rf(f);
                farea(f,counter)=pi*rf(f)*(rf(f)+bf(f));
            end
            
            fireall=rf;
            totaltarform(counter)=sum(firequenchflag);
            for mm=1:nfire
                if(fireall(mm)<0.5)
                    fireend(mm)=1;
                end
            end
            targetconfirm(counter)=nnz(targetdetect);
            totaltarmitigated(counter)=sum(fireend);
            
            uavmitigate(counter)=sum(ffseen,'all');
            uavsearch(counter)=nuav-uavmitigate(counter);
            totaltarremain(counter)=nfire-totaltarmitigated(counter);
            quenchswarm(counter)=sum(quenchflag,'all');
            
            searchswarm(counter)=nswarm-quenchswarm(counter);
            
            
            % plotting fire spots
            for f=1:nfire
                for kk=1:100
                    theta(kk)=(kk-1)*2*pi/99;
                    xf(f,kk)=xfire(f)+(rf(f)+bf(f))*cos(theta(kk));
                    yf(f,kk)=yfire(f)+(rf(f))*sin(theta(kk));
                end
            end
            
            
            
            
            if mod(counter,150)==0
                for i=1:nuav
                    plot(x1(i),y1(i),'>')
                    hold on
                end
                grid on
                
                axis equal
                for f=1:nfire
                    plot(xf(f,:),yf(f,:),'.-r')
                end
             
                axis([0,xmax+rcomm,0,ymax+rcomm]);
                
                
                xlabel ('X - Position')
                ylabel ('Y - Position')
                ang = 0:0.01:2*pi;
                for s=1:nswarm
                    plot(SwarmCen(s,1)+rswarm*cos(ang),SwarmCen(s,2)+rswarm*sin(ang),'--g') % plot circle....
                end
                pause(0.05);
                hold off
            end
            if tover==1 && lastcounter==0
                lastcounter=counter;
            end
            if tover==1 && counter>=lastcounter+200
                break
            end
            
            %             posx(counter,:)=x1;
            %             posy(counter,:)=y1;
            %             rfc(counter,:)=rf;
            %             bfc(counter,:)=bf;
            
        end
    end
    coverage_rate=sum(grid,'all')/(gr*gc);
    initialarea=sum(farea(:,1));
    totalarea=sum(max(farea,[],2));
    totalareamc(mc,:)=totalarea;
    targetconfirmmc(mc,:)=targetconfirm;
    totaltarformmc(mc,:)=totaltarform;
    totaltarmitigatedmc(mc,:)=totaltarmitigated;
    totaltarremainmc(mc,:)=totaltarremain;
    uavmitigatemc(mc,:)=uavmitigate;
    uavsearchmc(mc,:)=uavsearch;
    quenchswarmmc(mc,:)=quenchswarm;
    searchswarmmc(mc,:)=searchswarm;
    quenchtime=sum(timetoquench);
    searchtime=max(timestamp)-quenchtime;
    quenchtimeperc=quenchtime/max(timestamp);
    searchtimeperc=searchtime/max(timestamp);
    timestampmc(mc,:)=timestamp;
    timedetectmc(mc,:)=timedetect;
    quenchtimemc(mc,:)=quenchtime;
    searchtimemc(mc,:)=searchtime;
    quenchtimepercmc(mc,:)=quenchtimeperc;
    searchtimepercmc(mc,:)=searchtimeperc;
    coverage_ratemc(mc)=coverage_rate;
end
for v=1:counter
    t1avg(v)=(1/mc)*sum(totaltarformmc(:,v));
    t2avg(v)=(1/mc)*sum(totaltarmitigatedmc(:,v));
    t3avg(v)=(1/mc)*sum(uavmitigatemc(:,v));
    t4avg(v)=(1/mc)*sum(uavsearchmc(:,v));
    t5avg(v)=(1/mc)*sum(totaltarremainmc(:,v));
    t6avg(v)=(1/mc)*sum(targetconfirmmc(:,v));
end
figure
hold on
for f=1:nfire
    
    plot(tim,farea(f,:), 'LineWidth', 2)
end
xlabel('time')
ylabel('Fire area')
legend('fire 1','fire 2','fire 3','fire 4','fire 5')
grid on

timestampmc_asc=sort(timestampmc,2);
timedetectmc_asc=sort(timedetectmc,2);
spreadareapercmc=(totalareamc-initialarea)/initialarea*100;
totalareaavg=(1/mc)*sum(totalareamc);
spreadareaperc=(1/mc)*sum(spreadareapercmc);
firequenchtimemc=timestampmc_asc-timedetectmc_asc;
timestampavg=(1/mc)*sum(sort(timestampmc,2));
timedetectavg=(1/mc)*sum(sort(timedetectmc,2));
quenchtimeavg=(1/mc)*sum(quenchtimemc);
searchtimeavg=(1/mc)*sum(searchtimemc);
quenchtimepercavg=(1/mc)*sum(quenchtimepercmc);
searchtimepercavg=(1/mc)*sum(searchtimepercmc);
quenchswarmavg=(1/mc)*sum(quenchswarmmc);
searchswarmavg=(1/mc)*sum(searchswarmmc);
coverage_rateavg=(1/mc)*sum(coverage_ratemc);

quenchswarmstd=std(quenchswarmmc,1);
searchswarmstd=std(searchswarmmc,1);
uavmitigatestd=std(uavmitigatemc,1);
uavsearchmstd=std(uavsearchmc,1);
FFstd=std(totaltarformmc,1);
FDstd=std(totaltarmitigatedmc,1);
FRstd=std(totaltarremainmc,1);
display('Average time to mitigate targets in ascending order of detection')
display(timestampavg/60)
display('Average time to detect targets in ascending order of detection')
display(timedetectavg/60)
display('Average FER')
display(spreadareaperc/100)
display('Average coverage rate')
display(coverage_rateavg)




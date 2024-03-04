function search_refpoint=searchoppnew
global sense i j counter sw ns nswarm nfire distout lsense x1 y1 r1 xobs yobs xmax ymax xmin ymin robs rswarm dt r ldsense vx1 vy1 SwarmCen SwarmCenref j
                           if sw==1 || nfire==1
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(1:j,counter));
                                else
                                    [maxval,maxinfouav(sw)] = max(ldsense(1:ns(sw),counter));
                                    maxval_s(sw,counter)= max(lsense(1:ns(sw),counter));
                                end
                            else
                                if counter==1
                                    [maxval_s(sw,counter),maxinfouav(sw)] = max(lsense(sum(ns(1:sw-1))+1:j,counter));
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
                            levy_ang=-((up_lim-low_lim)*rand+low_lim);
                            z=abs(levy(1,1,1.0));  % Levy distribution

                            x1ref(j)=x1(maxinfouav(sw))+(dt*r1*z*cos(levy_ang)/sqrt(z*z'));
                            y1ref(j)=y1(maxinfouav(sw))+(dt*r1*z*sin(levy_ang)/sqrt(z*z'));

                            
                            if((x1ref(j)>xmax)||(x1ref(j)<xmin))
                                x1ref(j)=x1(maxinfouav(sw))-(dt*r1*z*cos(levy_ang)/sqrt(z*z'));
                            end
                            if((y1ref(j)>ymax) ||(y1ref(j)<ymin))
                                y1ref(j)=y1(maxinfouav(sw))-(dt*r1*z*sin(levy_ang)/sqrt(z*z'));
                            end
                            currentpoint=[x1ref(j) y1ref(j) x1(j) y1(j) SwarmCen(sw,1) SwarmCen(sw,2) rswarm];
                            newpoint=refpoint(currentpoint);
                            x1ref(j)=newpoint(1);
                            y1ref(j)=newpoint(2);

dist_way(j)=sqrt((x1ref(j)-x1(j))^2+(y1ref(j)-y1(j))^2);
search_refpoint=[x1ref(j) y1ref(j) dist_way(j)];
end
function fitit()

    t = 0:.01:1;
    function y = force1()
       y = 0*t;
       for k = 1:length(t)
           
          if t(k) > .5
             y(k) = polyval_bz([1 1 .8 0],(t(k)-.5)*2); 
          else
             y(k) = polyval_bz([0 .8 1 1],t(k)*2); 
          end
       end
    end

    function y  = force2(bez)
        y = polyval_bz([0 bez 0],t);
    end

    function e = err(x) 
        y  =force2(x);
        e =  y - force1();
    end
    
    b0 = [.8 1 1 1 1 .8];
    
    b = lsqnonlin(@err,b0,0*b0,100+0*b0);

    y = force2(b);
    f = force1();
    
    figure(1)
    clf
    hold on
    plot(t,f,'--')
    plot(t,y,'k');

    [0 b 0]'
    
    area = sum(y)*(t(2)-t(1))
    area1= mean([0 b 0])
    
end
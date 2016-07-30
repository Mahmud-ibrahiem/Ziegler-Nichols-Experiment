 timeConst=8.5; % time const of Equivalent first order system
 delaytime=8.2; % dead time
 %first solution
 kp(1)=1.2436;
 ki(1)=.0700;
 kd(1)=.301;
 %second solution
 kp(2)=1.200;
 ki(2)=.0700;
 kd(2)=.2800;

 % Genetic Algorithm  
 
 % the values of the controller converted to binary after it was multiplied
 % by 10000 to save the 1st four decimals
 binaryKp_1=de2bi(round(kp(1)*10000));
 binaryKi_1=de2bi(round(ki(1)*10000));
 binaryKd_1=de2bi(round(kd(1)*10000));
 binaryKp_2=de2bi(round(kp(2)*10000));
 binaryKi_2=de2bi(round(ki(2)*10000));
 binaryKd_2=de2bi(round(kd(2)*10000));

 % make sure that all the binary numbers have the same length
binaryKp_1=[binaryKp_1,zeros(1,16-length(binaryKp_1))];
binaryKi_1=[binaryKi_1,zeros(1,16-length(binaryKi_1))];
binaryKd_1=[binaryKd_1,zeros(1,16-length(binaryKd_1))];
binaryKp_2=[binaryKp_2,zeros(1,16-length(binaryKp_2))];
binaryKi_2=[binaryKi_2,zeros(1,16-length(binaryKi_2))];
binaryKd_2=[binaryKd_2,zeros(1,16-length(binaryKd_2))];

% combine the solutions of the controllers to get a combined solution
 totalSolution_1=[binaryKp_1,binaryKi_1,binaryKd_1];
 totalSolution_2=[binaryKp_2,binaryKi_2,binaryKd_2];
 totalSolution=[totalSolution_1;totalSolution_2];

% Cross Over Step
totsize=size(totalSolution); % get the size of the combined solution

 for i=1:4980
     
     for j=1:totsize(2)
     newsolution(j)= totalSolution(1+round(rand*(totsize(1)-1)),j); % this step gets a random value of each binary digit in the new solution from the previous ones
     end
     totalSolution=[totalSolution;newsolution]; % add the newly generated solution to the matrix of solutions
 end
totsize=size(totalSolution); % get the dimension of the total solution
 % mutation chooses random 8 digits and changes them
  for i=1:totsize(1)
     for j=1:8 % only 8 mutations
     totalSolution(i,round(1+(totsize(2)-1)*rand))=not( totalSolution(i,round(1+(totsize(2)-1)*rand)));
     end
  end
 
 % recover the values transfere from binary to decimal
   for i=1:totsize(1)
    kp(i)=0;
    kd(i)=0;
    ki(i)=0;
       for j=1:totsize(2)/3
    kp(i)=kp(i)+totalSolution(i,j)*2^(j-1);
    kd(i)=kd(i)+totalSolution(i,totsize(2)/3+j)*2^(j-1);
    ki(i)=ki(i)+totalSolution(i,2*totsize(2)/3+j)*2^(j-1);
       end
   end
  % retrieve the decimal values
 kp=kp/10000;
 kd=kd/10000;
 ki=ki/10000;

 % choosing min steady state value 
 num = 1; % the numenator of the open loop system transfere function
 den = [timeConst 1]; % the denumenator of the open loop system transfere function
 opensys= tf(num,den); % form the transfere function
 delaysys=tf([delaytime/-2,1],[delaytime/2,1]); % an approximation of the delay transfere function  where exp(x) = (1+.5x)/(1-.5x)
 opensys= series(opensys,delaysys); % create the "FOPDT" first order plus delay time system
 for i= 1 :totsize(1) % form the transfere function off all possible systems from all possible kp,kd,ki
     % that were acheived from the cross over process
 controller=pid(kp(i),ki(i),kd(i),1);
 [num,den] =tfdata(controller);
 num=cell2mat(num);
 den=cell2mat(den);
 forward=series(opensys,tf(num,den));
 closedsystem=feedback(forward,1);
  info=stepinfo(closedsystem);
  settle(i)=info.SettlingTime;
 end
     indexOfSettleMin=find(settle==min(settle)); % get the index of the minimum settling time
    fprintf('kp=%f\n',kp(indexOfSettleMin)) % show the kp corresponding to the minimum settling time
     fprintf('ki=%f\n',ki(indexOfSettleMin)) % show the ki corresponding to the minimum settling time
     fprintf('kd=%f\n',kd(indexOfSettleMin)) % show the kd corresponding to the minimum settling time
      fprintf('The settling Time= %f\n',settle(indexOfSettleMin)); % show the settling time


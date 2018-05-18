% cg_torso_sim (sean gillen 9/13/17)
% this script serves as the entry point for the walker simulation. The meat
% of the functionality is in the two classes, but this script is the one
% creating the objects and calling the right methods
%



%this will get the current git hash, I save this with the rest of my data

[~, git_hash] = system('git rev-parse HEAD');


%clear all % clears persistent variables
%format compact
ntrials = 20; 
nparam = 3;


Pinit = zeros(ntrials,nparam);
Pfinal =-1.*ones(ntrials,nparam);


lb = zeros(nparam,1);
ub = ones(nparam,1);
    
for i = 1:ntrials
    i
   % options = optimoptions('fmincon');
   % options = optimoptions(options, 'OptimalityTolerance', 1e-4);
    
   options = optimset('Display','iter');;
   options.display = 'iter';
   Pinit(i,:) = rand(1,nparam);

    %[Pfinal(i,:), step_height(i)] = fmincon(@(in)optimMaxStep(in),Pinit(i,:),[],[],[],[],lb,ub, [] ,options);
   [Pfinal(i,:), step_height(i)] = fminsearch(@(in)optimMaxStep(in),Pinit(i,:),options); 

end
    


% walker(grid_size,grid_size) = CGTorsoWalker();
% 
% step_height= -1.*ones(grid_size,grid_size);


% parfor i = 1:grid_size
%     for j = 1:grid_size
%         (i-1)*grid_size + (j-1)
%         walker[i,j] = CGTorsoWalker;
%         walker(i,j).controller.th3_ref = 20*pi/180;
%         
%         
%         walker(i,j).L3c = j/grid_size*1; 
%         
%         walker(i,j).m3 = i/grid_size*15 + 7.5;
%         walker(i,j).m2 = (30 - walker(i,j).m3)/2;
%         walker(i,j).m1 = walker(i,j).m2;
%         
%         walker(i,j).J1 = 1/3*walker(i,j).m1*walker(i,j).L1^2
%         walker(i,j).J2 = 1/3*walker(i,j).m2*walker(i,j).L2^2
%         walker(i,j).J3 = walker(i,j).m3*walker(i,j).L3c^2
% 
%         walker(i,j).controller.kp3 = i/grid_size*1000;
%         walker(i,j).controller.kd3 = j/grid_size*200;
%         step_height(i,j) = maxStep(walker(i,j),1);    
%     end
% end
% 


% parfor i = 1:grid_size
%     for j = 1:grid_size
%         
%         
%         (i-1)*grid_size + (j-1)
%         
%         walker(i,j).controller.CType = 1; %absolute reference for th3, rel for th2
%         %walker[i,j] = CGTorsoWalker;
%         walker(i,j).controller.th3_ref = (j/grid_size)*(80*pi/180);
%         walker(i,j).controller.th2_ref = ((180) + (i/grid_size)*90)*pi/180;
%         
%         
%         walker(i,j).m3 = 15.6; 
%         walker(i,j).m2 = 7.2;
%         walker(i,j).m1 = walker(i,j).m2;
%         
%         walker(i,j).J1 = 1/3*walker(i,j).m1*walker(i,j).L1^2
%         walker(i,j).J2 = 1/3*walker(i,j).m2*walker(i,j).L2^2
%         walker(i,j).J3 = 1/3*walker(i,j).m3*walker(i,j).L3^2
% 
%         %walker(i,j).controller.kp3 = i/grid_size*1000;
%         %walker(i,j).controller.kd3 = j/grid_size*200;
%         step_height(i,j) = maxStep(walker(i,j),1);    
%     end
% end


% 
% for i = 1:grid_size
%         i
%         
%         %walker[i,j] = CGTorsoWalker;
%         walker_mj(i).controller.th3_ref = 30*pi/180; 
%         walker_mj(i).m3 = i/grid_size*30;
%         walker_mj(i).m2 = (30 - walker_mj(i).m3)/2;
%         walker_mj(i).m1 = walker_mj(i).m2;
%         
%         walker_mj(i).J1 = 1/3*walker_mj(i).m1*walker_mj(i).L1^2
%         walker_mj(i).J2 = 1/3*walker_mj(i).m2*walker_mj(i).L2^2
%         walker_mj(i).J3 = 1/3*walker_mj(i).m3*walker_mj(i).L3^2
% 
% 
%         %walker(i,j).controller.kd3 = j/grid_size*200;
%         step_height_mj(i) = maxStep(walker_mj(i),1) 
%     
% end
% 
% 
% 
% for i = 1:grid_size
%         i
%         
%         %walker[i,j] = CGTorsoWalker;
%         walker_m(i).controller.th3_ref = 30*pi/180; 
%         walker_m(i).m3 = i/grid_size*30;
%         walker_m(i).m2 = (30 - walker_m(i).m3)/2;
%         walker_m(i).m1 = walker_m(i).m2;
%         
%        % walker(i).J1 = 1/3*walker(i).m1*walker(i).L1^2
%        % walker(i).J2 = 1/3*walker(i).m2*walker(i).L2^2
%        % walker(i).J3 = 1/3*walker(i).m3*walker(i).L3^2
% 
%         %walker(i,j).controller.kd3 = j/grid_size*200;
%         step_height_m(i) = maxStep(walker_m(i),1)  
% end


%name = 'constraint';
%ds = datestr(now,'_mm-dd-yyyy_HH-MM');
%save_name = strcat('saved/', name,ds,'.mat');
%save(save_name); 
%walker = CGTorsoWalker()
%walker.xy_step = [.1,.1]


%walker.controller.Ctype = 4;
%walker.controller.kp2 = 600;
%walker.controller.th2_ref = (180 + 45)*pi/180; 
%walker.controller.th3_ref = 30*pi/180;

%walker.runSim()
%walker.animate()

%walker.takeStep()
%walker.takeStep()
%walker.animate()

%maxStep(walker,1)
% 
% %what are we interested in changing?
% % walker.controller.Kp2
% % walker.controller.Kp3
% % walker.controller.th2_ref
% % walker.controller.th3_ref
% % walker.L1c + walker.L2c == these should be kept the same
% % walker.L3c > moments of inertia should change with these.. maybe point mass approx?
% 
% 
% max_steps = 1000;
% walker(max_steps) = CGTorsoWalker();
% 
% cur_max_step = zeros(1,max_steps);
% cur_max_step(1) = maxStep(walker(1))
% 
% %seed the random number generator
% %!!! if you add some noise to the walker you will reseed the rng! be
% %careful ( may need to change that part of the code) 
% rng(7);
% 
% %these are maximum values for our parameters
% MAX_KP_GAIN = 1000;
% MAX_KD_GAIN = 150;
% MAX_ANGLE = 2*pi; 
% MAX_LC_LEG = 1;
% MAX_LC_TORSO = .8;
% 
% for step_num = 2:max_steps
%     
%     step_size = .1; %this is actually a percentage of the max value, will need to find a smart way to change this
%     param = randperm(16); %we select a random order to try the parameters
%     
%     size(param,1)
%     
%     for i = 1:size(param,2)
%         %this is ugly, but efficient
%         switch param(i)
%             case 1
%                 walker(step_num).controller.kp2 = walker(step_num-1).controller.kp2 + step_size*MAX_KP_GAIN;
%             case 2
%                 walker(step_num).controller.kd2 = walker(step_num-1).controller.kd2 + step_size*MAX_KD_GAIN;
%             case 3
%                 walker(step_num).controller.kp3 = walker(step_num-1).controller.kp3 + step_size*MAX_KP_GAIN;
%             case 4
%                 walker(step_num).controller.kd3 = walker(step_num-1).controller.kd3 + step_size*MAX_KD_GAIN;
%             case 5
%                 walker(step_num).controller.th2_ref = walker(step_num-1).controller.th2_ref + step_size*MAX_ANGLE;
%             case 6
%                 walker(step_num).controller.th3_ref = walker(step_num-1).controller.th3_ref + step_size*MAX_ANGLE;
%             case 7
%                 walker(step_num).L1c = walker(step_num-1).L1c + step_size*MAX_LC_LEG;
%                 walker(step_num).L2c = walker(step_num-1).L2c + step_size*MAX_LC_LEG;
%             case 8
%                 walker(step_num).L3c = walker(step_num-1).L3c + step_size*MAX_LC_TORSO;
%                 
%                 
%             case 9
%                 walker(step_num).controller.kp2 = walker(step_num-1).controller.kp2 + step_size*-1*MAX_KP_GAIN;
%             case 10
%                 walker(step_num).controller.kd2 = walker(step_num-1).controller.kd2 + step_size*-1*MAX_KD_GAIN;
%             case 11
%                 walker(step_num).controller.kp3 = walker(step_num-1).controller.kp3 + step_size*-1*MAX_KP_GAIN;
%             case 12
%                 walker(step_num).controller.kd3 = walker(step_num-1).controller.kd3 + step_size*-1*MAX_KD_GAIN;
%             case 13
%                 walker(step_num).controller.th2_ref = walker(step_num-1).controller.th2_ref + step_size*-1*MAX_ANGLE;
%             case 14
%                 walker(step_num).controller.th3_ref = walker(step_num-1).controller.th3_ref + step_size*-1*MAX_ANGLE;
%             case 15
%                 walker(step_num).L1c = walker(step_num-1).L1c + step_size*-1*MAX_LC_LEG;
%                 walker(step_num).L2c = walker(step_num-1).L2c + step_size*-1*MAX_LC_LEG;
%             case 16
%                 walker(step_num).L3c = walker(step_num-1).L3c + step_size*-1*MAX_LC_TORSO;
%         end
%         
%         candidate_max_step = maxStep(walker(step_num))
%         cur_max_step(step_num-1)
%         step_num
%         i
%         
%         if candidate_max_step > cur_max_step(step_num-1)
%             cur_max_step(step_num) = candidate_max_step;
%             break; 
%         end
%  
%     end
%     
%     if i == size(param,2)
%         %this means we tried every option but didn't pick any of them
%         %will probably do somethig more intelligent in the future
%        
%         break
%     end
%     
% 
%     
% end

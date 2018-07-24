clear


[~, git_hash] = system('git rev-parse HEAD');


%clear all % clears persistent variables
%format compact
grid_size = 50; 
noise_levels = 0:.001:.020;


walker(grid_size,grid_size,size(noise_levels,2)) = CGTorsoWalker();

step_height= -1.*ones(grid_size,grid_size,size(noise_levels,2));

rng(0); %if you really want reproducible runs will have to be more careful with the parfor

for n = 1:size(noise_levels,2)
    parfor i = 1:grid_size
        for j = 1:grid_size
            
            (i-1)*grid_size + (j-1)
            walker(i,j,n) = CGTorsoWalker;
            walker(i,j,n).initSensorNoise(0, 0, noise_levels(n));
        
            
            walker(i,j,n).th3_ref = 20*pi/180;
            %walker(i,j,n).kp3 = i/grid_size*550 + 50;
        
            %walker(i,j).L3c = j/grid_size*.9 + .1;
            
            walker(i,j,n).m3 = i/grid_size*15 + 7.5;
            walker(i,j,n).m2 = (30 - walker(i,j,n).m3)/2;
            walker(i,j,n).m1 = walker(i,j,n).m2;
            
            walker(i,j,n).J1 = 1/3*walker(i,j,n).m1*walker(i,j,n).L1c^2;
            walker(i,j,n).J2 = 1/3*walker(i,j,n).m2*walker(i,j,n).L2c^2;
            walker(i,j,n).J3 = 1/3*walker(i,j,n).m3*walker(i,j,n).L3c^2;
            
            
            step_height(i,j,n) = maxStep(walker(i,j,n),1);
        end
    end
end


step_height_z = step_height;
step_height_z(step_height < 0) = 0;
zeros(size(noise_levels,2), grid_size);

for i = 1:size(noise_levels,2)
    avg_step_height(:,i) = mean(step_height_z(:,:,i)');
    
end


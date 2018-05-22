clear


[~, git_hash] = system('git rev-parse HEAD');


%clear all % clears persistent variables
%format compact
grid_size = 25; 
noise_levels = -1:.2:1;


walker(grid_size,grid_size) = CGTorsoWalker();

step_height= -1.*ones(grid_size,grid_size);


for n = 1:size(noise_levels,2)
    parfor i = 1:grid_size
        for j = 1:grid_size
            (i-1)*grid_size + (j-1)
            walker(i,j) = CGTorsoWalker;
            walker(i,j).bias = noise_levels(n);
            walker(i,j).controller.th3_ref = (j/grid_size)*40*pi/180;
            
            
            %walker(i,j).L3c = j/grid_size*.9 + .1;
            
            walker(i,j).m3 = i/grid_size*15 + 7.5;
            walker(i,j).m2 = (30 - walker(i,j).m3)/2;
            walker(i,j).m1 = walker(i,j).m2;
            
            walker(i,j).J1 = 1/3*walker(i,j).m1*walker(i,j).L1^2
            walker(i,j).J2 = 1/3*walker(i,j).m2*walker(i,j).L2^2
            walker(i,j).J3 = 1/3*walker(i,j).m3*walker(i,j).L3^2
            
            
            step_height(i,j) = maxStep(walker(i,j),1);
        end
    end
end




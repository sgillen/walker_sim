clear


[~, git_hash] = system('git rev-parse HEAD');


%clear all % clears persistent variables
%format compact
grid_size = 25; 
noise_levels = -.5:.1:.5;


walker(grid_size,grid_size,size(noise_levels,2)) = CGTorsoWalker();

step_height= -1.*ones(grid_size,grid_size,size(noise_levels,2));

for n = 1:size(noise_levels,2)
    for i = 1:grid_size
        for j = 1:grid_size
            
            (i-1)*grid_size + (j-1)
            walker(i,j,n) = CGTorsoWalker;
            %walker(i,j,n).bias =
            walker(i,j,n).initSensorNoise(0, noise_levels(n) ,0);
            
            walker(i,j,n).controller.th3_ref = (j/grid_size)*40*pi/180;
            
            
            %walker(i,j).L3c = j/grid_size*.9 + .1;
            
            walker(i,j,n).m3 = i/grid_size*15 + 7.5;
            walker(i,j,n).m2 = (30 - walker(i,j,n).m3)/2;
            walker(i,j,n).m1 = walker(i,j,n).m2;
            
            walker(i,j,n).J1 = 1/3*walker(i,j,n).m1*walker(i,j,n).L1^2;
            walker(i,j,n).J2 = 1/3*walker(i,j,n).m2*walker(i,j,n).L2^2;
            walker(i,j,n).J3 = 1/3*walker(i,j,n).m3*walker(i,j,n).L3^2;
            
            
            step_height(i,j,n) = maxStep(walker(i,j,n),1);
        end
    end
end




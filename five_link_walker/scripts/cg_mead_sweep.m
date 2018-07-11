clear


[~, git_hash] = system('git rev-parse HEAD');


%clear all % clears persistent variables
%format compact
grid_size = 10; 
%noise_levels = -.25:.025:.25;
noise_levels = 0;
nparam = 2;
walker(grid_size,grid_size,size(noise_levels,2)) = CGTorsoWalker();
step_height= -1.*ones(grid_size,grid_size,size(noise_levels,2));

lb = zeros(nparam,1);
ub = ones(nparam,1);

Pinit = -1.*ones(grid_size, grid_size, size(noise_levels,2), nparam);
Pfinal = -2.*ones(grid_size, grid_size, size(noise_levels,2), nparam);

for n = 1:size(noise_levels,2)
    parfor i = 1:grid_size
        for j = 1:grid_size
            
            (i-1)*grid_size + (j-1)
            walker(i,j,n) = CGTorsoWalker;
            %walker(i,j,n).bias =
            walker(i,j,n).initSensorNoise(0, noise_levels(n) ,0);
              
            options = optimset('Display','iter');;
            %options.display = 'iter';
            Pinit(i,j,n,:) = [i/grid_size, j/grid_size];
            
            %[Pfinal(i,:), step_height(i)] = fmincon(@(in)optimMaxStep(in),Pinit(i,:),[],[],[],[],lb,ub, [] ,options);
            [Pfinal(i,j,n,:), step_height(i,j,n)] = fminsearch(@(in)optimMaxStep(in),Pinit(i,j,n,:),options);
        end
    end
end




% Katie Byl, June 19, 2017 
%
% Example matlab code to make a couple of simple movies.
% Drawing objects are created in figure(11).
% Each movie is "replayed" in figure(12).
%
% Notes:
% - Using polar coordinates can to useful, to show rigid objects 
% moving in space. rotation matrices can also be useful to calculate
% coordinates of a rigid object moving in space.
% - The commands "plot" and "patch" are both useful.
% - After running this, type: get(p1)
% The "get" command will show you which elements may be changed for
% that drawing object. e.g., "FaceAlpha", etc.
% p1.FaceAlpha would show the value.
% get(p1,'FaceAlpha') does the same as above.
% (p1.FaceAlpha = 1 makes it "solid", which is the default for matlab...)
% - Changing the viewpoint can be helpful when making a movie, to
% get a full 3D idea of motion.
% - "axis equal" and "axis vis3d" are usually both needed to get the
% x and y axes to avoid re-scaling automatically. By default, matlab
% just scales axes to fit whatever data there are, but when drawing, you
% want "1 unit" to have the same length on the screen along the x axis
% as it does in the y axis.

figure(11); clf

x1 = [0 1 1.6 1.2 -.3 0];
y1 = [0 0 .8 1.5 .4 0];
id = 2; % A point to pivot about

% Save coordinates as "polar" coords:
a1 = atan2(y1-y1(id),x1-x1(id));
r1 = ((y1-y1(id)).^2 + (x1-x1(id)).^2).^.5;

da = a1(id) - a1(id+1);
alist = da*[0:.02:1];

% Draw the initial object:
pa = plot([-3 3],[0 0],'k-','LineWidth',2); hold on % ground plane...
% plot3 is like plot, but allows for z coords, too:
pb = plot3([1 1.7 1.7 1 1],[0 0 1 1 0],-0.3+[0 0 0 0 0],'r','LineWidth',2);
view(0,90); % this creates an "overhead view"
%p1 = plot(x1,y1,'b-');  % Try "plot" vs "patch" commands...
%set(p1,'LineWidth',3,'Color',[0 .5 .8]);
p1 = patch(x1,y1,'b-'); 
set(p1,'FaceColor',[.6 1 .9]);
set(p1,'EdgeColor',[0 .4 .8]);
set(p1,'LineWidth',2);
set(p1,'FaceAlpha',.8); % to make it "see through"...


% Be sure axes are 1:1
% help axis <-- Use this for more info in MATLAB
axis image 
axis([-1 2.5 -.5 2]); % hold axes as desired
axis off % if you do not want grid lines, etc.
% You can also use:
% axis square
% axis equal

mi=0; clear M % M will be our "movie" object
for n=1:length(alist)
    set(p1,'XData',x1(id) + r1.*cos(a1+alist(n)),...
        'YData',y1(id) + r1.*sin(a1+alist(n)));
    drawnow % "flush" output to the figure
    mi=mi+1; % increment movie index
    M(mi) = getframe; % "help getframe" in matlab...
end

figure(12)
axis off % otherwise, movie will play "on top of" background grid...
fps = 20; % frame per sec, to play movie
movie(M,-1,fps)

figure(11); % return to figure(11) and rotate the point of view:
axis vis3d % otherwise, picture will rescale during 3D viewpoint...
mi=0; clear M2 % M2 will be our NEW "movie" object
for n=1:length(alist)
    set(p1,'XData',x1(id) + r1.*cos(a1+alist(n)),...
        'YData',y1(id) + r1.*sin(a1+alist(n)));
    view(.4*mi+10,60-mi);
    drawnow % "flush" output to the figure
    mi=mi+1; % increment movie index
    M(mi) = getframe; % "help getframe" in matlab...
end

figure(12)
axis off % otherwise, movie will play "on top of" background grid...
fps = 20; % frame per sec, to play movie
movie(M,-1,fps)


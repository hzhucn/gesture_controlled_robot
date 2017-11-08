%% Dégrossir analyse image 
clear all; 
close all;

chemin='../DATA/RIGHT';
liste=dir(fullfile(chemin,'*.jpg'));

%% Define parameters
% limite pour le fond 

l_fond = 1e-6; 
flag_plot = 1; 
perimeters = zeros(600,1);
areas = zeros(600,1);
figure
for n=1:length(liste)
    
    nom=liste(n).name;
    X=double(imread(fullfile(chemin,nom)))/255.0;
    X_gray = rgb2gray(X);
    if flag_plot
        subplot(2,2,1);
        imagesc(X_gray)
        colormap(gray(256))
        axis equal
        title(['fichier go'])
        drawnow
    end
    
    [CI,XI]=imhist(X_gray,256); 
    CI=CI/numel(X_gray); 
    if flag_plot
        subplot(2,2,2);
        plot(XI,CI) 
    end
    
    X_bin = ones(200,200); 
    for i=1:200
        for j=1:200
            %v = find(XI == X(i,j));
            
           if X(i,j) < 0.15
               disp(X(i,j));
               X_bin(i,j) = 0;
           elseif X(i,j) > 0.8
               disp(X(i,j));
               X_bin(i,j) = 0;
           end
        end
    end
    if flag_plot
        subplot(2,2,3);
        imagesc(X_bin)
        axis equal
    end;
    
    stats = regionprops(X_bin, 'Centroid', 'Orientation');
    X_rotated = imrotate(X_bin, -stats.Orientation);
    stats_filled = regionprops(X_rotated, 'FilledImage');
    X_filled = stats_filled.FilledImage;
    stats_centroid = regionprops(X_filled, 'Centroid', 'Perimeter', 'Area');
    centroid = stats_centroid.Centroid;
    if flag_plot
        subplot(2,2,4);
        imagesc(X_filled); 
        hold on; 
        plot(centroid(:,1),centroid(:,2), 'b*'); 
        hold off;
        axis equal
    end;
    per = stats_centroid.Perimeter; 
    area = stats_centroid.Area;
    perimeters(n,1) = per;
    areas(n,1) = area;
    
end
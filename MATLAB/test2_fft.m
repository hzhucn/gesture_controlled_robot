%% Dégrossir analyse image 
clear all; 
close all;

chemin='../DATA/STOP';
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
        subplot(2,1,1);
        imagesc(X_gray)
        colormap(gray(256))
        axis equal
        title(['fichier go'])
        drawnow
    end
    
    bw = rgb2gray(X);
    F = fft2(bw);
    subplot(2,1,2);
    F2 = log(abs(F));
    imshow(F2,[-1 5],'InitialMagnification','fit');
    colormap(jet); colorbar
    
end
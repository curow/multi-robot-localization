% UTIAS Multi-Robot Cooperative Localization and Mapping Dataset
% produced by Keith Leung (keith.leung@robotics.utias.utoronto.ca) 2009
% Matlab script loadMRCLAMdataSet.m
% Description: This scripts parses the 17 text files that make up a 
% dataset into Matlab arrays. Run this script within the the dataset 
% directory.

%% specify which dataset to use (MRCLARM_Dataset[1-9]) and number of robots
NUM = 1;
PATH = ['MRCLAM_Dataset' num2str(NUM) '/'];
n_robots = 5;

%% start parsing
disp('Parsing Dataset')
disp('Reading barcode numbers')
[subject_num, barcode_num] = textread([PATH 'Barcodes.dat'], '%u %u','commentstyle','shell');
Barcodes = [subject_num, barcode_num];
clear subject_num barcode_num;

disp('Reading landmark groundtruth')
[subject_num x y x_sd y_sd] = textread([PATH 'Landmark_Groundtruth.dat'], '%f %f %f %f %f','commentstyle','shell');
Landmark_Groundtruth = [subject_num x y x_sd y_sd];
clear subject_num x y x_sd y_sd;
n_landmarks = length(Landmark_Groundtruth); 

for i=1:n_robots
 
    disp(['Reading robot ' num2str(i) ' groundtruth'])
    [time x y theta] = textread([PATH 'Robot' num2str(i) '_Groundtruth.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Groundtruth = [time x y theta];']); 
    clear time x y theta;

    disp(['Reading robot ' num2str(i) ' odometry'])
    [time, v, w] = textread([PATH 'Robot' num2str(i) '_Odometry.dat'], '%f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Odometry = [time v w];']);
    clear time v w;
    
    disp(['Reading robot ' num2str(i) ' measurements'])
    [time, barcode_num, r b] = textread([PATH 'Robot' num2str(i) '_Measurement.dat'], '%f %f %f %f','commentstyle','shell');
    eval(['Robot' num2str(i) '_Measurement = [time barcode_num r b];']);
    clear time barcode_num r b;

end
clear i
disp('Parsing Complete')

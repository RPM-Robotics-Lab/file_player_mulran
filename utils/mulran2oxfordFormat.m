clear; clc;

parent_dir = "/home/user/Documents/data/MulRan-icra/KAIST/20190902/sensor_data/radar/"; 
% change the above line to your path
% or you can try the example using parent_dir = "./data/";

data_dir = fullfile(parent_dir, "ray");

save_dir = fullfile(parent_dir, "oxford_form");
if ~exist(save_dir, 'dir')
   mkdir(save_dir)
end

files = dir(data_dir); 
files(1:2) = []; % remove . and .. 
files_names = {files(:).name};

for file_idx = 1:length(files_names)
    
    file_name = files_names{file_idx};
    data = readmatrix(fullfile(data_dir, file_name));

    img = data(:, 5:end);
    imwrite(img/255.0, 'img.png');
    img_unit8 = imread('img.png');
    
    times_ns = int64(data(:, 1)); 
    times_ns_uint8s = [];
    for ii=1:length(times_ns)
        uint8s = typecast(times_ns(ii), 'uint8');  % write left to right is corresponding to from lower digit to higher digit
        times_ns_uint8s = [times_ns_uint8s; uint8s];
    end
    
    azims_count = uint16(data(:, 3)); % encoder size is 5600
    azims_count_uint8s = [];
    for ii=1:length(azims_count)
        uint8s = typecast(azims_count(ii), 'uint8');  % write left to right is corresponding to from lower digit to higher digit
        azims_count_uint8s = [azims_count_uint8s; uint8s];
    end

    valids = uint8(255 * ones(size(azims_count_uint8s,1), 1));
    
    % 8 pixels for time, 2 pixels for azim counts, 1 pixels for validness, later then for intensities      
    img_augmented = [times_ns_uint8s, azims_count_uint8s, valids, img_unit8];
    file_name_png = strcat(file_name(1:end-4), '.png');
    imwrite(img_augmented, fullfile(save_dir, file_name_png));
    disp(strcat(file_name_png, " saved"));
    disp(100*file_idx/length(files_names));
end


          
          

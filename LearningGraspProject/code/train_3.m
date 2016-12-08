id = num2str(3);
init_poses = csvread(strcat('init_data_',id,'.csv'));
imdep = imread(strcat('scene',id,'_depth.png'));
target_poses = csvread(strcat('tar_data_',id,'.csv'));
cuboid_dim = [0.045,0.0175,0.055];
width = 340;
height = 210;
offset = [590 450];

%camera values
fx_d = 1.0 / 1051.89;
fy_d = 1.0 / 1060.192;
cx_d = 962.20;
cy_d = 535.165;
campos = [0, 0, 0.8];
camrot = [0.5, -0.5, 0.5, -0.5];
camtrotmat = qGetR(camrot);

imdep = double(imdep)/1000;
%Convert object to world frame
pt = zeros(width*height,3);
count = 0;
for i=offset(1):offset(1)+width
    for j=offset(2):offset(2)+height
        x = ((i - cx_d) * imdep(j,i) * fx_d);
        y = ((j - cy_d) * imdep(j,i) * fy_d);
        z = imdep(j,i);
        if x~=0 || y~=0 || z~=0;
            count = count +1;
            wpt = transpose(camtrotmat*transpose([x y z]) + transpose(campos));
            pt(count,:) = wpt;
        end
    end
end
pt(count+1:end,:) = [];

%convert scene to world frame
scenept = zeros(size(imdep,1)*size(imdep,2),3);
count = 0;
for k=1:size(imdep,2)
    for l=1:size(imdep,1)
        x = ((k - cx_d) * imdep(l,k) * fx_d);
        y = ((l - cy_d) * imdep(l,k) * fy_d);
        z = imdep(l,k);
        if x~=0 || y~=0 || z~=0;
            count = count + 1;
            scpt = transpose(camtrotmat*transpose([x y z]) + transpose(campos));
            scenept(count,:) = scpt;
        end
    end
end
scenept(count+1:end,:) = [];

score = zeros(size(target_poses,1),1);
features = zeros(size(target_poses,1),24);
disp('Starting Target_poses');
for j=1:size(target_poses,1);
    tic;
    tar = target_poses(j,:);
    init = init_poses(j,:);

    rotmat = qGetR([tar(7) tar(4) tar(5) tar(6)]);

    center_cuboid = [tar(1),tar(2),tar(3)];
    init_center_cuboid = [init(1), init(2), init(3)];
    % disp(init_center_cuboid);

    inliers = 0;
    %Convert segmented object points to gripper frame
    for i=1:size(pt,1);
        %gripper in target position for score

        cu_frame_pt = transpose(transpose(rotmat)*(transpose(pt(i,:) - [center_cuboid(1) center_cuboid(2) center_cuboid(3)])));
        if ( abs(cu_frame_pt(1)) < cuboid_dim(1) &&...
            abs(cu_frame_pt(2)) < cuboid_dim(2) && ...
            abs(cu_frame_pt(3)) < cuboid_dim(3))
            inliers = inliers+1;
        end

        %gripper in init position for training feature
        init_frame_pt = transpose(transpose(rotmat)*(transpose(pt(i,:) - [init_center_cuboid(1) init_center_cuboid(2) init_center_cuboid(3)])));
        if ( abs(init_frame_pt(1)) < cuboid_dim(1) &&...
            abs(init_frame_pt(2)) < cuboid_dim(2) )
            idx = int16(abs(init_frame_pt(1))/0.0025)+1;
            features(j,idx) = features(j,idx) + 1;
        end
    end
    %Convert scene points to gripper frame
    for i=1:size(scenept,1)
        %gripper in init position for training feature
        init_frame_pt = transpose(transpose(rotmat)*(transpose(scenept(i,:) - [init_center_cuboid(1) init_center_cuboid(2) init_center_cuboid(3)])));
        if ( abs(init_frame_pt(1)) < cuboid_dim(1) &&...
            abs(init_frame_pt(2)) < cuboid_dim(2) )
            idx = int16(abs(init_frame_pt(3))/0.05)+20;
            if idx<25
                features(j,idx) = features(j,idx) + 1;
            end
        end
    end
    score(j,1) = inliers;
    if mod(j,100)==0;
        toc;
    end
end
[~,max_index] = max(score);
disp(init_poses(max_index,:));
save(strcat('training_',id,'.mat'),'features','score');
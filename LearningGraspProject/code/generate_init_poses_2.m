function generate_init_poses_2()
    
    id = num2str(2);
    fid = fopen(strcat('dummy_data_',num2str(id),'.yaml'),'w');
    fid_initpose = fopen(strcat('init_data_',id,'.csv'),'w');
    
    width = 110;
    height = 200;
    offset = [730 450];
    imdep = imread(strcat('scene',id,'_depth.png'));
    imdep = double(imdep)/1000;

    fx_d = 1.0 / 1051.89;
    fy_d = 1.0 / 1060.192;
    cx_d = 962.20;
    cy_d = 535.165;

    %camera tf
    campos = [0, 0, 0.8];
    camrot = [0.5, -0.5, 0.5, -0.5];
    camtrotmat = qGetR(camrot);

    %cu = offset(2) + height/2;%555
    %cv = offset(1) + width/2;%695
    cu = 730;
    cv = 550;
    
    cx = ((cv - cx_d) * imdep(cu,cv) * fx_d);
    cy = ((cu - cy_d) * imdep(cu,cv) * fy_d);
    cz = imdep(cu,cv);
    c = [cx, cy, cz];
    wc = transpose(camtrotmat*transpose(c) + transpose(campos));
    dist = 0.25;


    for phi=90:-10:0;
        for theta=0:10:360;
            x_new = wc(1,1) + dist*cos(pi*theta/180)*sin(pi*phi/180);
            y_new = wc(1,2) + dist*sin(pi*theta/180)*sin(pi*phi/180);
            z_new = wc(1,3) + dist*cos(pi*phi/180);
            pt = [x_new,y_new,z_new];

            %pt = [0.5, 0.03, 0.9];
            rz = wc-pt;
            rz = rz/norm(rz);

            ry_orig = [rz(2)-rz(3), -rz(1), rz(1)];

            ax = rz(1,1);
            ay = rz(1,2);
            az = rz(1,3);
            bx = ry_orig(1,1);
            by = ry_orig(1,2);
            bz = ry_orig(1,3);

            for d=0:20:180
                syms x y z;
                eqn1 = ax*x + ay*y + az*z == 0;
                eqn2 = bx*x + by*y + bz*z == cos(pi*d/180);
                eqn3 = x^2 + y^2 + z^2 == 1;

                [v1,v2,v3] = solve([eqn1,eqn2,eqn3],x,y,z);
                for i=1:size(v1,1)
                    ry = [double(v1(i)),double(v2(i)),double(v3(i))];
                    rx = cross(ry, rz);

                    rot = [rx(1) ry(1) rz(1);rx(2) ry(2) rz(2);rx(3) ry(3) rz(3)];
                    %rot = [rx;ry;rz];
                    q = qGetQ(rot);
                    fprintf(fid,'-\n');
                    fprintf(fid,'  relative_config: [%f,%f,%f,%f,%f,%f,%f]\n',pt(1),pt(2),pt(3),q(2),q(3),q(4),q(1));
                    fprintf(fid,'  grasp_mode:  2\n');
                    fprintf(fid,'  release_mode:  3\n');

                    fprintf(fid_initpose,'%f,%f,%f,%f,%f,%f,%f\n',pt(1),pt(2),pt(3),q(2),q(3),q(4),q(1));
                end
            end  
        end
    end
    fclose(fid);
    fclose(fid_initpose);
    disp('Done');
end


%run('../toolbox/vl_setup')

kpoints = load('../bin/glx/pic2_3.sift');
pic = imread('../bin/glx/pic2_3.jpg');

kpoints = kpoints(:,1:4)';


figure;
image(pic);
hold on;
h = vl_plotframe(kpoints);
%kpointsA = load('../bin/glx/pic1_2.sift');
%kpointsB = load('../bin/glx/b_calc_desc/pic1_2.sift');

kpointsA = load('../bin/glx/pic2_3.sift');
kpointsB = load('../bin/glx/pic2_3.sift_dsp');


%kpointsA = load('../bin/glx/circle.sift');
%kpointsB = load('../bin/glx/b_calc_desc/circle.sift');



diff = kpointsA - kpointsB;

%diff'

disp(['max abweichung: ', num2str(max(max(abs(diff))))]);
disp(['mean abweichung: ', num2str(mean(mean(abs(diff))))]);
disp(['max MSQE: ', num2str(max(sum(diff.^2,1)))]);
disp(['mean MSQE: ', num2str(mean(sum(diff.^2,1)))]);


%%
da = kpointsA(:,5:end)';
db = kpointsB(:,5:end)';
[matches, scores] = vl_ubcmatch(da,db);
matchesA = kpointsA(matches(1,:),:);
matchesB = kpointsB(matches(2,:),:);
posdiff = matchesA(:,1:2) - matchesB(:,1:2);
max(posdiff)
mean(posdiff)
median(posdiff)
plot(posdiff(:,1));

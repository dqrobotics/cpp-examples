![ezgif-1-c04be41431](https://user-images.githubusercontent.com/23158313/158247464-945cb68b-d1ef-4d57-a6f9-601d92f91aaf.gif)

```matlab
clear all
close all
clc

torques_ref = readmatrix('list_torques_ref.csv')';
torques_read = readmatrix('list_torques_read.csv')';

h1 = figure;
set(h1, 'DefaultTextFontSize', 10);
set(h1, 'DefaultAxesFontSize', 10); % [pt]
set(h1, 'DefaultAxesFontName', 'mwa_cmr10');
set(h1, 'DefaultTextFontName', 'mwa_cmr10');
set(h1, 'Units', 'centimeters');
pos = get(h1, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 2*10; % Select the height of the figure in [cm] 6
set(h1, 'Position', pos);
set(h1, 'PaperType', 'a4letter');
set(h1,'PaperPositionMode','auto')
set(h1, 'Renderer', 'Painters');

a = 2;
b = 4;
w = 4;
fontsize = 20;
for i=1:7
subplot(a,b,i);
plot(torques_ref(i,:),'b', 'LineWidth',w);
hold on
plot(torques_read(i,:),':r', 'LineWidth',3);
set(gca, 'FontSize', fontsize );
fig = gcf;
fig.Color = [1 1 1];
box('off');

%title('$\tau$', 'Interpreter','latex', 'Rotation', 0)
title(['\tau_',num2str(i)])
end
legend('Reference', 'measurement')
```

![results](https://user-images.githubusercontent.com/23158313/153051922-420e29dc-4220-4f1c-8eb7-ca55d3ea9b5f.png)

clear
close all
clc
A = imread('1.png');
figure;
imshow(A)
str = sprintf('Data_1');
text(960, 200, 'Path-1', 'Color', 'k', 'FontSize', 10, 'Interpreter', 'latex','FontName','Times','FontUnit','points');
text(960, 230, 'Path-2', 'Color', 'k', 'FontSize', 10, 'Interpreter', 'latex','FontName','Times','FontUnit','points');
text(960, 260, 'Path-3', 'Color', 'k', 'FontSize', 10, 'Interpreter', 'latex','FontName','Times','FontUnit','points');
text(960, 290, 'Path-4', 'Color', 'k', 'FontSize', 10, 'Interpreter', 'latex','FontName','Times','FontUnit','points');
print('map','-depsc2')

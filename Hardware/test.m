h1 = plot(0,0,0,0);
h1(1).XData = [];
h1(2).XData = [];
h1(1).YData = [];
h1(2).YData = [];
legend('True $q_1$', 'Desired $q_1$', 'Interpreter', 'latex', 'FontSize', 14);

t = [1 2 3 4];
q1 = [2 4 5.5 6];
q1_des = [2 3 3 5];

size = length(t);

for i=1:size
    N = 1;
    h1(1).XData(end+1:end+N) = t(i);
    h1(2).XData(end+1:end+N) = t(i);
    h1(1).YData(end+1:end+N) = q1(i);
    h1(2).YData(end+1:end+N) = q1_des(i);
    pause(1);
end

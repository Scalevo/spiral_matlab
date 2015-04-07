function stop = outfun(x, optimValues, state)
stop = false;
subplot(3,1,3);
hold on;
plot(x(1),x(2),'o');
drawnow
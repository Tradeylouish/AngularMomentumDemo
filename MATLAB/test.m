close;

a = 0.00072;
b = 0.00044;
w0 = speed(1);
I = 0.0138;
x = time/1000;
y = (w0+(a/b))*exp((-b*x)/I) - (a/b);
figure; hold on
plot(x, y)
plot(x, speed)
legend();
hold off
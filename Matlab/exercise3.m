%%SERGE and Roland Master EES%%

fs = 48000; 

N=2^12

f1 = 1000; 
t1 = 1/f1;
n1 = fs*t1 


f2 = 8000;


f = linspace(-fs/2, fs/2, 2^12); 

t = linspace(0, t1, n1); 

x1 = sin(2*pi * f1 * t)
x2 =  sin(2*pi * f2 * t)
x= x1 + x2;
figure; 
plot(t,x); title("signal")


[B, A] = butter(5,8000/(fs/2)); 
%%[H2,~] = freqz(B, A,N, "whole")
%%figure;
%%plot(f,fftshift(abs(H2)));title("filter");

F = filter(B,A,x);
figure;
plot(t,F);



singleX = single(x);
singleB = single(B);
singleA = single(A);

F1 = myfilter1(singleB,singleA,singleX);
F2 = myfilter2(singleB,singleA,F1);

Root_MSE = sqrt(mean((F2-F).^2));

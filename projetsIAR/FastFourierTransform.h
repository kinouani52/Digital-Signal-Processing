/*
Copyright (C) 2016 Mauricio Kugler
Nagoya Institute of Technology, Japan
This software is intended for research purposes only;
its redistribution is forbidden at any circumstances.
*/

#ifndef FASTFOURIERTRANSFORMH
#define FASTFOURIERTRANSFORMH

#include "komplex.h"

#define M_PI 3.14159265358979323846

class FastFourierTransform
{
private:

	unsigned int N;
	unsigned int M;
	unsigned int K;

	unsigned int *D;

	komplex **z;
	komplex *w;
	float *y;

public:
	FastFourierTransform(unsigned int n);
	~FastFourierTransform();

	float inline *fft(float *x);

};

float inline *FastFourierTransform::fft(float *x)
{
	//Initialize the FFT
	for(unsigned int i=0;i<N;i++){
		z[0][i] = komplex(x[i],0);
	}

	//Calculate the FFT
	for(unsigned int i=0;i<K;i++){
		unsigned int mask = 0xffffffff<<i;
		for(unsigned int j=0;j<M;j++){
			unsigned int n=i%2;
			z[1-n][j<<1] = z[n][j] + z[n][j+M];
			z[1-n][(j<<1)+1] = w[j&mask]*(z[n][j] - z[n][j+M]);
		}
	}

	//Sort the output values & calculate magnitude
	for(unsigned int i=0;i<M;i++){
		unsigned int j = D[i];
		unsigned int n=K%2;
		y[i] = z[n][j].abs();
	}

	return(y);
}

#endif

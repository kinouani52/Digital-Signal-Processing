/*
Copyright (C) 2016 Mauricio Kugler
Nagoya Institute of Technology, Japan
This software is intended for research purposes only;
its redistribution is forbidden at any circumstances.
*/

#include "FastFourierTransform.h"

FastFourierTransform::FastFourierTransform(unsigned int n)
{
	K = (unsigned int)(log((float)n)/log((float)2));
	N = 2<<(K-1);
	M = N>>1;

	w = new komplex[M];
	z = new komplex*[2];
	y = new float[M];

	for(unsigned int i=0;i<2;i++) {
		z[i] = new komplex[N];
	}

	for(unsigned int i=0;i<M;i++) {
		float real = (float)cos(2*M_PI*i/N);
		float imag = (float)sin(2*M_PI*i/N)*(-1);
		w[i] = komplex(real,imag);
	}

	//Dizimation lookup table
	D = new unsigned int[N];
	for(unsigned int i=0;i<N;i++) {
		unsigned int k=0;
		for(unsigned int j=0;j<K;j++) {
			k = k<<1;
			k += i>>j & 1;
		}
		D[i] = k;
	}
}

FastFourierTransform::~FastFourierTransform()
{
	delete[] w;
	for(unsigned int i=0;i<2;i++) delete[] z[i];
	delete[] z;
	delete[] D;
	//'y' must be deleted externally!
}

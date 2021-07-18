/*
Copyright (C) 2016 Mauricio Kugler
Nagoya Institute of Technology, Japan
This software is intended for research purposes only;
its redistribution is forbidden at any circumstances.
*/

#ifndef KOMPLEXH
#define KOMPLEXH

#include <math.h>

class komplex 
{
private:

public:
	float real,imag;

	inline komplex() {};
	inline komplex(float r, float i);
	
	inline komplex operator + (komplex);
	inline komplex operator - (komplex);
	inline komplex operator * (komplex);
	inline komplex operator / (komplex);

	inline float abs();
	inline float ang();
};

komplex::komplex(float r, float i)
{
	real = r;
	imag = i;
}

komplex komplex::operator + (komplex x)
{
	komplex y;
	y.real = real + x.real;
	y.imag = imag + x.imag;
	return(y);
}

komplex komplex::operator - (komplex x)
{
	komplex y;
	y.real = real - x.real;
	y.imag = imag - x.imag;
	return(y);
}

komplex komplex::operator * (komplex x)
{
	komplex y;
	y.real = real*x.real - imag*x.imag;
	y.imag = imag*x.real + real*x.imag;
	return(y);
}

komplex komplex::operator / (komplex x)
{
	komplex y;
	float z = (x.real*x.real + x.imag*x.imag);
	y.real = (real*x.real + imag*x.imag)/z;
	y.imag = (imag*x.real - real*x.imag)/z;
	return(y);
}

float komplex::abs()
{
	float n = sqrt(real*real + imag*imag);
	return(n);
}

float komplex::ang()
{
	float n = atan2(imag,real);
	return(n);
}

#endif

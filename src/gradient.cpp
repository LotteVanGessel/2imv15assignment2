#include "Particle.h"
#include <vector>
#include "Force.h"
#include <math.h>
#include <stdio.h>
#include "linearSolver.h"
#include "Gradient.h"
std::vector<double> conjgrad2(std::vector<std::vector<double>> A, std::vector<double> b, std::vector<double> x, int n)
{
	std::vector<double> r;
	std::vector<double> p;
	std::vector<double> Ap;
	std::vector<double> product;
	double alpha = 0;
	double rs_old = 0;
	double rs_new = 0;
	double pAp = 0;

	product = matrix_multiplication(A, b, n);

	for (int i = 0; i < n; i++)
	{
		r[i] = b[i] - product[i];
	}
	p = r;

	for (int i = 0; i < n; i++)
	{
		rs_old += r[i] * r[i];
	}

	for (int i = 0; i < n; i++)
	{
		Ap = matrix_multiplication(A, p, n);
		for (int i = 0; i < n; i++)
		{
			pAp += p[i] * Ap[i];
		}
		alpha = rs_old / pAp;

		for (int i = 0; i < n; i++)
		{
			x[i] += alpha * p[i];
			r[i] -= alpha * Ap[i];
		}

		for (int i = 0; i < n; i++)
		{
			rs_new += r[i] * r[i];
		}

		if (sqrt(rs_new) < 0.00001)
		{
			break;
		}

		for (int i = 0; i < n; i++)
		{
			p[i] = r[i] + (rs_new / rs_old) * p[i];
			rs_old = rs_new;
		}
	}
	return (x);

}

std::vector<double> matrix_multiplication(std::vector<std::vector<double>> A, std::vector<double> b, int n)
{
	std::vector<double> product;
	for (int i = 0; i < n; i++)
	{
		product[i] = 0;
		for (int j = 0; j < n; j++)
		{
			product[i] += A[i][j] * b[i];
		}
	}
	return (product);
}

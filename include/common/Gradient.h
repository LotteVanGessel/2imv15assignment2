#include <math.h> 
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <vector>

std::vector<double> matrix_multiplication(std::vector<std::vector<double>> A, std::vector<double> b, int n);
std::vector<double> conjgrad2(std::vector<vector<double>> A, std::vector<double> b, std::vector<double> x, int n);
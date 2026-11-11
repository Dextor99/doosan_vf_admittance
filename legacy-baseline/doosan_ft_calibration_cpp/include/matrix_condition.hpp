#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

// Singular values of a small square matrix by one-sided Jacobi rotations.
// Orthogonal QR and column pivoting preserve the 2-norm condition number.
template<std::size_t N>
double matrix_condition_2(std::array<std::array<double,N>,N> matrix) {
    bool converged=false;
    for(int sweep=0;sweep<100;++sweep) {
        bool rotated=false;
        for(std::size_t p=0;p<N;++p)for(std::size_t q=p+1;q<N;++q) {
            double alpha=0,beta=0,gamma=0;
            for(std::size_t i=0;i<N;++i) {
                alpha+=matrix[i][p]*matrix[i][p];
                beta+=matrix[i][q]*matrix[i][q];
                gamma+=matrix[i][p]*matrix[i][q];
            }
            if(alpha==0 || beta==0 || std::abs(gamma)<=1e-12*std::sqrt(alpha)*std::sqrt(beta))continue;
            const double tau=(beta-alpha)/(2*gamma);
            const double t=std::copysign(1.0,tau)/(std::abs(tau)+std::hypot(1.0,tau));
            const double c=1/std::hypot(1.0,t),s=c*t;
            for(std::size_t i=0;i<N;++i) {
                const double a=matrix[i][p],b=matrix[i][q];
                matrix[i][p]=c*a-s*b;matrix[i][q]=s*a+c*b;
            }
            rotated=true;
        }
        if(!rotated){converged=true;break;}
    }
    if(!converged)throw std::runtime_error("Condition-number SVD did not converge");
    double largest=0,smallest=std::numeric_limits<double>::infinity();
    for(std::size_t j=0;j<N;++j) {
        double norm=0;
        for(std::size_t i=0;i<N;++i)norm=std::hypot(norm,matrix[i][j]);
        largest=std::max(largest,norm);smallest=std::min(smallest,norm);
    }
    if(smallest==0)return std::numeric_limits<double>::infinity();
    return largest/smallest;
}

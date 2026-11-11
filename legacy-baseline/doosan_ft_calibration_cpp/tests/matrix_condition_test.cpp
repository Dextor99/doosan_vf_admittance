#include "matrix_condition.hpp"
#include <array>
#include <cassert>
#include <cmath>

int main() {
    std::array<std::array<double,2>,2> shear{{{{1,1}},{{0,1}}}};
    const double expected=(3+std::sqrt(5.0))/2;
    assert(std::abs(matrix_condition_2<2>(shear)-expected)<1e-10);
    std::array<std::array<double,2>,2> rank_one{{{{1,1}},{{0,0}}}};
    assert(std::isinf(matrix_condition_2<2>(rank_one)) || matrix_condition_2<2>(rank_one)>1e14);

    std::array<std::array<double,10>,10> a{};
    for(int i=0;i<10;++i)a[i][i]=i+1;
    // Left and right orthogonal rotations preserve the known singular values 1..10.
    for(int k=0;k<9;++k) {
        const double angle=0.13*(k+1),c=std::cos(angle),s=std::sin(angle);
        for(int i=0;i<10;++i) {
            const double x=a[i][k],y=a[i][k+1];
            a[i][k]=c*x-s*y;a[i][k+1]=s*x+c*y;
        }
        for(int j=0;j<10;++j) {
            const double x=a[k][j],y=a[k+1][j];
            a[k][j]=c*x-s*y;a[k+1][j]=s*x+c*y;
        }
    }
    assert(std::abs(matrix_condition_2<10>(a)-10)<1e-8);
}

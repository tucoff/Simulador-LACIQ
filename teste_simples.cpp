#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout << "Teste básico funcionando!" << std::endl;
    
    Eigen::Vector2d v(1, 2);
    std::cout << "Vetor: " << v.transpose() << std::endl;
    
    return 0;
}

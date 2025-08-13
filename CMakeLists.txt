# Define a versão mínima do CMake necessária para o projeto
cmake_minimum_required(VERSION 3.12)

# Define o nome do projeto
project(MeuSimulador CXX)

# Habilita o uso de padrões C++17 ou mais recentes
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Habilita o suporte a warnings, o que é uma boa prática
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# Define os diretórios de inclusão para o seu projeto
include_directories(include)

# Configuração da biblioteca Eigen para álgebra linear
# Adiciona o diretório da Eigen como um subdiretório (header-only library)
add_subdirectory(libs/eigen)

# Configuração do Pybind11 para interface Python (opcional)
# Comentado temporariamente até Python dev headers estarem disponíveis
# add_subdirectory(libs/pybind11)

# Encontra as bibliotecas Python necessárias (opcional)
# find_package(PythonLibs REQUIRED)

# Cria a lista de arquivos fonte do projeto
# Lista explícita dos arquivos para melhor controle
set(SOURCES
    src/main.cpp
    src/core/QuantumGate.cpp
    src/core/QuantumCircuit.cpp
    src/core/Simulator.cpp
)

# Adiciona um executável ao projeto, a partir dos arquivos fonte
# e vincula as bibliotecas necessárias.
add_executable(meu_simulador ${SOURCES})

# Vincula a biblioteca Eigen ao executável
target_link_libraries(meu_simulador PRIVATE Eigen3::Eigen)

# Configuração opcional para bindings Python (descomentear quando Python dev headers estiverem disponíveis)
# target_include_directories(meu_simulador PRIVATE ${PYTHON_INCLUDE_DIRS})
# pybind11_add_module(simulador_python src/core/Simulador.cpp)
# target_link_libraries(simulador_python PRIVATE Eigen3::Eigen)
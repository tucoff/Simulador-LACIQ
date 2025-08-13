# Configuração das Bibliotecas Essenciais

## Bibliotecas Adicionadas

### ✅ Eigen - Álgebra Linear
- **Status**: Configurada e funcionando
- **Localização**: `libs/eigen/`
- **Uso**: Todas as operações de álgebra linear (vetores, matrizes, produtos tensoriais)
- **Configuração**: Automática via CMake

### 🔄 Pybind11 - Interface Python
- **Status**: Baixada, mas temporariamente desabilitada
- **Localização**: `libs/pybind11/`
- **Motivo**: Requer bibliotecas de desenvolvimento Python

## Como Habilitar Pybind11

Para habilitar o Pybind11, você precisa instalar as bibliotecas de desenvolvimento Python:

### No Arch Linux:
```bash
sudo pacman -S python-dev python-setuptools
```

### No Ubuntu/Debian:
```bash
sudo apt-get install python3-dev python3-setuptools
```

### No Fedora:
```bash
sudo dnf install python3-devel python3-setuptools
```

### Após instalar as dependências:

1. Descomente as linhas no `CMakeLists.txt`:
```cmake
# Descomente estas linhas:
add_subdirectory(libs/pybind11)
find_package(PythonLibs REQUIRED)

# E estas também:
target_include_directories(meu_simulador PRIVATE ${PYTHON_INCLUDE_DIRS})
pybind11_add_module(simulador_python src/core/Simulador.cpp)
target_link_libraries(simulador_python PRIVATE Eigen3::Eigen)
```

2. Recompile o projeto:
```bash
cd build
cmake ..
make
```

## Testando o Eigen

O projeto atual já inclui um teste básico do Eigen no `main.cpp`. Execute:

```bash
cd build
./meu_simulador
```

Você deve ver uma saída mostrando que o Eigen está funcionando corretamente.

## Estrutura do Projeto

```
Simulador-LACIQ/
├── libs/
│   ├── eigen/          # Biblioteca Eigen (funcionando)
│   └── pybind11/       # Biblioteca Pybind11 (pronta para uso)
├── include/
│   └── core/
├── src/
│   ├── main.cpp
│   └── core/
└── CMakeLists.txt      # Configuração das bibliotecas
```

## Próximos Passos

1. Desenvolver classes de simulação quântica usando Eigen
2. Habilitar Pybind11 quando necessário para interface Python
3. Criar testes unitários
4. Implementar algoritmos quânticos específicos

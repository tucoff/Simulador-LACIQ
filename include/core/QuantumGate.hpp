#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * NAMESPACE QuantumSim
 * 
 * Organiza todas as classes e funções relacionadas à simulação quântica.
 * Previne conflitos de nomes com outras bibliotecas.
 */
namespace QuantumSim {

    /**
     * ENUMERAÇÃO: Tipos de portas quânticas suportadas
     * 
     * Define todos os tipos de portas que o simulador pode processar.
     * Cada tipo corresponde a uma matriz matemática específica.
     * 
     * CATEGORIAS:
     * - Portas básicas: Hadamard, Pauli (X,Y,Z)
     * - Portas controladas: CNOT
     * - Portas parametrizadas: Phase, Rotações (precisam de ângulo θ)
     */
    enum class GateType {
        HADAMARD,    // H: Cria superposição |0⟩→(|0⟩+|1⟩)/√2, |1⟩→(|0⟩-|1⟩)/√2
        PAULI_X,     // X: NOT quântico |0⟩→|1⟩, |1⟩→|0⟩
        PAULI_Y,     // Y: Rotação em Y com fase |0⟩→i|1⟩, |1⟩→-i|0⟩
        PAULI_Z,     // Z: Flip de fase |0⟩→|0⟩, |1⟩→-|1⟩
        CNOT,        // Controlled-NOT: |c,t⟩→|c,c⊕t⟩
        PHASE,       // Phase: |0⟩→|0⟩, |1⟩→e^(iθ)|1⟩
        ROTATION_X,  // Rx(θ): Rotação em X por ângulo θ
        ROTATION_Y,  // Ry(θ): Rotação em Y por ângulo θ
        ROTATION_Z   // Rz(θ): Rotação em Z por ângulo θ
    };

    /**
     * ESTRUTURA: Representação de uma porta quântica individual
     * 
     * Cada porta no circuito é representada por esta estrutura.
     * Contém todas as informações necessárias para aplicar a porta:
     * - Tipo (qual operação matemática)
     * - Qubits alvo (onde aplicar)
     * - Parâmetros (ângulos para portas parametrizadas)
     * 
     * EXEMPLOS DE USO:
     * - Hadamard no qubit 0: QuantumGate(HADAMARD, 0)
     * - CNOT(0→1): QuantumGate(CNOT, 0, 1)
     * - Rotação X(π/2) no qubit 2: QuantumGate(ROTATION_X, 2, π/2)
     */
    struct QuantumGate {
        GateType type;                    // Tipo da porta (enum acima)
        std::vector<int> qubits;          // Índices dos qubits (0-based)
        std::vector<double> parameters;   // Parâmetros (ângulos em radianos)
        
        // CONSTRUTORES DE CONVENIÊNCIA:
        
        // Construtor para portas de 1 qubit sem parâmetros
        QuantumGate(GateType t, int qubit) 
            : type(t), qubits({qubit}) {}
        
        // Construtor para portas de 2 qubits (ex: CNOT)
        QuantumGate(GateType t, int control, int target) 
            : type(t), qubits({control, target}) {}
        
        // Construtor para portas parametrizadas de 1 qubit
        QuantumGate(GateType t, int qubit, double parameter) 
            : type(t), qubits({qubit}), parameters({parameter}) {}
    };

    /**
     * CLASSE: Fábrica de Matrizes para Portas Quânticas
     * 
     * Esta classe fornece todas as matrizes matemáticas correspondentes
     * às portas quânticas. É implementada como classe estática (todas
     * as funções são static) para facilitar o uso.
     * 
     * RESPONSABILIDADES:
     * 1. Definir matrizes corretas para cada porta
     * 2. Suportar portas parametrizadas (com ângulos)
     * 3. Fornecer interface uniforme via getMatrix()
     * 4. Utilitários (matriz identidade)
     * 
     * IMPLEMENTAÇÃO MATEMÁTICA:
     * - Usa números complexos (std::complex<double>)
     * - Matrizes são do tipo Eigen::Matrix[N]cd onde N é a dimensão
     * - Todas as matrizes são unitárias (preservam probabilidades)
     */
    class GateMatrix {
    public:
        // =============================================================
        // MATRIZES DAS PORTAS BÁSICAS (2x2)
        // =============================================================
        
        /**
         * Porta Hadamard: H = (1/√2) * [[1,1],[1,-1]]
         * Cria superposição uniforme entre |0⟩ e |1⟩
         */
        static Eigen::Matrix2cd hadamard();
        
        /**
         * Porta Pauli-X: X = [[0,1],[1,0]]
         * Equivalente quântico da porta NOT clássica
         */
        static Eigen::Matrix2cd pauli_x();
        
        /**
         * Porta Pauli-Y: Y = [[0,-i],[i,0]]
         * Rotação com introdução de fase imaginária
         */
        static Eigen::Matrix2cd pauli_y();
        
        /**
         * Porta Pauli-Z: Z = [[1,0],[0,-1]]
         * Adiciona fase -1 ao estado |1⟩
         */
        static Eigen::Matrix2cd pauli_z();
        
        // =============================================================
        // MATRIZES DE PORTAS DE 2 QUBITS (4x4)
        // =============================================================
        
        /**
         * Porta CNOT: Matriz 4x4 para emaranhamento
         * Base de estados: |00⟩, |01⟩, |10⟩, |11⟩
         */
        static Eigen::Matrix4cd cnot();
        
        // =============================================================
        // PORTAS PARAMETRIZADAS (dependem de ângulo θ)
        // =============================================================
        
        /**
         * Porta Phase: P(θ) = [[1,0],[0,e^(iθ)]]
         * @param theta Ângulo de fase em radianos
         */
        static Eigen::Matrix2cd phase(double theta);
        
        /**
         * Rotação X: Rx(θ) - rotação em torno do eixo X
         * @param theta Ângulo de rotação em radianos
         */
        static Eigen::Matrix2cd rotation_x(double theta);
        
        /**
         * Rotação Y: Ry(θ) - rotação em torno do eixo Y
         * @param theta Ângulo de rotação em radianos
         */
        static Eigen::Matrix2cd rotation_y(double theta);
        
        /**
         * Rotação Z: Rz(θ) - rotação em torno do eixo Z
         * @param theta Ângulo de rotação em radianos
         */
        static Eigen::Matrix2cd rotation_z(double theta);
        
        // =============================================================
        // INTERFACE PRINCIPAL
        // =============================================================
        
        /**
         * FUNÇÃO FACTORY: Obtém matriz para qualquer porta
         * 
         * @param gate Estrutura QuantumGate completa
         * @return Matriz Eigen correspondente
         * 
         * Esta é a função principal usada pelo simulador.
         * Ela automaticamente seleciona a matriz correta
         * baseada no tipo e parâmetros da porta.
         */
        static Eigen::MatrixXcd getMatrix(const QuantumGate& gate);
        
        // =============================================================
        // FUNÇÕES AUXILIARES
        // =============================================================
        
        /**
         * Cria matriz identidade de tamanho arbitrário
         * @param size Dimensão da matriz (size × size)
         * @return Matriz identidade I[size×size]
         */
        static Eigen::MatrixXcd identity(int size);
    };

} // namespace QuantumSim

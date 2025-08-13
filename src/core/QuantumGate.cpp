#include "core/QuantumGate.hpp"
#include <cmath>

namespace QuantumSim {

    // CONSTANTES MATEMÁTICAS IMPORTANTES
    const std::complex<double> I(0.0, 1.0);  // Unidade imaginária: i = √(-1)
    const double PI = 3.14159265358979323846; // Constante π

    /**
     * PORTA HADAMARD
     * 
     * A porta Hadamard é fundamental em computação quântica.
     * Ela cria superposição: transforma |0⟩ → (|0⟩+|1⟩)/√2 e |1⟩ → (|0⟩-|1⟩)/√2
     * 
     * Matriz matemática:
     * H = (1/√2) * [1  1 ]
     *              [1 -1 ]
     * 
     * Propriedade importante: H² = I (aplicar duas vezes = identidade)
     */
    Eigen::Matrix2cd GateMatrix::hadamard() {
        Eigen::Matrix2cd h;
        h << 1.0, 1.0,   // Primeira linha: [1, 1]
             1.0, -1.0;  // Segunda linha: [1, -1]
        
        // Divide por √2 para normalização
        return h / std::sqrt(2.0);
    }

    /**
     * PORTA PAULI-X (NOT quântico)
     * 
     * Equivalente quântico da porta NOT clássica.
     * Inverte o estado: |0⟩ → |1⟩ e |1⟩ → |0⟩
     * 
     * Matriz matemática:
     * X = [0 1]
     *     [1 0]
     */
    Eigen::Matrix2cd GateMatrix::pauli_x() {
        Eigen::Matrix2cd x;
        x << 0.0, 1.0,   // Primeira linha: [0, 1]
             1.0, 0.0;   // Segunda linha: [1, 0]
        return x;
    }

    /**
     * PORTA PAULI-Y
     * 
     * Rotação em torno do eixo Y na esfera de Bloch.
     * Transforma: |0⟩ → i|1⟩ e |1⟩ → -i|0⟩
     * 
     * Matriz matemática:
     * Y = [0 -i]
     *     [i  0]
     * 
     * Nota: Introduz fases complexas (números imaginários)
     */
    Eigen::Matrix2cd GateMatrix::pauli_y() {
        Eigen::Matrix2cd y;
        y << 0.0, -I,    // Primeira linha: [0, -i]
             I, 0.0;     // Segunda linha: [i, 0]
        return y;
    }

    /**
     * PORTA PAULI-Z (Flip de fase)
     * 
     * Adiciona fase -1 ao estado |1⟩, mantém |0⟩ inalterado.
     * Transforma: |0⟩ → |0⟩ e |1⟩ → -|1⟩
     * 
     * Matriz matemática:
     * Z = [1  0]
     *     [0 -1]
     */
    Eigen::Matrix2cd GateMatrix::pauli_z() {
        Eigen::Matrix2cd z;
        z << 1.0, 0.0,   // Primeira linha: [1, 0]
             0.0, -1.0;  // Segunda linha: [0, -1]
        return z;
    }

    /**
     * PORTA CNOT (Controlled-NOT)
     * 
     * A porta CNOT é essencial para criar emaranhamento quântico.
     * Opera em 2 qubits: um de controle e um alvo.
     * 
     * FUNCIONAMENTO:
     * - Se qubit controle = |0⟩ → qubit alvo permanece inalterado
     * - Se qubit controle = |1⟩ → qubit alvo é invertido (NOT)
     * 
     * TABELA DE VERDADE:
     * |00⟩ → |00⟩  (controle=0, não faz nada)
     * |01⟩ → |01⟩  (controle=0, não faz nada)  
     * |10⟩ → |11⟩  (controle=1, inverte alvo: 0→1)
     * |11⟩ → |10⟩  (controle=1, inverte alvo: 1→0)
     * 
     * Matriz 4x4 (base: |00⟩, |01⟩, |10⟩, |11⟩):
     * CNOT = [1 0 0 0]
     *        [0 1 0 0]
     *        [0 0 0 1]
     *        [0 0 1 0]
     */
    Eigen::Matrix4cd GateMatrix::cnot() {
        Eigen::Matrix4cd cnot;
        cnot << 1.0, 0.0, 0.0, 0.0,  // |00⟩ → |00⟩
                0.0, 1.0, 0.0, 0.0,  // |01⟩ → |01⟩
                0.0, 0.0, 0.0, 1.0,  // |10⟩ → |11⟩
                0.0, 0.0, 1.0, 0.0;  // |11⟩ → |10⟩
        return cnot;
    }

    /**
     * PORTA PHASE (Rotação de fase)
     * 
     * Adiciona uma fase e^(iθ) ao estado |1⟩, mantém |0⟩ inalterado.
     * Transforma: |0⟩ → |0⟩ e |1⟩ → e^(iθ)|1⟩
     * 
     * @param theta Ângulo de rotação em radianos
     * 
     * Matriz matemática:
     * P(θ) = [1    0   ]
     *        [0  e^(iθ)]
     * 
     * Casos especiais:
     * - θ = π/2 → Porta S
     * - θ = π/4 → Porta T  
     * - θ = π   → Porta Z
     */
    Eigen::Matrix2cd GateMatrix::phase(double theta) {
        Eigen::Matrix2cd p;
        p << 1.0, 0.0,                      // |0⟩ → |0⟩
             0.0, std::exp(I * theta);      // |1⟩ → e^(iθ)|1⟩
        return p;
    }

    /**
     * ROTAÇÃO X (Rx)
     * 
     * Rotação em torno do eixo X na esfera de Bloch por ângulo θ.
     * É uma generalização da porta Pauli-X.
     * 
     * @param theta Ângulo de rotação em radianos
     * 
     * Matriz matemática:
     * Rx(θ) = [cos(θ/2)   -i*sin(θ/2)]
     *         [-i*sin(θ/2)  cos(θ/2) ]
     * 
     * Casos especiais:
     * - θ = 0   → Identidade (nenhuma rotação)
     * - θ = π   → Porta Pauli-X (rotação completa)
     * - θ = π/2 → Rotação de 90 graus
     */
    Eigen::Matrix2cd GateMatrix::rotation_x(double theta) {
        Eigen::Matrix2cd rx;
        double cos_half = std::cos(theta / 2.0);    // cos(θ/2)
        double sin_half = std::sin(theta / 2.0);    // sin(θ/2)
        
        rx << cos_half, -I * sin_half,              // Primeira linha
              -I * sin_half, cos_half;              // Segunda linha
        return rx;
    }

    /**
     * ROTAÇÃO Y (Ry)
     * 
     * Rotação em torno do eixo Y na esfera de Bloch por ângulo θ.
     * É uma generalização da porta Pauli-Y.
     * 
     * @param theta Ângulo de rotação em radianos
     * 
     * Matriz matemática:
     * Ry(θ) = [cos(θ/2)  -sin(θ/2)]
     *         [sin(θ/2)   cos(θ/2)]
     * 
     * Nota: Esta é a única rotação que usa apenas números reais
     * (não introduz fases complexas).
     */
    Eigen::Matrix2cd GateMatrix::rotation_y(double theta) {
        Eigen::Matrix2cd ry;
        double cos_half = std::cos(theta / 2.0);    // cos(θ/2)
        double sin_half = std::sin(theta / 2.0);    // sin(θ/2)
        
        ry << cos_half, -sin_half,                  // Primeira linha
              sin_half, cos_half;                   // Segunda linha
        return ry;
    }

    /**
     * ROTAÇÃO Z (Rz)
     * 
     * Rotação em torno do eixo Z na esfera de Bloch por ângulo θ.
     * É uma generalização da porta Pauli-Z.
     * 
     * @param theta Ângulo de rotação em radianos
     * 
     * Matriz matemática:
     * Rz(θ) = [e^(-iθ/2)     0     ]
     *         [    0     e^(iθ/2) ]
     * 
     * Casos especiais:
     * - θ = 0 → Identidade
     * - θ = π → Porta Pauli-Z (com fase global)
     */
    Eigen::Matrix2cd GateMatrix::rotation_z(double theta) {
        Eigen::Matrix2cd rz;
        std::complex<double> exp_neg = std::exp(-I * theta / 2.0);  // e^(-iθ/2)
        std::complex<double> exp_pos = std::exp(I * theta / 2.0);   // e^(iθ/2)
        
        rz << exp_neg, 0.0,                         // Primeira linha
              0.0, exp_pos;                         // Segunda linha
        return rz;
    }

    /**
     * FUNÇÃO FACTORY: Obtém a matriz correspondente a uma porta específica
     * 
     * @param gate Estrutura QuantumGate contendo tipo, qubits e parâmetros
     * @return Matriz Eigen correspondente à porta
     * 
     * Esta função atua como um "factory pattern" - ela decide qual
     * matriz retornar baseada no tipo da porta e seus parâmetros.
     * 
     * TRATAMENTO DE PARÂMETROS:
     * - Portas sem parâmetros: usa implementação padrão
     * - Portas com parâmetros: usa valores fornecidos pelo usuário
     * - Fallback: retorna identidade se tipo não reconhecido
     */
    Eigen::MatrixXcd GateMatrix::getMatrix(const QuantumGate& gate) {
        switch (gate.type) {
            case GateType::HADAMARD:
                return hadamard();
            
            case GateType::PAULI_X:
                return pauli_x();
            
            case GateType::PAULI_Y:
                return pauli_y();
            
            case GateType::PAULI_Z:
                return pauli_z();
            
            case GateType::CNOT:
                return cnot();
            
            case GateType::PHASE:
                if (!gate.parameters.empty()) {
                    // Usa ângulo fornecido pelo usuário
                    return phase(gate.parameters[0]);
                }
                // Fallback: Porta S (phase gate padrão com θ=π/2)
                return phase(PI / 2.0);
            
            case GateType::ROTATION_X:
                if (!gate.parameters.empty()) {
                    return rotation_x(gate.parameters[0]);
                }
                // Fallback: Rotação π (equivale a Pauli-X)
                return rotation_x(PI);
            
            case GateType::ROTATION_Y:
                if (!gate.parameters.empty()) {
                    return rotation_y(gate.parameters[0]);
                }
                // Fallback: Rotação π (equivale a Pauli-Y)
                return rotation_y(PI);
            
            case GateType::ROTATION_Z:
                if (!gate.parameters.empty()) {
                    return rotation_z(gate.parameters[0]);
                }
                // Fallback: Rotação π (equivale a Pauli-Z)
                return rotation_z(PI);
            
            default:
                // Fallback de segurança: retorna matriz identidade
                // Isso garante que não há crash se tipo desconhecido for usado
                return identity(2);
        }
    }

    /**
     * FUNÇÃO AUXILIAR: Cria matriz identidade de tamanho arbitrário
     * 
     * @param size Dimensão da matriz (size × size)
     * @return Matriz identidade usando Eigen
     * 
     * A matriz identidade I tem a propriedade: I·ψ = ψ
     * Ou seja, não altera o estado quando aplicada.
     * 
     * Para qubits: I = [1 0]
     *                  [0 1]
     */
    Eigen::MatrixXcd GateMatrix::identity(int size) {
        return Eigen::MatrixXcd::Identity(size, size);
    }

} // namespace QuantumSim

#include "core/QuantumCircuit.hpp"
#include <stdexcept>
#include <sstream>
#include <algorithm>

namespace QuantumSim {

    /**
     * CONSTRUTOR do Circuito Quântico
     * 
     * @param num_qubits Número de qubits que o circuito irá operar
     * 
     * Um circuito quântico é essencialmente uma sequência de portas quânticas
     * que são aplicadas a um conjunto de qubits. Cada porta na sequência
     * transforma o estado quântico do sistema.
     * 
     * Exemplo: Para criar o estado de Bell |00⟩ + |11⟩
     * 1. Aplicar H no qubit 0 → cria superposição
     * 2. Aplicar CNOT(0,1) → cria emaranhamento
     */
    QuantumCircuit::QuantumCircuit(int num_qubits) 
        : num_qubits_(num_qubits) {
        // Validação: deve ter pelo menos 1 qubit
        if (num_qubits <= 0) {
            throw std::invalid_argument("Número de qubits deve ser positivo");
        }
        // gates_ começa vazio - portas serão adicionadas dinamicamente
    }

    /**
     * ADICIONA uma porta ao circuito
     * 
     * @param gate Estrutura QuantumGate a ser adicionada
     * 
     * Esta é a função principal para construir circuitos. Ela valida
     * se a porta é compatível com este circuito antes de adicionar.
     * 
     * VALIDAÇÕES REALIZADAS:
     * - Índices de qubits estão no range válido [0, num_qubits-1]
     * - Portas de 2 qubits não operam no mesmo qubit
     * - Número correto de qubits para cada tipo de porta
     */
    void QuantumCircuit::addGate(const QuantumGate& gate) {
        if (!validateGate(gate)) {
            throw std::invalid_argument("Porta inválida para este circuito");
        }
        // Adiciona à lista sequencial de portas
        gates_.push_back(gate);
    }

    /**
     * MÉTODOS DE CONVENIÊNCIA para adicionar portas específicas
     * 
     * Estas funções simplificam a criação de circuitos, permitindo
     * que o usuário adicione portas sem precisar criar estruturas
     * QuantumGate manualmente.
     * 
     * Exemplo de uso:
     *   circuit.addHadamard(0);
     *   circuit.addCNOT(0, 1);
     * 
     * É equivalente a:
     *   circuit.addGate(QuantumGate(GateType::HADAMARD, 0));
     *   circuit.addGate(QuantumGate(GateType::CNOT, 0, 1));
     */

    // Porta Hadamard: cria superposição
    void QuantumCircuit::addHadamard(int qubit) {
        addGate(QuantumGate(GateType::HADAMARD, qubit));
    }

    // Porta Pauli-X: NOT quântico
    void QuantumCircuit::addPauliX(int qubit) {
        addGate(QuantumGate(GateType::PAULI_X, qubit));
    }

    // Porta Pauli-Y: rotação em Y com fase
    void QuantumCircuit::addPauliY(int qubit) {
        addGate(QuantumGate(GateType::PAULI_Y, qubit));
    }

    // Porta Pauli-Z: flip de fase
    void QuantumCircuit::addPauliZ(int qubit) {
        addGate(QuantumGate(GateType::PAULI_Z, qubit));
    }

    // Porta CNOT: cria emaranhamento
    void QuantumCircuit::addCNOT(int control, int target) {
        addGate(QuantumGate(GateType::CNOT, control, target));
    }

    // Porta Phase: adiciona fase θ ao estado |1⟩
    void QuantumCircuit::addPhase(int qubit, double theta) {
        addGate(QuantumGate(GateType::PHASE, qubit, theta));
    }

    // Rotações parametrizadas com ângulo θ
    void QuantumCircuit::addRotationX(int qubit, double theta) {
        addGate(QuantumGate(GateType::ROTATION_X, qubit, theta));
    }

    void QuantumCircuit::addRotationY(int qubit, double theta) {
        addGate(QuantumGate(GateType::ROTATION_Y, qubit, theta));
    }

    void QuantumCircuit::addRotationZ(int qubit, double theta) {
        addGate(QuantumGate(GateType::ROTATION_Z, qubit, theta));
    }

    /**
     * LIMPA todas as portas do circuito
     * 
     * Remove todas as portas adicionadas, deixando o circuito vazio.
     * Útil para reutilizar o mesmo objeto QuantumCircuit para
     * diferentes experimentos.
     */
    void QuantumCircuit::clear() {
        gates_.clear();
    }

    /**
     * CONVERTE o circuito para representação textual
     * 
     * @return String descrevendo todas as portas do circuito
     * 
     * Esta função é extremamente útil para debug e visualização.
     * Ela mostra a sequência completa de operações que serão
     * executadas no simulador.
     * 
     * Exemplo de saída:
     * "Circuito Quântico (2 qubits, 3 portas):
     *  Porta 1: H(0)
     *  Porta 2: CNOT(0,1)
     *  Porta 3: Rz(1, θ=1.5708)"
     */
    std::string QuantumCircuit::toString() const {
        std::stringstream ss;
        ss << "Circuito Quântico (" << num_qubits_ << " qubits, " 
           << gates_.size() << " portas):\\n";
        
        // Itera por todas as portas em ordem sequencial
        for (size_t i = 0; i < gates_.size(); ++i) {
            const auto& gate = gates_[i];
            ss << "Porta " << i + 1 << ": ";
            
            // Formata cada tipo de porta diferentemente
            switch (gate.type) {
                case GateType::HADAMARD:
                    ss << "H(" << gate.qubits[0] << ")";
                    break;
                case GateType::PAULI_X:
                    ss << "X(" << gate.qubits[0] << ")";
                    break;
                case GateType::PAULI_Y:
                    ss << "Y(" << gate.qubits[0] << ")";
                    break;
                case GateType::PAULI_Z:
                    ss << "Z(" << gate.qubits[0] << ")";
                    break;
                case GateType::CNOT:
                    ss << "CNOT(" << gate.qubits[0] << "," << gate.qubits[1] << ")";
                    break;
                    
                // Portas parametrizadas mostram o ângulo
                case GateType::PHASE:
                    ss << "P(" << gate.qubits[0];
                    if (!gate.parameters.empty()) {
                        ss << ", θ=" << gate.parameters[0];
                    }
                    ss << ")";
                    break;
                case GateType::ROTATION_X:
                    ss << "Rx(" << gate.qubits[0];
                    if (!gate.parameters.empty()) {
                        ss << ", θ=" << gate.parameters[0];
                    }
                    ss << ")";
                    break;
                case GateType::ROTATION_Y:
                    ss << "Ry(" << gate.qubits[0];
                    if (!gate.parameters.empty()) {
                        ss << ", θ=" << gate.parameters[0];
                    }
                    ss << ")";
                    break;
                case GateType::ROTATION_Z:
                    ss << "Rz(" << gate.qubits[0];
                    if (!gate.parameters.empty()) {
                        ss << ", θ=" << gate.parameters[0];
                    }
                    ss << ")";
                    break;
            }
            ss << "\\n";
        }
        
        return ss.str();
    }

    /**
     * VALIDA se uma porta pode ser adicionada ao circuito
     * 
     * @param gate Porta a ser validada
     * @return true se válida, false caso contrário
     * 
     * VALIDAÇÕES REALIZADAS:
     * 
     * 1. RANGE DE QUBITS: Todos os qubits devem estar entre 0 e (num_qubits-1)
     * 2. QUBITS ÚNICOS: Portas de 2 qubits não podem operar no mesmo qubit
     * 3. CARDINALIDADE: Cada tipo de porta deve ter o número correto de qubits
     * 
     * Esta função previne erros comuns como:
     * - Tentar aplicar CNOT(0,0) (mesmo qubit)
     * - Usar qubit 5 em circuito de 3 qubits
     * - Criar porta Hadamard com 2 qubits
     */
    bool QuantumCircuit::validateGate(const QuantumGate& gate) const {
        // VALIDAÇÃO 1: Range de qubits
        for (int qubit : gate.qubits) {
            if (qubit < 0 || qubit >= num_qubits_) {
                return false;  // Qubit fora do range válido
            }
        }
        
        // VALIDAÇÃO 2: Qubits únicos para portas multi-qubit
        if (gate.qubits.size() == 2 && gate.qubits[0] == gate.qubits[1]) {
            return false;  // Mesmos qubits em porta de 2 qubits
        }
        
        // VALIDAÇÃO 3: Cardinalidade por tipo de porta
        switch (gate.type) {
            // Portas de 1 qubit: devem ter exatamente 1 qubit
            case GateType::HADAMARD:
            case GateType::PAULI_X:
            case GateType::PAULI_Y:
            case GateType::PAULI_Z:
            case GateType::PHASE:
            case GateType::ROTATION_X:
            case GateType::ROTATION_Y:
            case GateType::ROTATION_Z:
                return gate.qubits.size() == 1;
            
            // Portas de 2 qubits: devem ter exatamente 2 qubits
            case GateType::CNOT:
                return gate.qubits.size() == 2;
            
            // Tipo desconhecido
            default:
                return false;
        }
    }

} // namespace QuantumSim

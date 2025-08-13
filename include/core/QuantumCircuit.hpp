#pragma once

#include "QuantumGate.hpp"
#include <vector>
#include <string>

namespace QuantumSim {

    /**
     * CLASSE: Representação de um Circuito Quântico
     * 
     * Um circuito quântico é uma sequência ordenada de portas quânticas
     * que são aplicadas a um conjunto de qubits. Esta classe:
     * 
     * 1. ARMAZENA a sequência de portas em ordem de execução
     * 2. VALIDA portas antes de adicionar (qubits válidos, tipos corretos)
     * 3. FORNECE interface conveniente para construção de circuitos
     * 4. OFERECE representação textual para debug/visualização
     * 
     * EXEMPLO DE USO:
     * ```cpp
     * QuantumCircuit circuit(2);           // Circuito para 2 qubits
     * circuit.addHadamard(0);              // H no qubit 0
     * circuit.addCNOT(0, 1);               // CNOT entre qubits 0 e 1
     * circuit.addRotationZ(1, M_PI/4);     // Rz(π/4) no qubit 1
     * ```
     * 
     * REPRESENTAÇÃO INTERNA:
     * - std::vector<QuantumGate> para sequência de portas
     * - Validação automática de compatibilidade
     * - Índices de qubits são 0-based
     */
    class QuantumCircuit {
    private:
        int num_qubits_;                        // Número de qubits no circuito
        std::vector<QuantumGate> gates_;        // Sequência ordenada de portas
        
    public:
        // =============================================================
        // CONSTRUÇÃO E CONFIGURAÇÃO
        // =============================================================
        
        /**
         * Construtor do circuito quântico
         * 
         * @param num_qubits Número de qubits no circuito (deve ser > 0)
         * 
         * Cria um circuito vazio que pode operar em num_qubits qubits.
         * Todas as portas adicionadas posteriormente devem usar apenas
         * índices de qubits no range [0, num_qubits-1].
         */
        explicit QuantumCircuit(int num_qubits);
        
        // =============================================================
        // ADIÇÃO DE PORTAS (Interface Geral)
        // =============================================================
        
        /**
         * Adiciona uma porta ao circuito
         * 
         * @param gate Estrutura QuantumGate a ser adicionada
         * @throws std::invalid_argument se porta for inválida
         * 
         * Esta é a função principal para construção de circuitos.
         * Realiza validação completa antes de adicionar a porta.
         * 
         * VALIDAÇÕES:
         * - Índices de qubits no range [0, num_qubits-1]
         * - Portas de 2 qubits não operam no mesmo qubit
         * - Número correto de qubits para cada tipo de porta
         */
        void addGate(const QuantumGate& gate);
        
        // =============================================================
        // MÉTODOS DE CONVENIÊNCIA (Interface Simplificada)
        // =============================================================
        
        /**
         * Adiciona porta Hadamard (superposição)
         * @param qubit Índice do qubit alvo
         */
        void addHadamard(int qubit);
        
        /**
         * Adiciona porta Pauli-X (NOT quântico)
         * @param qubit Índice do qubit alvo
         */
        void addPauliX(int qubit);
        
        /**
         * Adiciona porta Pauli-Y (rotação Y com fase)
         * @param qubit Índice do qubit alvo
         */
        void addPauliY(int qubit);
        
        /**
         * Adiciona porta Pauli-Z (flip de fase)
         * @param qubit Índice do qubit alvo
         */
        void addPauliZ(int qubit);
        
        /**
         * Adiciona porta CNOT (emaranhamento)
         * @param control Índice do qubit de controle
         * @param target Índice do qubit alvo
         */
        void addCNOT(int control, int target);
        
        /**
         * Adiciona porta Phase (fase customizada)
         * @param qubit Índice do qubit alvo
         * @param theta Ângulo de fase em radianos
         */
        void addPhase(int qubit, double theta);
        
        /**
         * Adiciona rotação X (eixo X por ângulo θ)
         * @param qubit Índice do qubit alvo
         * @param theta Ângulo de rotação em radianos
         */
        void addRotationX(int qubit, double theta);
        
        /**
         * Adiciona rotação Y (eixo Y por ângulo θ)
         * @param qubit Índice do qubit alvo
         * @param theta Ângulo de rotação em radianos
         */
        void addRotationY(int qubit, double theta);
        
        /**
         * Adiciona rotação Z (eixo Z por ângulo θ)
         * @param qubit Índice do qubit alvo
         * @param theta Ângulo de rotação em radianos
         */
        void addRotationZ(int qubit, double theta);
        
        // =============================================================
        // CONSULTA E ACESSO (Getters)
        // =============================================================
        
        /**
         * Obtém número de qubits do circuito
         * @return Número de qubits configurado na criação
         */
        int getNumQubits() const { return num_qubits_; }
        
        /**
         * Obtém referência constante para lista de portas
         * @return Vector com todas as portas em ordem de execução
         * 
         * ATENÇÃO: Retorna referência constante para evitar modificação
         * acidental. Use apenas para leitura/iteração.
         */
        const std::vector<QuantumGate>& getGates() const { return gates_; }
        
        /**
         * Obtém número total de portas no circuito
         * @return Quantidade de portas adicionadas
         */
        size_t getNumGates() const { return gates_.size(); }
        
        // =============================================================
        // MANIPULAÇÃO DO CIRCUITO
        // =============================================================
        
        /**
         * Remove todas as portas do circuito
         * 
         * Limpa o circuito, deixando-o vazio. O número de qubits
         * permanece inalterado. Útil para reutilizar o mesmo objeto
         * para diferentes experimentos.
         */
        void clear();
        
        // =============================================================
        // REPRESENTAÇÃO E DEBUG
        // =============================================================
        
        /**
         * Converte circuito para representação textual
         * 
         * @return String descrevendo todas as portas do circuito
         * 
         * Gera representação legível do circuito para debug e visualização.
         * 
         * EXEMPLO DE SAÍDA:
         * "Circuito Quântico (2 qubits, 3 portas):
         *  Porta 1: H(0)
         *  Porta 2: CNOT(0,1)
         *  Porta 3: Rz(1, θ=1.5708)"
         */
        std::string toString() const;
        
        // =============================================================
        // VALIDAÇÃO INTERNA
        // =============================================================
        
        /**
         * Valida se uma porta é compatível com este circuito
         * 
         * @param gate Porta a ser validada
         * @return true se válida, false caso contrário
         * 
         * VALIDAÇÕES REALIZADAS:
         * 1. Todos os qubits estão no range [0, num_qubits-1]
         * 2. Portas de 2 qubits não operam no mesmo qubit
         * 3. Número correto de qubits para cada tipo de porta
         * 
         * Esta função é chamada automaticamente por addGate().
         */
        bool validateGate(const QuantumGate& gate) const;
    };

} // namespace QuantumSim

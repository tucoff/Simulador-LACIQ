#pragma once

#include "QuantumCircuit.hpp"
#include <Eigen/Dense>
#include <complex>
#include <vector>
#include <map>

namespace QuantumSim {

    /**
     * CLASSE PRINCIPAL: Simulador Quântico
     * 
     * Esta é a classe central que executa a simulação quântica.
     * Ela mantém o estado quântico do sistema e aplica portas
     * sequencialmente, calculando as transformações matemáticas.
     * 
     * FUNCIONAMENTO BÁSICO:
     * 1. ESTADO: Mantém vetor de amplitudes complexas (Eigen::VectorXcd)
     * 2. EXECUÇÃO: Aplica portas uma por uma ao vetor de estado
     * 3. MEDIÇÃO: Simula colapso quântico com probabilidades
     * 4. ANÁLISE: Fornece ferramentas para examinar resultados
     * 
     * REPRESENTAÇÃO DO ESTADO:
     * - Para n qubits: vetor de 2^n amplitudes complexas
     * - Cada posição representa um estado base |i⟩
     * - |amplitude|² = probabilidade de medir esse estado
     * - Soma de todas as probabilidades = 1 (normalização)
     * 
     * EXEMPLO PARA 2 QUBITS:
     * - Posição 0: amplitude de |00⟩
     * - Posição 1: amplitude de |01⟩  
     * - Posição 2: amplitude de |10⟩
     * - Posição 3: amplitude de |11⟩
     * 
     * LIMITAÇÕES:
     * - Máximo 20 qubits (2^20 ≈ 1M amplitudes)
     * - Simulação clássica (não simula decoerência)
     * - Precisão limitada por ponto flutuante
     */
    class Simulator {
    private:
        int num_qubits_;                    // Número de qubits sendo simulados
        Eigen::VectorXcd state_vector_;     // Vetor de estado: 2^n amplitudes complexas
        
        // =============================================================
        // MÉTODOS PRIVADOS (Algoritmos de Aplicação de Portas)
        // =============================================================
        
        /**
         * Aplica porta de um qubit ao vetor de estado
         * 
         * @param gate_matrix Matriz 2x2 da porta quântica
         * @param qubit Índice do qubit alvo (0-based)
         * 
         * ALGORITMO EFICIENTE:
         * Para cada amplitude no vetor de estado, verifica o valor
         * do qubit alvo e aplica a transformação 2x2 correspondente.
         * Usa manipulação de bits para extrair/modificar qubits específicos.
         */
        void applySingleQubitGate(const Eigen::Matrix2cd& gate_matrix, int qubit);
        
        /**
         * Aplica porta de dois qubits ao vetor de estado
         * 
         * @param gate_matrix Matriz 4x4 da porta quântica
         * @param control Qubit de controle
         * @param target Qubit alvo
         * 
         * ALGORITMO PARA PORTAS CONTROLADAS:
         * Para cada amplitude, extrai os valores dos 2 qubits,
         * mapeia para índice 4x4 e aplica transformação completa.
         */
        void applyTwoQubitGate(const Eigen::Matrix4cd& gate_matrix, int control, int target);
        
        /**
         * Dispatcher principal para aplicação de portas
         * 
         * @param gate Porta a ser aplicada
         * 
         * Decide automaticamente qual algoritmo usar baseado
         * no número de qubits da porta (1, 2, ou mais).
         */
        void applyGate(const QuantumGate& gate);
        
        /**
         * Constrói operador completo usando produto tensorial
         * 
         * @param gate_matrix Matriz da porta
         * @param target_qubits Qubits alvos da porta
         * @return Matriz completa 2^n × 2^n
         * 
         * NOTA: Atualmente não implementado (otimização futura)
         * Para sistemas grandes, seria mais eficiente que os
         * algoritmos atuais bit-a-bit.
         */
        Eigen::MatrixXcd buildFullOperator(const Eigen::MatrixXcd& gate_matrix, 
                                          const std::vector<int>& target_qubits);
        
    public:
        // =============================================================
        // CONSTRUÇÃO E CONFIGURAÇÃO
        // =============================================================
        
        /**
         * Construtor do simulador
         * 
         * @param num_qubits Número de qubits a simular (1-20)
         * @throws std::invalid_argument se num_qubits inválido
         * 
         * Cria simulador e inicializa no estado fundamental |00...0⟩.
         * Aloca 2^num_qubits amplitudes complexas em memória.
         */
        explicit Simulator(int num_qubits);
        
        // =============================================================
        // EXECUÇÃO DE CIRCUITOS
        // =============================================================
        
        /**
         * Executa um circuito quântico completo
         * 
         * @param circuit Circuito a ser executado
         * @throws std::invalid_argument se circuito incompatível
         * 
         * Aplica todas as portas do circuito sequencialmente.
         * Cada porta modifica o vetor de estado atual.
         * É equivalente a multiplicar uma série de matrizes unitárias.
         */
        void execute(const QuantumCircuit& circuit);
        
        // =============================================================
        // CONTROLE DE ESTADO
        // =============================================================
        
        /**
         * Reinicia simulador para estado fundamental
         * 
         * Coloca todos os qubits em |0⟩, zerando o vetor de estado
         * exceto a primeira posição (amplitude = 1.0).
         * Útil para executar múltiplos experimentos.
         */
        void reset();
        
        /**
         * Inicializa um qubit específico
         * 
         * @param qubit Índice do qubit
         * @param state true para |1⟩, false para |0⟩ (padrão)
         * 
         * Por padrão todos os qubits começam em |0⟩.
         * Esta função permite inicializar alguns em |1⟩.
         * Implementação: aplica porta X se state=true.
         */
        void initializeQubit(int qubit, bool state = false);
        
        // =============================================================
        // CONSULTA DE ESTADO (Getters)
        // =============================================================
        
        /**
         * Obtém referência ao vetor de estado
         * @return Vetor de amplitudes complexas (somente leitura)
         */
        const Eigen::VectorXcd& getStateVector() const { return state_vector_; }
        
        /**
         * Obtém número de qubits sendo simulados
         * @return Número de qubits configurado na criação
         */
        int getNumQubits() const { return num_qubits_; }
        
        // =============================================================
        // ANÁLISE DE PROBABILIDADES
        // =============================================================
        
        /**
         * Calcula probabilidades de todos os estados base
         * 
         * @return Mapa string → probabilidade para estados significativos
         * 
         * Para cada estado base |i⟩ com probabilidade > 10^-10,
         * retorna par (representação_binária, probabilidade).
         * 
         * EXEMPLO PARA 2 QUBITS:
         * Estado (|00⟩ + |11⟩)/√2 retorna:
         * {"00" → 0.5, "11" → 0.5}
         */
        std::map<std::string, double> getProbabilities() const;
        
        /**
         * Calcula probabilidade de um estado específico
         * 
         * @param state Estado em representação binária (ex: "101")
         * @return Probabilidade entre 0.0 e 1.0
         * @throws std::invalid_argument se estado inválido
         * 
         * Função "não-destrutiva" - não afeta o estado atual.
         * Útil para análise sem modificar o sistema.
         */
        double getStateProbability(const std::string& state) const;
        
        // =============================================================
        // MEDIÇÕES QUÂNTICAS
        // =============================================================
        
        /**
         * Mede um qubit específico
         * 
         * @param qubit Índice do qubit a ser medido
         * @return 0 ou 1 baseado na medição probabilística
         * @throws std::out_of_range se qubit inválido
         * 
         * ATENÇÃO: Esta é uma operação DESTRUTIVA!
         * 
         * PROCESSO:
         * 1. Calcula P(qubit=0) somando |amplitudes|² relevantes
         * 2. Gera número aleatório para decidir resultado
         * 3. Colapsa estado: zera amplitudes inconsistentes
         * 4. Renormaliza amplitudes restantes
         * 
         * Após medição, o sistema está em estado definitivo
         * para o qubit medido.
         */
        int measureQubit(int qubit);
        
        /**
         * Mede todos os qubits sequencialmente
         * 
         * @return String binária com resultado completo
         * 
         * ATENÇÃO: Mede UM QUBIT POR VEZ!
         * Cada medição colapsa o estado antes da próxima.
         * Para estados emaranhados, isso importa muito.
         * 
         * Exemplo: Estado |00⟩+|11⟩ → resultado sempre "00" ou "11"
         */
        std::string measureAll();
        
        
        // =============================================================
        // DIAGNÓSTICO E ANÁLISE
        // =============================================================
        
        /**
         * Converte estado atual para representação legível
         * 
         * @return String descrevendo amplitudes significativas
         * 
         * FORMATO DE SAÍDA:
         * "Estado: (0.707+0i)|00⟩ + (0.707+0i)|11⟩"
         * 
         * FILTROS:
         * - Apenas amplitudes > 10^-10 são mostradas
         * - Números complexos formatados com precisão controlada
         * - Estados em notação Dirac |abc⟩
         * 
         * Útil para debug e verificação de algoritmos.
         */
        std::string stateToString() const;
        
        /**
         * Verifica se o estado está normalizado
         * 
         * @return true se Σᵢ|αᵢ|² ≈ 1.0 (tolerância: 10^-10)
         * 
         * IMPORTÂNCIA:
         * - Estados não-normalizados indicam bugs
         * - Requerido pela física quântica
         * - Validação após operações complexas
         * 
         * CASOS DE FALHA:
         * - Estado zero (todos os amplitudes = 0)
         * - Erro numérico acumulado
         * - Operações incorretas em portas
         */
        bool isNormalized() const;
        
    private:
        // =============================================================
        // DADOS INTERNOS DO SIMULADOR
        // =============================================================
        
        /**
         * Vetor de estado quântico global
         * 
         * Representa |ψ⟩ = Σᵢ αᵢ|i⟩ onde:
         * - state_vector[i] = αᵢ (amplitude complexa)
         * - |i⟩ = estado base em binário
         * - tamanho = 2^numQubits
         * 
         * INDEXAÇÃO:
         * Para numQubits=3:
         * - state_vector[0] = amplitude de |000⟩
         * - state_vector[1] = amplitude de |001⟩
         * - state_vector[7] = amplitude de |111⟩
         * 
         * INVARIANTE: Σᵢ |αᵢ|² = 1 (normalização)
         */
        Eigen::VectorXcd state_vector;
        
        /**
         * Número total de qubits no sistema
         * 
         * Define:
         * - Dimensionalidade: 2^numQubits estados base
         * - Limites de validação para operações
         * - Tamanho do vetor de estado
         * 
         * LIMITES PRÁTICOS:
         * - Mínimo: 1 qubit (2 amplitudes)
         * - Máximo teórico: ~30 qubits (limitado por memória)
         * - Típico para simulação: 10-20 qubits
         */
        int numQubits;
        
        // =============================================================
        // FUNÇÕES UTILITÁRIAS INTERNAS
        // =============================================================
        
        /**
         * Converte índice inteiro para representação binária
         * 
         * @param index Índice no vetor de estado (0 a 2^numQubits-1)
         * @return String binária padronizada (ex: "0101" para 4 qubits)
         * 
         * ALGORITMO:
         * 1. Converte número para binário
         * 2. Padroniza com zeros à esquerda
         * 3. Garante largura = numQubits
         * 
         * Exemplo (3 qubits):
         * - index 0 → "000"
         * - index 5 → "101"
         * - index 7 → "111"
         */
        std::string indexToBinary(int index) const;
        
        /**
         * Converte string binária para índice no vetor
         * 
         * @param binaryString Estado em binário (ex: "101")
         * @return Índice correspondente no state_vector
         * @throws std::invalid_argument se formato inválido
         * 
         * VALIDAÇÕES:
         * - Comprimento deve ser exatamente numQubits
         * - Apenas caracteres '0' e '1' permitidos
         * 
         * ALGORITMO:
         * String "abc" → a×2² + b×2¹ + c×2⁰
         */
        int binaryToIndex(const std::string& binaryString) const;
    };

} // namespace QuantumSim

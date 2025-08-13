#include "core/Simulator.hpp"
#include "core/QuantumGate.hpp"
#include <stdexcept>
#include <random>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <bitset>

namespace QuantumSim {

    /**
     * Construtor do Simulador Quântico
     * 
     * @param num_qubits Número de qubits que o simulador irá gerenciar
     * 
     * O simulador funciona mantendo um vetor de estado complexo que representa
     * todas as possíveis combinações de estados dos qubits. Para n qubits,
     * precisamos de 2^n amplitudes complexas.
     * 
     * Exemplo: Para 2 qubits, temos 4 estados possíveis:
     * |00⟩, |01⟩, |10⟩, |11⟩
     * O vetor de estado tem 4 posições, uma para cada estado.
     */
    Simulator::Simulator(int num_qubits) : num_qubits_(num_qubits) {
        // Validação: limitamos a 20 qubits para evitar problemas de memória
        // 2^20 = ~1 milhão de amplitudes complexas ainda é gerenciável
        if (num_qubits <= 0 || num_qubits > 20) {
            throw std::invalid_argument("Número de qubits deve estar entre 1 e 20");
        }
        
        // Calcula o tamanho do vetor de estado: 2^num_qubits
        // Operador << é shift à esquerda: 1 << n = 2^n
        int state_size = 1 << num_qubits;
        
        // Cria vetor de amplitudes complexas, inicialmente todo zerado
        state_vector_ = Eigen::VectorXcd::Zero(state_size);
        
        // Inicializa no estado fundamental |00...0⟩
        // Isso significa que a primeira posição (índice 0) tem amplitude 1.0
        // e todas as outras posições têm amplitude 0.0
        state_vector_(0) = 1.0;
    }

    /**
     * Executa um circuito quântico completo
     * 
     * @param circuit O circuito quântico a ser executado
     * 
     * Esta função pega um circuito (que é uma lista de portas) e aplica
     * cada porta sequencialmente ao vetor de estado. É como executar
     * o programa quântico passo a passo.
     */
    void Simulator::execute(const QuantumCircuit& circuit) {
        // Verifica se o circuito é compatível com este simulador
        // O número de qubits deve ser o mesmo
        if (circuit.getNumQubits() != num_qubits_) {
            throw std::invalid_argument("Circuito incompatível com o número de qubits do simulador");
        }
        
        // Itera por todas as portas do circuito em ordem sequencial
        // Cada porta modifica o vetor de estado atual
        for (const auto& gate : circuit.getGates()) {
            applyGate(gate);  // Aplica a porta ao estado atual
        }
    }

    /**
     * Reinicia o simulador para o estado inicial |00...0⟩
     * 
     * Útil quando queremos executar um novo experimento sem
     * criar um novo simulador.
     */
    void Simulator::reset() {
        // Zera todas as amplitudes
        state_vector_.setZero();
        // Coloca amplitude 1.0 apenas no estado fundamental
        state_vector_(0) = 1.0;
    }

    /**
     * Inicializa um qubit específico no estado |1⟩
     * 
     * @param qubit Índice do qubit (0-based)
     * @param state true para |1⟩, false para manter |0⟩
     * 
     * Por padrão todos os qubits começam em |0⟩. Esta função
     * permite inicializar alguns qubits em |1⟩ se necessário.
     */
    void Simulator::initializeQubit(int qubit, bool state) {
        // Validação do índice do qubit
        if (qubit < 0 || qubit >= num_qubits_) {
            throw std::out_of_range("Índice de qubit inválido");
        }
        
        if (state) {
            // Para colocar um qubit em |1⟩, aplicamos a porta Pauli-X (NOT quântico)
            // que inverte o estado: |0⟩ → |1⟩ e |1⟩ → |0⟩
            applySingleQubitGate(GateMatrix::pauli_x(), qubit);
        }
    }

    /**
     * Aplica uma porta quântica ao vetor de estado
     * 
     * @param gate A porta a ser aplicada
     * 
     * Esta é a função central que decide como aplicar cada tipo de porta.
     * Portas de 1 qubit e 2 qubits têm algoritmos diferentes para eficiência.
     */
    void Simulator::applyGate(const QuantumGate& gate) {
        // Obtém a matriz matemática correspondente à porta
        // Por exemplo: Hadamard → matriz 2x2, CNOT → matriz 4x4
        Eigen::MatrixXcd gate_matrix = GateMatrix::getMatrix(gate);
        
        if (gate.qubits.size() == 1) {
            // Porta de um qubit: Hadamard, Pauli-X, Y, Z, rotações, etc.
            // Usa algoritmo otimizado para portas de 1 qubit
            applySingleQubitGate(gate_matrix, gate.qubits[0]);
            
        } else if (gate.qubits.size() == 2) {
            // Porta de dois qubits: CNOT, CZ, etc.
            // Usa algoritmo para portas controladas de 2 qubits
            applyTwoQubitGate(gate_matrix, gate.qubits[0], gate.qubits[1]);
            
        } else {
            // Portas de 3+ qubits não implementadas ainda (Toffoli, etc.)
            throw std::invalid_argument("Portas com mais de 2 qubits não implementadas ainda");
        }
    }

    /**
     * Aplica uma porta de um qubit ao vetor de estado
     * 
     * @param gate_matrix Matriz 2x2 da porta quântica
     * @param qubit Índice do qubit alvo (0-based)
     * 
     * ALGORITMO DETALHADO:
     * 
     * Para aplicar uma porta de 1 qubit, precisamos considerar que ela
     * afeta apenas UM qubit específico, mas o vetor de estado contém
     * informações sobre TODOS os qubits simultaneamente.
     * 
     * Exemplo com 2 qubits:
     * - Estado atual: a|00⟩ + b|01⟩ + c|10⟩ + d|11⟩
     * - Aplicar H no qubit 0: deve transformar |0⟩→(|0⟩+|1⟩)/√2 e |1⟩→(|0⟩-|1⟩)/√2
     * - Resultado: (a+c)/√2|00⟩ + (b+d)/√2|01⟩ + (a-c)/√2|10⟩ + (b-d)/√2|11⟩
     * 
     * O algoritmo itera por todos os estados possíveis e calcula como
     * cada amplitude se transforma baseada no valor do qubit alvo.
     */
    void Simulator::applySingleQubitGate(const Eigen::Matrix2cd& gate_matrix, int qubit) {
        int state_size = state_vector_.size();  // 2^num_qubits
        Eigen::VectorXcd new_state = Eigen::VectorXcd::Zero(state_size);
        
        // Itera por cada amplitude do estado atual
        for (int i = 0; i < state_size; ++i) {
            // Otimização: pula amplitudes que são zero (muito pequenas)
            if (std::abs(state_vector_(i)) < 1e-10) continue;
            
            // Extrai o valor do qubit alvo no estado i
            // Operação bit: (i >> qubit) pega o bit na posição 'qubit'
            // & 1 garante que pegamos apenas 0 ou 1
            int qubit_bit = (i >> qubit) & 1;
            
            // A porta 2x2 pode transformar |0⟩ em combinação de |0⟩ e |1⟩
            // e |1⟩ em combinação de |0⟩ e |1⟩
            // Por isso iteramos os dois possíveis valores de saída
            for (int new_bit = 0; new_bit < 2; ++new_bit) {
                // Calcula qual seria o novo índice de estado
                int new_index = i;
                if (qubit_bit != new_bit) {
                    // Se o bit mudou, fazemos flip usando XOR
                    new_index ^= (1 << qubit);
                }
                
                // Aplica a transformação da matriz:
                // new_amplitude = gate_matrix[new_bit][qubit_bit] * old_amplitude
                new_state(new_index) += gate_matrix(new_bit, qubit_bit) * state_vector_(i);
            }
        }
        
        // Substitui o estado antigo pelo novo
        state_vector_ = new_state;
    }

    /**
     * Aplica uma porta de dois qubits ao vetor de estado
     * 
     * @param gate_matrix Matriz 4x4 da porta quântica
     * @param control Índice do qubit de controle
     * @param target Índice do qubit alvo
     * 
     * ALGORITMO DETALHADO:
     * 
     * Portas de 2 qubits operam em pares de qubits simultaneamente.
     * A matriz 4x4 representa todas as transformações possíveis:
     * 
     * Estados de entrada:  |00⟩, |01⟩, |10⟩, |11⟩ (índices 0,1,2,3)
     * Estados de saída:    |00⟩, |01⟩, |10⟩, |11⟩ (índices 0,1,2,3)
     * 
     * Exemplo CNOT:
     * |00⟩ → |00⟩, |01⟩ → |01⟩, |10⟩ → |11⟩, |11⟩ → |10⟩
     * 
     * Matriz CNOT:
     * [1 0 0 0]
     * [0 1 0 0]  
     * [0 0 0 1]
     * [0 0 1 0]
     */
    void Simulator::applyTwoQubitGate(const Eigen::Matrix4cd& gate_matrix, int control, int target) {
        int state_size = state_vector_.size();
        Eigen::VectorXcd new_state = Eigen::VectorXcd::Zero(state_size);
        
        // Itera por todos os estados possíveis do sistema
        for (int i = 0; i < state_size; ++i) {
            // Otimização: pula amplitudes zero
            if (std::abs(state_vector_(i)) < 1e-10) continue;
            
            // Extrai os bits dos qubits de controle e alvo
            int control_bit = (i >> control) & 1;  // 0 ou 1
            int target_bit = (i >> target) & 1;    // 0 ou 1
            
            // Calcula o índice na matriz 4x4
            // Convertemos (control_bit, target_bit) para índice linear:
            // (0,0)→0, (0,1)→1, (1,0)→2, (1,1)→3
            int input_index = control_bit * 2 + target_bit;
            
            // A matriz 4x4 pode distribuir esta amplitude entre
            // todos os 4 possíveis estados de saída
            for (int output_index = 0; output_index < 4; ++output_index) {
                // Converte índice linear de volta para bits
                int new_control_bit = output_index / 2;  // 0 ou 1
                int new_target_bit = output_index % 2;   // 0 ou 1
                
                // Constrói o novo índice do estado global
                int new_state_index = i;
                
                // Atualiza bit de controle se necessário
                if (control_bit != new_control_bit) {
                    new_state_index ^= (1 << control);
                }
                
                // Atualiza bit alvo se necessário  
                if (target_bit != new_target_bit) {
                    new_state_index ^= (1 << target);
                }
                
                // Aplica a transformação matricial
                new_state(new_state_index) += gate_matrix(output_index, input_index) * state_vector_(i);
            }
        }
        
        state_vector_ = new_state;
    }

    /**
     * Calcula as probabilidades de cada estado base
     * 
     * @return Mapa string → probabilidade para cada estado significativo
     * 
     * Em mecânica quântica, a probabilidade de medir um estado específico
     * é o quadrado da magnitude da amplitude complexa desse estado.
     * 
     * Por exemplo, se o estado é: 0.6|00⟩ + 0.8|11⟩
     * Então: P(|00⟩) = |0.6|² = 0.36 = 36%
     *        P(|11⟩) = |0.8|² = 0.64 = 64%
     */
    std::map<std::string, double> Simulator::getProbabilities() const {
        std::map<std::string, double> probabilities;
        int state_size = state_vector_.size();
        
        for (int i = 0; i < state_size; ++i) {
            // Calcula probabilidade: |amplitude|²
            double probability = std::norm(state_vector_(i));
            
            // Só inclui probabilidades significativas (> 0.0000000001)
            if (probability > 1e-10) {
                // Converte índice i para representação binária
                // std::bitset<32> cria string binária de 32 bits
                std::string binary_state = std::bitset<32>(i).to_string();
                
                // Pega apenas os bits relevantes (os últimos num_qubits_ bits)
                // Por exemplo, para 2 qubits, pega apenas os últimos 2 bits
                binary_state = binary_state.substr(32 - num_qubits_);
                
                probabilities[binary_state] = probability;
            }
        }
        
        return probabilities;
    }

    /**
     * Realiza medição de um qubit específico
     * 
     * @param qubit Índice do qubit a ser medido
     * @return 0 ou 1 baseado na medição probabilística
     * 
     * PROCESSO DE MEDIÇÃO QUÂNTICA:
     * 
     * 1. CALCULAR PROBABILIDADES: Soma todas as amplitudes onde o qubit está em |0⟩
     * 2. ESCOLHA ALEATÓRIA: Gera número aleatório para decidir 0 ou 1
     * 3. COLAPSO DO ESTADO: Zera amplitudes inconsistentes com o resultado
     * 4. RENORMALIZAÇÃO: Ajusta amplitudes restantes para manter norma = 1
     * 
     * Exemplo: Estado inicial = 0.6|00⟩ + 0.8|11⟩
     * Medindo qubit 0:
     * - P(qubit0=0) = |0.6|² = 0.36 = 36%  
     * - P(qubit0=1) = |0.8|² = 0.64 = 64%
     * Se resultado = 1, novo estado = |11⟩ (amplitude normalizada para 1.0)
     */
    int Simulator::measureQubit(int qubit) {
        // Validação do índice
        if (qubit < 0 || qubit >= num_qubits_) {
            throw std::out_of_range("Índice de qubit inválido");
        }
        
        // PASSO 1: Calcula probabilidade de medir |0⟩
        double prob_zero = 0.0;
        int state_size = state_vector_.size();
        
        for (int i = 0; i < state_size; ++i) {
            // Verifica se o qubit está em |0⟩ neste estado
            if (((i >> qubit) & 1) == 0) {
                // Soma a probabilidade (|amplitude|²)
                prob_zero += std::norm(state_vector_(i));
            }
        }
        
        // PASSO 2: Escolha aleatória baseada na probabilidade
        static std::random_device rd;                    // Fonte de entropia
        static std::mt19937 gen(rd());                  // Gerador Mersenne Twister
        std::uniform_real_distribution<> dis(0.0, 1.0); // Distribuição uniforme [0,1]
        
        // Se número aleatório < prob_zero, resultado é 0, senão é 1
        int result = (dis(gen) < prob_zero) ? 0 : 1;
        
        // PASSO 3: Colapso do estado
        // Calcula fator de normalização para manter norma = 1
        double norm_factor = (result == 0) ? std::sqrt(prob_zero) : std::sqrt(1.0 - prob_zero);
        
        for (int i = 0; i < state_size; ++i) {
            // Extrai o valor do qubit medido neste estado
            int qubit_state = (i >> qubit) & 1;
            
            if (qubit_state != result) {
                // PASSO 4a: Zera amplitudes inconsistentes com o resultado
                state_vector_(i) = 0.0;
            } else {
                // PASSO 4b: Renormaliza amplitudes consistentes
                state_vector_(i) /= norm_factor;
            }
        }
        
        return result;
    }

    /**
     * Mede todos os qubits sequencialmente
     * 
     * @return String binária com resultado da medição
     * 
     * ATENÇÃO: Esta função mede os qubits UM POR VEZ, em ordem.
     * Cada medição colapsa o estado, afetando as medições subsequentes.
     * 
     * Exemplo: Estado |00⟩ + |11⟩ (emaranhado)
     * - Mede qubit 0 primeiro → resultado aleatório (50% chance de 0 ou 1)
     * - Se resultado = 0 → estado colapsa para |00⟩ → qubit 1 será definitivamente 0
     * - Se resultado = 1 → estado colapsa para |11⟩ → qubit 1 será definitivamente 1
     */
    std::string Simulator::measureAll() {
        std::string result;
        
        // Mede cada qubit sequencialmente da esquerda para direita
        for (int i = 0; i < num_qubits_; ++i) {
            // Cada measureQubit() colapsa o estado antes da próxima medição
            result += std::to_string(measureQubit(i));
        }
        
        return result;
    }

    /**
     * Calcula probabilidade de um estado específico SEM medição
     * 
     * @param state String binária representando o estado (ex: "101")
     * @return Probabilidade entre 0.0 e 1.0
     * 
     * Esta função é "não-destrutiva" - ela apenas calcula a probabilidade
     * sem afetar o estado atual do sistema. Útil para análise.
     */
    double Simulator::getStateProbability(const std::string& state) const {
        // Validação: string deve ter mesmo tamanho que número de qubits
        if (state.length() != static_cast<size_t>(num_qubits_)) {
            throw std::invalid_argument("Estado deve ter o mesmo número de bits que qubits");
        }
        
        // Converte string binária para índice numérico
        int index = 0;
        for (size_t i = 0; i < state.length(); ++i) {
            if (state[i] == '1') {
                // Se bit i é '1', adiciona 2^i ao índice
                index |= (1 << i);
            } else if (state[i] != '0') {
                throw std::invalid_argument("Estado deve conter apenas '0' e '1'");
            }
        }
        
        // Retorna |amplitude|² para este estado
        return std::norm(state_vector_(index));
    }

    /**
     * Converte o estado quântico atual para representação legível
     * 
     * @return String no formato "a|00⟩ + b|01⟩ + c|10⟩ + d|11⟩"
     * 
     * Esta função é extremamente útil para debug e visualização.
     * Ela mostra todas as amplitudes complexas não-zero do estado atual.
     * 
     * Exemplos de saída:
     * - Estado fundamental: "1.0000|00⟩"
     * - Superposição: "0.7071|00⟩ + 0.7071|11⟩"
     * - Com fase: "(0.5000 + 0.5000i)|01⟩ + (0.5000 - 0.5000i)|10⟩"
     */
    std::string Simulator::stateToString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4);  // 4 casas decimais
        
        int state_size = state_vector_.size();
        bool first = true;  // Flag para controlar o primeiro termo
        
        // Itera por todos os possíveis estados
        for (int i = 0; i < state_size; ++i) {
            std::complex<double> amplitude = state_vector_(i);
            
            // Só inclui amplitudes significativas (não-zero)
            if (std::abs(amplitude) > 1e-10) {
                // Adiciona " + " entre termos (exceto o primeiro)
                if (!first) ss << " + ";
                first = false;
                
                // FORMATAÇÃO DA AMPLITUDE COMPLEXA:
                if (std::abs(amplitude.imag()) < 1e-10) {
                    // Número real puro: "0.7071"
                    ss << amplitude.real();
                } else if (std::abs(amplitude.real()) < 1e-10) {
                    // Número imaginário puro: "0.5000i"
                    ss << amplitude.imag() << "i";
                } else {
                    // Número complexo completo: "(0.5000 + 0.3000i)"
                    ss << "(" << amplitude.real() << " + " << amplitude.imag() << "i)";
                }
                
                // FORMATAÇÃO DO ESTADO BASE:
                // Converte índice i para string binária
                std::string binary_state = std::bitset<32>(i).to_string();
                // Pega apenas os bits relevantes
                binary_state = binary_state.substr(32 - num_qubits_);
                // Adiciona notação quântica: "|101⟩"
                ss << "|" << binary_state << "⟩";
            }
        }
        
        // Se nenhuma amplitude foi encontrada, estado é zero
        if (first) ss << "0";
        
        return ss.str();
    }

    /**
     * Verifica se o vetor de estado está normalizado
     * 
     * @return true se ||ψ||² = 1, false caso contrário
     * 
     * Em mecânica quântica, o vetor de estado deve sempre ter norma 1
     * (soma de todas as probabilidades = 100%). Esta função verifica
     * se esta condição está sendo mantida após as operações.
     * 
     * ||ψ||² = Σ|aᵢ|² onde aᵢ são as amplitudes
     */
    bool Simulator::isNormalized() const {
        // Calcula ||ψ||² usando função otimizada do Eigen
        double norm_squared = state_vector_.squaredNorm();
        
        // Verifica se está próximo de 1.0 (considerando erros de ponto flutuante)
        return std::abs(norm_squared - 1.0) < 1e-10;
    }

} // namespace QuantumSim

/**
 * PROGRAMA PRINCIPAL - Demonstração do Simulador Quântico LACIQ
 * 
 * Este programa demonstra todas as funcionalidades principais do
 * simulador quântico implementado. Ele serve como:
 * 
 * 1. TESTE DE FUNCIONAMENTO: Verifica se todas as partes funcionam
 * 2. EXEMPLO DE USO: Mostra como usar as classes implementadas
 * 3. TUTORIAL: Demonstra conceitos de computação quântica
 * 
 * FLUXO DO PROGRAMA:
 * 1. Testa biblioteca Eigen (álgebra linear)
 * 2. Cria circuito quântico com portas
 * 3. Executa simulação passo a passo
 * 4. Analisa resultados (probabilidades, normalização)
 */

#include <iostream>
#include <Eigen/Dense>
#include "core/Simulator.hpp"
#include "core/QuantumCircuit.hpp"

int main() {
    std::cout << "=== Simulador Quântico LACIQ ===" << std::endl;
    std::cout << "Demonstrando o núcleo da simulação\\n" << std::endl;
    
    try {
        // =================================================================
        // TESTE 1: VERIFICAÇÃO DA BIBLIOTECA EIGEN
        // =================================================================
        std::cout << "1. Teste do Eigen:" << std::endl;
        
        // Cria vetor 3D simples
        Eigen::Vector3d v(1, 2, 3);
        // Cria matriz identidade 3x3
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
        
        std::cout << "   Vetor: " << v.transpose() << std::endl;
        std::cout << "   Matriz identidade 3x3:" << std::endl << m << std::endl;
        
        // =================================================================
        // TESTE 2: CONSTRUÇÃO DE CIRCUITO QUÂNTICO
        // =================================================================
        std::cout << "" << std::endl;
        std::cout << "2. Criando circuito quântico (2 qubits):" << std::endl;
        
        // Cria circuito para operar em 2 qubits
        QuantumSim::QuantumCircuit circuit(2);
        
        // SEQUÊNCIA DE PORTAS para criar estado de Bell:
        circuit.addHadamard(0);     // H(0): |00⟩ → (|00⟩+|10⟩)/√2
        circuit.addCNOT(0, 1);      // CNOT(0,1): (|00⟩+|10⟩)/√2 → (|00⟩+|11⟩)/√2
        circuit.addPauliX(1);       // X(1): Inverte qubit 1 por demonstração
        
        // Mostra a sequência de portas que será executada
        std::cout << "   " << circuit.toString() << std::endl;
        
        // =================================================================
        // TESTE 3: SIMULAÇÃO PASSO A PASSO
        // =================================================================
        std::cout << "" << std::endl;
        std::cout << "3. Executando simulação:" << std::endl;
        
        // Cria simulador para 2 qubits
        QuantumSim::Simulator simulator(2);
        
        // ESTADO INICIAL: Todos os qubits começam em |0⟩
        // Para 2 qubits: estado inicial = |00⟩
        std::cout << "   Estado inicial: " << simulator.stateToString() << std::endl;
        
        // PASSO 1: Aplica apenas Hadamard no qubit 0
        QuantumSim::QuantumCircuit simple_circuit(2);
        simple_circuit.addHadamard(0);
        simulator.execute(simple_circuit);
        
        // RESULTADO: |00⟩ → (|00⟩+|10⟩)/√2 (superposição)
        std::cout << "   Após H(0): " << simulator.stateToString() << std::endl;
        
        // PASSO 2: Aplica CNOT entre qubits 0 e 1
        QuantumSim::QuantumCircuit cnot_circuit(2);
        cnot_circuit.addCNOT(0, 1);
        simulator.execute(cnot_circuit);
        
        // RESULTADO: (|00⟩+|10⟩)/√2 → (|00⟩+|11⟩)/√2 (emaranhamento!)
        std::cout << "   Após CNOT(0,1): " << simulator.stateToString() << std::endl;

        // PASSO 3: Inverte qubit 1 (aplica Pauli-X)
        QuantumSim::QuantumCircuit x_circuit(2);
        x_circuit.addPauliX(1);
        simulator.execute(x_circuit);

        // RESULTADO: (|00⟩+|11⟩)/√2 → (|01⟩+|10⟩)/√2 (inverte qubit 1)
        std::cout << "   Após X(1): " << simulator.stateToString() << std::endl;
        
        // =================================================================
        // TESTE 4: ANÁLISE DE PROBABILIDADES
        // =================================================================
        std::cout << "" << std::endl;
        std::cout << "4. Probabilidades dos estados:" << std::endl;

        // Obtém mapa de todos os estados com probabilidade > 0
        auto probabilities = simulator.getProbabilities();
        
        for (const auto& [state, prob] : probabilities) {
            std::cout << "   |" << state << "⟩: " << prob * 100 << "%" << std::endl;
        }
        
        // =================================================================
        // TESTE 5: VERIFICAÇÃO DE INTEGRIDADE
        // =================================================================
        std::cout << "" << std::endl;
        std::cout << "5. Estado normalizado: " 
                  << (simulator.isNormalized() ? "Sim" : "Não") << std::endl;
        
        // A normalização é crucial: soma de todas as probabilidades = 100%
        // Se não estiver normalizado, há bug no simulador

        std::cout << "=== Núcleo da simulação funcionando corretamente! ===" << std::endl;

    } catch (const std::exception& e) {
        // TRATAMENTO DE ERROS
        // Captura qualquer exceção (erros de validação, índices inválidos, etc.)
        std::cerr << "Erro: " << e.what() << std::endl;
        return 1;  // Código de saída indicando erro
    }
    
    return 0;  // Sucesso!
}
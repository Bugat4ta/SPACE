import numpy as np

class QuantumCalculator:
    def __init__(self, num_qubits):
        self.num_qubits = num_qubits
        self.state = np.zeros((2**num_qubits, 1), dtype=complex)

    def initialize(self):
        self.state = np.zeros((2**self.num_qubits, 1), dtype=complex)
        self.state[0] = 1

    def store(self, index):
        if index < 0 or index >= 2**self.num_qubits:
            raise ValueError("Index out of bounds for quantum storage.")
        self.state = np.zeros((2**self.num_qubits, 1), dtype=complex)
        self.state[index] = 1

    def add(self, a, b):
        if a < 0 or a >= 2**self.num_qubits or b < 0 or b >= 2**self.num_qubits:
            raise ValueError("Operands out of bounds for addition.")
        result = (a + b) % (2 ** self.num_qubits)
        self.store(result)

    def retrieve(self):
        probabilities = np.abs(self.state) ** 2
        return probabilities.flatten()

    def print_state(self):
        binary_state = np.argmax(np.abs(self.state)**2)
        binary_representation = f'{binary_state:0{self.num_qubits}b}'
        print(f"Current Quantum State: |{binary_representation}>")

    def binary_to_decimal(self, binary_str):
        return int(binary_str, 2)

    def custom_input(self):
        a_str = input(f"Enter the first binary number (up to {self.num_qubits} bits): ")
        b_str = input(f"Enter the second binary number (up to {self.num_qubits} bits): ")
        
        if len(a_str) > self.num_qubits or len(b_str) > self.num_qubits:
            raise ValueError(f"Input exceeds {self.num_qubits} bits.")
        
        a_decimal = self.binary_to_decimal(a_str)
        b_decimal = self.binary_to_decimal(b_str)
        
        return a_decimal, b_decimal

num_qubits = 10
quantum_calculator = QuantumCalculator(num_qubits)

quantum_calculator.initialize()
quantum_calculator.print_state()

a, b = quantum_calculator.custom_input()

quantum_calculator.store(a)
quantum_calculator.print_state()

quantum_calculator.store(b)
quantum_calculator.print_state()

quantum_calculator.add(a, b)
quantum_calculator.print_state()

probabilities = quantum_calculator.retrieve()
print("\nMeasurement Probabilities:")
for i, prob in enumerate(probabilities):
    if prob > 0:
        print(f"|{i:0{num_qubits}b}>: {prob:.4f}")

import numpy as np

class ParallelUniverseSimulator:
    def __init__(self, num_qubits):
        self.num_qubits = num_qubits
        self.state = np.zeros((2**num_qubits, 1), dtype=complex)
        self.initialize()

    def initialize(self):
        self.state[0] = 1  # Start in state |000...0>

    def apply_hadamard(self):
        # Create the full Hadamard gate for all qubits
        hadamard_matrix = self.hadamard_matrix(self.num_qubits)
        # Apply the Hadamard gate to the entire state
        self.state = np.dot(hadamard_matrix, self.state)

    def hadamard_matrix(self, num_qubits):
        """Create the Hadamard matrix for multiple qubits."""
        H = np.array([[1, 1], [1, -1]]) / np.sqrt(2)  # 2x2 Hadamard matrix
        # Kronecker product to get Hadamard for num_qubits
        for _ in range(num_qubits - 1):
            H = np.kron(H, np.array([[1, 1], [1, -1]]) / np.sqrt(2))
        return H

    def measure(self):
        # Calculate probabilities for each state
        probabilities = np.abs(self.state.flatten()) ** 2
        measurement = np.random.choice(range(2**self.num_qubits), p=probabilities)
        return measurement

    def print_state(self):
        print("Current Quantum State Amplitudes:")
        for i in range(2**self.num_qubits):
            amplitude = self.state[i][0]
            print(f"|{i:0{self.num_qubits}b}>: {amplitude:.4f}")

# Example usage
num_qubits = 3
simulator = ParallelUniverseSimulator(num_qubits)

# Create superposition
simulator.apply_hadamard()
simulator.print_state()

# Simulate a measurement
measurement_result = simulator.measure()
print(f"\nMeasurement Result: |{measurement_result:0{num_qubits}b}>")

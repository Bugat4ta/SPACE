import numpy as np

class QuantumSimulator:
    def __init__(self, num_qubits):
        self.num_qubits = num_qubits
        self.state = self.initialize_state()

    def initialize_state(self):
        # Initialize state vector for qubits (|0...0>)
        return np.zeros((2**self.num_qubits, 1), dtype=complex)

    def set_state(self, index):
        if index < 0 or index >= 2**self.num_qubits:
            raise ValueError("Index out of bounds.")
        self.state = np.zeros((2**self.num_qubits, 1), dtype=complex)
        self.state[index] = 1  # Set the state to |index>

    def apply_hadamard(self, qubit_index):
        # Apply Hadamard gate on a specific qubit
        h_matrix = (1/np.sqrt(2)) * np.array([[1, 1], [1, -1]], dtype=complex)
        current_state = self.state.copy()

        for i in range(2**self.num_qubits):
            if (i >> qubit_index) & 1:  # Check if the qubit is in state |1>
                # Apply the Hadamard operation
                basis_index = i ^ (1 << qubit_index)  # Flip the target qubit
                current_state[basis_index] += current_state[i]
                current_state[i] = 0  # Set original to 0 after operation
            else:
                # Apply the Hadamard operation
                current_state[i] = (current_state[i] * h_matrix[0, 0] +
                                    current_state[i ^ (1 << qubit_index)] * h_matrix[1, 0])

        self.state = current_state

    def measure(self):
        # Calculate probabilities for each state
        probabilities = np.abs(self.state.flatten()) ** 2
        probabilities /= np.sum(probabilities)  # Normalize
        measurement = np.random.choice(range(2**self.num_qubits), p=probabilities)
        return measurement

    def print_state(self):
        print("Current Quantum State Amplitudes:")
        for i in range(2**self.num_qubits):
            amplitude = self.state[i][0]
            if amplitude != 0:
                print(f"|{i:0{self.num_qubits}b}>: {amplitude:.4f}")

# Example usage
num_qubits = 30
quantum_simulator = QuantumSimulator(num_qubits)

# Initialize to the |000...0> state
quantum_simulator.set_state(0)
quantum_simulator.print_state()

# Apply Hadamard gates to the first few qubits to create superposition
for qubit in range(5):  # Change this to apply to more qubits if needed
    quantum_simulator.apply_hadamard(qubit)

quantum_simulator.print_state()

# Measure the quantum state
measurement_result = quantum_simulator.measure()
print(f"\nMeasurement Result: |{measurement_result:0{num_qubits}b}>")

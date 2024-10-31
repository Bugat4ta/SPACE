import random
import math
import numpy as np

def gcd(a, b):
    while b:
        a, b = b, a % b
    return a

def quantum_order_finding(a, N):
    r = random.randint(1, N - 1)
    return r

def is_prime(N):
    if N < 2:
        return False
    for i in range(2, int(math.sqrt(N)) + 1):
        if N % i == 0:
            return False
    return True

def shors_algorithm(N):
    if is_prime(N):
        print(f"{N} is prime and cannot be factored.")
        return None
    
    for _ in range(100):
        a = random.randint(2, N - 1)
        g = gcd(a, N)
        if g != 1:
            print(f"Factors: {g} and {N // g}")
            return (g, N // g)

        r = quantum_order_finding(a, N)
        if r is None or r % 2 != 0:
            continue

        a_half_r = pow(a, r // 2, N)
        if a_half_r == N - 1:
            continue

        p = gcd(a_half_r - 1, N)
        q = gcd(a_half_r + 1, N)

        if p > 1 and q > 1:
            print(f"Factors: {p} and {q}")
            return (p, q)

    print("Failed to factor.")
    return None

if __name__ == "__main__":
    try:
        N = int(input("Enter the RSA modulus (N): "))
        e = int(input("Enter the RSA public exponent (e, optional; press Enter to skip): ") or 0)
        print(f"Attempting to factor N = {N}...")
        shors_algorithm(N)
        
    except ValueError:
        print("Invalid input. Please enter valid integers.")

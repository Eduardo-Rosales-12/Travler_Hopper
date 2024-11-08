import sympy as sp
import math

# Define the symbolic variables
l, l1, l2, l3, g = sp.symbols('l l1 l2 l3 g')

# Example equation: x^2 + y = z
equation = sp.Eq(l3 + (l1*sp.cos(g)) + sp.sqrt((l2**2)-((l1**2)*(sp.sin(g))**2)), l)

# Solve for 'x'
solution = sp.solve(equation, g)

# Display the solution
print("Solution for g:")
print(solution)

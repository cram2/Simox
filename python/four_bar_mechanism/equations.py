import os
import os.path

from sympy import symbols, sin, cos, sqrt, asin, atan

from hemisphere_joint_demo.sympy_to_code import SympyToCpp
sp.init_printing(use_latex='mathjax')

# Actuation (P1_z, P2_z)

# active joint
theta = symbols('theta')

# passive joint
psi = symbols('psi')

# Constants defining geometry.
theta0 = symbols('theta0')

shank, p1, p2, p3 = symbols('shank p1 p2 p3')


k1 = shank / p1
k2 = shank / p3
k3 = (shank**2 + p1**2 + p3**2 - p2**2 ) / (2 * p1 * p3)

cT = cos(theta)
sT = sin(theta)

A = k1 * cT + k2 + k3 + cT # C.34
B = -2*sT # C.35
C = k1 * cT - k2 + k3 - cT # C.36

psi = 2 * atan((-B + sqrt(B * B - 4 * A * C)) / (2 * A)) #  C.39


def forwardKinematics(theta):
  

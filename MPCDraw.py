import OCP
import numpy as np
import matplotlib.pyplot as plt


test_model = OCP.OCP(-74.40007019042969, -9.511345863342285, -77.89087677001953, -1.5707623714909982, 4.095040687328225, 0.0)
output = test_model.calculate()

print(output)


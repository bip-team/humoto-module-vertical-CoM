
import numpy as np
from scipy.optimize import minimize

def main():
    cons=[
        {'type': 'ineq',
         'fun': lambda x: x[0],
         'jac': lambda x: np.array([1,0])},
        {'type': 'ineq',
         'fun': lambda x: x[1],
         'jac': lambda x: np.array([0,1])}
        ]
    cons.append({'type': 'ineq',
         'fun': lambda x: x[1]-5,
         'jac': lambda x: np.array([0,1])})

    res = minimize(lambda x: x[0]**2 + x[1]**2,
            np.array([3000.3,80.3]),
            jac= lambda x: np.array([2*x[0],2*x[1]]),
            constraints=cons,
            method='SLSQP', options={'disp': True})
    print('res', res.x)

main()

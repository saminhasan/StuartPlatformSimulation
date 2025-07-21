import sympy as sp

#--- define symbols ---
phi, theta, psi, phidot, thetadot, psidot, phiddot, thetaddot, psiddot = sp.symbols(
    'phi theta psi phidot thetadot psidot phiddot thetaddot psiddot', real=True
)

#--- rotation matrices (extrinsic X–Y–Z) ---
Rx = sp.Matrix([
    [1,          0,           0],
    [0,  sp.cos(phi), -sp.sin(phi)],
    [0,  sp.sin(phi),  sp.cos(phi)]
])
Ry = sp.Matrix([
    [ sp.cos(theta), 0, sp.sin(theta)],
    [             0, 1,             0],
    [-sp.sin(theta), 0, sp.cos(theta)]
])
Rz = sp.Matrix([
    [sp.cos(psi), -sp.sin(psi), 0],
    [sp.sin(psi),  sp.cos(psi), 0],
    [          0,            0, 1]
])

#--- overall rotation ---
R = Rx * Ry * Rz

#--- time derivative of R via chain rule ---
Rdot = (
    sp.diff(R, psi)   * psidot
  + sp.diff(R, theta) * thetadot
  + sp.diff(R, phi)   * phidot

)

#--- skew‐symmetric rate matrix ---
Omega = sp.simplify(Rdot * R.T)

#--- vee map: body‐angular‐velocity vector ---
omega = sp.simplify(sp.Matrix([
    Omega[2, 1],
    Omega[0, 2],
    Omega[1, 0]
]))

#--- prepare for second derivative ---
vars    = [phi, theta, psi, phidot, thetadot, psidot]
varsdot = [phidot, thetadot, psidot, phiddot, thetaddot, psiddot]

#--- time derivative of omega ---
omega_dot = sp.simplify(sp.Matrix(omega).jacobian(vars) * sp.Matrix(varsdot))

#--- display results ---
print(omega)
print(omega_dot)

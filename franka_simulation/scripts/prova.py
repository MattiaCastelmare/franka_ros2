#!/usr/bin/env python3
import pinocchio as pin
import numpy as np

# === PERCORSI ===
model_dir = "/ros2_ws/src/franka_description"
urdf_path  = "/ros2_ws/src/franka_description/robots/fr3/fr3.urdf"

# === CARICA MODELLO ===
model, cmodel, vmodel = pin.buildModelsFromUrdf(urdf_path, model_dir)
data = model.createData()

# Gravità esplicita (Z negativo)
model.gravity.linear = np.array([0.0, 0.0, -9.81])

print(f"nq={model.nq}, nv={model.nv}")

# === CONFIGURAZIONE DI TEST ===
q  = np.zeros(model.nq)   # posizioni
dq = np.zeros(model.nv)   # velocità

# === INERZIA M(q) ===
M = pin.crba(model, data, q).copy()          # (nv x nv)

# === CORIOLIS C(q, dq) ===
pin.computeCoriolisMatrix(model, data, q, dq)
C = data.C.copy()                            # (nv x nv)

# === GRAVITÀ g(q) ===
g = pin.computeGeneralizedGravity(model, data, q).copy()   # (nv,)

# === STAMPA COMPLETA ===
np.set_printoptions(precision=6, suppress=True)
print("\n--- Matrice di inerzia M (completa) ---")
print(M)
print("\n--- Matrice di Coriolis C (completa) ---")
print(C)
print("\n--- Vettore di gravità g (completo) ---")
print(g)

# === OPZIONALE: SOLO BRACCIO (prime 7 dof) ===
if model.nv >= 7:
    M_arm = M[:7, :7]
    C_arm = C[:7, :7]
    g_arm = g[:7]
    print("\n=== SOLO BRACCIO (7 DOF) ===")
    print("\nM_arm (7x7):")
    print(M_arm)
    print("\nC_arm (7x7):")
    print(C_arm)
    print("\ng_arm (7):")
    print(g_arm)

# Sanity check: nle = C*dq + g
nle = pin.nonLinearEffects(model, data, q, dq).copy()
print("\nCheck: ||C*dq + g - nle|| =", np.linalg.norm(C.dot(dq) + g - nle))

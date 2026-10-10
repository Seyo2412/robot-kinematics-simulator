import numpy as np

def forward_kinematics_3r(angles, link_lengths):
    """
    Berechnet die (x, y) Positionen der Gelenke für einen planaren 3R-Arm.
    Gibt ein 2D-Array der Form (4, 2) zurueck:
    [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
    """
    t1, t2, t3 = angles
    l1, l2, l3 = link_lengths

    # Winkel kumulieren (ebene Kinematik)
    phi1 = t1
    phi2 = t1 + t2
    phi3 = t1 + t2 + t3

    # Positionen Schritt fuer Schritt berechnen
    x0, y0 = 0.0, 0.0
    x1 = float(x0 + l1 * np.cos(phi1))
    y1 = float(y0 + l1 * np.sin(phi1))
    
    x2 = float(x1 + l2 * np.cos(phi2))
    y2 = float(y1 + l2 * np.sin(phi2))
    
    x3 = float(x2 + l3 * np.cos(phi3))
    y3 = float(y2 + l3 * np.sin(phi3))

    return np.array([
        [x0, y0],
        [x1, y1],
        [x2, y2],
        [x3, y3]
    ], dtype=float)

def inverse_kinematics_3r(target_x, target_y, target_phi_rad, link_lengths, elbow_up=True):
    """
    Berechnet analytisch die Gelenkwinkel [theta1, theta2, theta3] fuer ein Ziel (x, y, phi).
    """
    l1, l2, l3 = link_lengths

    # 1. Position des Handgelenks zurueckrechnen
    xw = target_x - l3 * np.cos(target_phi_rad)
    yw = target_y - l3 * np.sin(target_phi_rad)

    # 2. Reichweitenpruefung via Kosinussatz
    r_sq = xw**2 + yw**2
    cos_t2 = (r_sq - l1**2 - l2**2) / (2.0 * l1 * l2)

    if cos_t2 < -1.0 or cos_t2 > 1.0:
        raise ValueError("Zielpunkt liegt ausserhalb des zulaessigen Arbeitsraums!")

    # 3. Gelenkwinkel theta2
    sin_t2 = np.sqrt(1.0 - cos_t2**2)
    if not elbow_up:
        sin_t2 = -sin_t2
    t2 = np.arctan2(sin_t2, cos_t2)

    # 4. Gelenkwinkel theta1
    beta = np.arctan2(yw, xw)
    gamma = np.arctan2(l2 * np.sin(t2), l1 + l2 * np.cos(t2))
    t1 = beta - gamma

    # 5. Gelenkwinkel theta3
    t3 = target_phi_rad - t1 - t2

    return np.array([t1, t2, t3], dtype=float)

class RobotArm6DOF:

    def __init__(self):
        self.d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
        self.a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        self.alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]

    def dh_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1]
        ], dtype=float)

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        points = [T[0:3, 3]]

        for i in range(6):
            Ti = self.dh_matrix(joint_angles[i], self.d[i], self.a[i], self.alpha[i])
            T = T @ Ti
            points.append(T[0:3, 3])

        return np.array(points)
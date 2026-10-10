import numpy as np

def plan_quintic_trajectory(q_start, q_goal, duration, steps = 100):
    """
    Berechnet eine ruckfreie Gelenktrajektorie (Polynom 5. Grades).
    Randbedingunen:
    - Geschwindigkeit bei t = 0 und t = duration ist 0
    - Beschleunigung bei t = 0 und t = duration ist 0
    
    q_start:  Startwinkel-Vektor
    q_goal:   Zielwinkel-Vektor
    duration: Gesamtzeit in Sekunden
    steps:    Anzahl der Zwischenschritte
    """

    t = np.linspace(0, duration, steps)
    tau = t / duration

    s = 10 * (tau**3) - 15 * (tau**4) + 6 * (tau**5)

    trajectory = []
    for step_s in s:
        q_current = q_start + step_s * (q_goal - q_start)
        trajectory.append(q_current)

    return np.array(trajectory), t
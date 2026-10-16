import numpy as np
import matplotlib.pyplot as plt

# parameters
L1 = 91.3       
L2 = 40.0        
SERVO1_RANGE = (0, 180)    
SERVO2_RANGE = (0, 180)     
GEAR_RATIO = 2.0            # for servo 2
THETA1_OFFSET = 0           
THETA2_OFFSET = 0           
N = 400                    

# finds positions that respond to servo angle values
def forward_kinematics(s1_deg, s2_deg):
    t1 = np.radians(s1_deg + THETA1_OFFSET)
    t2 = np.radians(s2_deg * GEAR_RATIO + THETA2_OFFSET)   # gearing applied here
    ex, ey = L1 * np.cos(t1), L1 * np.sin(t1)
    x = ex + L2 * np.cos(t1 + t2)
    y = ey + L2 * np.sin(t1 + t2)
    return (ex, ey), (x, y)


def compute_workspace():
    s1 = np.linspace(*SERVO1_RANGE, N)
    s2 = np.linspace(*SERVO2_RANGE, N)
    S1, S2 = np.meshgrid(s1, s2)
    _, (X, Y) = forward_kinematics(S1, S2)
    return S1, S2, X, Y


def main():
    # visualization of arm range
    S1, S2, X, Y = compute_workspace()
    R = np.hypot(X, Y)

    joint2_span = (SERVO2_RANGE[1] - SERVO2_RANGE[0]) * GEAR_RATIO
    print(f"Link 1 = {L1} mm, Link 2 = {L2} mm")
    print(f"Joint 1 span = {SERVO1_RANGE[1]-SERVO1_RANGE[0]} deg, "
          f"Joint 2 span = {joint2_span:.0f} deg (after {GEAR_RATIO:g}:1 gearing)")
    print(f"Max reach  = {R.max():.1f} mm  (theoretical L1+L2 = {L1+L2:.1f} mm)")
    print(f"Min reach  = {R.min():.1f} mm  (theoretical |L1-L2| = {abs(L1-L2):.1f} mm)")
    print(f"X range    = {X.min():.1f} to {X.max():.1f} mm")
    print(f"Y range    = {Y.min():.1f} to {Y.max():.1f} mm")

    fig, ax = plt.subplots(figsize=(9, 7))

    # Reachable points, colored by servo 1 angle
    sc = ax.scatter(X.ravel(), Y.ravel(), c=S1.ravel(), s=1, cmap="viridis", alpha=0.35)
    fig.colorbar(sc, ax=ax, label="Servo 1 angle (deg)")

    # Reference circles: max and min reach
    t = np.linspace(0, 2 * np.pi, 400)
    for r, lbl, ls in [(L1 + L2, f"max reach {L1+L2:.1f} mm", "--"),
                       (abs(L1 - L2), f"min reach {abs(L1-L2):.1f} mm", ":")]:
        ax.plot(r * np.cos(t), r * np.sin(t), "k", ls=ls, lw=1, label=lbl)

    # Path of the elbow (end of link 1)
    s1_line = np.linspace(*SERVO1_RANGE, 200)
    (ex, ey), _ = forward_kinematics(s1_line, 0)
    ax.plot(ex, ey, color="tab:red", lw=2, label="elbow path (end of link 1)")

    # A few example arm poses
    for s1, s2 in [(30, 20), (90, 120), (150, 60)]:
        (ex, ey), (x, y) = forward_kinematics(s1, s2)
        ax.plot([0, ex, x], [0, ey, y], "-o", color="tab:orange", lw=2.5, ms=5)
        ax.annotate(f"({s1}°, {s2}°)", (x, y), textcoords="offset points",
                    xytext=(5, 5), fontsize=8)
    ax.plot([], [], "-o", color="tab:orange", label="example poses (servo1, servo2)")

    ax.plot(0, 0, "ks", ms=8, label="base (servo 1)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title(f"2-DOF arm workspace  (L1 = {L1} mm, L2 = {L2} mm, "
                 f"{GEAR_RATIO:g}:1 elbow gearing)")
    ax.legend(loc="lower left", fontsize=8)
    plt.tight_layout()
    plt.savefig("arm_workspace.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
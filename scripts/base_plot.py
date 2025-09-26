from robot import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
    ax: a matplotlib axis, e.g., as output from plt.gca().
    """
    ax = plt.gca()
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.45 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def show(ax):
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.set_edgecolor("w")
        axis.pane.set_alpha(0)
        axis.line.set_color((1, 1, 1, 0))
    ax.grid(False)
    set_axes_equal(ax)
    plt.show()


if __name__ == "__main__":
    R = 0.08
    N = 0.27999999999999997 / R
    rB, dB = 0.10160254037844389, 0.030000000000000016
    rP, dP = 0.0716025403784439, 0.020000000000000004
    L = 2e-2
    SP = StuartPlatform(R, N, rB, dB, rP, dP)
    angles, P = SP.move(array([0.0, 0.0, 0.01, np.deg2rad(0), np.deg2rad(-30), np.deg2rad(0)]))
    B = SP.B
    H = SP.h(angles)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    # ----- origins -----
    O_b = np.zeros(3)
    O_p = P.mean(axis=0)  # plate origin in world (centroid; symmetric layout -> exact)

    # ----- base circle (Z=0 plane) -----
    tt = np.linspace(0, 2 * np.pi, 360)
    base_circle = np.column_stack([rB * np.cos(tt), rB * np.sin(tt), np.zeros_like(tt)])
    ax.plot(base_circle[:, 0], base_circle[:, 1], base_circle[:, 2], color="k", linewidth=1, linestyle="--")
    # ----- vectors Ob->Bk as r_B then d_B components -----
    # Origins and directions for the rb "radii" parts
    rb_origins = []
    rb_dirs = []

    # Origins and directions for the db "spokes" parts
    db_origins = []
    db_dirs = []

    for k in range(6):
        u_gB = np.array([np.cos(SP.gammaB[k]), np.sin(SP.gammaB[k]), 0.0])
        u_bB = np.array([np.cos(SP.betaB[k]), np.sin(SP.betaB[k]), 0.0])
        v_rB = rB * u_gB
        v_dB = dB * u_bB

        # rb: from Ob to Ob + v_rB
        rb_origins.append(O_b)
        rb_dirs.append(v_rB)

        # db: from Ob + v_rB to Ob + v_rB + v_dB
        db_origins.append(O_b + v_rB)
        db_dirs.append(v_dB)

    # Convert to arrays
    rb_origins = np.array(rb_origins)
    rb_dirs = np.array(rb_dirs)
    db_origins = np.array(db_origins)
    db_dirs = np.array(db_dirs)

    # Plot with quiver
    ax.quiver(
        rb_origins[:, 0],
        rb_origins[:, 1],
        rb_origins[:, 2],  # origins
        rb_dirs[:, 0],
        rb_dirs[:, 1],
        rb_dirs[:, 2],  # directions
        color="k",
        linewidth=1,
        arrow_length_ratio=0.1,
    )
    # Label rb arrows at midpoints
    for i in range(len(rb_origins)):
        mid_point = rb_origins[i] + 0.5 * rb_dirs[i]
        ax.text(mid_point[0], mid_point[1], mid_point[2] + 0.005, "rB", color="k", fontsize=10, ha="center", va="center")
    ax.quiver(
        db_origins[:, 0],
        db_origins[:, 1],
        db_origins[:, 2],
        db_dirs[:, 0],
        db_dirs[:, 1],
        db_dirs[:, 2],
        color="k",
        linewidth=1,
        arrow_length_ratio=0.2,
    )
    # Label db arrows at midpoints
    for i in range(len(db_origins)):
        mid_point = db_origins[i] + 0.5 * db_dirs[i]
        ax.text(mid_point[0], mid_point[1], mid_point[2] + 0.005, "dB", color="k", fontsize=10, ha="center", va="center")
    # ----- RGB triads at Ob (world frame) and Op (plate frame) -----

    # World triad at Ob
    ax.quiver(
        [O_b[0]] * 3,
        [O_b[1]] * 3,
        [O_b[2]] * 3,
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        length=L,
        normalize=False,
        colors=["r", "g", "b"],
        arrow_length_ratio=0.1,
    )
    # optional tiny labels
    ax.text(O_b[0] + L * 1.05, O_b[1], O_b[2], "$B_x$", fontsize=14, color="r")
    ax.text(O_b[0], O_b[1] + L * 1.05, O_b[2], "$B_y$", fontsize=14, color="g")
    ax.text(O_b[0], O_b[1], O_b[2] + L * 1.05, "$B_z$", fontsize=14, color="b")
    ax.text(O_b[0], O_b[1], O_b[2] - 0.02, "$O_B$", fontsize=14, color="k")
    for k in range(6):
        c = B[k]  # center at B_k
        beta = SP.betaB[k]  # base orientation about z
        Rz = np.array([[np.cos(beta), -np.sin(beta), 0.0], [np.sin(beta), np.cos(beta), 0.0], [0.0, 0.0, 1.0]])
        ex, ey, ez = Rz[:, 0], Rz[:, 1], Rz[:, 2]
        ax.text(c[0], c[1], c[2] - 0.02, f"$B_{{{k+1}}}$", fontsize=14, color="k")
        # reference triad at B_k (translated global axes)
        ax.quiver(
            [c[0]] * 3,
            [c[1]] * 3,
            [c[2]] * 3,
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            length=L,
            normalize=False,
            colors=["r", "g", "b"],
            arrow_length_ratio=0.1,
        )
        # # rotated triad at B_k: Rz(beta) * Ry(-alpha)
        alpha_k = 0  # servo arm rotation about local y

        Rz = np.array([[np.cos(beta), -np.sin(beta), 0.0], [np.sin(beta), np.cos(beta), 0.0], [0.0, 0.0, 1.0]])
        ca, sa = np.cos(alpha_k), np.sin(alpha_k)
        Ry = np.array([[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]])
        Rba = Rz @ Ry
        ex, ey, ez = Rba[:, 0], Rba[:, 1], Rba[:, 2]
        ax.quiver(
            [c[0]] * 3,
            [c[1]] * 3,
            [c[2]] * 3,
            [ex[0], ey[0], ez[0]],
            [ex[1], ey[1], ez[1]],
            [ex[2], ey[2], ez[2]],
            length=L,
            normalize=False,
            colors=["r", "g", "b"],
            arrow_length_ratio=0.1,
        )
        # --- arc for β_k in the XY plane at B_k (from global x to Rz(beta)*x) ---
        r1 = 0.5 * L
        t1 = np.linspace(0.0, beta, 64)
        arc1 = np.column_stack([c[0] + r1 * np.cos(t1), c[1] + r1 * np.sin(t1), np.full_like(t1, c[2])])
        ax.plot(arc1[:, 0], arc1[:, 1], arc1[:, 2], "b-", linewidth=1.0)
        mid1 = arc1[len(t1) // 2]
        ax.text(mid1[0], mid1[1], mid1[2], rf"$\beta_{{{k+1}}}$", fontsize=14, color="b")
    show(ax)

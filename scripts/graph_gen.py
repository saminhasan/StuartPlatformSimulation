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
    R = 0.1
    N = 0.27999999999999997 / R
    rB, dB = 0.10160254037844389, 0.030000000000000016
    rP, dP = 0.0716025403784439, 0.020000000000000004
    L = 2e-2
    SP = StuartPlatform(R, N, rB, dB, rP, dP)
    angles, P = SP.move(array([0.0, 0.0, 0.01, np.deg2rad(5), np.deg2rad(-5), np.deg2rad(5)]))
    B = SP.B
    H = SP.h(angles)
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(B[:, 0], B[:, 1], B[:, 2], color="k", alpha=1.0, s=10)
    ax.scatter(H[:, 0], H[:, 1], H[:, 2], color="k", alpha=1.0, s=10)
    for k in range(H.shape[0]):
        # ax.text(H[k, 0], H[k, 1], H[k, 2] + L / 2, f"$H_{k+1}$", fontsize=14, color="k")
        pass
    # ax.scatter(P[:, 0], P[:, 1], P[:, 2], color="k", alpha=1.0, s=10)
    lines_BH = np.stack([B, H], axis=1)
    lines_HP = np.stack([H, P], axis=1)
    # lines_BP = np.stack([B, P], axis=1)

    # all_lines = np.concatenate([lines_BH, lines_HP, lines_BP], axis=0)
    all_lines = np.concatenate([lines_BH, lines_HP], axis=0)

    lc = Line3DCollection(all_lines, colors="k", linewidths=1, alpha=1.0)
    poly = Poly3DCollection([P], alpha=0.25, facecolor="k")
    ax.add_collection3d(lc)
    ax.add_collection3d(poly)
    # Label BH segments
    for seg in lines_BH:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$r$",
            color="k",
            ha="center",
            va="center",
            fontsize=24,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break
    # Label HP segments
    for seg in lines_HP:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$l=nr$",
            color="k",
            ha="center",
            va="center",
            fontsize=24,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break
    # ----- origins -----
    O_b = np.zeros(3)
    O_p = P.mean(axis=0)  # plate origin in world (centroid; symmetric layout -> exact)

    # ----- base circle (Z=0 plane) -----
    tt = np.linspace(0, 2 * np.pi, 360)
    base_circle = np.column_stack([rB * np.cos(tt), rB * np.sin(tt), np.zeros_like(tt)])
    ax.plot(base_circle[:, 0], base_circle[:, 1], base_circle[:, 2], color="k", linewidth=1, linestyle="--")

    # ----- Kabsch solve for plate rotation R (maps plate local -> world) -----
    # X: local plate points (3x6), zero-mean; Y: world plate points relative to O_p (3x6)
    X = SP.P.T - SP.P.mean(axis=0).reshape(3, 1)  # local plate frame (already centered in your param)
    Y = (P - O_p).T  # world points centered at O_p
    Hk = X @ Y.T
    U, S, Vt = np.linalg.svd(Hk)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # ensure proper rotation (no reflection)
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # ----- plate circle (in plate plane, radius rP) -----
    plate_local_circle = np.column_stack([rP * np.cos(tt), rP * np.sin(tt), np.zeros_like(tt)])
    plate_world_circle = (R @ plate_local_circle.T).T + O_p
    ax.plot(
        plate_world_circle[:, 0], plate_world_circle[:, 1], plate_world_circle[:, 2], color="k", linewidth=1, linestyle="--"
    )

    # ----- vectors Ob->Bk as r_B then d_B components -----
    rb_segments = []
    db_segments = []
    for k in range(6):
        u_gB = np.array([np.cos(SP.gammaB[k]), np.sin(SP.gammaB[k]), 0.0])
        u_bB = np.array([np.cos(SP.betaB[k]), np.sin(SP.betaB[k]), 0.0])
        v_rB = rB * u_gB
        v_dB = dB * u_bB
        # Ob -> Ob + rB*u_gB
        rb_segments.append(np.vstack([O_b, O_b + v_rB]))
        # (Ob + rB*u_gB) -> (Ob + rB*u_gB + dB*u_bB) == Bk
        db_segments.append(np.vstack([O_b + v_rB, O_b + v_rB + v_dB]))

    rb_lc = Line3DCollection(rb_segments, colors="k", linewidths=1, alpha=1.0)
    db_lc = Line3DCollection(db_segments, colors="k", linewidths=1, alpha=1.0)
    ax.add_collection3d(rb_lc)
    ax.add_collection3d(db_lc)
    # Add labels for r_B and d_B segments
    for seg in rb_segments:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$r_B$",
            color="k",
            ha="center",
            va="center",
            fontsize=14,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break

    for seg in db_segments:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$d_B$",
            color="k",
            ha="center",
            va="center",
            fontsize=14,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break
    # ----- vectors Op->Pk as r_P then d_P components (in plate plane) -----
    rp_segments = []
    dp_segments = []
    for k in range(6):
        # plate local unit directions, then rotate to world
        u_gP_local = np.array([np.cos(SP.gammaP[k]), np.sin(SP.gammaP[k]), 0.0])
        u_bP_local = np.array([np.cos(SP.betaP[k]), np.sin(SP.betaP[k]), 0.0])
        u_gP_world = R @ u_gP_local
        u_bP_world = R @ u_bP_local
        v_rP = rP * u_gP_world
        v_dP = dP * u_bP_world
        # Op -> Op + rP*u_gP_world
        rp_segments.append(np.vstack([O_p, O_p + v_rP]))
        # (Op + rP*u_gP_world) -> (Op + rP*u_gP_world + dP*u_bP_world) == Pk
        dp_segments.append(np.vstack([O_p + v_rP, O_p + v_rP + v_dP]))

    rp_lc = Line3DCollection(rp_segments, colors="k", linewidths=1, alpha=1.0)
    dp_lc = Line3DCollection(dp_segments, colors="k", linewidths=1, alpha=1.0)
    ax.add_collection3d(rp_lc)
    ax.add_collection3d(dp_lc)
    for seg in rp_segments:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$r_P$",
            color="k",
            ha="center",
            va="center",
            fontsize=14,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break

    for seg in dp_segments:
        mid = seg.mean(axis=0)
        ax.text(
            mid[0],
            mid[1],
            mid[2],
            r"$d_P$",
            color="k",
            ha="center",
            va="center",
            fontsize=14,
            bbox=dict(boxstyle="round", fc="w", ec="k"),
        )
        break
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
    # ax.text(O_b[0] + L * 1.05, O_b[1], O_b[2], "$B_x$", fontsize=14, color="r")
    # ax.text(O_b[0], O_b[1] + L * 1.05, O_b[2], "$B_y$", fontsize=14, color="g")
    # ax.text(O_b[0], O_b[1], O_b[2] + L * 1.05, "$B_z$", fontsize=14, color="b")
    ax.text(O_b[0], O_b[1], O_b[2] - 0.02, "$O_B$", fontsize=14, color="k")

    # Plate triad at Op (rotated by R)
    # Columns of R are the rotated basis vectors in world coords
    ex_p, ey_p, ez_p = R[:, 0], R[:, 1], R[:, 2]
    ax.quiver(
        [O_p[0]] * 3,
        [O_p[1]] * 3,
        [O_p[2]] * 3,
        [ex_p[0], ey_p[0], ez_p[0]],
        [ex_p[1], ey_p[1], ez_p[1]],
        [ex_p[2], ey_p[2], ez_p[2]],
        length=L,
        normalize=False,
        colors=["r", "g", "b"],
    )
    # optional tiny labels
    # ax.text(*(O_p + ex_p * L * 1.05), "$P_x$", fontsize=14, color="r")
    # ax.text(*(O_p + ey_p * L * 1.05), "$P_y$", fontsize=14, color="g")
    # ax.text(*(O_p + ez_p * L * 1.05), "$P_z$", fontsize=14, color="b")
    ax.text(O_p[0], O_p[1], O_p[2] - 0.02, "$O_P$", fontsize=14, color="k")
    # ----- local frames at each P_k -----
    Lp = 0.7 * L  # frame axis length at anchors
    for k, c in enumerate(P):
        exk = R @ np.array([np.cos(SP.betaP[k]), np.sin(SP.betaP[k]), 0.0])
        exk /= np.linalg.norm(exk)
        ezk = ez_p  # plate normal (unit)
        eyk = np.cross(ezk, exk)
        eyk /= np.linalg.norm(eyk)

        ax.quiver(
            [c[0]] * 3,
            [c[1]] * 3,
            [c[2]] * 3,
            [exk[0], eyk[0], ezk[0]],
            [exk[1], eyk[1], ezk[1]],
            [exk[2], eyk[2], ezk[2]],
            length=Lp,
            normalize=False,
            colors=["r", "g", "b"],
            arrow_length_ratio=0.2,
        )

        # ax.text(*(c + exk * 1.05 * Lp), f"$P_{{x,{k+1}}}$", fontsize=10, color="r")
        # ax.text(*(c + eyk * 1.05 * Lp), f"$P_{{y,{k+1}}}$", fontsize=10, color="g")
        # ax.text(*(c + ezk * 1.05 * Lp), f"$P_{{z,{k+1}}}$", fontsize=10, color="b")
        # ax.text(c[0], c[1], c[2] - 0.01, f"$P_{k+1}$", fontsize=10, color="k")

    for k in range(6):
        c = B[k]  # center at B_k
        beta = SP.betaB[k]  # base orientation about z
        alpha_k = -angles[k]  # servo arm rotation about local y
        Rz = np.array([[np.cos(beta), -np.sin(beta), 0.0], [np.sin(beta), np.cos(beta), 0.0], [0.0, 0.0, 1.0]])
        ex, ey, ez = Rz[:, 0], Rz[:, 1], Rz[:, 2]
        v = np.asarray(ex) / np.linalg.norm(ex)
        tip = np.asarray(c) + 1.05 * L * v
        x, y, z = tip
        # ax.text(
        #     x, y, z - 1e-2, f"$B_{{{k+1}}}$", color="k", ha="center", va="center", fontsize=14
        # )  # reference triad at B_k (translated global axes)
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

        # rotated triad at B_k: Rz(beta) * Ry(-alpha)
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
        v = np.asarray(ex) / np.linalg.norm(ex)
        tip = np.asarray(c) + 1.05 * L * v
        x, y, z = tip
        # ax.text(x, y, z + 1e-2, f"$M_{{{k+1}}}$", color="k", ha="center", va="center", fontsize=14)

        # show the intermediate x-axis after Rz(beta) as a dashed guide
        x_beta = Rz @ np.array([1.0, 0.0, 0.0])
        # ax.plot(
        #     [c[0], c[0] + 0.8 * Lf * x_beta[0]],
        #     [c[1], c[1] + 0.8 * Lf * x_beta[1]],
        #     [c[2], c[2] + 0.8 * Lf * x_beta[2]],
        #     "r--",
        #     linewidth=1.0,
        # )

        # --- arc for α_k in the local x–z plane of the servo (about the local y after Rz(beta)) ---
        r2 = 0.5 * L
        t2 = np.linspace(0.0, alpha_k, 64)
        pts2 = []
        for t in t2:
            ct, st = np.cos(-t), np.sin(-t)
            # Rz(beta) * Ry(-t) * e_x
            v = Rz @ np.array([ct, 0.0, st])
            pts2.append(c + r2 * v)
        pts2 = np.asarray(pts2)
        ax.plot(pts2[:, 0], pts2[:, 1], pts2[:, 2], "g-", linewidth=1.0)
        mid2 = pts2[len(t2) // 2]
        # ax.text(mid2[0], mid2[1], mid2[2], rf"$\alpha_{{{k+1}}}$", fontsize=14, color="g")

    show(ax)

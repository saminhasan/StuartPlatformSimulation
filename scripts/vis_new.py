from robot import *
import numpy as np
import open3d as o3d


# ---------- helpers ----------
def make_frame(T, size):
    m = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    m.transform(T)
    return m


def lineset_from_segments(segments, color=(0.0, 0.0, 0.0)):
    pts, lines = [], []
    for i, (p, q) in enumerate(segments):
        pts.extend([p, q])
        lines.append([2 * i, 2 * i + 1])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.asarray(pts))
    ls.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
    return ls


def lineset_from_loop(points, color=(0.0, 0.0, 0.0)):
    n = len(points)
    lines = np.column_stack([np.arange(n), (np.arange(n) + 1) % n]).astype(np.int32)
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.asarray(points))
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (n, 1)))
    return ls


def make_circle(center, ex, ey, radius, n=240, color=(0.0, 0.0, 0.0)):
    t = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    pts = center + radius * (np.outer(np.cos(t), ex) + np.outer(np.sin(t), ey))
    return lineset_from_loop(pts, color=color)


def arc_lineset(center, pts, color):
    # pts: (M,3) sequence; connect sequentially
    M = pts.shape[0]
    lines = np.column_stack([np.arange(M - 1), np.arange(1, M)]).astype(np.int32)
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (M - 1, 1)))
    return ls


def draw_thick(geoms, line_width=8.0, bg=(1, 1, 1, 1), show_skybox=False):
    """Render geoms with consistent thick lines via the modern renderer."""
    items = []
    for i, g in enumerate(geoms):
        if isinstance(g, o3d.geometry.LineSet):
            mat = o3d.visualization.rendering.MaterialRecord()
            mat.shader = "unlitLine"  # REQUIRED for line_width to work
            mat.line_width = float(line_width)
            # If your LineSet lacks per-segment colors, uncomment to force black:
            # mat.base_color = (0.0, 0.0, 0.0, 1.0)
            items.append({"name": f"line_{i}", "geometry": g, "material": mat})
        elif isinstance(g, o3d.geometry.TriangleMesh):
            # Keep default material; or make it unlit/black if you want a flat look:
            mat = o3d.visualization.rendering.MaterialRecord()
            mat.shader = "defaultUnlit"
            # mat.base_color = (0.0, 0.0, 0.0, 1.0)
            items.append({"name": f"mesh_{i}", "geometry": g, "material": mat})
        else:
            # Points / other geoms: let Open3D pick a default
            items.append({"name": f"geom_{i}", "geometry": g})

    o3d.visualization.draw(items, bg_color=bg, show_skybox=show_skybox)


if __name__ == "__main__":
    R = 0.08
    N = 0.27999999999999997 / R
    rB, dB = 0.10160254037844389, 0.030000000000000016
    rP, dP = 0.0716025403784439, 0.020000000000000004
    SP = StuartPlatform(R, N, rB, dB, rP, dP)
    angles, P = SP.move(array([0.0, 0.0, 0.01, np.deg2rad(5), np.deg2rad(-20), np.deg2rad(15)]))
    B = SP.B
    H = SP.h(angles)

    # ---------- origins ----------
    O_b = np.zeros(3)
    O_p = P.mean(axis=0)  # plate origin in world (centroid is exact for your symmetric layout)

    # ---------- recover plate rotation R (maps plate local -> world) ----------
    X = (SP.P - SP.P.mean(axis=0)).T  # 3x6 local centered
    Y = (P - O_p).T  # 3x6 world centered
    Hk = X @ Y.T
    U, S, Vt = np.linalg.svd(Hk)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # ---------- scale: keep your previous ratios ----------
    L_main = 0.3 * rP  # triad length at Ob/Op
    L_servo = 0.4 * rP  # per-servo mini triads
    r_arc = 0.75 * L_servo

    # ---------- frames at Ob and Op ----------
    Tb = np.eye(4)
    Tb[:3, 3] = O_b
    Tp = np.eye(4)
    Tp[:3, :3] = R
    Tp[:3, 3] = O_p
    cf_base = make_frame(Tb, size=L_main)
    cf_plate = make_frame(Tp, size=L_main)

    # ---------- base & plate circles ----------
    base_circle = make_circle(O_b, np.array([1, 0, 0]), np.array([0, 1, 0]), rB, color=(0.0, 0.0, 0.0))
    plate_circle = make_circle(O_p, R[:, 0], R[:, 1], rP, color=(0.0, 0.0, 0.0))

    # ---------- base & plate polygons (connect B_k and P_k)
    base_poly = lineset_from_loop(B, color=(0.0, 0.0, 0.0))
    plate_poly = lineset_from_loop(P, color=(0.0, 0.0, 0.0))

    # ---------- lines: arms (B->H), rods (H->P), reference (B->P) ----------
    segments_BH = [(B[k], H[k]) for k in range(6)]
    segments_HP = [(H[k], P[k]) for k in range(6)]
    segments_BP = [(B[k], P[k]) for k in range(6)]  # "back" reference lines
    ls_BH = lineset_from_segments(segments_BH, color=(0.0, 0.0, 0.0))
    ls_HP = lineset_from_segments(segments_HP, color=(0.0, 0.0, 0.0))
    ls_BP = lineset_from_segments(segments_BP, color=(0.0, 0.0, 0.0))

    # ---------- decomposition vectors: Ob->(rB) and ->(dB) to each Bk ----------
    rb_segments, db_segments = [], []
    for k in range(6):
        ug = np.array([np.cos(SP.gammaB[k]), np.sin(SP.gammaB[k]), 0.0])
        ub = np.array([np.cos(SP.betaB[k]), np.sin(SP.betaB[k]), 0.0])
        v_rB = rB * ug
        v_dB = dB * ub
        rb_segments.append((O_b, O_b + v_rB))
        db_segments.append((O_b + v_rB, O_b + v_rB + v_dB))  # = Bk
    ls_rB = lineset_from_segments(rb_segments, color=(0.0, 0.0, 0.0))
    ls_dB = lineset_from_segments(db_segments, color=(0.0, 0.0, 0.0))

    # ---------- decomposition vectors: Op->(rP) and ->(dP) to each Pk (in plate plane) ----------
    rp_segments, dp_segments = [], []
    for k in range(6):
        ug_local = np.array([np.cos(SP.gammaP[k]), np.sin(SP.gammaP[k]), 0.0])
        ub_local = np.array([np.cos(SP.betaP[k]), np.sin(SP.betaP[k]), 0.0])
        ug = R @ ug_local
        ub = R @ ub_local
        v_rP = rP * ug
        v_dP = dP * ub
        rp_segments.append((O_p, O_p + v_rP))
        dp_segments.append((O_p + v_rP, O_p + v_rP + v_dP))  # = Pk
    ls_rP = lineset_from_segments(rp_segments, color=(0.0, 0.0, 0.0))
    ls_dP = lineset_from_segments(dp_segments, color=(0.0, 0.0, 0.0))

    # ---------- per-servo mini triads and angle arcs at each B_k ----------
    servo_frames = []
    angle_arcs = []
    alpha_local = angles  # keep your earlier convention

    for k in range(6):
        c = B[k]
        beta = SP.betaB[k]
        a_k = alpha_local[k]

        Rz = np.array([[np.cos(beta), -np.sin(beta), 0.0], [np.sin(beta), np.cos(beta), 0.0], [0.0, 0.0, 1.0]])
        ca, sa = np.cos(-a_k), np.sin(-a_k)
        Ry = np.array([[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]])
        Rba = Rz @ Ry

        Tmini = np.eye(4)
        Tmini[:3, :3] = Rba
        Tmini[:3, 3] = c
        servo_frames.append(make_frame(Tmini, size=L_servo))

        # beta arc in XY at Bk (green)
        t1 = np.linspace(0.0, beta, 72)
        arc1_pts = np.column_stack([c[0] + r_arc * np.cos(t1), c[1] + r_arc * np.sin(t1), np.full_like(t1, c[2])])
        angle_arcs.append(arc_lineset(c, arc1_pts, color=(0.0, 0.0, 1.0)))

        # alpha arc in local x–z plane after Rz(beta) (red)
        t2 = np.linspace(0.0, a_k, 72)
        v2 = np.stack([np.cos(t2), np.zeros_like(t2), np.sin(t2)], axis=1)  # Ry(t)*ex = [cos t,0,sin t]
        arc2_pts = c + r_arc * (Rz @ v2.T).T
        angle_arcs.append(arc_lineset(c, arc2_pts, color=(0.0, 1.0, 0.0)))

    # ---------- collect & draw ----------
    geoms = [
        cf_base,
        cf_plate,
        base_circle,
        plate_circle,
        base_poly,
        plate_poly,
        ls_BH,
        ls_HP,
        ls_BP,
        ls_rB,
        ls_dB,
        ls_rP,
        ls_dP,
        *servo_frames,
        *angle_arcs,
    ]
    # o3d.visualization.draw_geometries(geoms)
    # ================================================================
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    opt = vis.get_render_option()
    opt.background_color = np.array([0.5, 0.5, 0.5])
    opt.line_width = 5.0
    for g in geoms:
        vis.add_geometry(g)

    vis.run()
    vis.destroy_window()
    # ================================================================

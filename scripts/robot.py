from numpy import *

set_printoptions(precision=6, suppress=True)


def rotX(phi):
    return array([[1, 0, 0], [0, cos(phi), -sin(phi)], [0, sin(phi), cos(phi)]])


def rotY(theta):
    return array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])


def rotZ(psi):
    return array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])


class StuartPlatform:
    def __init__(self, r, n, rB, dB, rP, dP):
        self.r = r
        self.n = n
        self.rB = rB
        self.dB = dB
        self.rP = rP
        self.dP = dP
        self.d = self.n * self.r
        self.gammaB = radians(repeat([0, 120, 240], 2))
        self.betaB = self.gammaB + tile([-pi / 2, pi / 2], 3)
        self.gammaP = self.gammaB
        self.betaP = self.betaB
        self.B = array(
            [
                rB * cos(self.gammaB) + dB * cos(self.betaB),
                rB * sin(self.gammaB) + dB * sin(self.betaB),
                zeros_like(self.gammaB),
            ]
        ).T
        self.H = array([(rotZ(beta_k) @ [self.r, 0, 0]) + B_k for beta_k, B_k in zip(self.betaB, self.B)])
        self.P = array(
            [
                rP * cos(self.gammaP) + dP * cos(self.betaP),
                rP * sin(self.gammaP) + dP * sin(self.betaP),
                zeros_like(self.gammaP),
            ]
        ).T
        self.Zhome = mean(sqrt(full(6, self.d**2) - sum((self.H - self.P) ** 2, axis=1)))
        print(self.gammaB)
        print(self.gammaP)
    def move(self, pose):
        # pose = [Tx, Ty, Tz, psi, theta, phi]
        pose[2] += self.Zhome
        R = rotZ(pose[3]) @ rotY(pose[4]) @ rotX(pose[5])
        P = pose[:3] + (R @ self.P.T).T
        L = P - self.B
        ek = 2 * self.r * L[:, 2]
        fk = 2 * self.r * (cos(self.betaB) * L[:, 0] + sin(self.betaB) * L[:, 1])
        alpha_i = arcsin((linalg.norm(L, axis=1) ** 2 - ((self.n**2 - 1) * self.r**2)) / sqrt(ek**2 + fk**2)) - arctan2(fk, ek)
        return alpha_i, P

    def h(self, alpha=zeros((6))):
        return array(
            [rotZ(beta_k) @ rotY(-alpha_k) @ [self.r, 0, 0] + B_k for alpha_k, beta_k, B_k in zip(alpha, self.betaB, self.B)]
        )


if __name__ == "__main__":
    R = 0.08
    N = 0.27999999999999997 / R
    rB, dB = 0.10160254037844389, 0.030000000000000016
    rP, dP = 0.0716025403784439, 0.020000000000000004
    robot = StuartPlatform(R, N, rB, dB, rP, dP)
    print(robot.move(array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
    # print("Base Points:\n", platform.B)
    # print("Platform Points:\n", platform.P)
    # print("Link Lengths:\n", platform.l)
    # print(rb.h())

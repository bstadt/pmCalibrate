import numpy as np
from functools32 import lru_cache
from scipy.linalg import fractional_matrix_power

def tupilize(mat):
    return tuple([tuple(elem) for elem in mat])

@lru_cache(maxsize=256)
def cacheInverse(mat):
    return np.linalg.inv(mat)

def getLogR(rotation):
    theta = np.arccos(((np.trace(rotation)-1)/2.))
    logRH = theta/(2.*np.sin(theta))*(rotation-np.transpose(rotation))
    return [logRH[2, 1], logRH[0, 2], logRH[1, 0]]

def getRotation(quaternians):
    quaternians = quaternians/np.linalg.norm(quaternians)

    qx = quaternians[0]
    qy = quaternians[1]
    qz = quaternians[2]
    qw = quaternians[3]

    return np.stack([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
                     [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
                     [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])

def axxb(Qee, QMarker):
    M = np.zeros((3, 3))
    LHS = np.empty((0, 3))
    RHS_A = []
    RHS_B = []

    for baseIdx in range(Qee.shape[0]):

        #get the current position quaternians
        E_1_Q = np.array(Qee[baseIdx][3:])
        S_1_Q = np.array(QMarker[baseIdx][3:])

        #convert quaternians to rotation matrix
        E_1_R = np.array(getRotation(E_1_Q))
        S_1_R = np.array(getRotation(S_1_Q))

        #get the base translations
        E_1_T = np.array(Qee[baseIdx][:3])
        S_1_T = np.array(QMarker[baseIdx][:3])

        #generate the honogenous matrix
        E_1 = np.vstack([np.hstack([E_1_R, E_1_T.reshape(3, 1)]),
                        [0., 0., 0., 1.]])

        S_1 = np.vstack([np.hstack([S_1_R, S_1_T.reshape(3, 1)]),
                        [0., 0., 0., 1.]])


        #for every way to pair the current arm position with
        #a target pos it has not yet been paired with
        for targetIdx in range(baseIdx+1, Qee.shape[0]):

            #get target quaternians
            E_2_Q = np.array(Qee[targetIdx][3:])
            S_2_Q = np.array(QMarker[targetIdx][3:])

            #convert quaternians to rotation matrix
            E_2_R = np.array(getRotation(E_2_Q))
            S_2_R = np.array(getRotation(S_2_Q))

            #get the target translations
            E_2_T = np.array(Qee[targetIdx][:3])
            S_2_T = np.array(QMarker[targetIdx][:3])

            #generate the homogenous matrix
            E_2 = np.vstack([np.hstack([E_2_R, E_2_T.reshape(3, 1)]),
                            [0., 0., 0., 1.]])

            S_2 = np.vstack([np.hstack([S_2_R, S_2_T.reshape(3, 1)]),
                            [0., 0., 0., 1.]])

            #get the current homogenous transforms
            HA = np.matmul(cacheInverse(tupilize(E_1)), E_2)
            HB = np.matmul(S_1, cacheInverse(tupilize(S_2)))

            #extract the rotations and translations from the transforms
            RA = HA[:3, :3]
            TA = HA[:3][:,3]
            RB = HB[:3, :3]
            TB = HB[:3][:,3]

            #build the least squares translation stack
            LHS = np.concatenate([LHS, np.identity(3)-RA], 0)
            for elem in TA:
                RHS_A.append([elem])
            RHS_B.append(TB)

            #get current alpha and beta
            alpha = np.array(getLogR(RA))
            beta = np.array(getLogR(RB))

            #iterate M
            M = M + np.outer(beta, alpha)

    #solve the rotation
    Rx = np.matmul(fractional_matrix_power(np.matmul(np.transpose(M),M), -.5),
                   np.transpose(M))

    #stack RHS_A to get it to the correct shape
    RHS_A = np.stack(RHS_A)

    #perform elementwise matrix multiplication
    #with rotation for all translations in RHS_B
    #then stack it to make it the correct shape
    Rx_RHS_B = []
    for elem in RHS_B:
        Rx_elem = np.matmul(Rx, elem)
        for subElem in Rx_elem:
            Rx_RHS_B.append([subElem])

    Rx_RHS_B = np.stack(Rx_RHS_B)

    #solve for translation
    t = np.linalg.lstsq(LHS, (RHS_A - Rx_RHS_B))[0]

    return Rx, t

if __name__ == '__main__':
    Qee = np.stack([[0.504,-0.082,0.371,0.141,0.737,-0.124,0.649],
                    [0.511,-0.018,0.371,0.094,0.744,-0.083,0.656],
                    [0.485,-0.071,0.312,0.291,0.724,-0.302,0.547],
                    [0.490,-0.013,0.312,0.247,0.740,-0.269,0.564],
                    [0.262,0.014,0.333,0.304,0.470,-0.284,0.778],
                    [0.259,0.046,0.333,0.274,0.489,-0.234,0.795],
                    [0.251,0.078,0.333,0.242,0.505,-0.184,0.808],
                    [0.201,0.038,0.320,0.302,0.418,-0.276,0.811],
                    [0.185,0.087,0.320,0.247,0.452,-0.173,0.840],
                    [0.122,0.095,0.356,0.253,0.341,-0.148,0.893]])

    QMarker = np.stack([[-0.079,0.038,0.362,0.373,0.648,-0.561,-0.355],
                        [-0.046,-0.016,0.360,0.335,0.669,-0.583,-0.315],
                        [-0.102,0.042,0.296,0.535,0.574,-0.413,-0.462],
                        [-0.098,-0.017,0.291,0.509,0.601,-0.444,-0.427],
                        [-0.099,0.046,0.388,-0.431,-0.346,0.624,0.553],
                        [-0.094,-0.016,0.396,-0.392,-0.380,0.649,0.530],
                        [-0.082,-0.077,0.401,-0.350,-0.412,0.673,0.506],
                        [-0.111,0.042,0.417,-0.406,-0.299,0.655,0.562],
                        [-0.093,-0.081,0.434,-0.321,-0.367,0.701,0.521],
                        [-0.002,-0.047,0.515,-0.259,-0.272,0.749,0.546]])

    Rx, t = axxb(Qee, QMarker)
    print Rx
    print t

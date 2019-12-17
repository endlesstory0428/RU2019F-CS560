import sys
import numpy as np
from matplotlib import pyplot as plt


class kalman2d(object):
    def __init__(self, data, x10, x20, P0, Q = None, R = None, A = None, B = None, H = None):
        super(kalman2d, self).__init__()
        self.data = np.array(data, dtype = np.float64)
        self.u = self.data[:, 0:2]
        self.z = self.data[:, 2:4]
        self.x10 = x10
        self.x20 = x20
        self.scaler = P0
        self.P0 = self.scaler * np.identity(2, dtype = np.float64)
        if Q is None:
            self.Q = np.array([[1e-4, 2e-5], [2e-5, 1e-4]], dtype = np.float64)
        if R is None:
            self.R = np.array([[1e-2, 5e-3], [5e-3, 2e-2]], dtype = np.float64)
        if A is None:
            self.A = np.identity(2, dtype = np.float64)
        if B is None:
            self.B = np.identity(2, dtype = np.float64)
        if H is None:
            self.H = np.identity(2, dtype = np.float64)

        self.X = np.empty_like(self.z, dtype = np.float64)
        self.tempX = np.array([self.x10, self.x20], dtype = np.float64)
        self.tempP = self.P0
        return

    def tUpdate(self, k):
        if k == 0:
            self.tempXm = np.dot(self.A, self.tempX)
        else:
            self.tempXm = np.dot(self.A, self.tempX) + np.dot(self.B, self.u[k])
        self.tempPm = np.dot(np.dot(self.A, self.tempP), np.transpose(self.A)) + self.Q
        return

    def mUpdate(self, k):
        factor = np.dot(self.tempPm, np.transpose(self.H))
        self.tempK = np.dot(factor, np.linalg.inv(np.dot(self.H, factor) + self.R))
        self.tempX = self.tempXm + np.dot(self.tempK, (self.z[k] - np.dot(self.H, self.tempXm)))
        self.tempP = np.dot((np.identity(2, dtype = np.float64) - np.dot(self.tempK, self.H)), self.tempPm)
        return self.tempX

    def uptade(self):
        length = self.data.shape[0]
        for k in range(length):
            self.tUpdate(k)
            self.X[k] = self.mUpdate(k)
        return

    def plot(self):
        lineZ, = plt.plot(self.z[:, 0], self.z[:, 1], 'rx-')
        lineX, = plt.plot(self.X[:, 0], self.X[:, 1], 'bo-')
        plt.legend(handles = [lineX, lineZ], labels = ['x', 'z'])
        plt.title('kalman2d result for x = (%.2f, %.2f), lambda = %.2f ' %(self.x10, self.x20, self.scaler))
        plt.show()
        return



if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 5):
        print ("Four arguments required: python kalman2d.py [datafile] [x1] [x2] [lambda]")
        exit()
    
    filename = sys.argv[1]
    x10 = float(sys.argv[2])
    x20 = float(sys.argv[3])
    scaler = float(sys.argv[4])

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    data = []
    for line in range(0, len(lines)):
        data.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print ("The input data points in the format of 'k [u1, u2, z1, z2]', are:")
    for it in range(0, len(data)):
        print (str(it + 1) + ": ", end='')
        print (*data[it])

    k2d = kalman2d(data, x10, x20, scaler)
    k2d.uptade()
    k2d.plot()
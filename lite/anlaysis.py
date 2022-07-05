import pandas as pd
import numpy as np

if __name__ == '__main__':
    rrt_path = "../build/lite/path.txt"
    rrt_data = pd.read_csv(rrt_path)
    rrt_array = np.array(rrt_data).astype('int')
    # print(rrt_array)

    uu = np.genfromtxt("../resources/uu.csv", delimiter=',', skip_header = 0)
    vv = np.genfromtxt("../resources/vv.csv", delimiter=',', skip_header = 0)
    puu = np.genfromtxt("../resources/puu.csv", delimiter=',', skip_header = 0)
    pvv = np.genfromtxt("../resources/pvv.csv", delimiter=',', skip_header = 0)

    # print(uu.shape, vv.shape)

    gt_path_uu = uu[rrt_array[:, 0], rrt_array[:, 1]]
    gt_path_vv = vv[rrt_array[:, 0], rrt_array[:, 1]]


    pred_path_uu = puu[rrt_array[:, 0], rrt_array[:, 1]]
    pred_path_vv = pvv[rrt_array[:, 0], rrt_array[:, 1]]

    UUMSE = np.square(np.subtract(gt_path_uu, pred_path_uu)).mean()
    VVMSE = np.square(np.subtract(gt_path_vv, pred_path_vv)).mean()

    # Calculating the Root Mean Squared Error (MSE) for the UU and VV values
    RMSE = np.sqrt(UUMSE + VVMSE)

    print("Root Mean Square Error: {:.4f}".format(RMSE))
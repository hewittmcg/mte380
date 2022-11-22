import matplotlib.pyplot as plt
import argparse
import csv

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filepath")
    args = parser.parse_args()
    
    with open(args.filepath, "r") as f:
        reader = csv.reader(f)
        data = []
        frontdata = []
        imudata = []
        angledata = []
        for row in reader:
            try:
                row = [int(ele) for ele in row]
                drow = [0,0,0,0,0]
                if len(row) == 4:
                    if row[0] == 1:
                        row[2] = min(row[2], 500)
                        row[3] = min(row[3], 500)
                        data.append(row)
                    elif row[0] == 2:
                        imudata.append(row)
                    elif row[0] == 0:
                        row[2] = min(row[2], 1800)
                        frontdata.append(row)
                    elif row[0] == 3:
                        angledata.append(row)
            except Exception as e:
                print(f"Exception adding row to data: {e}")    

        print(imudata)
        print(data)
        fig, axs = plt.subplots(5)
        fig.suptitle('Data subplots')
        x = [row[1] for row in imudata]
        y = [row[2] for row in imudata]
        axs[0].plot(x, y, label = "Gyro DPS")
        y = [row[3] for row in imudata]
        axs[1].plot(x, y, label = "Accel")
        x = [row[1] for row in data]
        y = [row[2] for row in data]
        axs[2].plot(x, y, label = "Front Side Tof")
        y = [row[3] for row in data]
        axs[2].plot(x, y, label = "Rear Side Tof")
        x = [row[1] for row in frontdata]
        y = [row[2] for row in frontdata]
        axs[3].plot(x, y, label = "Forward Tof")
        x = [row[1] for row in angledata]
        y = [row[2] for row in angledata]
        axs[4].plot(x, y, label = "Angle Data")
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
        axs[3].legend()
        axs[4].legend()
        plt.show()
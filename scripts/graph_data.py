import matplotlib.pyplot as plt
import argparse
import csv

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filepath")
    args = parser.parse_args()
    
    with open(args.filepath, "r") as f:
        reader = csv.reader(f)
        sidetofdata = []
        frontdata = []
        imudata = []
        angledata = []
        photodata = []
        imuturndata = []
        for row in reader:
            try:
                row = [int(ele) for ele in row]
                if len(row) == 4:
                    if row[0] == 0:
                        row[2] = min(row[2], 1800)
                        frontdata.append(row)
                    elif row[0] == 1:
                        row[2] = min(row[2], 500)
                        row[3] = min(row[3], 500)
                        sidetofdata.append(row)
                    elif row[0] == 2:
                        imudata.append(row)
                    elif row[0] == 3:
                        angledata.append(row)
                    elif row[0] == 4:
                        photodata.append(row)
                    elif row[0] == 5:
                        imuturndata.append(row)
            except Exception as e:
                print(f"Exception adding row to data: {e}")    

        print("The IMU Data is:",imudata)
        print("The Side TOF Data is:"sideotofdata)
        print("The Front TOF Data is:", frontdata)
        print("The Integrated X angle is:", angledata)
        print("The IMU Turning Data is:" imuturndata)
        print("The Photoresistor Data is:", photodata)
        
        fig, axs = plt.subplots(7)
        fig.suptitle('Data subplots')

        x = [row[1] for row in imudata]
        y = [row[2] for row in imudata]
        axs[0].plot(x, y, label = "Gyro X-Axis DPS")

        y = [row[3] for row in imudata]
        axs[1].plot(x, y, label = "Accelerometer Z")

        x = [row[1] for row in sidetofdata]
        y = [row[2] for row in sidetofdata]
        axs[2].plot(x, y, label = "Front Side Tof")
        y = [row[3] for row in sidetofdata]
        axs[2].plot(x, y, label = "Rear Side Tof")

        x = [row[1] for row in frontdata]
        y = [row[2] for row in frontdata]
        axs[3].plot(x, y, label = "Forward Tof")

        x = [row[1] for row in angledata]
        y = [row[2] for row in angledata]
        axs[4].plot(x, y, label = "Integrated Angle")

        x = [row[1] for row in imuturndata]
        y = [row[2] for row in imuturndata]
        axs[5].plot(x, y, label = "Integrated Turn Degrees")
        y = [row[3] for row in imuturndata]
        axs[5].plot(x, y, label = "Degrees Turned")

        x = [row[1] for row in photodata]
        y = [row[2] for row in photodata]
        axs[6].plot(x, y, label = "Photoresistor Voltage")

        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
        axs[3].legend()
        axs[4].legend()
        axs[5].legend()
        axs[6].legend()

        plt.show()
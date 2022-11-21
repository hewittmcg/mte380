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
        for row in reader:
            try:
                row = [int(ele) for ele in row]
                if len(row) == 4:
                    if row[0] == 1:
                        data.append(row)
            except Exception as e:
                print(f"Exception adding row to data: {e}")    

        print(data)
        x = [row[1] for row in data]
        y = [row[2] for row in data]
        plt.plot(x, y, label = "Front Side ToF")
        y = [row[3] for row in data]
        plt.plot(x, y, label = "Rear Side ToF")
        plt.legend()
        plt.show()
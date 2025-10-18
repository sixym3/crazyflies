#!/usr/bin/env python3
import glob
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def latest_csv(log_dir="logs"):
    files = sorted(glob.glob(os.path.join(log_dir, "*.csv")))
    if not files:
        print("No CSVs found in ./logs")
        sys.exit(1)
    return files[-1]

def load_logs(path):
    # Use the first line as header (matches how np.savetxt wrote the file)
    df = pd.read_csv(path, header=0)
    # If someone loaded it without header earlier and saved again, recover:
    if list(df.columns) == [str(i) for i in range(9)] and isinstance(df.iloc[0,0], str):
        new_header = df.iloc[0]
        df = df[1:]
        df.columns = new_header

    # Ensure numeric dtype
    df = df.apply(pd.to_numeric, errors='coerce')

    # Drop completely empty rows (all NaN) and all-zero rows
    df = df.dropna(how='all')
    df = df.loc[~(df.fillna(0) == 0).all(axis=1)]

    expected_cols = ["x","y","z","yaw","front","back","left","right","up"]
    # If columns are unnamed, try to assign
    if set(expected_cols).issubset(set(df.columns)) is False and df.shape[1] == 9:
        df.columns = expected_cols

    return df

def main():
    path = latest_csv("logs")
    print(f"Loading {path}")
    df = load_logs(path)
    print(df.info())
    print(df.head())

    # Position traces
    plt.figure()
    df[["x","y","z"]].plot()
    plt.title("Position (x, y, z)")
    plt.xlabel("Sample")
    plt.ylabel("Meters")
    plt.legend()
    plt.show()

    # Yaw
    plt.figure()
    df["yaw"].plot()
    plt.title("Yaw (deg)")
    plt.xlabel("Sample")
    plt.ylabel("Degrees")
    plt.show()

    # Ranges
    plt.figure()
    df[["front","back","left","right","up"]].plot()
    plt.title("Multiranger distances")
    plt.xlabel("Sample")
    plt.ylabel("mm")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()

import csv

with open("LH_FOOT_5cm_0_5Hz.csv", "r", encoding="utf-8") as file:
    reader = csv.reader(file)
    for _ in range(1):
        next(reader)
    for row in reader:
        print(row[3] * 10)

import os

if __name__ == '__main__':
    file = open("mh-dynamic", "w")
    time_interval = 20000 # 20s
    for i in range(time_interval):
        for j in range(2): # *12Mbit/s
            file.write(str(i)+"\n")
        
    for i in range(time_interval):
        for j in range(2): # *24Mbit/s
            file.write(str(i+time_interval) + "\n" + str(i+time_interval) + "\n")
import serial
import matplotlib.pyplot as plt
import numpy as np
import serial

connected = False

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

#while not connected:
#    serin = ser.read()
#    connected = True


plt.ion()                    #sets plot to animation mode

length = 500                 #determines length of data taking session (in data points)
x = [0]*length               #create empty variable of length of test
y = [0]*length
z = [0]*length

xline, = plt.plot(x)         #sets up future lines to be modified
yline, = plt.plot(y)
zline, = plt.plot(z)
plt.ylim(-180,180)        #sets the y axis limits

for i in range(length):     #while you are taking data
    # request data by sending a dot
    ser.write(".")
    data = ser.readline()    #reads until it gets a carriage return. MAKE SURE THERE IS A CARRIAGE RETURN OR IT READS FOREVER
#    print("%d: %s"%(i, data))
    sep = data.split(", ")      #splits string into a list at the tabs
    #print sep
   
    if len(sep) == 3:
        x.append(int(sep[0]))   #add new value as int to current list
        y.append(int(sep[1]))
        z.append(int(sep[2]))

        print("%d: x=%d y=%d z=%d "%(i, int(sep[0]), int(sep[1]), int(sep[2])))
  
    del x[0]
    del y[0]
    del z[0]
   
    xline.set_xdata(np.arange(len(x))) #sets xdata to new list length
    yline.set_xdata(np.arange(len(y)))
    zline.set_xdata(np.arange(len(z)))
   
    xline.set_ydata(x)                 #sets ydata to new list
    yline.set_ydata(y)
    zline.set_ydata(z)
 
    plt.pause(0.001)                   #in seconds
    plt.draw()                         #draws new plot


rows = zip(x, y, z)                  #combines lists together

row_arr = np.array(rows)               #creates array from list
np.savetxt("test_radio2.txt", row_arr) #save data in file (load w/np.loadtxt())

ser.close() #closes serial connection (very important to do this! if you have an error partway through the code, type this into the cmd line to close the connection)

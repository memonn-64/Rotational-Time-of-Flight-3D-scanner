#2DX3

import serial
import math

s = serial.Serial('COM3',115200) # change com port to match your computer 

s.open
s.reset_output_buffer()
s.reset_input_buffer()

f = open("point_array.xyz", "w") 
step = 0
x = 0 # initial x-displacement (mm)
depth = 500 # depth in the x-axis(mm)
num_inc = int(input("Enter the Number of scans:"))
count=0

while(count<num_inc):
    raw = s.readline()
    data = raw.decode("utf-8") # decodes the byte data to string format (from the UART) 
    data = data[0:-2] 

    if (data.isdigit() == True):
        angle = (step/512)*2*math.pi # Calculate angle in radians
        r = int(data)
        y = r*math.cos(angle) # calulates y
        z = r*math.sin(angle) # Calcualtes z

        print(y)
        print(z)
        
        f.write('{} {} {}\n'.format(x,y,z)) # sends data to the file in the file format .xyz   
        step = step+ 32 # increment step by 32 (512 steps in a full rotation, 32 steps per degree)


    if (step == 512): #reset steps and increment depth by 500

        step = 0
        x = x + depth # increment x by depth (500mm)
        count=count+1

    print(data)
    

f.close() #close file when done so data saves


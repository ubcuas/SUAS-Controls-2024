INPUT_FILE = r'C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\UAS-PPES\RecordedData\Dec_12_23_TestData.txt'

ORIENTATION_OUTPUT_FILE = r'C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\UAS-PPES\RecordedData\Dec_12_23_TestData_Orientation.csv'
SENSOR_OUTPUT_FILE = r'C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\UAS-PPES\RecordedData\Dec_12_23_TestData_Sensor.csv'

Sensor_output_file_header = "P_Pos, P_Vel, Altitude, Pressure, Acc_X, Acc_Y, Acc_Z, Latitude, Longitude, GPS_Altitude, Lock, Satellites" 

HEADER = "4097"

#input file
f = open(INPUT_FILE, "r")

#output file
f1 = open(ORIENTATION_OUTPUT_FILE, "w")
f2 = open(SENSOR_OUTPUT_FILE, "w")

#clear the file
f1.truncate(0)
f2.truncate(0)

#write header to sensor file
f2.write(Sensor_output_file_header + '\n')

#read input file
data = f.read()

#split data into lines
lines = data.split('\n')

#check if line begins with header, if yes then write the line into the orientation file, else write into the sensor file
for line in lines:
    #if there is a 'nan' anywhere in the line, skip it
    if 'nan' in line:
        continue
    # Search for the substring within the line
    index = line.find(HEADER)
    # Check if the substring was found
    if index != -1:
        # Write everything after "$GPGGA" to the output file
        # The index is adjusted by the length of "$GPGGA" to start writing after this substring
        f1.write(line[index:] + '\n')
    else:
        f2.write(line + '\n')

#close files
f.close()
f1.close()
f2.close()

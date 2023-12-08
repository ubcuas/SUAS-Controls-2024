#python file to remove timestamp from serial output and convert to text file

#input file location
INPUT_FILE_LOC = r"C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\icm20948_test\python\GPS_DATA\gpsData.txt"

#output file location
OUTPUT_FILE_LOC = r"C:\Users\nisch\OneDrive - UBC\UBC_UAS\2023\icm20948_test\python\GPS_DATA\Gps_out.txt"

#which conversion ype wanted:
# 1 - remove timestamp and keep all data
# 2 - remove timestamp and keep only a specifi nema data
CONV_TYPE = 1
NEMA_TYPE = "$GPGGA"

#input file
f = open(INPUT_FILE_LOC, "r")

#output file
f1 = open(OUTPUT_FILE_LOC, "w")
#clear the file
f1.truncate(0)

#read input file
data = f.read()

#split data into lines
lines = data.split('\n')

#remove timestamp from each line
print("Nmber of lines: ", len(lines))
if CONV_TYPE == 1:
    for line in lines:
        # Initialize an index variable
        index = 0
        # Iterate over the line with an index
        for index, char in enumerate(line):
            # If we encounter the '$' character, break out of the loop
            if char == '$':
                break
        # Write the substring from the '$' character to the end of the line to the output file
        # We add 1 to the index because we want to start writing from the character after '$'
        f1.write(line[index:] + '\n')

elif CONV_TYPE == 2:
    for line in lines:
        # Search for the "$GPGGA" substring within the line
        index = line.find(NEMA_TYPE)
        # Check if the substring was found
        if index != -1:
            # Write everything after "$GPGGA" to the output file
            # The index is adjusted by the length of "$GPGGA" to start writing after this substring
            f1.write(line[index:] + '\n')

#close files
f.close()
f1.close()
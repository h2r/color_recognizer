#!/usr/bin/python
import sys
counter=0
colourString = ""
redString = ""
greenString = ""
blueString = ""
counterColors = 0
with open('colorWithRGBCode.txt') as fileRead:
	for line in fileRead:
		line = line.rstrip('\n')
		counter=counter+1

		print counter
		if counter%4 == 1:
			colourString += "\"" + line +  "\"," 
			counterColors +=1
		elif counter%4 ==2:
			redString +=  str(int(line[2:4],16)) +  ", " 
			greenString +=  str(int(line[4:6],16)) +  ", " 
			blueString +=  str(int(line[6:8],16)) +  ", " 
			print counterColors

with open('coloursRGBDecCode.txt', 'w') as fileWrite:
	fileWrite.write(colourString)
	fileWrite.write("\n colour red \n")
	fileWrite.write(redString)
	fileWrite.write("\n red green \n")
	fileWrite.write(greenString)
	fileWrite.write("\n green blue \n")
	fileWrite.write(blueString)

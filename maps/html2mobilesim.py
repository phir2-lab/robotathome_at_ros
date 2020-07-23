#!/usr/bin/python

import sys
import os

import numpy as np
import math

def getBigTranslation(lines):
    # find mins and maxs
	minX = min([min([x[0] for x in lines]),min([x[2] for x in lines])])
	maxX = max([max([x[0] for x in lines]),max([x[2] for x in lines])])
	minY = min([min([x[1] for x in lines]),min([x[3] for x in lines])])
	maxY = max([max([x[1] for x in lines]),max([x[3] for x in lines])])

	diff_x = abs(minX - maxX)
	diff_y = abs(minY - maxY)

	if(diff_x > diff_y):
		return diff_x/2
	else:
		return diff_y/2

def getLimits(lines):
    # find mins and maxs
    minX = min([min([x[0] for x in lines]),min([x[2] for x in lines])])
    maxX = max([max([x[0] for x in lines]),max([x[2] for x in lines])])
    minY = min([min([x[1] for x in lines]),min([x[3] for x in lines])])
    maxY = max([max([x[1] for x in lines]),max([x[3] for x in lines])])
    return minX,maxX,minY,maxY

def scaleTranslateLines(lines, minX, minY, scale):
	output = []
	for l in lines:
		l[0] = (l[0]+minX)*scale
		l[1] = (l[1]+minY)*scale
		l[2] = (l[2]+minX)*scale
		l[3] = (l[3]+minY)*scale
		l = [int(i) for i in l]
		output.append(l)
	return output

def scaleTranslateDoors(lines, minX, minY, scale):
	output = []
	for l in lines:  
		l[0] = (l[0]+minX)*scale
		l[1] = (l[1]+minY)*scale 
		output.append(l)
	return output

def generateDotMapFile(lines, outputName, scale, minX,maxX,minY,maxY):
	lines = scaleTranslateLines(lines, -minX, -minY, scale)

	minX,maxX,minY,maxY = getLimits(lines)
	print minX,maxX,minY,maxY

	print outputName
	f = open(outputName, 'w')
	f.write("2D-Map\n")
	f.write("Resolution: 0\n")
	f.write("LineMinPos: %d %d\n" % (minX, minY) )
	f.write("LineMaxPos: %d %d\n" % (maxX, maxY) )
	f.write("NumLines: %d\n" % len(lines) )
	f.write("LinesAreSorted: false\n")
	f.write("Cairn: RobotHome 4000 4000 0.000000 \"\" ICON \"\"\n")
	f.write("LINES\n")
	for l in lines:
		f.write("%d %d %d %d\n" % (l[0],l[1],l[2],l[3]) )
	f.write("DATA\n")

def extractMapLines(lines):
	t_matrix = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	saved_matrix = t_matrix

	map_lines = []

	origin = []

	last_origin = []

	first = True

	for line in lines:
		if (line[0].strip() == "ctx.save"):
			saved_matrix = t_matrix

		if (line[0].strip() == "ctx.restore"):
			t_matrix = saved_matrix

		if (line[0].strip() == "ctx.transform"):
			v = line[1].split(")")[0].split(",")
			t_matrix = np.matrix([[float(v[0].strip()), float(v[2].strip()), float(v[4].strip())],
								  [float(v[1].strip()), float(v[3].strip()), float(v[5].strip())],
								  [0                  ,                   0, 1                  ]])

		if (line[0].strip() == "ctx.moveTo"):
#			if not first:
#				map_lines.append([float(origin[0]), -float(origin[1]), float(last_origin[0]), -float(last_origin[1])])
			v = line[1].split(")")[0].split(",")
			origin = np.matrix([[float(v[0].strip())], [float(v[1].strip())],[1]])		
			origin = np.matmul(t_matrix, origin)
			last_origin = origin
			first = False

		if (line[0].strip() == "ctx.lineTo"):
			v = line[1].split(")")[0].split(",")
			dest = np.matrix([[float(v[0].strip())], [float(v[1].strip())],[1]])		
			dest = np.matmul(t_matrix, dest)
			map_lines.append([float(origin[0]), -float(origin[1]), float(dest[0]), -float(dest[1])])
			origin = dest;

	#if not first:
	#	map_lines.append([float(origin[0]), -float(origin[1]), float(last_origin[0]), -float(last_origin[1])])

	return map_lines

def generateDoorsFile(doors, outputName, scale, minX,maxX,minY,maxY):

	doors = scaleTranslateDoors(doors, -minX, -minY, scale/1000)

	doors.sort(key=lambda x: x[2])

	print outputName
	f = open(outputName, 'w')

	for l in doors:
		f.write("%f %f %d %d\n" % (l[0],l[1],l[2],l[3]) )

def extractMapDoors(lines):
	t_matrix = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	saved_matrix = t_matrix

	map_doors = []

	for line in lines:
		if (line[0].strip() == "ctx.save"):
			saved_matrix = t_matrix

		if (line[0].strip() == "ctx.restore"):
			t_matrix = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

		if (line[0].strip() == "ctx.transform"):
			v = line[1].split(")")[0].split(",")
			t_matrix = np.matrix([[float(v[0].strip()), float(v[2].strip()), float(v[4].strip())],
								  [float(v[1].strip()), float(v[3].strip()), float(v[5].strip())],
								  [0                  ,                   0, 1                  ]])

		if (line[0].strip() == "ctx.fillText"):
			v = line[1].split(")")[0].split(",")
			pos = np.matrix([[float(v[1].strip())], [float(v[2].strip())], [1]])		
			pos = np.matmul(t_matrix, pos)

			fake_point1 = np.matrix([[0], [0], [1]])
			fake_point2 = np.matrix([[1], [0], [1]])

			fake_point1 = np.matmul(t_matrix, fake_point1)
			fake_point2 = np.matmul(t_matrix, fake_point2)	

			th = np.arctan2([fake_point2[1] - fake_point1[1]], [fake_point1[0] - fake_point2[0]])*180/math.pi 

			th -= 90

			map_doors.append([float(pos[0]), -float(pos[1]), int(v[0].strip().replace("\"", "")), th])

	return map_doors

def main(inputFile, scale):
	print 'InputFile:', inputFile

	with open(inputFile) as f:
		lines = f.readlines()

	lineCommands = []
	doorCommands = []

	for line in lines:
		command = line.split("(")
		if (command[0].strip() == "ctx.restore") or (command[0].strip() == "ctx.save") or (command[0].strip() == "ctx.transform"):
			doorCommands.append(command)
			lineCommands.append(command)
		if (command[0].strip() == "ctx.lineTo") or (command[0].strip() == "ctx.moveTo"):
			lineCommands.append(command)
		if (command[0].strip() == "ctx.fillText"):
			doorCommands.append(command)

	mapLines = extractMapLines(lineCommands)

#	mapDoors = extractMapDoors(doorCommands)

	filename, file_extension = os.path.splitext(inputFile)

	minX,maxX,minY,maxY = getLimits(mapLines)

	generateDotMapFile(mapLines, filename+".map", float(scale), minX,maxX,minY,maxY)

#	generateDoorsFile(mapDoors, filename+".doors", float(scale), minX,maxX,minY,maxY)
	

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "USAGE: python html2mobilesim.py INPUTFILE.html scale"
    else:
        main(sys.argv[1], sys.argv[2])

#!/usr/bin/python

import sys
import os
import shutil
import os.path as path
import open3d as o3d

# Check if input is right
if len(sys.argv) != 2:
    print("Not good number of arguments. Please just send the directory.")
else:
    # Get directory
    directory = sys.argv[1]
    # Create directory if it does not exists
    plyFilesDirectory = path.join(directory, "plyFiles")
    if not path.isdir(plyFilesDirectory):
        os.mkdir(plyFilesDirectory)

    # Check if directory exists
    if path.isdir(directory):
        # Read through input directory
        for filename in os.listdir(directory):
            # Check if file is .ply
            name = path.splitext(filename)[0]
            extension = path.splitext(filename)[1]

            if extension == ".ply":
                # Get input and output file names
                inputFile = path.join(directory, filename)
                outputFileName = name + ".pcd"
                outputFile = path.join(directory, outputFileName)
                movePlyOutputDir = path.join(plyFilesDirectory, filename)

                # Read file
                pcd = o3d.io.read_point_cloud(inputFile)
                # Save file
                print("Saving PLY {} to PCD {}".format(inputFile, outputFile))
                o3d.io.write_point_cloud(outputFile, pcd)
                
                # Move files 
                shutil.move(inputFile, movePlyOutputDir)
            else:
                print("Ignoring file {} as it is not ply".format(filename))

    else:
        print("Argument is not a directory")
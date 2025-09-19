#2DX3

import numpy as np
import open3d as o3d

if __name__ == "__main__":
    scans = int(input("How many scans did you take:"))
    spins = 16

    #Read data from the file made       
    pcd = o3d.io.read_point_cloud("point_array.xyz", format="xyz")

    #Print numerical point cloud data        
    print(np.asarray(pcd.points))

    #Print graphical point cloud data       
    o3d.visualization.draw_geometries([pcd])
    
    # give each vertex a different number
    yz_slice_vertex = []
    for x in range(0,scans*spins):
        yz_slice_vertex.append([x])

    #coordinates defined that connect the points in the yz plane 
    # this is the yz plane because the x axis is the depth axis (x-axis)
    # and the y and z axis are the coordinates of the point cloud data        
    lines = []  
    for x in range(0,scans*spins,spins): # x is the index of the yz slice
        #This loop connects the points in the yz plane (x is the index of the yz slice)
        for i in range(spins):
            if i==spins-1:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x]])
            else:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+1]])
            


    #This loop connects the points in the yz plane (x is the index of the yz slice)
    # This is the yz plane because the x axis is the depth axis (x-axis)       
    for x in range(0,scans*spins-spins-1,spins):
        for i in range(spins):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+spins]])

    #Mapping the 3d coordinate 
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #getting 3d visualization      
    o3d.visualization.draw_geometries([line_set])
                                    
    
 


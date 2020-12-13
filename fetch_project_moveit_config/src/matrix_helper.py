import numpy as np

#returns 4x4 rotation matrix from roll pitch and yaw
def get_rotation_matrix(rpy):
    x = rpy[2]
    y = rpy[1]
    z = rpy[0]

    matrix = np.matrix([
        [np.cos(z)*np.cos(y), (np.cos(z)*np.sin(y)*np.sin(x)) - (np.sin(z)*np.cos(x)), (np.cos(z)*np.sin(y)*np.cos(x)) + (np.sin(z)*np.sin(x)), 0],
        [np.sin(z)*np.cos(y), (np.sin(z)*np.sin(y)*np.sin(x)) + (np.cos(z)*np.cos(x)), (np.sin(z)*np.sin(y)*np.cos(x)) - (np.cos(z)*np.sin(x)), 0],
        [-np.sin(y), np.cos(y)*np.sin(x), np.cos(y)*np.cos(x), 0],
        [0, 0, 0, 1]])
    
    return matrix

#returns 4x4 translation matrix from x,y,z coordinate 
def get_translation_matrix(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    matrix = np.matrix([
        [1,0,0,x], 
        [0,1,0,y], 
        [0,0,1,z], 
        [0,0,0,1]])

    return matrix

#takes a translation and rotation matrix and takes the dot product
def get_world_transform_matrix(rotation, translation):
    return np.dot(translation, rotation)
    #return np.dot(rotation, translation)


#returns a point that is transformed from matrixA global to matrixB local coordinates
def global_to_local(matrixA, matrixB):
    inverseB = np.linalg.inv(matrixB)
    combine = np.dot(inverseB, matrixA)
    #combine = np.dot(matrixA,inverseB)
    
    empty = np.matrix([[0],[0],[0],[1]])
    point = np.dot(combine, empty)
    point = point[:-1].transpose()
    return np.array(point)[0].tolist()

 

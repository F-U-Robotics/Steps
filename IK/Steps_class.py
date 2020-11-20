# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 10:42:43 2020

@author: liorfa
"""
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits import mplot3d

########################## Moshe remarks ###########################

# legs defined by index in the following order: LF, RF, LR, RR
# Please create a new index by inserting two letter (e.g. ii not i)

####################################################################
T_DR = np.pi / 180


# T_RS=500/np.pi

# Defining the spot class
class Steps:
    def __init__(self, Shin_L, Thigh_L, Hip_L, Body_W, Body_L):
       

        # step size for iterative inverse kinematic calculation
        self.S_inc = 0.01  # defined in meters

        # Initialized position for  the body
        self.pos = np.array([0, 0, 0])
        

        # these are the linkage length of the robot, in general they should be fixed
        self.S_L = Shin_L
        self.T_L = Thigh_L
        self.H_L = Hip_L
        self.B_W = Body_W
        self.B_L = Body_L
        
        #The following array contains the tarnslation needed to construct the homogeneous transformation
        #Body Shoulder translation
        self.Tr01=np.array([[self.B_L / 2 ,self.B_W / 2 ,0],
                            [self.B_L / 2 ,-self.B_W / 2,0],
                            [-self.B_L / 2,self.B_W / 2 ,0],
                            [-self.B_L / 2,-self.B_W / 2,0]])
        #Shoulder to hip translation
        self.Tr12=np.array([[0,self.H_L ,0],
                            [0,-self.H_L,0],
                            [0,self.H_L ,0],
                            [0,-self.H_L,0]])
        #Hip to knee translation
        self.Tr23=np.array([[0,0,-self.T_L],
                            [0,0,-self.T_L],
                            [0,0,-self.T_L],
                            [0,0,-self.T_L]])
        
        self.Temp = []
        
        #The inverse jacobian
        self.inv_J = np.zeros((4,3,3))
        #The foot positions
        self.F_pos = np.zeros((4,4))
        self.K_pos = np.zeros((4,4))
        self.S_pos = np.zeros((4,4))

        self.Joint_Ang = np.array([[0, 10, -10], #order of angels is shoulder, hip,knee and each row represents a leg
                                   [0, 10, -10],
                                   [0, 10, -10],
                                   [0, 10, -10]]) * T_DR
        
        #Initialization of the foot positions in the body coardinate system
        for ii in range(4):
            self.Forw_kin(ii)

       
        
        
    def T01_fun(self,ii):
        T01 = np.array([[1, 0                           , 0                           , self.Tr01[ii,0]], 
                        [0, np.cos(self.Joint_Ang[ii,0]),-np.sin(self.Joint_Ang[ii,0]), self.Tr01[ii,1]],
                        [0, np.sin(self.Joint_Ang[ii,0]), np.cos(self.Joint_Ang[ii,0]), self.Tr01[ii,2]],
                        [0, 0                           , 0                           , 1]])
        
        DT01 = np.array([[0, 0                            , 0                            , 0],
                         [0, -np.sin(self.Joint_Ang[ii,0]), -np.cos(self.Joint_Ang[ii,0]), 0],
                         [0, np.cos(self.Joint_Ang[ii,0]) , -np.sin(self.Joint_Ang[ii,0]), 0], 
                         [0, 0                            , 0                            , 0]])
        return T01, DT01
    
    def T12_fun(self,ii):
        T12 = np.array([[np.cos(self.Joint_Ang[ii,1]), 0, np.sin(self.Joint_Ang[ii,1]),self.Tr12[ii,0]],
                        [0                           , 1, 0                           ,self.Tr12[ii,1]],
                        [-np.sin(self.Joint_Ang[ii,1]),0, np.cos(self.Joint_Ang[ii,1]),self.Tr12[ii,2]],
                        [0                           , 0, 0                           , 1]])
        
        DT12 = np.array([[-np.sin(self.Joint_Ang[ii,1]), 0, np.cos(self.Joint_Ang[ii,1]), 0],
                         [0                            , 0, 0                           , 0],
                         [-np.cos(self.Joint_Ang[ii,1]), 0, -np.sin(self.Joint_Ang[ii,1]),0],
                         [0                            , 0, 0                            ,0]])
        return T12,DT12

    def T23_fun(self,ii):
        T23 = np.array([[np.cos(self.Joint_Ang[ii,2]) , 0, np.sin(self.Joint_Ang[ii,2]), self.Tr23[ii,0]],
                        [0                            , 1, 0                           , self.Tr23[ii,1]],
                        [-np.sin(self.Joint_Ang[ii,2]), 0, np.cos(self.Joint_Ang[ii,2]), self.Tr23[ii,2]],
                        [0                            , 0, 0                           , 1]])
        
        DT23 = np.array([[-np.sin(self.Joint_Ang[ii,2]), 0, np.cos(self.Joint_Ang[ii,2]) , 0],
                         [0                            , 0, 0                            , 0],
                         [-np.cos(self.Joint_Ang[ii,2]), 0, -np.sin(self.Joint_Ang[ii,2]), 0], 
                         [0                            , 0, 0                            , 0]])
        return T23, DT23
                        


  
    def Forw_kin(self,ii):
        
        #Computation of the transformation
        T01,DT01=self.T01_fun(ii)
        T12,DT12=self.T12_fun(ii)
        T23,DT23=self.T23_fun(ii)
        
        #comutation of the Jackobian and its inverse
        J1 = np.matmul(DT01, np.matmul(T12, np.matmul(T23, np.array([0, 0, -self.S_L, 1]))))
        J2 = np.matmul(T01, np.matmul(DT12, np.matmul(T23, np.array([0, 0, -self.S_L, 1]))))
        J3 = np.matmul(T01, np.matmul(T12, np.matmul(DT23, np.array([0, 0, -self.S_L, 1]))))
        J = np.array([J1, J2, J3])
        J = np.transpose(J)
        J = J[0:3, 0:3]
        self.inv_J[ii]=np.linalg.inv(J)
        self.S_pos[ii]=np.matmul(T01, np.matmul(T12, np.array([0, 0, 0, 1])))
        self.K_pos[ii]=np.matmul(T01, np.matmul(T12, np.matmul(T23, np.array([0, 0, 0, 1]))))
        self.F_pos[ii]=np.matmul(T01, np.matmul(T12, np.matmul(T23, np.array([0, 0, -self.S_L, 1]))))
        

      


    def Inv_kin(self, ii, DX):
        
        XF = np.array([self.F_pos[ii,0] + DX[0], self.F_pos[ii, 1] + DX[1], self.F_pos[ii,2] + DX[2]])
        S = np.sqrt(DX.dot(DX))
        step_num = int(S / self.S_inc) + 1
        dx = DX / step_num;
        for i in range(0, step_num, 1):
                X = np.array([self.F_pos[ii,0], self.F_pos[ii,1], self.F_pos[ii,2]])
                dx = (XF - X) / (step_num - i)
                # print(dx)
                DT = np.matmul(self.inv_J[ii], dx)
                # print(DT)
                self.Joint_Ang[ii, 2] += DT[2]
                self.Joint_Ang[ii,1] += DT[1]
                self.Joint_Ang[ii,0] += DT[0]
                self.Forw_kin(ii)
        
        

def Att_corr(A,Pitch,Roll):
    if abs(Roll)>0.06:
        DR=np.array([0,0,np.sin(Roll)*A.B_W/2])
    else:
        DR=np.array([0,0,0])
    if abs(Pitch)>0.06:
        DP=np.array([0,0,np.sin(Pitch)*A.B_L/2])
    else:
        DP=np.array([0,0,0])
    A.Inv_kin(0,DR+DP)
    A.Inv_kin(1,-DR+DP)
    A.Inv_kin(2,DR-DP)
    A.Inv_kin(3,-DR-DP)


# # Define the suport function for plotting the robots pos
def plot_steps(A):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    body_x = np.array([A.B_L / 2, A.B_L / 2, -A.B_L / 2, -A.B_L / 2, A.B_L / 2])
    body_y = np.array([A.B_W / 2, -A.B_W / 2, -A.B_W / 2, A.B_W / 2, A.B_W / 2])
    body_z = np.array([0, 0, 0, 0, 0])
    ax.plot3D(body_x, body_y, body_z, 'gray')
    # LF
    LF_S_x = np.array([A.B_L / 2, A.S_pos[0,0]])
    LF_S_y = np.array([A.B_W / 2, A.S_pos[0,1]])
    LF_S_z = np.array([0, A.S_pos[0,2]])
    ax.plot3D(LF_S_x, LF_S_y, LF_S_z, 'green')
    LF_H_x = np.array([A.S_pos[0,0], A.K_pos[0,0]])
    LF_H_y = np.array([A.S_pos[0,1], A.K_pos[0,1]])
    LF_H_z = np.array([A.S_pos[0,2], A.K_pos[0,2]])
    ax.plot3D(LF_H_x, LF_H_y, LF_H_z, 'red')
    LF_F_x = np.array([A.K_pos[0,0], A.F_pos[0,0]])
    LF_F_y = np.array([A.K_pos[0,1], A.F_pos[0,1]])
    LF_F_z = np.array([A.K_pos[0,2], A.F_pos[0,2]])
    ax.plot3D(LF_F_x, LF_F_y, LF_F_z, 'blue')
    # RF
    RF_S_x = np.array([A.B_L / 2, A.S_pos[1,0]])
    RF_S_y = np.array([-A.B_W / 2, A.S_pos[1,1]])
    RF_S_z = np.array([0, A.S_pos[1,2]])
    ax.plot3D(RF_S_x, RF_S_y, RF_S_z, 'green')
    RF_H_x = np.array([A.S_pos[1,0], A.K_pos[1,0]])
    RF_H_y = np.array([A.S_pos[1,1], A.K_pos[1,1]])
    RF_H_z = np.array([A.S_pos[1,2], A.K_pos[1,2]])
    ax.plot3D(RF_H_x, RF_H_y, RF_H_z, 'red')
    RF_F_x = np.array([A.K_pos[1,0], A.F_pos[1,0]])
    RF_F_y = np.array([A.K_pos[1,1], A.F_pos[1,1]])
    RF_F_z = np.array([A.K_pos[1,2], A.F_pos[1,2]])
    ax.plot3D(RF_F_x, RF_F_y, RF_F_z, 'blue')
    # LR
    LR_S_x = np.array([-A.B_L / 2, A.S_pos[2,0]])
    LR_S_y = np.array([A.B_W / 2, A.S_pos[2,1]])
    LR_S_z = np.array([0, A.S_pos[2,2]])
    ax.plot3D(LR_S_x, LR_S_y, LR_S_z, 'green')
    LR_H_x = np.array([A.S_pos[2,0], A.K_pos[2,0]])
    LR_H_y = np.array([A.S_pos[2,1], A.K_pos[2,1]])
    LR_H_z = np.array([A.S_pos[2,2], A.K_pos[2,2]])
    ax.plot3D(LR_H_x, LR_H_y, LR_H_z, 'red')
    LR_F_x = np.array([A.K_pos[2,0], A.F_pos[2,0]])
    LR_F_y = np.array([A.K_pos[2,1], A.F_pos[2,1]])
    LR_F_z = np.array([A.K_pos[2,2], A.F_pos[2,2]])
    ax.plot3D(LR_F_x, LR_F_y, LR_F_z, 'blue')
    # RR
    RR_S_x = np.array([-A.B_L / 2, A.S_pos[3,0]])
    RR_S_y = np.array([-A.B_W / 2, A.S_pos[3,1]])
    RR_S_z = np.array([0, A.S_pos[3,2]])
    ax.plot3D(RR_S_x, RR_S_y, RR_S_z, 'green')
    RR_H_x = np.array([A.S_pos[3,0], A.K_pos[3,0]])
    RR_H_y = np.array([A.S_pos[3,1], A.K_pos[3,1]])
    RR_H_z = np.array([A.S_pos[3,2], A.K_pos[3,2]])
    ax.plot3D(RR_H_x, RR_H_y, RR_H_z, 'red')
    RR_F_x = np.array([A.K_pos[3,0], A.F_pos[3,0]])
    RR_F_y = np.array([A.K_pos[3,1], A.F_pos[3,1]])
    RR_F_z = np.array([A.K_pos[3,2], A.F_pos[3,2]])
    ax.plot3D(RR_F_x, RR_F_y, RR_F_z, 'blue')
    
    ax.view_init(45, -45)


# Initializing the spot class
A = Steps(0.13, 0.105, 0.05, 0.08, 0.222)

plot_steps(A)

Att_corr(A,0.1,0.1)

plot_steps(A)
    
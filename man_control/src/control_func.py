#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 16 17:57:27 2020

@author: Devdutt Subhasish

Some common functions which will be used across multiple files, classes and files.
Also loads up the mass-inertia and kinematic parameters from the URDF.
Should be imported individually in every python file.
"""

import numpy as np#numpy
from numba import jit
import rospy#for interfacing with ROS
from urdf_parser_py.urdf import URDF#to use URDF from ROS Parameter Server
from tf.transformations import *#to convert orientations between various conventions
import pickle
# import time
from typing import Tuple

# some functions which will be used frequently
sin=np.sin
cos=np.cos
pi=np.pi
inv=np.linalg.inv#inverse
pinv=np.linalg.pinv

@jit(nopython=True,cache=True)
def tilda(p:np.ndarray)->np.ndarray:
    """Returns the 3x3 skew-symmetric matix corresponding to the cross-product
    operation for a given 3x1 vector.

    Parameters
    ----------
    p : list or numpy.ndarray of shape (3,1)
         3x1 input vector

    Returns
    -------
    til : numpy.ndarray of shape (3,3)
         3x3 skew-symmetric matix corresponding to the cross-product operation

    """
    
    til=np.array([[0,-p[2,0],p[1,0]],[p[2,0],0,-p[0,0]],[-p[1,0],p[0,0],0]])
    return til

@jit(nopython=True,cache=True)
def inv_tilda(til:np.ndarray)->np.ndarray:
    """Inverse function of the tilda function

    Parameters
    ----------
    til : numpy.ndarray of shape (3,3)
         3x3 skew-symmetric matix corresponding to the cross-product operation

    Returns
    -------
    p : numpy.ndarray of shape (3,1)
         3x1 vector corresponding to the input skew-symmetric matrix

    """
    
    p=np.zeros((3,1),dtype=np.float64)
    p[0]=-til[1,2]
    p[1]=til[0,2]
    p[2]=-til[0,1]
    return p

def tilda6(P:np.ndarray)->np.ndarray:
    """Tilda operation used in Spatial Vector Algebra.

    Parameters
    ---------- 
    P : list or numpy.ndarray of shape (6,)
         Input spatial vector

    Returns
    -------
    til : numpy.ndarray of shape (6,6)
         6x6 matrix corresponding to tilda operation on P
    """
    
    x=P[0:3]
    y=P[3:6]
    x_tilda=tilda(x)
    y_tilda=tilda(y)
    til=np.zeros((6,6),dtype=np.float64)
    til[0:3,0:3]=x_tilda
    til[3:6,3:6]=x_tilda
    til[3:6,0:3]=y_tilda
    return til

@jit(nopython=True,cache=True)
def Rx(theta:np.float64)->np.ndarray:
    """Returns rotation matrix for a rotation about x axis by theta radians

    Parameters
    ----------
    theta : float
         angle in radians 

    Returns
    -------
    R : numpy.ndarray of shape (3,3)
         3x3 Rotation Matrix

    """
    
    R=np.array([[1.0,0.0,0.0],
                [0.0,np.cos(theta),-np.sin(theta)],
                [0.0,np.sin(theta),np.cos(theta)]])
    return R

@jit(nopython=True,cache=True)
def Ry(theta:float)->np.ndarray:
    """Returns rotation matrix for a rotation about y axis by theta radians

    Parameters
    ----------
    theta : float
         angle in radians 

    Returns
    -------
    R : numpy.ndarray of shape (3,3)
         Rotation Matrix

    """
    
    R=np.array([[np.cos(theta),0.0,np.sin(theta)],
                [0.0,1.0,0.0],
                [-np.sin(theta),0.0,np.cos(theta)]])
    return R

@jit(nopython=True,cache=True)
def Rz(theta:float)->np.ndarray:
    """Returns rotation matrix for a rotation about z axis by theta radians

    Parameters
    ----------
    theta : float
         angle in radians 

    Returns
    -------
    R : numpy.ndarray of shape (3,3)
         Rotation Matrix

    """
    
    R=np.array([[np.cos(theta),-np.sin(theta),0.0],
                [np.sin(theta),np.cos(theta),0.0],
                [0.0,0.0,1.0]])
    return R

@jit(nopython=True,cache=True)
def phi_link(d:float,a:float,alpha:float)->np.ndarray:
    """Returns Rigid body transformation matrix corresponding to DH parameters d,a and alpha

    Parameters
    ----------
    d : float
          DH parameter d (joint offset in metres)
    a : float
          DH parameter a (link length in metres)
    alpha : float
          DH parameter alpha (link twist in radians)

    Returns
    -------
    phi_l : numpy.ndarray of shape (6,6)
          Rigid Body Transformation Matrix

    """
    
    R=Rx(alpha)
    l_tilda=tilda(np.array([[a,-d*np.sin(alpha),d*np.cos(alpha)]]).T)
    phi_l=np.zeros((6,6))
    phi_l[0:3,0:3]=phi_l[3:6,3:6]=R
    phi_l[0:3,3:6]=l_tilda @ R
    
    return phi_l

@jit(nopython=True,cache=True)
def phi_hinge(theta:float)->np.ndarray:
    """Returns Rigid body transformation matrix corresponding to DH parameter theta

    Parameters
    ----------
    theta : float
          DH parameter theta (joint angle in radians)

    Returns
    -------
    phi_h : numpy.ndarray of shape (6,6)
          Rigid Body Transformation Matrix

    """
    phi_h=np.zeros((6,6))
    phi_h[0:3,0:3]=phi_h[3:6,3:6]=Rz(theta)
    return phi_h

@jit(nopython=True,cache=True)
def get_phi(alpha:np.ndarray,a:np.ndarray,theta0:np.ndarray,theta:np.ndarray,d:np.ndarray,a0:float,alpha0:float)->np.ndarray:
    """Returns a 3D arrays of rigid body transformation matrixes for all 6 joints in the Manipulator

    Parameters
    ----------
    alpha : numpy.ndarray of shape (6,1)
          array of DH parameter alpha for all 6 joints
    a : numpy.ndarray of shape (6,1)
          array of DH parameter a for all 6 joints
    theta0 : numpy.ndarray of shape (6,1)
          array of DH parameter theta_0 which is the initial joint angle at home position
    theta : numpy.ndarray of shape (6,1)
          array of DH parameter theta which is to be added to initial joint angle to get total joint angle.
    d : numpy.ndarray of shape (6,1)
          array of DH parameter d for all 6 joints
    a0 : float
          base link length
    alpha0 : float
          base link twist

    Returns
    -------
    phis : numpy.ndarray of shape (6,6,6)
          array of Rigid body transformation matrices. phis[0] to phis[5] are the rigid body transformation matrices
        phi_i-1_i for i=1 to 6. 
    phis_l : numpy.ndarray of shape (6,6,6)
          array of link Rigid body transformation matrices. phis[0] to phis[5] are the rigid body transformation matrices
        phi_i-1_i for i=1 to 6.
    phis_h : numpy.ndarray of shape (6,6,6)
          array of hinge Rigid body transformation matrices. phis[0] to phis[5] are the rigid body transformation matrices
        phi_i-1_i for i=1 to 6.
    """

    phis_l=np.zeros((6,6,6))
    phis_h=np.zeros((6,6,6))
    phis=np.zeros((6,6,6))
    for i in range(6):
        if i==0 :
            phis_l[i]=phi_link(d[i,0],a0,alpha0)
        else:
            phis_l[i]=phi_link(d[i,0],a[i-1,0],alpha[i-1,0])
        phis_h[i,:,:]=phi_hinge(theta[i,0]+theta0[i,0])
        phis[i]=phis_l[i] @ phis_h[i]
    return phis,phis_l,phis_h

@jit(nopython=True,cache=True)
def get_cummulative_phi(phis:np.ndarray)->np.ndarray:
    """Returns a array of cummulative rigid body transformation matrices.

    Parameters
    ----------
    phis : numpy.ndarray of shape (6,6,6)
          array of 6 rigid body transformation matrices phi_i-1_i for i=1 to 6

    Returns
    -------
    cphis : numpy.ndarray of shape (6,6,6)
          array of Cummulative Rigid body transformation matrices. cphis[0] to cphis[5] are the cummulative rigid body transformation matrices
        phi_i-1_6 for i=1 to 6. 

    """
    
    cphis=np.zeros((6,6,6))
    cphis[5,:,:]=phis[5,:,:]
    for i in [4,3,2,1,0]:
        cphis[i]=phis[i]@cphis[i+1]
        
    return cphis

@jit(nopython=True,cache=True)
def pos_from_phi(phi:np.ndarray)->np.ndarray:
    """returns the position coordinates from a rigid body transformation matrix

    Parameters
    ----------
    phi : numpy.ndarray of shape (6,6)
         rigid body transformation matrix

    Returns
    -------
    pos : numpy.ndarray
         numpy array of shape (3,1) which is the position coordinates from phi

    """
    
    pos=inv_tilda(phi[0:3,3:6] @ phi[0:3,0:3].T)
    return pos

def create_IM(link)->Tuple[float,np.ndarray,np.ndarray]:
    """Gets the mass-inertia properties of the link from the Robot Description (URDF) loaded in the ROS parameter server
    returns a tuple of the form (m,COM,I)
    where m is the mass of the link
    COM is the coordinates of the Centre of Mass of the link in terms of the link frame
    I is the inertia tensor w.r.t the link frames
    
    Parameters
    ----------
    link : URDF.Link
         urdf.link object corresponding to the link whose mass-inertia properties need to be extracted

    Returns
    -------
    m : float
         Mass of the link
    COM : numpy.ndarray of shape (3,)
         Coordinates of Centre of Mass of the links in terms of the link frame
    I : numpy.ndarray of shape (3,3)
         Inertia Tensor in terms of the link frame

    """
    
    link=link[0]
    ixx=link.inertial.inertia.ixx
    ixy=link.inertial.inertia.ixy
    ixz=link.inertial.inertia.ixz
    iyy=link.inertial.inertia.iyy
    iyz=link.inertial.inertia.iyz
    izz=link.inertial.inertia.izz
    I=np.array([[ixx,ixy,ixz],
                [ixy,iyy,iyz],
                [ixz,iyz,izz]],dtype=np.float64)
    
    m=link.inertial.mass
    
    COM=np.array(link.inertial.origin.xyz,dtype=np.float64)

    COM=COM.reshape((3,1))

    It1=m*np.matmul(COM.T,COM)*np.eye(3,dtype=np.float64)
    It2=-m*np.matmul(COM,COM.T)

    I=I+1*It1+1*It2
    
    return m,COM.flatten(),I

def get_SM(m:list,COMs:np.ndarray,IMs:list)->list:
    """returns SMs the list of 6 Link Spatial Inertia matrices.
    SMs[0]-SMs[5] are the Spatial Inertias of links 1-6.

    Parameters
    ----------
    m : list
         list of link masses
    COMs : numpy.ndarray of shape (3,6)
         array of Centres of Mass of the links
    IMs : list
         list of Inertia Tensors of the links

    Returns
    -------
    SMs : list
         list of 12 Spatial Inertia Matrices.

    """
    
    SMs=np.zeros((len(m),6,6))
    for i in range(len(m)):
        SM=np.zeros((6,6),dtype=np.float64)
        SM[0:3,0:3]=IMs[i]
        til=tilda(COMs[:,i][:,np.newaxis])
        SM[0:3,3:6]=m[i]*til
        SM[3:6,0:3]=-m[i]*til
        SM[3:6,3:6]=m[i]*np.eye(3,dtype=np.float64)
        SMs[i]=SM
    return SMs    
    

@jit(nopython=True,cache=True)
def compute_D(SMs:np.ndarray, phis:np.ndarray,H:np.ndarray)->np.ndarray:
    """returns the 6x6 Joint Space Inertia Matrix D computed using the Composite Rigid Body Algorithm

    Parameters
    ----------
    SMs : list
          list of 6 link spatial inertias 
    phis : numpy.ndarray of shape (6,6,6)
          array of 6 rigid body transformation matrices for frames 1 to 6
    H : numpy.ndarray
          Hinge Map Matrix

    Returns
    -------
    D : numpy.ndarray of shape (6,6)
          6x6 Joint Space Inertia Matrix

    """
    
    D=np.zeros((6,6))
    R=np.zeros((6,6))
    F=np.zeros((6,1))
    R[:,:]=SMs[5]
    F[:,:]=R @ H.T
    D[5,5]=(H @ F)[0,0]
    for j in range(4,-1,-1):
        F[:,:]=phis[j+1] @ F
        D[j,5]=D[5,j]=(H @ F)[0,0]
            
    for k in range(4,-1,-1):
        R[:,:]=SMs[k]+phis[k+1] @ R @ phis[k+1].T
        F[:,:]=R @ H.T
        D[k,k]=(H @ F)[0,0]
        for j in np.arange(k-1,-1,-1):
            F[:,:]=phis[j+1] @ F
            D[j,k]=D[k,j]=(H @ F)[0,0]            
    return D

def compute_D_(SMs:np.ndarray, phis:np.ndarray,H:np.ndarray)->np.ndarray:
    """returns the 6x6 Joint Space Inertia Matrix D computed using the Composite Rigid Body Algorithm

    Parameters
    ----------
    SMs : list
          list of 6 link spatial inertias 
    phis : numpy.ndarray of shape (6,6,6)
          array of 6 rigid body transformation matrices for frames 1 to 6
    H : numpy.ndarray
          Hinge Map Matrix

    Returns
    -------
    D : numpy.ndarray of shape (6,6)
          6x6 Joint Space Inertia Matrix

    """
    
    D=np.zeros((6,6))
    R=np.zeros((6,6))
    F=np.zeros((6,1))
    R[:,:]=SMs[5]
    F[:,:]=R @ H.T
    D[5,5]=(H @ F)[0,0]
    for j in range(4,-1,-1):
        F[:,:]=phis[j+1] @ F
        D[j,5]=D[5,j]=(H @ F)[0,0]
            
    for k in range(4,-1,-1):
        R[:,:]=SMs[k]+phis[k+1] @ R @ phis[k+1].T
        F[:,:]=R @ H.T
        D[k,k]=(H @ F)[0,0]
        for j in np.arange(k-1,-1,-1):
            F[:,:]=phis[j+1] @ F
            D[j,k]=D[k,j]=(H @ F)[0,0]            
    return D

@jit(nopython=True,cache=True)
def forward_sweep(theta0:np.ndarray,theta:np.ndarray,dtheta:np.ndarray,phis:np.ndarray,H:np.ndarray)->Tuple[np.ndarray,np.ndarray,np.ndarray]:
    """Carries out the forward sweep of the Recursive Newton Euler Algorithm
    
    Parameters
    ----------
    theta0 : numpy.ndarray of shape (6,1)
         array of initial joint angles at home position
    theta : numpy.ndarray of shape (6,1)
         array of additional joint angle applied from home position
    dtheta : numpy.ndarray of shape (6,1)
         array of joint velocities
    phis : numpy.ndarray of shape (6,1)
         array rigid body transformation matrices for frames 1 to 12
    H : numpy.ndarray
         Hinge Map Matrix

    Returns
    -------
    V : numpy.ndarray of shape (6,6,1)
         array of 12 link spatial velocities
    A : numpy.ndarray of shape (6,6,1)
         array of 12 link spatial accelerations
    g : numpy.ndarray of shape (6,6,1)
         array of 12 link spatial acceleration due to gravity

    """
    
    V=np.zeros((6,6,1))
    A=np.zeros((6,6,1))
    g=np.zeros((6,6,1))

    Vk=np.zeros((6,1))
    Ak=np.zeros((6,1))
    gk=np.zeros((6,1))
    gk[5]=-9.8
    kak=np.zeros((6,1))
    kwk=np.zeros((3,1))
    for k in np.arange(1,7,1):
        Vk[:,:]=phis[k-1].T @ Vk+H.T*dtheta[k-1]
        kwk[:,:]=(dtheta[k-1]*H.T)[0:3,:]
        kak[0:3,:]=-tilda(kwk) @ Vk[0:3]
        kak[3:6,:]=-tilda(kwk) @ Vk[3:6]
        Ak[:,:]=phis[k-1].T @ Ak+kak
        gk[:,:]=phis[k-1].T @ gk

        V[k-1,:,:]=Vk[:,:]
        A[k-1,:,:]=Ak[:,:]
        g[k-1,:,:]=gk[:,:]        
    return V,A,g

@jit(nopython=True,cache=True)
def reverse_sweep(phis:np.ndarray,SMs:np.ndarray,m:np.ndarray,V:np.ndarray,A:np.ndarray,g:np.ndarray,H:np.ndarray,COMs:np.ndarray)->np.ndarray:
    """Carries out the forward sweep of the Recursive Newton Euler Algorithm
    
    Parameters
    ----------
    phis : numpy.ndarray of shape (6,6,6)
         array rigid body transformation matrices for frames 1 to 6
    SMs : list
         list of 6 link spatial inertias 
    m : list
         list of link masses
    V : numpy.ndarray of shape (6,6,1)
         array of link spatial velocities
    A : numpy.ndarray of shape (6,6,1)
         array of link spatial accelerations
    g : numpy.ndarray of shape (6,6,1)
         array of link spatial acceleration due to gravity
    H : numpy.ndarray
         Hinge Map Matrix
    COMs : numpy.ndarray of shape (3,6)
         array of Centres of Mass of the links

    Returns
    -------
    CG : numpy.ndarray of shape (6,1)
         Vector of Generalised Centripetal, Coriolis and Gravity forces
   
    """
    
    CG=np.zeros((6,1))
    
    kbk=np.zeros((6,1))
    w=np.zeros((3,1))
    p=np.zeros((3,1))
    v=np.zeros((3,1))
    J=np.zeros((3,3))
    w[:]=V[5][0:3]
    J[:,:]=SMs[5][0:3,0:3]
    p[:,0]=COMs[:,5]
    v[:]=V[5][3:6]
    mk=m[5]
    kbk[0:3]=tilda(w) @ J @ w+mk*tilda(p) @ tilda(w) @ v
    kbk[3:6]=mk* tilda(w) @ tilda(w) @ p+mk*tilda(w) @ v
    Fk=SMs[5] @ (A[5]-g[5])+kbk
    CG[5]=H @ Fk
        
    for k in range(5,0,-1):
        w[:]=V[k-1][0:3]
        J[:,:]=SMs[k-1][0:3,0:3]
        p[:,0]=COMs[:,k-1]
        v[:]=V[k-1][3:6]
        mk=m[k-1]
        kbk[0:3]=tilda(w) @ J @ w +mk*tilda(p) @ tilda(w) @ v
        kbk[3:6]=mk*tilda(w) @ tilda(w) @ p+mk*tilda(w) @ v
        Fk[:,:]=phis[k] @ Fk+SMs[k-1] @ (A[k-1]-g[k-1])+kbk
        CG[k-1]=H @ Fk
               
    return CG

def get_DH(joints:np.ndarray,joint_names:np.ndarray)->Tuple[np.ndarray,np.ndarray,np.ndarray]:
    """Gets the DH parameters from the Robot Description (URDF) loaded in the ROS parameter server
    
    Parameters
    ----------
    joints : numpy.ndarray
         array of 6 URDF.Joint objects
    joint_names : numpy.ndarray
         array of 6 Joint Names used to indicate the order in which the URDF.Joint objects exist in joints array

    Returns
    -------
     a : numpy.ndarray
         numpy array of shape (6,1) having the link length for links i=1 to 6
     alpha : numpy.ndarray
         numpy array of shape (6,1) having the link twist for links i=1 to 6
     d : numpy.ndarray
         numpy array of shape (6,1) having the joint offsets for joints i=1 to 6
     theta0 : numpy.ndarray
         numpy array of shape (6,1) having the nominal joint angle for joints i=1 to 6

    """
    
    a=np.zeros((6,1))
    alpha=np.zeros((6,1))
    d=np.zeros((6,1))
    theta0=np.zeros((6,1))
    
    for i in range(1,7):
        joint_name="J"+str(i)
        joint=joints[joint_names==joint_name][0]
        #these are the xyz and rpy from the joint tag. You can derive the used formulas by equating 
        #the transformation matrix from DH parameters with the transformation matrix from xyz-rpy
        x=joint.origin.xyz[0]
        y=joint.origin.xyz[1]
        z=joint.origin.xyz[2]
        roll=joint.origin.rpy[0]
        pitch=joint.origin.rpy[1]
        yaw=joint.origin.rpy[2]
        if i>1:
            a[i-2,0]=x
            alpha[i-2,0]=-np.arctan2(np.cos(roll)*np.sin(pitch)*np.sin(yaw) - np.cos(yaw)*np.sin(roll),np.cos(pitch)*np.cos(roll))
            a_=a[i-2,0]
            alpha_=alpha[i-2,0]
        else:
            a_=a0
            alpha_=alpha0
        if alpha_!=0:
            d[i-1,0]=-y/np.sin(alpha_)
        else:
            d[i-1,0]=z/np.cos(alpha_)
        theta0[i-1,0]=-np.arctan2(np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.cos(roll)*np.sin(yaw),np.cos(pitch)*np.cos(yaw))
    
    return a,alpha,d,theta0

@jit(nopython=True,cache=True)
def euler2omega_jacobian(alpha:float,beta:float,gamma:float)->np.ndarray:
    """returns the 3x3 Jacobian Matrix for converting angular velocities to Euler Angle Rates for the ZYZ Euler Angles

    Parameters
    ----------
    alpha : float
         first angle in a ZYZ Euler Angle notation
    beta : float
         second angle in a ZYZ Euler Angle notation
    gamma : float
         third angle in a ZYZ Euler Angle notation

    Returns
    -------
    Te : numpy.ndarray of shape (3,3)
         3x3 Jacobian Matrix for converting angular velocities to Euler Angle Rates for the ZYZ Euler Angles

    """
    
    Te = np.array([[0.0, -np.sin(alpha), np.cos(alpha)*np.sin(beta)],
                     [0.0, np.cos(alpha), np.sin(alpha)*np.sin(beta)],
                     [1.0, 0.0, np.cos(beta)]])
    return Te

@jit(nopython=True,cache=True)
def geometric2analytic_jacobian(euler:np.ndarray)->np.ndarray:
    """returns the 6x6 matrix which needs to be post multiplied by the Geometric Jacobian 
    for converting it into Analytical Jacobian for use with ZYZ Euler Angles

    Parameters
    ----------
    euler : np.ndarray of shape (3,1)
         array of euler angles

    Returns
    -------
    Ta_inv : numpy.ndarray of shape (6,6)
         6x6 matrix which needs to be post multiplied by the 6x6 Geometric Jacobian 
        of a single side for converting it into Analytical Jacobian for use with ZYZ Euler Angles

    """
    
    alpha=euler[0,0]
    beta=euler[1,0]
    gamma=euler[2,0]
    Te=euler2omega_jacobian(alpha,beta,gamma)
    Te_inv=inv(Te)
    Ta_inv=np.eye(6)
    Ta_inv[0:3,0:3]=Te_inv
    return Ta_inv

@jit(nopython=True,cache=True)
def geometric_jacobian(cphis:np.ndarray)->Tuple[np.ndarray,np.ndarray,np.ndarray]:
    """returns the 6x6 Geometric Jacobian.

    Parameters
    ----------
    cphis : np.ndarray of shape (6,6,6)
         array of 6 6x6 Cummulative Rigid Body Transformation Matrices

    Returns
    -------
    J : numpy.ndarray of shape 6x6
         6x6 Geometric Jacobian.

    """
    
    J_=np.zeros((6,6))
    R=cphis[0,0:3,0:3]
    R06=np.zeros((6,6))
    R06[0:3,0:3]=R06[3:6,3:6]=R
    for i in range(1,6):
        J_[:,i-1]=(cphis[i].T @ H.T).ravel()
    J_[:,5]=(H.T).ravel()
    J=R06 @ phi6_ef.T @ J_ 
    return J,J_,R06
    
def joint2task(theta:np.ndarray)->np.ndarray:
    """Converts Joint Space Coordinates to Task Space Coordinates for the ZYZ Euler Angle Representation of Orientation
    
    Parameters
    ----------
    theta : numpy.ndarray of shape (6,1)
         Joint Space Coordinate

    Returns
    -------
    X : numpy.ndarray of shape (6,1)
         Task Space Coordinate
    """
    
    X=np.zeros((6,1))
    phis=get_phi(alpha,a,theta0,theta,d,a0,alpha0)[0]
    cphis=get_cummulative_phi(phis)
    ef=np.matmul(cphis[0,:,:],phi6_ef)
    X[0:3]=np.array(euler_from_matrix(ef[0:3,0:3],'rzyz')).reshape((3,1))
    X[3:6]=pos_from_phi(ef)
    
    return X

@jit(nopython=True,cache=True)
def dphis_dq(theta:np.ndarray,phis_l:np.ndarray,n:int=6)->np.ndarray:
    """returns 3D matrix of stacked derivative of rigid body transformation matrices wrt respective joint variable

    Parameters
    ----------
    theta : np.ndarray of shape (n,1)
        vector of current joint variables
    phis_l : np.ndarray of shape (n,6,6)
        3D array of stacked link rigid body transformation matrices
    n : int, optional
        Number of DOF. The default is 6.

    Returns
    -------
    dphis : np.ndarray of shape (n,6,6)
        3D matrix of stacked derivative of rigid body transformation matrices wrt respective joint variable

    """
    
    dphis=np.zeros((n,6,6))
    for j in range(1,n+1):
        dphis[j-1]=phis_l[j-1] @ dphih_dq(j,theta[j-1,0])
    return dphis


@jit(nopython=True,cache=True)
def dphih_dq(j:int,q:float)->np.ndarray:
    """returns derivative of hinge rigid body transformation matrix wrt the respective joint angle.

    Parameters
    ----------
    j : int
        joint variable index.
    q : float
        joint variable value.

    Returns
    -------
    dphi_h : np.ndarray of shape (6,6)
        derivative of hinge rigid body transformation matrix wrt the respective joint angle.
    """   
    
    theta=q+theta0[j-1,0]
    R=np.array([[-np.sin(theta),-np.cos(theta),0.0],
                [np.cos(theta),-np.sin(theta),0.0],
                [0.0,0.0,0.0]])
    dphi_h=np.zeros((6,6))
    dphi_h[0:3,0:3]=dphi_h[3:6,3:6]=R
    return dphi_h

@jit(nopython=True,cache=True)
def dTe(euler:np.ndarray,deuler:np.ndarray)->np.ndarray:
    """returns time derivative of Te

    Parameters
    ----------
    euler : np.ndarray of shape (3,1)
        euler angles.
    deuler : np.ndarray of shape (3,1)
        euler angle rates.

    Returns
    -------
    DTE : np.ndarray of shape (6,6)
        time derivative of Te

    """
    
    alpha=euler[0,0]
    beta=euler[1,0]
    gamma=euler[2,0]
    dalpha=deuler[0,0]
    dbeta=deuler[1,0]
    dgamma=deuler[2,0]
    dte=np.array([[0.0, -np.cos(alpha)*dalpha, np.cos(alpha)*np.cos(beta)*dbeta-np.sin(alpha)*np.sin(beta)*dalpha],
                  [0.0, -np.sin(alpha)*dalpha, np.cos(alpha)*np.sin(beta)*dalpha+np.sin(alpha)*np.cos(beta)*dbeta],
                  [0.0, 0.0, -np.sin(beta)*dbeta]])
    DTE=np.zeros((6,6))
    DTE[0:3,0:3]=dte
    return DTE

@jit(nopython=True)
def dJi_dt(phis:np.ndarray,dphis:np.ndarray,cphis:np.ndarray,dq:np.ndarray,n:int,i:int)->np.ndarray:
    """returns time derivative of i'th column of J_hat

    Parameters
    ----------
    phis : np.ndarray of shape (n,6,6)
        3D array of stacked rigid body transformation matrices
    dphis : np.ndarray of shape (n,6,6)
        3D array of stacked derivative of rigid body transformation matrices wrt respective joint variable
    cphis : np.ndarray of shape (n,6,6)
        3D array of stacked cummulative rigid body transformation matrices
    dq : np.ndarray of shape (n,1)
        array of current joint velocities
    n : int
        Number of DOF.
    i : int
        column number.

    Returns
    -------
    dj_ : np.ndarray
        time derivative of the i'th column of J_hat

    """    
    phi_p=np.zeros((n-i,1,6))
    phi_p[0]=H
    phi_s=np.zeros((n-i,6,6))
    phi_s[0:-1]=cphis[i+1:n]
    phi_s[-1]=np.eye(6)
    for k in range(i+1,n):
        phi_p[k-i]=phi_p[k-i-1] @ phis[k-1]
    dj_=np.zeros((6,1))
    for k in range(i,n):
        dj_+=dq[k,0] * (phi_p[k-i] @ dphis[k] @ phi_s[k-i])
    return dj_.ravel()

# def dJi_dt(phis:np.ndarray,dphis:np.ndarray,cphis:np.ndarray,dq:np.ndarray,n:int,i:int)->np.ndarray:
#     """returns time derivative of i'th column of J_hat

#     Parameters
#     ----------
#     phis : np.ndarray of shape (n,6,6)
#         3D array of stacked rigid body transformation matrices
#     dphis : np.ndarray of shape (n,6,6)
#         3D array of stacked derivative of rigid body transformation matrices wrt respective joint variable
#     cphis : np.ndarray of shape (n,6,6)
#         3D array of stacked cummulative rigid body transformation matrices
#     dq : np.ndarray of shape (n,1)
#         array of current joint velocities
#     n : int
#         Number of DOF.
#     i : int
#         column number.

#     Returns
#     -------
#     dj_ : np.ndarray
#         time derivative of the i'th column of J_hat

#     """
    
#     phi_p=np.zeros((n-i,1,6))
#     phi_p[0]=H
#     phi_s=np.zeros((n-i,6,6))
#     phi_s[0:-1]=cphis[i+1:n]
#     phi_s[-1]=np.eye(6)
#     for k in range(i+1,n):
#         phi_p[k-i]=phi_p[k-i-1] @ phis[k-1]
#     dj_=np.sum(dq[i:n] *(phi_p @ dphis[i:n] @ phi_s)[:,0,:],axis=0)
#     return dj_

@jit(nopython=True,cache=True)
def dJ_dt_(phis:np.ndarray,dphis:np.ndarray,cphis:np.ndarray,dq:np.ndarray,n:int)->np.ndarray:
    """returns time derivative of J_hat
    
    Parameters
    ----------
    phis : np.ndarray of shape (n,6,6)
        3D array of stacked rigid body transformation matrices
    dphis : np.ndarray of shape (n,6,6)
        3D array of stacked derivative of rigid body transformation matrices wrt respective joint variable
    cphis : np.ndarray of shape (n,6,6)
        3D array of stacked cummulative rigid body transformation matrices
    dq : np.ndarray of shape (n,1)
        array of current joint velocities
    n : int
        number of DOF.

    Returns
    -------
    dJ_ : np.ndarray
        time derivative of J_hat.

    """
    
    dJ_=np.zeros((6,n))
    for i in range(1,n):
        dJ_[:,i-1]=dJi_dt(phis,dphis,cphis,dq,n,i)
    return dJ_

@jit(nopython=True,cache=True)
def dR_dt(phis:np.ndarray,dphis:np.ndarray,cphis:np.ndarray,dq:np.ndarray,n:int)->np.ndarray:
    """returns time derivative of R0n

    Parameters
    ----------
    phis : np.ndarray of shape (n,6,6)
        3D array of stacked rigid body transformation matrices
    dphis : np.ndarray of shape (n,6,6)
        3D array of stacked derivative of rigid body transformation matrices wrt corresponding joint variable
    cphis : np.ndarray of shape (n,6,6)
        3D array of stacked cummulative rigid body transformation matrices
    dq : np.ndarray of shape (n,1)
        array of current joint velocities
    n : int
        number of DOF.

    Returns
    -------
    dR : np.ndarray
        time derivative of R0n.

    """
    
    R_p=np.zeros((n,3,3))
    R_p[0]=np.eye(3)
    for k in range(1,n):
        R_p[k]=R_p[k-1] @ phis[k-1,0:3,0:3]
    R_s=np.zeros((n,3,3))
    R_s[0:-1]=cphis[1:n,0:3,0:3]
    R_s[-1]=np.eye(3)
    dr=np.zeros((3,3))
    for i in range(n):
        dr+= dq[i,0] * (R_p[i] @ dphis[i,0:3,0:3] @ R_s[i])
    dR=np.zeros((6,6))
    dR[0:3,0:3]=dR[3:6,3:6]=dr
    return dR

# def dR_dt(phis:np.ndarray,dphis:np.ndarray,cphis:np.ndarray,dq:np.ndarray,n:int)->np.ndarray:
#     """returns time derivative of R0n

#     Parameters
#     ----------
#     phis : np.ndarray of shape (n,6,6)
#         3D array of stacked rigid body transformation matrices
#     dphis : np.ndarray of shape (n,6,6)
#         3D array of stacked derivative of rigid body transformation matrices wrt corresponding joint variable
#     cphis : np.ndarray of shape (n,6,6)
#         3D array of stacked cummulative rigid body transformation matrices
#     dq : np.ndarray of shape (n,1)
#         array of current joint velocities
#     n : int
#         number of DOF.

#     Returns
#     -------
#     dR : np.ndarray
#         time derivative of R0n.

#     """
    
#     R_p=np.zeros((n,3,3))
#     R_p[0]=np.eye(3)
#     for k in range(1,n):
#         R_p[k]=R_p[k-1] @ phis[k-1,0:3,0:3]
#     R_s=np.zeros((n,3,3))
#     R_s[0:-1]=cphis[1:n,0:3,0:3]
#     R_s[-1]=np.eye(3)
#     dr= np.sum(dq * (R_p @ dphis[:,0:3,0:3] @ R_s).reshape((n,9)), axis=0).reshape(3,3)
#     dR=np.zeros((6,6))
#     dR[0:3,0:3]=dR[3:6,3:6]=dr
#     return dR

@jit(nopython=True,cache=True)
def dJ_dt(J_:np.ndarray,R06:np.ndarray,phis:np.ndarray,phis_l:np.ndarray,cphis:np.ndarray,q:np.ndarray,dq:np.ndarray,n:int)->np.ndarray:
    """returns time derivative of geometric jacobian

    Parameters
    ----------
    J_ : np.ndarray of shape (6,n)
        J_hat
    R06 : np.ndarray of shape (6,6)
        Spatial rotation from #0 to #n
    phis : np.ndarray of shape (n,6,6)
        3D array of stacked rigid body transformation matrices
    phis_l : np.ndarray of shape (n,6,6)
        3D array of stacked link rigid body transformation matrices
    cphis : np.ndarray of shape (n,6,6)
        3D array of stacked cummulative rigid body transformation matrices
    q : np.ndarray of shape (n,1)
        array of current joint position
    dq : np.ndarray of shape (n,1)
        array of current joint velocities
    n : int
        number of DOF.

    Returns
    -------
    dJ : np.ndarray
        time derivative of geometric jacobian

    """
    
    dphis=dphis_dq(q,phis_l,n)
    dJ_=dJ_dt_(phis,dphis,cphis,dq,n)
    dR=dR_dt(phis,dphis,cphis,dq,n)
    dJ=dR @ phi6_ef.T @ J_ + R06 @ phi6_ef.T @ dJ_
    return dJ

@jit(nopython=True,cache=True)
def R2Euler(R:np.ndarray)->np.ndarray:
    beta=np.arctan2(-np.sqrt(1-R[2,2]**2),R[2,2])
    alpha=np.arctan2(-R[1,2],-R[0,2])
    gamma=np.arctan2(-R[2,1],R[2,0])
    return np.array([[alpha,beta,gamma]]).T

@jit(nopython=True,cache=True)
def regularize(A,cond):
    """
    regularizes input matrix A using a modified truncated SVD regularization using
    cond as the threshold condition number

    Parameters
    ----------
    A : numpy.ndarray
         input matrix
    cond : float
         threshold condition number

    Returns
    -------
    A_ : numpy.ndarray
         regularized matrix
    A_pinv: numpy.ndarray
        pseudo inverse of regularized matrix

    """
    U,S,Vt=np.linalg.svd(A)

    S=S[S[0]/S<cond]

    S_=np.diag(S)
    n=len(S)
    A_=U[:,0:n] @ S_ @ Vt[0:n,:]#regularized matrix

    S1=1/S
    S1_=np.diag(S1)
    A_pinv=Vt[0:n,:].T @ S1_ @ U[:,0:n].T
    return A_,A_pinv

@jit(nopython=True,cache=True)
def state_difference(Xg,Xs):
    """
    The function which gives the difference between task space coordinates
    Xg and Xs as per the Euler Angle based scheme.

    Parameters
    ----------
    Xg : numpy.ndarray
         goal task space coordinate/desired task space coordinate
    Xs : numpy.ndarray
         start task space coordinate/current task space coordinate

    Returns
    -------
    Xd : numpy.ndarray
         difference between task space coordinates Xg and Xs as per the control scheme.

    """
    Xd=np.zeros(Xg.shape)
    for i in range(3):
        diff=Xg[i,0]-Xs[i,0]
        if abs(diff)>pi:
            Xd[i,0]=-np.sign(diff)*(2*pi-abs(diff))
        else:
            Xd[i,0]=diff
    Xd[3:6]=Xg[3:6]-Xs[3:6]
    # Xd=Xg-Xs
    # Xde=Xd[0:3]
    # #converting angles greater than pi to equivalent negative angle
    # #done so that the difference indicates the shortest possible path
    # idx=np.abs(Xde)>pi
    # Xde[idx]=-np.sign(Xde[idx])*(2*pi-np.abs(Xde[idx]))
    # Xd[0:3,0]=Xde[0:3,0]
    return Xd

#initializing kinematic parameters of the Manipulator  
print("Initializing Kinematic properties of the Manipulator") 
robot = URDF.from_parameter_server() #getting robot description from ROS parameter server
link_names=np.array([l.name for l in robot.links]) #list of link names
links=np.array(robot.links) #list of URDF.Link
joint_names=np.array([j.name for j in robot.joints]) #list of joint names
joints=np.array(robot.joints) #list of URDF.Joint

H=np.array([[0,0,1,0,0,0]],dtype=np.float64) #Hinge Map Matrix
"""Hinge Map Matrix for a revolute joint"""

a0=0 #link length for base link
"""Link length for base link"""

alpha0=0 #link twist for base link
"""Link twist for base link"""

a=np.zeros((6,1))
"""array of DH parameters a"""
alpha=np.zeros((6,1))
"""array of DH parameters alpha"""
d=np.zeros((6,1))
"""array of DH parameters d"""
theta0=np.zeros((6,1))
"""array of nominal joint angles"""
a[:,:],alpha[:,:],d[:,:],theta0[:,:]=get_DH(joints,joint_names)#get DH parameters

#rigid body transformation matrices from #6 to end effector frame
phi6_ef=np.zeros((6,6),dtype=np.float64)
"""rigid body transformation matrices from #6 to end effector frame"""

phi6_ef[0:3,3:6]=tilda(np.array([[0,0,0.25722535]]).T)
phi6_ef[0:3,0:3]=phi6_ef[3:6,3:6]=np.eye(3)


#initializing mass-inertia properties of the Manipulator
print("Initializing Mass-Inertia properties of the Manipulator")
m=np.array(6*[0],dtype=np.float64) #list of link masses
"""list of link masses"""
COMs=np.zeros((3,6),dtype=np.float64) #list of link centre of mass
"""array of link centre of masses"""
IMs=6*[0] #list of Inertia Tensor
"""list of link rotational inertias"""

for i in range(1,7):
    m[i-1],COMs[:,i-1],IMs[i-1]=create_IM(links[link_names=='L'+str(i)])

SMs=get_SM(m,COMs,IMs)#spatial inertias
"""list of link spatial inertias"""
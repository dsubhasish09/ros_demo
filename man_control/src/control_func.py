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
# cimport numpy as np
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

def tilda(p:np.ndarray)->np.ndarray:
    """Returns the 3x3 skew-symmetric matix corresponding to the cross-product
    operation for a given 3x1 vector.

    Parameters
    ----------
    p : list or numpy.ndarray of shape (3,)
         3x1 input vector

    Returns
    -------
    til : numpy.ndarray of shape (3,3)
         3x3 skew-symmetric matix corresponding to the cross-product operation

    """
    
    til=np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]],dtype=np.float64)
    return til


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

def Rx(theta:float)->np.ndarray:
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
    
    R=np.array([[1,0,0],
                [0,cos(theta),-sin(theta)],
                [0,sin(theta),cos(theta)]],dtype=np.float64)
    return R

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
    
    R=np.array([[cos(theta),0,sin(theta)],
                [0,1,0],
                [-sin(theta),0,cos(theta)]],dtype=np.float64)
    return R

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
    
    R=np.array([[cos(theta),-sin(theta),0],
                [sin(theta),cos(theta),0],
                [0,0,1]],dtype=np.float64)
    return R

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
    l=[a,-d*sin(alpha),d*cos(alpha)]
    l_tilda=tilda(l)
    phi_l=np.zeros((6,6),dtype=np.float64)
    phi_l[0:3,0:3]=R
    phi_l[0:3,3:6]=np.matmul(l_tilda,R)
    phi_l[3:6,3:6]=R
    return phi_l

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
    
    R=Rz(theta)
    phi_h=np.zeros((6,6),dtype=np.float64)
    phi_h[0:3,0:3]=R
    phi_h[3:6,3:6]=R
    return phi_h


def get_phi(alpha:np.ndarray,a:np.ndarray,theta0:np.ndarray,theta:np.ndarray,d:np.ndarray,a0:float,alpha0:float)->np.ndarray:
    """Returns a list of combined rigid body transformation matrixes for all 12 joints in the Half-humanoid

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
    """

    phis_l=np.zeros((6,6,6))
    phis_h=np.zeros((6,6,6))
    for i in range(6):
        if i==0 :
            phis_l[i,:,:]=phi_link(d[i,0],a0,alpha0)
        else:
            phis_l[i,:,:]=phi_link(d[i,0],a[i-1,0],alpha[i-1,0])
        phis_h[i,:,:]=phi_hinge(theta[i]+theta0[i])
    phis=phis_l@phis_h
    return phis,phis_l,phis_h

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
        cphis[i,:,:]=np.matmul(phis[i,:,:],cphis[i+1,:,:])
        
    return cphis

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
    
    pos=inv_tilda(np.matmul(phi[0:3,3:6],phi[0:3,0:3].T))
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
    
    SMs=[]
    for i in range(len(m)):
        SM=np.zeros((6,6),dtype=np.float64)
        SM[0:3,0:3]=IMs[i]
        SM[0:3,3:6]=m[i]*tilda(COMs[:,i])
        SM[3:6,0:3]=-m[i]*tilda(COMs[:,i])
        SM[3:6,3:6]=m[i]*np.eye(3,dtype=np.float64)
        SMs.append(SM)
    return SMs    
    


def compute_D(SMs:list, phis:np.ndarray,H:np.ndarray)->np.ndarray:
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
    
    D=np.zeros((6,6),dtype=np.float64)
    R=np.zeros((6,6))
    F=np.zeros((6,1))
    R[:,:]=SMs[5]
    F[:,:]=np.matmul(R,H.T)
    D[5,5]=np.matmul(H,F)
    for j in np.arange(4,-1,-1):
        F[:,:]=np.matmul(phis[j+1],F)
        D[j,5]=np.matmul(H,F)
        D[5,j]=D[j,5]
            
    for k in [4,3,2,1,0]:
        R[:,:]=SMs[k]+np.matmul(np.matmul(phis[k+1],R),phis[k+1].T)
        F[:,:]=np.matmul(R,H.T)
        D[k,k]=np.matmul(H,F)
        for j in np.arange(k-1,-1,-1):
            F[:,:]=np.matmul(phis[j+1],F)
            D[j,k]=np.matmul(H,F)
            D[k,j]=D[j,k]
            
    return D

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
    V0=np.zeros((6,1))
    A0=np.zeros((6,1))
    g0=np.zeros((6,1))
    Vk=np.zeros((6,1))
    Ak=np.zeros((6,1))
    gk=np.zeros((6,1))
    g0[5]=-9.8
    kak=np.zeros((6,1))
    kwk=np.zeros((3,1))
    for k in np.arange(1,7,1):
        Vk[:,:]=np.matmul(phis[k-1].T,V0)+H.T*dtheta[k-1]
        kwk[:,:]=(dtheta[k-1]*H.T)[0:3,:]
        kak[0:3,:]=-np.matmul(tilda(kwk),Vk[0:3])
        kak[3:6,:]=-np.matmul(tilda(kwk),Vk[3:6])
        Ak[:,:]=np.matmul(phis[k-1].T,A0)+kak
        gk[:,:]=np.matmul(phis[k-1].T,g0)

        V[k-1,:,:]=Vk[:,:]
        A[k-1,:,:]=Ak[:,:]
        g[k-1,:,:]=gk[:,:]
        
        V0[:,:]=Vk[:,:]
        A0[:,:]=Ak[:,:]
        g0[:,:]=gk[:,:]
        
    return V,A,g

def reverse_sweep(phis:np.ndarray,SMs:list,m:list,V:np.ndarray,A:np.ndarray,g:np.ndarray,H:np.ndarray,COMs:np.ndarray)->np.ndarray:
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
    
    F0=np.zeros((6,1))
    CG=np.zeros((6,1))
    
    kbk=np.zeros((6,1))
    w=np.zeros((3,))
    p=np.zeros((3,))
    v=np.zeros((3,))
    J=np.zeros((3,3))
    w[:]=V[5][0:3,0]
    J[:,:]=SMs[5][0:3,0:3]
    p[:]=COMs[:,5]
    v[:]=V[5][3:6,0]
    mk=m[5]
    kbk[0:3,0]=np.matmul(np.matmul(tilda(w),J),w)+mk*np.matmul(np.matmul(tilda(p),tilda(w)),v)
    kbk[3:6,0]=mk*np.matmul(np.matmul(tilda(w),tilda(w)),p)+mk*np.matmul(tilda(w),v)
    Fk=np.matmul(SMs[5],(A[5]-g[5]))+kbk
    CG[5]=np.matmul(H,Fk)
        
    F0[:,:]=Fk[:,:]
    for k in np.arange(5,0,-1):
        w[:]=V[k-1][0:3,0]
        J[:,:]=SMs[k-1][0:3,0:3]
        p[:]=COMs[:,k-1]
        v[:]=V[k-1][3:6,0]
        mk=m[k-1]
        kbk[0:3,0]=np.matmul(np.matmul(tilda(w),J),w)+mk*np.matmul(np.matmul(tilda(p),tilda(w)),v)
        kbk[3:6,0]=mk*np.matmul(np.matmul(tilda(w),tilda(w)),p)+mk*np.matmul(tilda(w),v)
        Fk[:,:]=np.matmul(phis[k],F0)+np.matmul(SMs[k-1],(A[k-1]-g[k-1]))+kbk
        CG[k-1]=np.matmul(H,Fk)
        
        F0[:,:]=Fk[:,:]
        
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
            alpha[i-2,0]=-np.arctan2(cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll),cos(pitch)*cos(roll))
            a_=a[i-2,0]
            alpha_=alpha[i-2,0]
        else:
            a_=a0
            alpha_=alpha0
        if alpha_!=0:
            d[i-1,0]=-y/sin(alpha_)
        else:
            d[i-1,0]=z/cos(alpha_)
        theta0[i-1,0]=-np.arctan2(cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw),cos(pitch)*cos(yaw))
    
    return a,alpha,d,theta0

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
    
    Te = np.array([[0, -sin(alpha), cos(alpha)*sin(beta)],
                     [0, cos(alpha), sin(alpha)*sin(beta)],
                     [1, 0, cos(beta)]])
    return Te


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
    
    J_=np.zeros((6,6),dtype=np.float64)
    R=cphis[0,0:3,0:3]
    R06=np.zeros((6,6),dtype=np.float64)
    R06[0:3,0:3]=R06[3:6,3:6]=R
    J_[0:6,0:5]=np.array([np.matmul(cphis[i,:,:].T,H.T) for i in [1,2,3,4,5]],dtype=np.float64).squeeze().T
    J_[0:6,5]=(H.T).squeeze()
    J=np.matmul(R06,np.matmul(phi6_ef.T,J_))

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
    
    dphis_h=np.zeros((n,6,6))
    for j in range(1,n+1):
        dphis_h[j-1,:,:]=dphih_dq(j,theta[j-1,0])
    dphis=phis_l @ dphis_h
    return dphis

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
    
    theta=q+theta0[j-1]
    R=np.array([[-sin(theta),-cos(theta),0],
                [cos(theta),-sin(theta),0],
                [0,0,0]])
    dphi_h=np.zeros((6,6))
    dphi_h[0:3,0:3]=dphi_h[3:6,3:6]=R
    return dphi_h

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
    dte=np.array([[0, -cos(alpha)*dalpha, cos(alpha)*cos(beta)*dbeta-sin(alpha)*sin(beta)*dalpha],
                  [0, -sin(alpha)*dalpha, cos(alpha)*sin(beta)*dalpha+sin(alpha)*cos(beta)*dbeta],
                  [0, 0, -sin(beta)*dbeta]])
    DTE=np.zeros((6,6))
    DTE[0:3,0:3]=dte
    return DTE

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
    dj_=np.sum(dq[i:n] *(phi_p @ dphis[i:n] @ phi_s)[:,0,:],axis=0)
    return dj_

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
    dr= np.sum(dq * (R_p @ dphis[:,0:3,0:3] @ R_s).reshape((n,9)), axis=0).reshape(3,3)
    dR=np.zeros((6,6))
    dR[0:3,0:3]=dR[3:6,3:6]=dr
    return dR

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

#initializing kinematic parameters of the half humanoid  
print("Initializing Kinematic properties of the Half Humanoid") 
robot = URDF.from_parameter_server() #getting robot description from ROS parameter server
link_names=np.array([l.name for l in robot.links]) #list of link names
links=np.array(robot.links) #list of URDF.Link
joint_names=np.array([j.name for j in robot.joints]) #list of joint names
joints=np.array(robot.joints) #list of URDF.Joint

H=np.array([[0,0,1,0,0,0]],dtype=np.float64) #Hinge Map Matrix
a0=0 #link length for base link
alpha0=0 #link twist for base link
a,alpha,d,theta0=get_DH(joints,joint_names)#get DH parameters



#rigid body transformation matrices for Hand Centres
phi6_ef=np.zeros((6,6),dtype=np.float64)
phi6_ef[0:3,3:6]=tilda([0,0,0.25722535])
phi6_ef[0:3,0:3]=phi6_ef[3:6,3:6]=np.eye(3)


#initializing mass-inertia properties of the half humanoid
print("Initializing Mass-Inertia properties of the Half Humanoid")
m=6*[0] #list of link masses
COMs=np.zeros((3,6),dtype=np.float64) #list of link centre of mass
IMs=6*[0] #list of Inertia Tensor

for i in range(1,7):
    m[i-1],COMs[:,i-1],IMs[i-1]=create_IM(links[link_names=='L'+str(i)])

SMs=get_SM(m,COMs,IMs)#spatial inertias
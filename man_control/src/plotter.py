#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 28 20:02:38 2021

@author: dsubhasish
"""

import pickle
import matplotlib.pyplot as plt
import numpy as np

file=open("time.pickle","rb")
t=np.array(pickle.load(file))
file.close()

file=open("err.pickle","rb")
err=np.array(pickle.load(file))
file.close()

file=open("time_.pickle","rb")
t_=np.array(pickle.load(file))
file.close()

file=open("err_.pickle","rb")
err_=np.array(pickle.load(file))
file.close()

err=err[np.logical_and(t>=2.00,t<=3)]
t=t[np.logical_and(t>=2.00,t<=3)]-2.005

err_=err_[np.logical_and(t_>=1.00,t_<=2)]
t_=t_[np.logical_and(t_>=1.00,t_<=2)]-1.00

plt.figure(1,dpi=300)

E0=np.max(err)
t__=np.arange(0,200)*0.005
err__=(E0+10*E0*t__)*np.exp(-10*t__)
plt.plot(t__,err__,label="Analytical Result")
plt.plot(t,err,label="Exact Derivative at 200Hz")
plt.plot(t_,err_,label="Exact Derivative at 10kHz")
plt.ylim((0,E0+0.05))
plt.legend()
plt.xlabel("time (s)")
plt.ylabel("error magnitude")
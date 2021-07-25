#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 10 18:23:51 2021

@author: dsubhasish
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt

# file=open('for_loop.pickle','rb')
# for_data=pickle.load(file)
# file.close()

# Tf,Nf,xf,yf,y_f,betaf=for_data

# file=open('vectorized.pickle','rb')
# vect_data=pickle.load(file)
# file.close()

# Tv,Nv,xv,yv,y_v,betav=vect_data

# plt.figure(1,dpi=300)
# # plt.plot(Nf,Tf,label=r'Measured Running time')
# # plt.plot(Nf,np.exp(y_f),label=r'Fitted Running time ($O(n^{1.965}))$')
# plt.plot(Nv,Tv,label=r'Measured Running time')
# plt.plot(Nv,np.exp(y_v),label=r'Fitted Running time ($O(n^{1.668}))$')
# plt.xlabel("DOFs")
# plt.ylabel("time (s)")
# plt.legend()
# plt.title("vectorized running time")
# plt.grid()

file=open("t_.pickle","rb")
t=pickle.load(file)
file.close()

file=open("j_err__.pickle","rb")
jerr=pickle.load(file)
file.close()

plt.figure(1,dpi=300)
plt.plot(t,jerr)
plt.title(r"$e_{rel}$ vs time")
plt.xlabel("time (s)")
plt.ylabel(r"$e_{rel}$")
print("Average Error-",np.average(jerr))

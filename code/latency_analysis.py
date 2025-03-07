"""
Script Name: stability_analysis.py
Description: Contains a class to analyze the stability of a two-supplier district heating system as discussed in the paper 

P. Friedrich, et. al. "Stability Analysis and Mitigation of Thermo-Hydraulic Oscillations in Multi-Supplier District Heating Systems", 
MDPI Energies 2025, https://doi.org/10.3390/en18051126. 

Execution of the script produces Figs. 4, 5, 6 and 10 shown in the paper, and makes Nyquist- and Bode-plots not shown in the paper.
Author: Pascal Friedrich, Kirill Kuroptev, Thanh Huynh
Date: 2025-03-06
Version: 2.0
License: Creative commons CC-BY
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import ScalarFormatter
import seaborn as sns
import os
from typing import List, Tuple
import seaborn as sns
import control as ctl
import sympy as sp
import pandas as pd

plt.style.use(style="ggplot")

# constants
cp = 4183 # specific heat capacity of water (J/kg.K)
rho = 1000 # density of water (kg/m^3)

# Definiere den Style mit einem rc-Dictionary
custom_style = {
    'font.family': 'Charter',
    'figure.figsize': (0.9*21/2.54, 0.9*21/2.54),  # Width and height in inch
}

# Plot style
sns.set_style("darkgrid") 
sns.set_context("paper", rc=custom_style)
plt.rcParams.update(custom_style)

class ControlStabilityAnalyzor:
    """Class to analyze the stability of a delayed two-supplier DHS via the linearized transfer function, as in P. Friedrich, et. al.
    "Stability Analysis and Mitigation of Thermo-Hydraulic Oscillations in Multi-Supplier District Heating Systems", MDPI Energies 2025, https://doi.org/10.3390/en18051126
    """
    approximation_order = 4

    def __init__(self, L, d, Qf_C, T_R, T_C0, T_S, T_D):
        self.L      = L
        self.d      = d
        self.Qf_C   = Qf_C
        self.T_R    = T_R
        self.T_C0   = T_C0
        self.T_S    = T_S
        self.T_D    = T_D
        
        # Pre-Processing
        self.mf_C0 = Qf_C/(cp*(T_C0-T_R)) # constant share (=set-point) of mass flow at consumer
        self.mf_D = self.mf_C0*(T_S-T_C0)/(T_S-T_D) # mass flow at deployed supplier
        self.Qf_D = self.mf_D*cp*(T_D-T_R) # Heat flow at deployed supplier
        self.mf_D = self.Qf_D/(cp*(T_D-T_R)) 
        # gains 
        self.K_M = self.mf_D*(T_S-T_D)/(self.mf_C0**2) # Eq. 8: Gain of mixing node
        self.K_C = - self.mf_C0/(T_C0-T_R) # Eq. 21: Gain of consumer: Derivative of mf for T_c at setpoint 
        self.K = (T_S-T_C0)/(T_R-T_C0) # overall system gain
        # control
        print(f'Control of values (should be equal): K_M*K_D={self.K_C*self.K_M}, K={self.K}')
        self.m_P = rho*L*np.pi*d**2/4 # mass of water in pipe
        self.t_P = self.m_P/self.mf_C0 # Eq. 15: transport delay of pipe
        print(f'Qf_D/QF_C = {self.Qf_D/Qf_C}')
        
        #sympy
        # Define symbolic variables
        self.sp_K, self.sp_t_T, self.sp_t_P, self.sp_s = sp.symbols('K t_T t_P s') #for pole analysis

        print("Input for t_P:", self.t_P)
        print("Input for K:", self.K)

    def get_V_tank(self, t_T:float) -> float:
        return self.mf_C0*t_T/rho
    
    def F_open_loop(self, w, t_T):
        """Transfer function of open loop"""
        return self.K/(1+1j*w*t_T)*np.e**(-1j*w*self.t_P)

    def F_closed_loop(self, w, t_T):
        """Transfer function of closed loop"""
        Fol = self.F_open_loop(w, t_T)
        return Fol/(1-Fol)

    def F_open_loop_approx(self, t_T) -> ctl.TransferFunction:
        """Transfer function of open loop with Padé approximation"""
        # Pade approximation of the time delay e^(-s*t_P)
        num_delay, den_delay = ctl.pade(self.t_P, self.approximation_order)  # Xth order approximation, adjust as needed
        # Transfer function without the delay
        G_s = ctl.tf([self.K], [t_T, 1])
        # Transfer function of the Pade approximation
        G_delay = ctl.tf(num_delay, den_delay)
        # Complete transfer function with delay approximation
        G_total = G_s * G_delay
        return G_total

    def F_closed_loop_approx(self, t_T) -> ctl.TransferFunction:
        """Transfer function of closed loop with Padé approximation"""
        G_total = self.F_open_loop_approx(t_T)
        return G_total/(1-G_total)

    def F_open_loop_sympy(self):
        """Exact transfer function of open loop with sympy"""
        # Define the transfer function expression
        F = self.sp_K / (1 + self.sp_s*self.sp_t_T) * sp.exp(-self.sp_s*self.sp_t_P)
        return F

    def F_closed_loop_sympy(self):
        """Exact transfer function of closed loop with sympy"""
        F = self.F_open_loop_sympy()
        # Closed-loop transfer function
        return F / (1 - F)

    def get_poles_zeros(self, open_loop=True):
        """Returns poles with sympy, including their real and imaginary parts"""
        if open_loop:
            F = self.F_open_loop_sympy()
        else:
            F = self.F_closed_loop_sympy()

        # Use sympy's fraction method to get the numerator and denominator of the expression
        numerator, denominator = sp.fraction(sp.simplify(F))
        
        # Find the poles and zeros by solving the denominator equation in terms of 's'
        poles = sp.solve(denominator, self.sp_s)
        zeros = sp.solve(numerator, self.sp_s)

        return poles, zeros

    def get_poles_no_tank(self, n:int):
        """Returns poles of system without tank"""
        n = np.arange(-n,n+1,1)
        if self.K >=0:
            return (np.log(self.K)+np.pi*(2*n+1)*1j)/self.t_P
        else:
            return (np.log(abs(self.K))+np.pi*2*n*1j)/self.t_P

    def evaluate_poles_zeros(self, poles, zeros, t_T):
        eval_poles = [pole.subs({self.sp_K: self.K, self.sp_t_T: t_T, self.sp_t_P: self.t_P}) for pole in poles]
        eval_zeros = [zero.subs({self.sp_K: self.K, self.sp_t_T: t_T, self.sp_t_P: self.t_P}) for zero in zeros]
        return eval_poles, eval_zeros

    def plot_poles_zeros(self, eval_poles, zeros):
        """Plots poles and zeros in the complex plane with matplotlib. Poles as black crosses, zeros as black circles"""
        
        # Extract real and imaginary parts of the poles/zeros
        poles_real_imag = [(sp.re(pole), sp.im(pole)) for pole in eval_poles]
        zeros_real_imag = [(sp.re(zero), sp.im(zero)) for zero in zeros]

        # Unpack poles and zeros into real and imaginary parts
        poles_real, poles_imag = zip(*poles_real_imag) if poles_real_imag else ([], [])
        zeros_real, zeros_imag = zip(*zeros_real_imag) if zeros_real_imag else ([], [])

        fig = plt.figure(figsize=(6, 6))
        
        # Plot zeros as 'o'
        plt.plot(zeros_real, zeros_imag, 'o', markersize=10, label='Zeros', color='black')
        
        # Plot poles as 'x'
        plt.plot(poles_real, poles_imag, 'x', markersize=12, label='Poles', color='black')
        
        plt.xlabel('Real part')
        plt.ylabel('Imaginary part')
        plt.title('Pole-Zero Plot')
        plt.grid(True)
        plt.axhline(y=0, color='k', linewidth=0.8)  # Horizontal axis
        plt.axvline(x=0, color='k', linewidth=0.8)  # Vertical axis
        plt.legend()

        return fig

    def get_phase_reserve(self, t_T:float):
        """returns phase reserve of system"""
        # Calculate frequency of norm = 1
        w = np.sqrt(self.K**2-1)/t_T
        phi = np.angle(self.F_open_loop(w,t_T), deg=True)
                
        # Get phase reserve
        if phi<0:
            phi_reserve = 180+phi
        else:
            phi_reserve = phi-180

        return phi_reserve
    
    def plot_custom_nyquists(self, t_T_list:List, open_loop=True):
        """self made nyquist plot of exact solution of system"""
        fig = plt.figure(figsize=(6, 6))
        plt.title(f'Nyquist Plot of DH-Pipe with Dampening Mixing Node')
        plt.xlabel('Real part')
        plt.ylabel('Imaginary part')
        plt.grid(True)
        plt.axhline(0, color='black', lw=0.5)
        plt.axvline(0, color='black', lw=0.5)
        # Define a seaborn color palette for the lines
        palette = sns.color_palette("husl", len(t_T_list))  # "husl" is just one of the many available palettes

        w = np.linspace(0,10000,int(5e6))
        def get_first_zero_idx(list:List):
            for i in range(len(list)):
                val = list[i]
                if -0.1 <= val <= 0.1:
                    return i
        for t_T, color in zip(t_T_list,palette):      
            if open_loop:
                F_w = self.F_open_loop(w, t_T)
            else:
                F_w = self.F_closed_loop(w, t_T)
            # Extract real and imaginary parts
            real_part = F_w.real
            imaginary_part = F_w.imag
            # Plot the Nyquist plot: Imaginary vs Real
            plt.plot(real_part, imaginary_part, color=color, label=f'{t_T/self.t_P}')
            # Choose a point for the arrow; for example, halfway along the line
            arrow_index = get_first_zero_idx(real_part)  # get an index close to first crossing of real axis
            plt.annotate('', xy=(real_part[arrow_index], imaginary_part[arrow_index]),
                         xytext=(real_part[arrow_index - 1], imaginary_part[arrow_index - 1]),
                         arrowprops=dict(arrowstyle="-|>,head_length=0.8,head_width=0.4", color=color, lw=1,
                                         connectionstyle="arc3"))
            plt.plot(real_part[0], imaginary_part[0], 'o', markersize=10, color=color)  # 'c' is the marker for a circle, markersize adjusts the size
        circle = plt.Circle((0, 0), 1, color='black', linewidth=0.5, fill=False)
        plt.gca().add_artist(circle)  # This line adds the circle to the current axes
        plt.legend(title='t_T/t_P')
        if open_loop:
            # To set a red cross at (0, -1)
            plt.plot(-1, 0, 'xr', markersize=10)  # 'x' is the marker for a cross, 'r' specifies red color, and markersize adjusts the size

        return fig

    def plot_ctrl_pckg_nyquists(self, t_T_list:List, open_loop=True):
        """nyquist plot (of system with Padé approximation) with control library"""
        
        fig = plt.figure(figsize=(6, 6))

        # Define a seaborn color palette for the lines
        palette = sns.color_palette("husl", len(t_T_list))  # "husl" is just one of the many available palettes

        # Create custom legend handles: these are simple lines with the desired colors
        handles = []
        # Create custom legend labels
        labels = []
        for t_T, color in zip(t_T_list,palette):
            
            # Use closed loop function if desired
            if open_loop:
                G_total = self.F_open_loop_approx(t_T)
            else:   
                G_total = self.F_closed_loop_approx(t_T)
            # Nyquist plot of the system with delay approximation
            ctl.nyquist(G_total, color=color, label=f'{t_T/self.t_P}')
            # Create custom legend handles: these are simple lines with the desired colors
            handles.append(plt.Line2D([0], [0], color=color, linewidth=2, linestyle='-'))
            # Create custom legend labels
            labels.append(f'{t_T/self.t_P}')
        
        # Add the legend using the custom handles, and the labels corresponding to your systems
        plt.legend(handles, labels, title='t_T/t_P')
        plt.title(f'Nyquist Plot of DH-Pipe with Dampening Mixing Node')

    def plot_ctrl_pckg_bodes(self, t_T_list:List, open_loop=True):
        """Bode plots with control library (system with Padé approximation)"""
        
        fig = plt.figure(figsize=(6, 6))

        # Define a seaborn color palette for the lines
        palette = sns.color_palette("husl", len(t_T_list))  # "husl" is just one of the many available palettes
        # Create custom legend handles: these are simple lines with the desired colors
        handles = []
        # Create custom legend labels
        labels = []
        for t_T, color in zip(t_T_list,palette):
            # Use closed loop function if desired
            if open_loop:
                G_total = self.F_open_loop_approx(t_T)
            else:   
                G_total = self.F_closed_loop_approx(t_T)
            # Bode plot of the system with delay approximation
            ctl.bode(G_total, color=color, label=f'{t_T/self.t_P}')
            # Create custom legend handles: these are simple lines with the desired colors
            handles.append(plt.Line2D([0], [0], color=color, linewidth=2, linestyle='-'))
            # Create custom legend labels
            labels.append(f'{t_T/self.t_P}')
        
        # Add the legend using the custom handles, and the labels corresponding to your systems
        plt.legend(handles, labels, title='t_T/t_P')
        plt.title(f'Bode Plot of DH-Pipe with Dampening Mixing Node')

    def plot_ctrl_pckg_pole_zero(self, t_T:float):
        """pole zero diagram of closed loop with control package (pade-approximation of latency)"""
        G_total = self.F_closed_loop_approx(t_T)
        ratio = round(t_T/self.t_P, 1)
        ctl.pzmap(G_total, Plot=True, title=f't_T/t_P = {ratio}')

    def plot_ctrl_pckg_pole_zero_grid(self, t_T_list:List[float], approx_orders:List[int], xlim:tuple()=(), ylim:tuple()=()):
        """A grid of pole-zero plots. Different tank sizes in rows, Padé approximation orders in columns."""
        # number of rows and columns
        J = len(approx_orders)
        I = len(t_T_list)

        # define responsive figsize
        x_size = min(2+J*4/2.54, custom_style['figure.figsize'][0])
        y_size = min((4+I*1)/2.54, custom_style['figure.figsize'][1])
        figsize= (x_size, y_size)
        
        # Create the figure and a grid of subplots
        fig, axs = plt.subplots(I, J, sharex=True, sharey=True, figsize=figsize)

        # Loop over the parameters to create each subplot
        for i, t_T in enumerate(t_T_list):
            for j, ao in enumerate(approx_orders):
                self.approximation_order = ao
                G_total = self.F_closed_loop_approx(t_T)
                ax = axs.flat[i*J+j]
                # Compute poles and zeros
                poles = ctl.pole(G_total)
                zeros = ctl.zero(G_total)
                
                # Plot poles and zeros manually on the provided axes
                ax.scatter(np.real(zeros), np.imag(zeros), marker='o', color='blue', label='Zeros')
                ax.scatter(np.real(poles), np.imag(poles), marker='x', color='red', label='Poles')
                
                # ax.set_Pitle(f'p1: {t_T}, p2: {ao}', fontsize=10)
                ax.axhline(0, color='black', lw=0.5)
                ax.axvline(0, color='black', lw=0.5)
                ax.grid(True)

                # Inside your plotting function:
                ax.xaxis.set_major_formatter(ScalarFormatter(useMathText=True))
                ax.xaxis.get_major_formatter().set_powerlimits((-3, 3))

                ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
                ax.yaxis.get_major_formatter().set_powerlimits((-3, 3))

                # Avoid placing the offset (e.g., 1e3) in the middle of the plot
                ax.get_xaxis().get_offset_text().set_position((0.9, 0.2))  # Move it to the end of the axis
                ax.get_yaxis().get_offset_text().set_position((-0.385, 0.6))  # Move it to the end of the axis

        # Add one legend at the lower left corner        
        axs.flat[J*I-J].legend(loc='upper right', handletextpad=0.1, borderpad=0.2, bbox_to_anchor=(-0.1/x_size, -0.1/x_size))

        # Add column titles and axis labels for the approximation order
        if J>1:
            fig.text(0.582, 0.97, 'Order of Padé approximation', ha='center', va='center', fontsize=10, weight='bold')
            for j, ao in enumerate(approx_orders):
                axs.flat[j].annotate(str(ao), xy=(0.5, 1.1), xycoords='axes fraction',
                                ha='center', fontsize=9, weight='bold')
        for j, ao in enumerate(approx_orders):
            axs.flat[-(j+1)].set_xlabel('Re', loc='left')

        # Add row titles and axis labels for t_T/t_P
        for i, t_T in enumerate(t_T_list):
            axs.flat[i*J].set_ylabel('$\\mathbf{'+str(round(t_T/self.t_P,3))+'}$ \n Im')

        fig.text(0.11/x_size, 0.5, r'Time constant ratio $\vartheta$', ha='center', va='center', fontsize=10, weight='bold', rotation='vertical')

        # Overall annotation
        if J == 1:
            text = f'Gain K = {round(self.K, 3)}, Padé appr. order: {approx_orders[0]}'
            x = 0.5
            ha = 'center'
        else:
            text = f'Gain $K$ = {round(self.K, 3)}'
            x = 0.2/x_size #0.2 inch fix
            ha = 'left'
        fig.text(x, 0.97, text, ha=ha, va='center', fontsize=14, weight='bold')

        # Adjust layout to prevent overlap
        plt.tight_layout(rect=[0.05, 0.05, 1, 0.96], pad=0.4)
        plt.subplots_adjust(wspace=0.03, hspace=0.1, left=1.1/x_size, bottom=0.5/y_size, right=1-0.14/x_size)#, right=1, top=1, bottom=0)

        # Set the same x and y axis limits for all subplots
        if xlim:
            for ax in axs.flat:
                ax.set_xlim(xlim[0], xlim[1])  # Set x-axis limits
        if ylim:
            for ax in axs.flat:
                ax.set_ylim(ylim[0], ylim[1])  # Set x-axis limits

        return fig, axs
    
def plot_poles_zeros_K_comparison_no_tank(system_params:List[dict], approx_orders:List[int]=[], xlim:tuple()=(), ylim:tuple()=(), n_poles_exact=10):
    """Plots poles (and zeros) for a system with pipe and without tank. Visualizes the differences for different degrees of Padé approximation and the exact solution.
    Makes a subplots with different systems (-->K-values) in the rows i and approximation orders j in columns.
    Plots the exact solution for an element j in approx_orders[j] in (np.inf, 'exact').
    The K-values K[i] come from system_params[i].

    :param system_params: Parameters (temperatures, pipe geometry, thermal powers) of different systems to show.
    :type system_params: List[dict]
    :param approx_orders: orders of Padé-approximation. An element j of approx_orders[j] in (np.inf, 'exact') makes the exact poles. defaults to []
    :type approx_orders: List[int], optional
    :param xlim: limits for x-axes, tuple of (xmin,xmax). None cause automatic scaling, defaults to ()
    :type xlim: tuple, optional
    :param ylim: limits for y-axes, tuple of (ymin,ymax). None cause automatic scaling, defaults to ()
    :type ylim: tuple, optional
    :param n_poles_exact: number of poles to plot for exact solution, defaults to 10
    :type n_poles_exact: int, optional
    :return: fig, axes
    :rtype: tuple
    """

    # number of rows and columns
    J = len(approx_orders)
    I = len(system_params)
       
    # define responsive figsize
    x_size = min(2+J*1.6/2.54, custom_style['figure.figsize'][0])
    y_size = min((4+I*1)/2.54, custom_style['figure.figsize'][1])
    figsize= (x_size, y_size)

    # Create the figure and a grid of subplots
    fig, axs = plt.subplots(len(system_params), len(approx_orders), sharex=True, sharey=True, figsize=figsize)

    # create a list of all systems to plot
    systems = [ControlStabilityAnalyzor(**sp) for sp in system_params]
    
    # Loop over the parameters to create each subplot
    for i, s in enumerate(systems):
        for j, ao in enumerate(approx_orders):
            ax = axs.flat[i*J+j]
            # get poles for exact solution
            if ao == np.inf or ao == 'exact':
                # Compute poles 
                poles = s.get_poles_no_tank(n_poles_exact)
            # get poles for approximations
            else:
                s.approximation_order = ao
                G_total = s.F_closed_loop_approx(0)
                # Compute poles and zeros
                poles = ctl.pole(G_total)
                zeros = ctl.zero(G_total)
                # Plot zeros manually on the provided axes
                ax.scatter(np.real(zeros), np.imag(zeros), marker='o', color='blue', label='Zeros')
            
                            
            # Plot poles manually on the provided axes
            ax.scatter(np.real(poles), np.imag(poles), marker='x', color='red', label='Poles')
            
            # ax.set_Pitle(f'p1: {t_T}, p2: {ao}', fontsize=10)
            ax.axhline(0, color='black', lw=0.5)
            ax.axvline(0, color='black', lw=0.5)
            ax.grid(True)

            # Inside your plotting function:
            ax.xaxis.set_major_formatter(ScalarFormatter(useMathText=True))
            ax.xaxis.get_major_formatter().set_powerlimits((-3, 3))

            ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
            ax.yaxis.get_major_formatter().set_powerlimits((-3, 3))

            # Avoid placing the offset (e.g., 1e3) in the middle of the plot
            ax.get_xaxis().get_offset_text().set_position((0.9, 0.2))  # Move it to the end of the axis
            ax.get_yaxis().get_offset_text().set_position((-0.385, 0.6))  # Move it to the end of the axis

    # Add one legend at the lower left corner        
        axs.flat[J*I-J].legend(loc='upper right', handletextpad=0.1, borderpad=0.2, bbox_to_anchor=(-0.1/x_size, -0.1/x_size))

    # Add column titles and axis labels for the approximation order
    for j, ao in enumerate(approx_orders):
        fig.text(0.582, 0.97, 'Time delay representation', ha='center', va='center', fontsize=10, weight='bold')
        if ao == np.inf or ao == 'exact':
            axs[0, j].annotate(str(ao), xy=(0.5, 1.1), xycoords='axes fraction',
                            ha='center', fontsize=9, weight='bold')
        else:
            axs[0, j].annotate(f'Padé, order: {ao}', xy=(0.5, 1.1), xycoords='axes fraction',
                            ha='center', fontsize=9, weight='bold')
        axs.flat[-(j+1)].set_xlabel('Re', loc='left')

    # Add row titles and axis labels for t_T/t_P
    fig.text(0.11/x_size, 0.5, r'Overall gain $K$', ha='center', va='center', fontsize=10, weight='bold', rotation='vertical')
    for i, s in enumerate(systems):
        axs.flat[i*J].set_ylabel('$\\mathbf{'+str(round(s.K,3))+'}$ \n Im', loc='center')

    # general annotation of tank size 0
    fig.text(0.2/x_size, 0.96, f'$t_T$ = {0}', ha='left', va='center', fontsize=14, weight='bold')

    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0.05, 0.05, 1, 0.96], pad=0.4)
    plt.subplots_adjust(wspace=0.03, hspace=0.1, left=1.1/x_size, bottom=0.5/y_size, right=1-0.14/x_size)#, right=1, top=1, bottom=0)

    # Set the same x and y axis limits for all subplots
    if xlim:
        for ax in axs.flat:
            ax.set_xlim(xlim[0], xlim[1])  # Set x-axis limits
    if ylim:
        for ax in axs.flat:
            ax.set_ylim(ylim[0], ylim[1])  # Set x-axis limits

    return fig, axs

def get_ctrl_pckg_tank_stabilization_curve(K_range:List[float], time_constant_ratio_stepsize:float=0.01, approx_order:int=None)->list:
    """This function return the curve of time constant ratio t_T/t_P that stabilizes a system over K.
    Uses a simple iterative approach by successively increasing the time-constant-ratio and storing the value
    for which all poles crossed the imaginary axis. Might run forever if no such point is found!

    :param K_range: Values for overall gain K for which the curve is returned
    :type K_range: List[float]
    :param time_constant_ratio_stepsize: stepsize of t_T/t_P by which the algorithm procedes.
    :type time_constant_ratio_stepsize: float
    :param approx_order: order of Padé approximation. Defaults to value of class instance.
    :type approx_order: int, optional
    :return: values of t_T/t_P that stabilizes operation.
    :rtype: list
    """
    
    if approx_order is not None:
        assert(isinstance(approx_order, int))
    assert(all(K <= -1 for K in K_range))
    
    time_constant_ratio = 0
    stable_time_constant_ratios = list()
    for K in K_range:
        # Compute T_S to get the wanted K 
        T_R  = 40
        T_C0 = 70
        T_S  = T_C0 - K*(T_C0 - T_R)
        # Set up system params (most are arbitrary)
        system_params = dict(
                    L = 750,
                    d = 0.3,
                    Qf_C = 4e6,
                    T_R     = T_R,
                    T_C0    = T_C0,
                    T_S = T_S,
                    T_D     = 50,
                )
        # initialize system with K
        system = ControlStabilityAnalyzor(**system_params)

        # loop over time_constants
        # time_constant_ratio = 0 #no re-initialisation needed: Curve is monotinocally increasing
        biggest_real_part_of_poles = 1
        while biggest_real_part_of_poles > 0: #abort when all poles are in left half-plane
            # get t_T according to wanted ratio
            t_T = system.t_P * time_constant_ratio 
            # get closed-loop transfer function
            F_total = system.F_closed_loop_approx(t_T)
            # get poles
            poles = ctl.pole(F_total)
            # get real part of poles
            real_part_of_poles = np.real(poles)
            # get the real value of rightmost pole
            biggest_real_part_of_poles = max(real_part_of_poles)
            # update time constant ratio for next run of loop
            time_constant_ratio += time_constant_ratio_stepsize
        # save stabilizing time constant ratio
        stable_time_constant_ratios.append(time_constant_ratio)

    return stable_time_constant_ratios

def plot_inertia_stabilization_curve(Ks:np.ndarray, K_bend_limit=-1.5) -> plt.Figure:
    """This function plots a curve of the ratio of time constants (inertia and transport delay, vartheta=t_T/t_P) that stabilizes a system with gain K.
    Also computes and draws a linear approximation (regression) of that curve.

    :param Ks: Values for gain K to plot the curve for. All values must be <1.
    :type Ks: np.ndarray
    :param K_bend_limit: From where we think the bent start of the curve ends and the curve proceeds as a straight line, defaults to -1.5. Important to filter values for linear regression.
    :type K_bend_limit: float, optional
    :return: Figure
    :rtype: plt.Figure
    """
   
    assert all(K<=-1 for K in Ks)

    Ks_bent = Ks[Ks>=K_bend_limit]
    Ks_straight = Ks[Ks<K_bend_limit]

    # copmpute stabilizing time-constant-ratio
    # only the first values where curve is bent until ~ -1.5
    stabilizing_t_ratios = get_ctrl_pckg_tank_stabilization_curve(Ks_bent, approx_order=10) 
    # rest of curve
    stabilizing_t_ratios_end = get_ctrl_pckg_tank_stabilization_curve(Ks_straight, approx_order=10) 
    stabilizing_t_ratios.extend(stabilizing_t_ratios_end)

    # parameters of linear line
    import scipy.stats as stats
    slope, intercept, r_value, p_value, std_err = stats.linregress(Ks_straight, stabilizing_t_ratios_end)
    print(f'linear curve between K and stabilizing vartheta=t_T/t_P has parameters: slope = {slope}, intercept = {intercept}, r_value = {r_value}, p_value = {p_value}, std_err = {std_err}')
    # we get linear curve between K and stabilizing t_T/t_P has parameters: slope = -0.6502201888931962, intercept = -0.462503369540342, r_value = -0.9999287795236964, p_value = 8.145906379121496e-177, std_err = 0.0008135427655268334
    slope = np.round(slope, 2)
    intercept = np.round(intercept, 2)

    # approximated linear curve
    approx = (Ks*slope+intercept)

    # merge data
    data = np.array([stabilizing_t_ratios, approx]).transpose()
    columns = ["exact", "approximate"]
    data = pd.DataFrame(data, Ks, columns)
 
    # plot
    fig = plt.figure(figsize=(custom_style['figure.figsize'][0], 2))
    ax = sns.lineplot(data=data )

    # Set title and labels directly
    ax.set_title("Stabilizing DHS with Thermal Inertia", fontsize=12, weight='bold')
    ax.set_xlabel("Overall gain $K$")
    ax.set_ylabel("Stabilizing time \n constant ratio $\\vartheta_\mathrm{ntr}$")
    
    # Set x-ticks with a spacing of 1 
    ax.set_xticks(np.arange(np.floor(min(Ks)), np.ceil(max(Ks)) + 0.5, 0.5))
    ax.set_yticks(np.arange(np.floor(min(stabilizing_t_ratios)), np.ceil(max(stabilizing_t_ratios)) + 0.5, 0.5))
    fig.tight_layout()
    
    return fig

def plot_Ttable_region(figsize:Tuple[int], limits_K_theta:Tuple[float]=(-6,2,-1,4), theta_ntr_slope:float=-0.65, theta_ntr_intercept:float=-0.46)->plt.Figure:
    """This function makes a plot of the stable region, visualizing the control law. 

    :param figsize: size of the figure, (width,height), in inch.
    :type figsize: Tuple[int]
    :param limits_K_theta: axes limits (x=K, y=vartheta), defaults to (-6,2,-1,4)
    :type limits_K_theta: Tuple[float], optional
    :param theta_ntr_slope: slope of approximated stabilizing time-constant-ratio vartheta, defaults to -0.65
    :type theta_ntr_slope: float, optional
    :param theta_ntr_intercept: intercept of approximated stabilizing time-constant-ratio vartheta, defaults to -0.46
    :type theta_ntr_intercept: float, optional
    :return: Figure
    :rtype: plt.Figure
    """
    # make this plot and this one only in white style
    with sns.axes_style('whitegrid', {'font.family': 'Charter'}): 
        fig = plt.figure(figsize=figsize)
        K_min, K_max, theta_min, theta_max = limits_K_theta
        theta_ntr = lambda K: K*theta_ntr_slope+theta_ntr_intercept
        
        # line of theta_ntr
        theta_ntr_line = theta_ntr(np.array([K_min, -1]))
        sns.lineplot([K_min, -1], theta_ntr_line)
        # vertical line for K=-1
        sns.lineplot([-1, -1], [0, theta_ntr(-1)])
        # vertical line for K=1
        ax = sns.lineplot([1, 1], [0, theta_max])

        # make lines for the axes
        ax.axhline(y=0, color='black')
        ax.axvline(x=0, color='black')

        # fill instable region with red
        plt.fill_between([K_min, -1], theta_ntr_line, 0, color='red', alpha=.5, hatch='/')
        # fill stable region with green
        stable_region_bottom_line = list(theta_ntr_line)
        stable_region_bottom_line.extend([0,0])
        plt.fill_between([K_min, -1, -1, +1], stable_region_bottom_line, theta_max, color='green', alpha=.5, hatch='\\')
        # fill impossible regions with grey
        plt.fill_between([K_min, 1, 1, K_max], [0, 0, theta_max, theta_max], theta_min, color='slategrey', alpha=0.2, hatch='x')

        # annotate regions
        bbox_dict = dict(facecolor='white', edgecolor='none', pad=2)
        plt.annotate('stable set-\npoints $\Sigma$',      (K_min+(1-K_min)/2, theta_max-1.5),         color='green',      fontsize=9, fontweight='bold', bbox=bbox_dict, ha='center',  va='center',)
        plt.annotate('unstable\nset-points',    (-1+2*(K_min+1)/3, theta_ntr_line[0]/3),    color='red',        fontsize=9, fontweight='bold', bbox=bbox_dict, ha='center',  va='center',)
        plt.annotate('unfeasible\nset-points',  (1.5, -0.5),                                color='slategrey',  fontsize=9, fontweight='bold', bbox=bbox_dict, ha='center',  va='center',)

        # annotate line for neutral stability
        plt.annotate('$\\vartheta_\mathrm{ntr} (K)$',      (-2.5, theta_ntr(-2.5)), rotation=np.rad2deg(np.arctan(theta_ntr_slope)),         color='black',      fontsize=9, fontweight='bold', bbox=bbox_dict, ha='center',  va='center',)


        # annotate axis directions
        plt.annotate('bigger\ninertia', rotation=90, ha='right',  va='center', xy=(-1.25, 3), fontweight='bold', bbox=bbox_dict)
        arrow = mpatches.FancyArrowPatch((-1.1, 2.25), (-1.1, 3.75), shrinkA=4, shrinkB=4, mutation_scale=20)
        ax.add_patch(arrow)
        plt.annotate('bigger\ndelay', rotation=90, ha='right',  va='center', xy=(-1.25, 1.5), fontweight='bold', bbox=bbox_dict)
        arrow = mpatches.FancyArrowPatch((-1.1, 2.25), (-1.1, 0.75), shrinkA=4, shrinkB=4, mutation_scale=20)
        ax.add_patch(arrow)

        plt.annotate('cold slack', ha='center',  va='bottom', xy=(0.5, 0.7), fontweight='bold', bbox=bbox_dict)
        arrow = mpatches.FancyArrowPatch((0, 0.35), (-1, 0.35), shrinkA=4, shrinkB=4, mutation_scale=20)
        ax.add_patch(arrow)
        plt.annotate('hot slack', ha='center',  va='bottom', xy=(-0.5, 0.7), fontweight='bold', bbox=bbox_dict)
        arrow = mpatches.FancyArrowPatch((0, 0.35), (1, 0.35), shrinkA=4, shrinkB=4, mutation_scale=20)
        ax.add_patch(arrow)

        # Set title and labels directly
        ax.set_title("Visualized Control Law", fontsize=12, weight='bold')
        ax.set_xlabel("Overall gain $K$")
        ax.set_ylabel("Time constant ratio $\\vartheta$")
        
        # Set x-ticks with a spacing of 1
        ax.set_xticks(np.arange(K_min, K_max + 1, 1))
        ax.set_yticks(np.arange(theta_min, theta_max + 1, 1))
        fig.tight_layout()

    return fig




if __name__ == '__main__':
    
    figure_name = 'stable_but_not_good'
    
    L = 750
    d = 0.3
    Qf_C = 4e6
    param_collection = {
        'very_unstable': # very unstable K=-3.6144 
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 178.43,
                T_D     = 50,
            ), #Qf_D/Qf_C = 0,25
        
        'unstable': # unstable K=-2
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 130,
                T_D     = 50,
            ), #Qf_D/Qf_C = 0,25
        'indifferent': # indifferent K=-1
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 100,
                T_D     = 50,
            ), #Qf_D/Qf_C = 0,2
        'slightly_stable': # sligthly stable K=-0.933
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 98,
                T_D     = 50,
            ), #Qf_D/Qf_C = 0,2
        'stable_but_not_good': # stable (K = - 0,67), but small heat pump (here: deployed supplier) production share
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 90,
                T_D     = 50,
            ), #Qf_D/Qf_C = 0.16
        'stable': # stable (K = + 0,67), with big heat pump (here: slack supplier) production share
            dict(
                L = L,
                d = d,
                Qf_C = Qf_C,
                T_R     = 40,
                T_C0    = 70,
                T_S     = 50,
                T_D     = 130,
            ), #Qf_D/Qf_C = 0.67
        }

    # Folder to store figures
    fig_folder = os.path.abspath(__file__+'/../../figures')

    # order of Padé approximation
    approx_order = 10

    # Fig. 4: Pole-zero plot of system without tank, comparison of K-values, validation of Padé approximation
    param_list = list(param_collection.values())
    # exclude K=0.677 as we don't need it: stable anyways
    del param_list[-1]
    approx_orders = [approx_order, 'exact'] # to plot exact solution, too!
    for xlim, ylim, filename in zip([(-0.014, 0.014),(-0.0012, 0.0012)] , [(), ()], ['Fig_4a_macro', 'Fig_4b_micro']):
        plot_poles_zeros_K_comparison_no_tank(param_list, approx_orders, xlim, ylim, 10)
        filename = f'{filename}_pole_zero_plot_without_tank.pdf'
        path = os.path.join(fig_folder,filename)
        plt.savefig(path)
    
    # Fig. 5: Pole-zero plots of system with tank and pipe
    figure_name = 'pole_zero_plot_with_delay_and_tank.pdf'
    sys_params = param_collection['unstable']
    print(f'\n################\n {figure_name}\n################\n')
    system = ControlStabilityAnalyzor(**sys_params)
    t_T_array = np.array([0, 0.1, 0.4, 0.86, 1.5, 3, 1000])*system.t_P
    volumes = {t_T/system.t_P:system.get_V_tank(t_T) for t_T in t_T_array}
    print(f'tank volumes: {volumes}')
    for xlim, ylim, sub_fig_name in zip([(-0.014, 0.014),(-0.0016, 0.0016)] , [(), ()], ['Fig_5a_macro', 'Fig_5b_micro']):
        system.plot_ctrl_pckg_pole_zero_grid(t_T_array, [approx_order], xlim, ylim)
        filename = f'{sub_fig_name}_{figure_name}.pdf'
        path = os.path.join(fig_folder,filename)
        plt.savefig(path)
        print(filename)

    # Fig. 6: Visualize vartheta
    Ks = np.linspace(-1, -5, 200)
    plot_inertia_stabilization_curve(Ks)
    path = os.path.join(fig_folder,'Fig_6_stabilizing_with_thermal_inertia.pdf')
    plt.savefig(path)

    # Fig. 10: visualize the control law
    plot_Ttable_region((custom_style['figure.figsize'][0], 3))
    path = os.path.join(fig_folder,'Fig_10_stability_control_law_regions.png') # png needed! Missing hatches with pdf!
    plt.savefig(path, dpi=900)
    
    plt.show()

    # additional plots
    system.plot_ctrl_pckg_bodes(t_T_array)
    system.plot_ctrl_pckg_nyquists(t_T_array)
    system.plot_custom_nyquists(t_T_array)

    plt.show()
    
   



# Project Title
Thermo-Hydraulic Oscillations in District Heating Systems: Stability Analysis

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)

## Introduction

This repository contains the code used in the scientific publication (open access)

Friedrich, P.; Kuroptev, K.; Huynh, T.; Niessen, S. Stability Analysis and Mitigation of Thermo-Hydraulic Oscillations in Multi-Supplier District Heating Systems. Energies 2025, 18, 1126. https://doi.org/10.3390/en18051126 

The paper discusses thermo-hydraulic asymptotic stability of a two-supplier district heating system (DHS) with transport delay in the network. The work first employs pole-zero analysis of the linearized state equations using Python, and then performs simulations of the original, non-linear system model using Modelica, by the aid of this code.

## Installation
1. Clone the repository:
```bash
 git clone https://github.com/EINS-TUDa/DHS-ThermoHydraulicOscillations.git
```

2. Install all eight required packages with pip, if neccessary:
```bash
 pip install -r requirements.txt
 ```

Python 3.7.4. was used in this work.

## Usage
Figs. 4, 5, 6, 10 shown in the stability analysis in Secs. 3 were made with the Python-script found in "code/stability_analysis.py". The script helps analyzing the stability of a two-supplier DHS with transport delay in the network, using a Laplace-transform of the linearized system model. To reproduce the results shown in the paper, the script simply needs to be executed. The used system parameters to are found in the script.

An applicable and updated version of the Modelica model which was used to validate the analytical findings (Sec. 4) is found in code/validation_model_total.mo. The model represents the original, non-linear, delayed two-supplier DHS and allows the simulation of the time-dependent system state. In the paper, Dassault Systèmes Dymola was used for execution, but OpenModelica (https://openmodelica.org/) might be an open-source alternative (not tested). The parameters required to reproduce the results are found in the model and in simulation_parameters.pdf.

Simulation results presented in the paper are stored in folder simulation_results as .mat-files exported from Dymola. The Python.script to load the data and produce Figs. 7, 8, 9 is found in code/modelica_plots.py. The plots are produced by simple execution of the script.

## License
This project is licensed under creative commons (CC-BY).

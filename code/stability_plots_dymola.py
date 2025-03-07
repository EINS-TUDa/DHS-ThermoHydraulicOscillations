"""
Script Name: stability_plots_modelica.py
Description: Makes plots of dynamic simulations (Figs. 7, 8, 9) shown in the paper 

P. Friedrich, et. al. "Stability Analysis and Mitigation of Thermo-Hydraulic Oscillations in Multi-Supplier District Heating Systems", 
MDPI Energies 2025, https://doi.org/10.3390/en18051126. 

Requires simulation results as .mat files in the folder "data_root" (see __main__).
Note that to load the simulation results shown in the paper, list var_names_paper_results is used (line 426).
To load simulation results of the published Modelica-model, list var_names_published_modelica_model needs to be used in line 426, as path data_root needs to be updated.

Author: Pascal Friedrich, Thanh Huynh
Date: 2025-03-06
Version: 2.0
License: Creative commons CC-BY
"""

import os
import DyMat
import pandas as pd
from typing import Callable, List, Dict
from matplotlib import pyplot as plt
import seaborn as sns

# define seaborn style with rc-Dictionary
custom_style = {
    'font.family': 'Charter',
}

# set plot style with seaborn
sns.set_style("darkgrid")  # Hintergrundstil setzen (z.B., 'white', 'dark', 'whitegrid', 'darkgrid')
sns.set_context("paper", rc=custom_style)
plt.rcParams.update(custom_style)

var_names_published_modelica_model = [
    dict(
        dym_var_name='S_slack_supplier.Q_gen',
        plot_var_name=r'$\dot{Q}_S$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='D_deployed_supplier.out_Q_consume',
        plot_var_name=r'$\dot{Q}_C$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='C_consumer.Q_gen',
        plot_var_name=r'$\dot{Q}_D$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='S_slack_supplier.T_b',
        plot_var_name=r'$T_S$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
    dict(
        dym_var_name='C_consumer.T_a',
        plot_var_name=r'$T_C$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
    dict(
        dym_var_name='D_deployed_supplier.T_b',
        plot_var_name=r'$T_D$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
]

var_names_paper_results = [
    dict(
        dym_var_name='producer_pT.Q_gen',
        plot_var_name=r'$\dot{Q}_S$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='consumerSimple.out_Q_consume',
        plot_var_name=r'$\dot{Q}_C$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='producer_QT_eq.Q_gen',
        plot_var_name=r'$\dot{Q}_D$',
        conversion_factor=1e-6,
        offset=0,
        group=r'$\dot{Q}$ (MW)'
    ),
    dict(
        dym_var_name='producer_pT.T_b',
        plot_var_name=r'$T_S$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
    dict(
        dym_var_name='consumerSimple.T_a',
        plot_var_name=r'$T_C$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
    dict(
        dym_var_name='producer_QT_eq.T_b',
        plot_var_name=r'$T_D$',
        conversion_factor=1.0,
        offset=-273.15,
        group=r'$T$ (°C)'
    ),
]

def load_data(mat_files: List[os.PathLike], var_names:List[Dict]) -> pd.DataFrame:
    """This function loads dymola data into a DataFrame. The columns are multi-indexed. 
    First level: File name, second level: dymola variable name.
    
    :param mat_files: mat files to load
    :type mat_files: List[os.PathLike]
    :return: data
    :rtype: pd.DataFrame
    """

    data_frames = []
    for mat_file in mat_files:
        mat_data = DyMat.DyMatFile(mat_file)
        file_name = os.path.basename(mat_file)
        
        file_data = {}
        for var_name in var_names:
            dym_var_name = var_name['dym_var_name']
            group = var_name['group']
            if dym_var_name in mat_data.names():
                time = mat_data.abscissa(dym_var_name)
                values = mat_data.data(dym_var_name)
                series = pd.Series(data=values, index=time[0])
                # index may contain duplicated values: Keep only the first occurrence of each index
                series = series[~series.index.duplicated(keep='first')]
                file_data[(group, file_name, dym_var_name)] = series
        
        if file_data:
            df_file = pd.DataFrame(file_data)
            data_frames.append(df_file)
    
    # Combine all the DataFrames into a single DataFrame
    df = pd.concat(data_frames, axis=1)
    
    # Set the column names as a MultiIndex
    df.columns = pd.MultiIndex.from_tuples(df.columns, names=['Group', "File", "Variable"])

    # fill gaps
    df = df.interpolate(method='linear')
    
    return df


def convert_data(df: pd.DataFrame, var_names:List[dict]) -> pd.DataFrame:
    """This function converts the data according to the var_names list of dicts.
    
    :param df: The dataframe containing simulation data, with Modelica variable names
    :type df: pd.DataFrame
    :param var_names: a list of dictionaries mapping variable names of Modelica to what is shown in the plots.
    :rtype: List[dict]
    :return: The dataframe containing simulation data, with plot variable names
    :rtype: pd.DataFrame
    """
    
    for var in var_names:
        dym_var_name = var['dym_var_name']
        plot_var_name = var['plot_var_name']
        conversion_factor = var['conversion_factor']
        offset = var['offset']
        
        # Rename columns in the second level of multi-index
        df.rename(columns={dym_var_name: plot_var_name}, level=2, inplace=True)
        
        # Apply conversion factor and offset
        df.loc[:, (slice(None),slice(None), plot_var_name)] = df.loc[:, (slice(None),slice(None), plot_var_name)] * conversion_factor + offset
    
    # convert index to hours
    diff_index = df.index[-1] - df.index[0]
    if diff_index > 2*3600:
        df.index = df.index / 3600
        df.index.name = 'Time (h)'
    elif diff_index > 600:
        df.index = df.index / 60
        df.index.name = 'Time (min)'
    else:
        df.index.name = 'Time (s)'
    
    return df


def get_K_value_title(file_path: os.PathLike) -> float:
    """This function returns a string to describe K-values from the path.

    :param file_path: _description_
    :type file_path: os.PathLike
    :return: title for a plot describing K
    :rtype: str
    """
    
    base_name = os.path.basename(file_path)
    k_value = base_name.split('_')[1].replace(',', '.')
    return float(k_value) # using a simple float value sorts the plots...which is very usefull f'$K={k_value}$'

def get_switch_titles(file_path: os.PathLike) -> float:
    """This function returns a string to describe K-values from the path.

    :param file_path: _description_
    :type file_path: os.PathLike
    :return: title for a plot describing K
    :rtype: str
    """
    
    base_name = os.path.basename(file_path)
    k_value = base_name.split('_')[1].replace(',', '.')
    if 'low' in base_name:
        return 'b) Lowered $T_S$ \n ($K=−0.667$)'  # using a simple float value sorts the plots...which is very usefull f'$K={k_value}$'
    elif 'switch' in base_name:
        return 'd) Slack switch \n ($K=0.667$)' # using a simple float value sorts the plots...which is very usefull f'$K={k_value}$'
    elif base_name == 'unstable_3.mat':
        return 'c) Tank at \n $\\vartheta=3$'
    elif 'K_-2_unstable_no_tank.mat' in base_name:
        return 'a) Initial \n ($K=−2$)'
    else:
        raise ValueError('unknown file name')

def get_time_constant_ratio(file_path: os.PathLike) -> float:
    """This function returns the ratio of time constants from tank (PT1) and pipe (delay) t_T/t_P resp. t_s/t_t.

    :param file_paths: _description_
    :type file_paths: List[os.PathLike]
    :return: tank volume
    :rtype: str
    """

    base_name = os.path.basename(file_path)
    tcr = base_name.split('.')[0].split('_')[-1].replace(',', '.')
    return float(tcr) # using a simple float value sorts the plots...which is very usefull f'$t_T/t_P={tcr}$'

def dymola_multi_plot_file_rows(df: pd.DataFrame, suptitle:str='', row_explainer:str='', column_explainer:str='', column_titles:List[str]=[], one_legend_only:bool=False, ylims=List[tuple]):
    """This function creates line-subplots of a multi-indexed dymola DataFrame, for time series or similar. 
    The data frame has three column index levels:
    1st level: columns of subplots
    2nd level: rows of subplots
    3rd level: individual variables

    :param df: data
    :type df: pd.DataFrame
    """
    
    # Create subplots
    num_rows = len(df.columns.levels[1])
    num_cols = len(df.columns.levels[0])
    
    fig, axes = plt.subplots(num_rows, num_cols, sharex=True, figsize=(0.9*21/2.54, 1*num_rows+0.5))

    # Initialize lists to store min and max values for each variable (column)
    y_min = [float('inf')] * num_cols
    y_max = [float('-inf')] * num_cols
        
    # First, determine the common y-limits for each variable across all files
    for i, col_group in enumerate(df.columns.levels[0]):
        
        # check if static limits are given
        if ylims:
            if ylims[i]:
                y_min[i] = ylims[i][0]
                y_max[i] = ylims[i][1]
                continue # next iteration
        # else: set in min/max
        series = df[col_group]
        y_min[i] = series.min().min()
        y_max[i] = series.max().max()
    
    # Then, plot the data with consistent y-axis scaling
    for j, col_group in enumerate(df.columns.levels[0]):        
        for i, row_group in enumerate(df.columns.levels[1]):
            ax = axes[i, j] if num_rows > 1 else axes[j]
            ax.plot(df[col_group, row_group], label=df[col_group, row_group].columns) 
            ncols = df[col_group, row_group].columns.size
            ax.set_ylabel(col_group)            
            # Set the y-limits based on the previously determined common limits
            ax.set_ylim(y_min[j]-0.05*abs(y_min[j]), y_max[j]+0.05*abs(y_max[j]))
        
    # set row and column labels
    for i, row_group in enumerate(df.columns.levels[1]):
        row_str = str(row_group)
        # replace hyphens by minus
        row_str = row_str.replace("\u2010", "\u2212").replace("\u2011", "\u2212") 
        axes[i,0].annotate(row_str, xy=(-0.25, 0.5), xycoords='axes fraction',
                            ha='center', va='center', fontsize=9, weight='bold', rotation=90)
    for j, col_group in enumerate(df.columns.levels[0]):
        axes[-1,j].set_xlabel(df.index.name)
        axes[0,j].set_title(column_titles[j])
    
    # create legends:
    if one_legend_only:
        for j, col_group in enumerate(df.columns.levels[0]): 
        # Add one legend at the upper left corner        
            axes[-1, j].legend(loc='upper center', bbox_to_anchor=(0.5,-0.6), ncol=ncols, handletextpad=0.1, borderpad=0.2)
    else:
        for j, col_group in enumerate(df.columns.levels[0]):        
            for i, row_group in enumerate(df.columns.levels[1]):
                axes[i, j].legend(loc='best', handletextpad=0.1, borderpad=0.2)

    # General titles for columns and rows
    if column_explainer:
        for j, col_group in enumerate(df.columns.levels[0]):        
            axes[0, j].annotate(column_explainer, xy=(0.5, 1.1), xycoords='axes fraction',
                                ha='center', fontsize=9, weight='bold')
    if row_explainer:
        fig.text(0.02, 0.5, row_explainer, ha='center', va='center', fontsize=10, weight='bold', rotation='vertical')

    if suptitle:
        fig.suptitle(suptitle, fontweight='bold', fontsize=12)

    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0.05, 0.05, 1, 0.96], pad=0.1)
    plt.subplots_adjust(left=0.16, bottom=0.17, wspace=0.19, hspace=0.03,)#,right=1, top=1, bottom=0)

def dymola_multi_plot_one_row(df: pd.DataFrame):
    """This function creates subplots of a multi-indexed dymola DataFrame.

    :param df: data
    :type df: pd.DataFrame
    """
    
    # remove duplicate series, show only once
    df = df.T.drop_duplicates().T

    # swap levels to prioritize variables over files
    df = df.swaplevel(1,2,axis=1)

    # if variable exists only once, it has been a duplicate. Remove the identifier on (what is now) level 2, as this column is for all files...
    for i in df.columns.get_level_values(0):
        for j in df[i].columns.get_level_values(0):
            if df[i,j].columns.size == 1:
                df[i,j].columns = ['']

    # Create subplots
    num_rows = 1 
    num_cols = len(df.columns.levels[0])
    
    fig, axes = plt.subplots(num_rows, num_cols, sharex=True)

    # Initialize lists to store min and max values for each variable (column)
    y_min = [float('inf')] * num_cols
    y_max = [float('-inf')] * num_cols
    
    # First, determine the common y-limits for each variable across all files
    for i, group_name in enumerate(df.columns.levels[0]):
        series = df[group_name]
        y_min[i] = series.min().min()
        y_max[i] = series.max().max()
    
    # Then, plot the data with consistent y-axis scaling
    for j, group_name in enumerate(df.columns.levels[0]):
        ax = axes[i, j] if num_rows > 1 else axes[j]
        df[group_name].plot(ax=ax)
        ax.set_title(f"{group_name}")
        ax.set_xlabel("t (h)")
        ax.set_ylabel(group_name)

        # Set the y-limits based on the previously determined common limits
        # ax.set_ylim(y_min[j]-0.05*abs(y_min[j]), y_max[j]+0.05*abs(y_max[j]))
    
    plt.tight_layout()
    
def prepare_data(mat_files: List[os.PathLike], file_name_replacement: Callable or dict, var_names:List[dict]):
    """This function creates plots that compare different systems

    :param mat_files: mat files to load
    :type folder: List[os.PathLike]
    """
    
    # Load data
    df = load_data(mat_files, var_names)
    
    # Rename first column index level with the given function/dict
    new_names = {file_name: file_name_replacement(file_name) for file_name in df.columns.levels[1]}
    df.rename(columns=new_names, level=1, inplace=True)
    
    # Convert data
    df = convert_data(df, var_names)
        
    return df

if __name__ == '__main__':
    # folders
    data_root = os.path.abspath(__file__+'../../../simulation_results')
    plot_root = os.path.abspath(__file__+'../../../figures')

    # folder names of results / base for plot file names
    result_IDs = ['Fig_7_no_tank_K_comparison', 'Fig_8_stabilization_with_tank', 'Fig_9_strategy_comparison'] 
    
    data_folders = [os.path.join(data_root, id) for id in result_IDs]
    plot_paths = [os.path.join(plot_root, id+'.pdf') for id in result_IDs] 
    
    # functions to extract name features from result file names
    file_name_replacements = [get_K_value_title, get_time_constant_ratio, get_switch_titles] 

    # plot attributes
    row_titles = [r'Overall gain $\mathbf{K}$', r'Time constant ratio $\mathbf{\vartheta}$', 'Stabilization approach']
    fig_titles = ['Stabilizing by Adjusting Slack Supply Temperature', 'Stabilizing with a Tank at K=−2', r'Comparing Approaches: Benefits of a Well-Chosen Slack Generator']
    ylims = [[(),(-1.5,6)],[(),(-1.5,6)],[(),(-1.5,6)]]

    # make plots
    for data_folder, replacement_fun, plot_path, row_title, title, yl in zip(data_folders, file_name_replacements, plot_paths, row_titles, fig_titles, ylims):
        
        # List all mat files in the folder
        mat_files = [os.path.join(data_folder, f) for f in os.listdir(data_folder) if f.endswith('.mat')]
        
        # Make the comparison plot
        df = prepare_data(mat_files, replacement_fun, var_names_paper_results)
        # Make the plot
        dymola_multi_plot_file_rows(df, title, row_explainer=row_title, column_titles=['Supply temperatures', 'Thermal powers'], one_legend_only=True, ylims=yl)
        
        # Save the plot
        plt.savefig(plot_path)

    # Show the plots
    plt.show() 
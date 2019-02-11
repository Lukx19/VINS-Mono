import argparse
import pandas as pd
import os
import glob
import json
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt


convert_dict = {
    "estimator:processImage": "Pose Estimator",
    "readImage": "Feature Tracker",
}

def convert(name):
    if name in convert_dict:
        return convert_dict[name]
    else:
        return name

# def genMetricsTable(experiment_folder,result_dir):
#     table = pd.DataFrame({})
#     for path in sorted(glob.glob(experiment_folder + '/**/*.json', recursive=True)):
#         with open(path, 'r') as content_file:
#             json_str = content_file.read()
#             json_data = json.loads(json_str)
#         split_path = path.split("/")
#         reverse_path = split_path[::-1]

#         json_props = reverse_path[0].split(".")
#         if len(json_props) != 3:
#             continue
#         dataset = json_props[1]
#         experiment = reverse_path[1]
#         # print(dataset, experiment)
#         # print(json_data)
#         if dataset not in table.columns:
#             table[dataset] = None
#         for category in ["ATE"]:
#             for df_field in ["rmse"]:
#                 experiment_name = experiment+"/"+category +"/"+ df_field
#                 if experiment_name not in table.index:
#                     table = table.append(pd.Series(name=experiment_name))
#                 dt_pt = float(json_data[category][df_field])
#                 if dt_pt < 300:
#                     table.loc[experiment_name, dataset] = json_data[category][df_field]
#                 else:
#                     table.loc[experiment_name, dataset] = None
#         experiment_name = experiment + "/scale"
#         if experiment_name not in table.index:
#             table = table.append(pd.Series(name=experiment_name))
#         table.loc[experiment_name, dataset] = json_data["scale"]

#     table.transpose().to_csv(result_dir+"/metrics.csv")

def genMetricsTable2(experiment_folder,result_dir):
    headers = ["dataset","experiment","round","category","sub_category","value"]
    table = []
    for path in sorted(glob.glob(experiment_folder + '/**/*.json', recursive=True)):
        with open(path, 'r') as content_file:
            json_str = content_file.read()
            json_data = json.loads(json_str)
        split_path = path.split("/")
        reverse_path = split_path[::-1]

        json_props = reverse_path[0].split(".")
        if len(json_props) != 3:
            continue

        dataset = json_props[1]
        round_n = json_props[0][0]
        experiment = reverse_path[1]

        scale = json_data["scale"]
        for category in ["ATE"]:
            for df_field in ["rmse"]:
                value = float(json_data[category][df_field])
                if value > 100 or scale < 0.8:
                    value = np.nan
                table.append([convert(dataset), convert(experiment), round_n, category, df_field, value])

    table = pd.DataFrame(table, columns=headers)
    table.to_csv(result_dir+"/metrics.csv")
    return table


    #

def genPerfTable(experiment_folder, result_dir):
    table = pd.DataFrame({})
    for path in sorted(glob.glob(experiment_folder + '/**/*.txt', recursive=True)):
        data = pd.read_csv(path, sep="\s+", header=None, names=["Elapsed time", "CPU (%)", "Real (MB)", "Virtual (MB)"], skiprows=1)
        data = data[data["CPU (%)"] > 50]
        stats = data.describe()
        split_path = path.split("/")
        experiment = split_path[-2]
        name_and_ext = split_path[-1].split(".")
        dataset = name_and_ext[0][8:]
        # print(dataset, experiment)
        if dataset not in table.columns:
            table[dataset] = None
        for stats_cat in ["mean","std"]:
            experiment_name = experiment+"/"+ stats_cat
            if experiment_name not in table.index:
                table = table.append(pd.Series(name=experiment_name))
            table.loc[experiment_name, dataset] = stats.loc[stats_cat,"CPU (%)"]
    table.transpose().to_csv(result_dir+"/runtime_stats.csv")

# def genTimeTable(experiment_folder, result_dir,metrics=["readImage","estimator:processImage"]):
#     table = pd.DataFrame({})
#     for path in sorted(glob.glob(experiment_folder + '/**/*.log', recursive=True)):
#         split_path = path.split("/")
#         if split_path[-1] == "output.log" or split_path[-1] == "results.log":
#             continue
#         experiment = split_path[-2]
#         name_and_ext = split_path[-1].split(".")
#         dataset = name_and_ext[0][8:]
#         if dataset not in table.columns:
#             table[dataset] = None
#         with open(path, "r") as f:
#             for line in f:
#                 if len(line) < 5:
#                     continue
#                 if line[0:5] != "Desc:":
#                     continue
#                 line_parts = line.split(": ")
#                 time_name = line_parts[1][0:-5]
#                 if time_name not in metrics:
#                     continue
#                 mean = line_parts[3].split(" ")[0]
#                 experiment_name = experiment + "/" + time_name
#                 if experiment_name not in table.index:
#                     table = table.append(pd.Series(name=experiment_name))
#                 table.loc[experiment_name, dataset] = float(mean)
#                 # print(dataset,experiment,time_name,mean)
#     # print(table)
#     table.transpose().to_csv(result_dir+"/time_stats.csv")

def barplot(table,query, ylabel,ylimit=None,title=None, output_file=None):
    if query:
        selected_table = table.query(query)
    else:
        selected_table = table
    plt.clf()
    plt.close()
    # print(selected_table)
    ax = sns.barplot(x='dataset', y='value', hue="experiment",
                 data=selected_table, estimator=np.mean, saturation=0.7)
    sns.despine(ax=ax,left=True)
    ax.set_xticklabels(ax.get_xticklabels(),rotation=30)
    ax.legend(frameon=True, loc='upper center', ncol=5)
    # plt.title('My title')
    ax.set_xlabel('Datasets')
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(ylabel)
    # print(np.max(selected_table.loc[:,'value'].max()))
    # sns.plt.ylim(0, selected_table.loc[:,'value'].max()*1.5)

    # ax.set(ylim=(0, selected_table.loc[:,'value'].max()*2.5))
    if ylimit:
        ax.set_ylim(0, ylimit)
    else:
        ax.set_ylim(0,selected_table.loc[:,'value'].max()*1.2)
    if output_file is None:
        plt.show()
    else:
        fig = ax.get_figure()
        fig.tight_layout()
        fig.savefig(output_file)


def genTimeTable2(experiment_folder, result_dir,metrics=["readImage","estimator:processImage"]):
    headers = ["dataset", "experiment", "round","timer_name","value"]

    table = []
    for path in sorted(glob.glob(experiment_folder + '/**/*.log', recursive=True)):
        split_path = path.split("/")
        if split_path[-1] == "output.log" or split_path[-1] == "results.log":
            continue
        experiment = split_path[-2]
        name_parts = split_path[-1].split(".")[0].split("_")
        round_n = name_parts[0]
        # print(name_parts)
        dataset = "_".join(name_parts[2:])
        # print(dataset)
        # if dataset not in table.columns:
        #     table[dataset] = None
        # metric_data =  dict(zip(metrics, [0 for i in metrics]))
        with open(path, "r") as f:
            for line in f:
                if len(line) < 5:
                    continue
                if line[0:5] != "Desc:":
                    continue
                line_parts = line.split(": ")
                time_name = line_parts[1][0:-5]
                if time_name not in metrics:
                    continue
                mean = line_parts[3].split(" ")[0]
                table.append([convert(dataset),convert(experiment),round_n,convert(time_name),float(mean)])
                # metric_data[time_name] = float(mean)
        # row = [dataset, experiment, round_n]
        # for metric_val in metric_data:
            # row.append(metric_val)
        # table.append(row)
                # print(dataset,experiment,time_name,mean)
    table = pd.DataFrame(table, columns=headers)
    table.to_csv(result_dir+"/time_stats.csv")
    return table

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script generates csv table from multiple experiments by concatenating experiment results
    ''')
    parser.add_argument('folder', help='where to start search for experiments')
    parser.add_argument('--results_dir', help='Existing directory where to save all outputs from this graph')
    args = parser.parse_args()


    output_folder = args.folder
    if "--result_dir" in args:
        output_folder = args.result_dir

    estiamtor_timer = '"' + convert('estimator:processImage') + '"'
    tracker_timer = '"'+convert('readImage')+'"'
    # print("ahoooj")
    sns.set_palette("muted")
    sns.set_style("whitegrid")

    table = genMetricsTable2(args.folder, output_folder)
    barplot(table,'category == "ATE" and sub_category=="rmse"','ATE-RMSE [cm]',output_file=output_folder+"/ate_rmse.png")

    genPerfTable(args.folder, output_folder)


    table = genTimeTable2(args.folder, output_folder)
    table_grouped = table.query(
                        'timer_name==' + estiamtor_timer + " or timer_name==" + tracker_timer).groupby(['dataset', 'experiment', 'round']).sum().reset_index(inplace=False)
    # print(table_grouped)
    barplot(table_grouped,query=None,ylabel='Execution time [ms]',output_file=output_folder+"/total_exec.png")
    barplot(table,'timer_name=='+estiamtor_timer,'Execution time [ms]',output_file=output_folder+"/estimator_exec.png")
    barplot(table,'timer_name=='+tracker_timer,'Execution time [ms]',output_file=output_folder+"/feature_exec.png")



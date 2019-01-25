import argparse
import pandas as pd
import os
import glob
import json

def genMetricsTable(experiment_folder,result_dir):
    table = pd.DataFrame({})
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
        experiment = reverse_path[1]
        # print(dataset, experiment)
        # print(json_data)
        if dataset not in table.columns:
            table[dataset] = None
        for category in ["ATE"]:
            for df_field in ["rmse"]:
                experiment_name = experiment+"/"+category +"/"+ df_field
                if experiment_name not in table.index:
                    table = table.append(pd.Series(name=experiment_name))
                dt_pt = float(json_data[category][df_field])
                if dt_pt < 300:
                    table.loc[experiment_name, dataset] = json_data[category][df_field]
                else:
                    table.loc[experiment_name, dataset] = None
        experiment_name = experiment + "/scale
        if experiment_name not in table.index:
            table = table.append(pd.Series(name=experiment_name))
        table.loc[experiment_name, dataset] = json_data["scale"]

    table.transpose().to_csv(result_dir+"/metrics.csv")

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

def genTimeTable(experiment_folder, result_dir,metrics=["readImage","estimator:processImage"]):
    table = pd.DataFrame({})
    for path in sorted(glob.glob(experiment_folder + '/**/*.log', recursive=True)):
        split_path = path.split("/")
        if split_path[-1] == "output.log" or split_path[-1] == "results.log":
            continue
        experiment = split_path[-2]
        name_and_ext = split_path[-1].split(".")
        dataset = name_and_ext[0][8:]
        if dataset not in table.columns:
            table[dataset] = None
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
                experiment_name = experiment + "/" + time_name
                if experiment_name not in table.index:
                    table = table.append(pd.Series(name=experiment_name))
                table.loc[experiment_name, dataset] = float(mean)
                # print(dataset,experiment,time_name,mean)
    # print(table)
    table.transpose().to_csv(result_dir+"/time_stats.csv")


if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script generates csv table from multiple experiments by concatenating experiment results
    ''')
    parser.add_argument('folder', help='ground truth trajectory (format: timestamp tx ty tz qw qx qy qz)')
    parser.add_argument('--results_dir', help='Existing directory where to save all outputs from this graph')
    args = parser.parse_args()


    output_folder = args.folder
    if "--result_dir" in args:
        output_folder = args.result_dir

    genMetricsTable(args.folder, output_folder)
    genPerfTable(args.folder, output_folder)
    genTimeTable(args.folder, output_folder)




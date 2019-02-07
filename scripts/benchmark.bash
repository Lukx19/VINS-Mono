#!/bin/bash

# MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult


# datasets="MH_01_easy MH_03_medium V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult"
datasets="V1_01_easy V1_02_medium V1_03_difficult"

output_path=$(pwd -P)
alg_run=1
orb_ygz_folder="/home/osboxes/3duniversum/ORB-YGZ-SLAM"
datasets_folder="/home/osboxes/3duniversum/datasets"
algorithm="VINS"
config_file=""
rosbag_flags=""
experiment_file=""
current_dir=$(pwd -P)
rounds=1

while getopts "h?so:d:c:r:a:e:f:t:" opt; do
    case "$opt" in
    h|\?)
        echo "-d "V1_01_easy V1_02_medium V1_03_difficult"  -o "output_path"  -s[will not run algorithm only statistics]  -c "path to config file"  -r '-s 10 --clock'  -a VINS -e "experiment_file_path" -t num_of_trys_for_each_experiment"
        exit 0
        ;;
    d)  datasets=$OPTARG
        ;;
    o)  output_path=$(realpath -m $OPTARG)
        ;;
    s) alg_run=0
        ;;
    c) config_file=$(realpath -m $OPTARG)
        ;;
    r) rosbag_flags=$OPTARG
        ;;
    a) algorithm=$OPTARG
        ;;
    e) experiment_file=$(realpath $OPTARG)
        ;;
    f) datasets_folder=$(realpath $OPTARG)
        ;;
    t) rounds=$OPTARG
        ;;
    esac
done

mkdir -p ${output_path}
cd ${output_path}
if [ "$alg_run" == 1 2> /dev/null ]
then
    rm *.csv
    rm *.log
    rm *.txt
fi
rm *.png
rm results.log
touch results.log

if [ ! -z "$experiment_file" ]
then
    echo "Sourcing experiment file: "${experiment_file}
     . $experiment_file
    echo ${dataset_bag_flags[MH_01_easy]}
    echo ${dataset}
    echo ${algorithm}
fi

echo "Datasets: "${datasets}
for i in $(seq ${rounds})
do
    for dataset in $datasets
    do
        echo -e "\n-------------------------${dataset}\n" >> "${output_path}/results.log"
        echo "Evaluate dataset ${dataset}"
        ground_truth="${current_dir}/../benchmark_publisher/config/$dataset/data.csv"
        recording_file="$output_path/${i}_recording_${dataset}.csv"
        roslaunch_args="dataset_folder:=${datasets_folder} record_file:=${recording_file} sequence_name:=${dataset}"
        if [ ! -z "$config_file" ]
        then
            roslaunch_args=${roslaunch_args}" config_path:=${config_file}"
        fi

        if [ ! -z "$experiment_file" ] && [ -z "$rosbag_flags" ]
        then
            rosbag_flags=${dataset_bag_flags[$dataset]}
        fi

        if [ ! -z "$rosbag_flags" ]
        then
            roslaunch_args=${roslaunch_args}" play_bag_flags:='${rosbag_flags}'"
        fi
        echo "Ros launch args "${roslaunch_args}
        if [ "$alg_run" == 1 2> /dev/null ]
        then
            rm $recording_file
            touch $recording_file
            echo "Running algorithm: "$algorithm
            executable_cmd=""
            case "$algorithm" in
                "VINS")
                executable_cmd="roslaunch benchmark_recorder benchmark_vins.launch ${roslaunch_args}"
                ;;
                "VINS_RGBD")
                executable_cmd="roslaunch benchmark_recorder benchmark_vins_rgbd.launch ${roslaunch_args}"
                ;;
                "SVO")
                executable_cmd="roslaunch benchmark_recorder benchmark_svo.launch ${roslaunch_args}"
                ;;
                "OKVIS")
                executable_cmd="roslaunch benchmark_recorder benchmark_okvis.launch ${roslaunch_args}"
                ;;
                "ORB_XYZ_IMU")
                dataset_shorthand=$(echo ${dataset} | cut -f"1,2" -d"_" --output-delimiter="")
                executable_cmd="${orb_ygz_folder}/Examples/Monocular/mono_euroc_vins \
                    ${orb_ygz_folder}/Vocabulary/ORBvoc.bin \
                    ${orb_ygz_folder}/Examples/Monocular/EuRoC.yaml \
                    ${datasets_folder}/${dataset}/mav0/cam0/data \
                    ${datasets_folder}/${dataset}/mav0/cam0/data.csv \
                    ${datasets_folder}/${dataset}/mav0/imu0/data.csv \
                    $recording_file"
                ;;
                "ORB_XYZ_SVO")
                executable_cmd="dataset_shorthand=$(echo ${dataset} | cut -f"1,2" -d"_" --output-delimiter="")
                    ${orb_ygz_folder}/Examples/Monocular/mono_euroc \
                    ${orb_ygz_folder}/Vocabulary/ORBvoc.bin \
                    ${orb_ygz_folder}/Examples/Monocular/EuRoC.yaml \
                    ${datasets_folder}/${dataset}/mav0/cam0/data \
                    ${orb_ygz_folder}/Examples/Monocular/EuRoC_TimeStamps/${dataset_shorthand}.txt \
                    $recording_file"
                ;;
            esac
            # install this to strack cpu and memory usage pip install psrecord
            psrecord "${executable_cmd} | tee ${output_path}/${i}_output_${dataset}.log" --log "${output_path}/${i}_cpu_mem_${dataset}.txt" --plot "${output_path}/${i}_cpu_mem_${dataset}.png" --include-children --interval 1
            # time ${executable_cmd}

        fi
        python ${current_dir}/evaluate_ate_rpe.py $ground_truth $recording_file --plot --verbose --save_associations --results_dir $output_path --prefix ${i} --postfix $dataset >> "${output_path}/${i}_results.log"

    done
done
#!/bin/bash

# MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult


# datasets="MH_01_easy MH_03_medium V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult"
datasets="V1_01_easy V1_02_medium V2_01_easy"

output_path=$(pwd -P)
no_alg_run=0
orb_ygz_folder="/home/osboxes/3duniversum/ORB-YGZ-SLAM"
datasets_folder="/home/osboxes/3duniversum/datasets"
algorithm="VINS"

while getopts "h?bo:d:" opt; do
    case "$opt" in
    h|\?)
        show_help
        exit 0
        ;;
    d)  datasets=$OPTARG
        ;;
    o)  output_path=$(realpath $OPTARG)
        ;;
    b) no_alg_run=1
        ;;
    esac
done

# create result file
rm "${output_path}/results.log"
touch "${output_path}/results.log"
echo "Datasets: "${datasets}
for dataset in $datasets
do
    echo -e "\n-------------------------${dataset}\n" >> "${output_path}/results.log"
    echo "Evaluate dataset ${dataset}"
    ground_truth="../benchmark_publisher/config/$dataset/data.csv"
    recording_file="$output_path/recording_${dataset}.csv"
    if [ "$no_alg_run" == 0 2> /dev/null ]
    then
        rm $recording_file
        touch $recording_file
        echo "Running algorithm: "$algorithm
        case "$algorithm" in
            "VINS")
            roslaunch benchmark_publisher benchmark.launch record_file:=$recording_file sequence_name:=$dataset
            ;;
            "SVO")
            roslaunch benchmark_publisher benchmark_svo.launch record_file:=$recording_file     sequence_name:=$dataset
            ;;
            "ORB_XYZ_IMU")
            dataset_shorthand=$(echo ${dataset} | cut -f"1,2" -d"_" --output-delimiter="")
            ${orb_ygz_folder}/Examples/Monocular/mono_euroc_vins \
                ${orb_ygz_folder}/Vocabulary/ORBvoc.bin \
                ${orb_ygz_folder}/Examples/Monocular/EuRoC.yaml \
                ${datasets_folder}/${dataset}/mav0/cam0/data \
                ${datasets_folder}/${dataset}/mav0/cam0/data.csv \
                ${datasets_folder}/${dataset}/mav0/imu0/data.csv \
                $recording_file
            ;;
            "ORB_XYZ_SVO")
            dataset_shorthand=$(echo ${dataset} | cut -f"1,2" -d"_" --output-delimiter="")
                ${orb_ygz_folder}/Examples/Monocular/mono_euroc \
                ${orb_ygz_folder}/Vocabulary/ORBvoc.bin \
                ${orb_ygz_folder}/Examples/Monocular/EuRoC.yaml \
                ${datasets_folder}/${dataset}/mav0/cam0/data \
                ${orb_ygz_folder}/Examples/Monocular/EuRoC_TimeStamps/${dataset_shorthand}.txt \
                $recording_file
            ;;
        esac

    fi
    python evaluate_ate_rpe.py $ground_truth $recording_file --plot --verbose --save_associations --results_dir $output_path --prefix $dataset >> "${output_path}/results.log"
    #  echo "\n---------RPE\n" >> "${output_path}/results.log"
    #  python evaluate_rpe.py --verbose $ground_truth $recording_file >> "${output_path}/results.log"

done
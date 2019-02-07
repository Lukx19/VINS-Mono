#!/bin/bash

alg_run=1
use_ops=0
datasets_folder="/home/osboxes/3duniversum/datasets"
rounds=1

while getopts "h?se:d:t:" opt; do
    use_ops=1
    case "$opt" in
    h|\?)
        echo "-d "V1_01_easy V1_02_medium V1_03_difficult"  -o "output_path"  -s[will not run algorithm only statistics]  -c "path to config file"  -r '-s 10 --clock'  -a VINS -e "experiment_file_path" -d dataset folder -t num_of_trys_for_each_experiment"
        exit 0
        ;;
    s) alg_run=0
        ;;
    e) experiment_path=$(realpath $OPTARG)
        ;;
    d) datasets_folder=$(realpath $OPTARG)
        ;;
    t) rounds=$OPTARG
        ;;
    *) echo "unknown command"
       exit 0
        ;;
    esac
done
if [ "$use_ops" == 0 ]
then
   echo "Please use some of cli ops"
   exit 0
fi

experiments=$(find $experiment_path -type f -name "*.conf" -print | sort --unique)
echo ${experiments}



for experiment in $experiments
do
    echo ${experiment}
    dir=$(dirname $experiment)
    if [ "$alg_run" == 1 2> /dev/null ]
    then
        bash benchmark.bash -e ${experiment} -o${dir} -f ${datasets_folder} -t ${rounds} | tee ${dir}/output.log
    else
        bash benchmark.bash -s -e ${experiment} -o${dir} -f ${datasets_folder} -t ${rounds} | tee ${dir}/output.log
    fi
done
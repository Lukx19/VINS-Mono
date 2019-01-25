#!/bin/bash

alg_run=1

while getopts "h?se:" opt; do
    case "$opt" in
    h|\?)
        echo "-d "V1_01_easy V1_02_medium V1_03_difficult"  -o "output_path"  -s[will not run algorithm only statistics]  -c "path to config file"  -r '-s 10 --clock'  -a VINS -e "experiment_file_path" "
        exit 0
        ;;
    s) alg_run=0
        ;;
    e) experiment_path=$(realpath $OPTARG)
        ;;
    esac
done

experiments=$(find $experiment_path -type f -name "*.conf" -print | sort --unique)
echo ${experiments}



for experiment in $experiments
do
    echo ${experiment}
    dir=$(dirname $experiment)
    if [ "$alg_run" == 1 2> /dev/null ]
    then
        bash benchmark.bash -e ${experiment} -o${dir} | tee ${dir}/output.log
    else
        bash benchmark.bash -s -e ${experiment} -o${dir} | tee ${dir}/output.log
    fi
done
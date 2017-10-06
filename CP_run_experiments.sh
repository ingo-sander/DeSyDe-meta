#!/bin/bash
Experiment="--- Experiments ---"
echo $Experiment
echo $Experiment > log.txt
#number of experiments
total_experiments=1

echo -n "Insert the experiment number (Enter for all experiments) > "
read exp_no
if [ "$exp_no" == "" ]
then
    echo "Running all experiments"
    first_exp=1
    last_exp=$total_experiments
else
    first_exp=$exp_no
    echo -n "Insert last experiment > "
    read last_exp    
    echo "Running experiment "$exp_no " to " $last_exp 
fi


for (( i=$first_exp;i<=$last_exp;i++)); do
    printf "\n" >> log.txt
    echo "--- Experiment "$i >> log.txt
    echo "--- Experiment "$i 
    date >> log.txt
    ls exp_$i/sdfs >> log.txt
    SECONDS=0
    sudo chrt --rr 99 ./bin/adse --config config.cfg 
    duration=$SECONDS
    echo "$(($duration / 60)) minutes and $(($duration % 60)) seconds elapsed." >> log.txt
    
done
exit 

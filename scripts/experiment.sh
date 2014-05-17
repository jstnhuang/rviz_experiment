DATA_DIR=/home/jstn/Dropbox/experiment_data

if [ -e $DATA_DIR/$1_$2.bag ]
then
  echo "$DATA_DIR/$1_$2.bag already exists! Stopping launch."
else
  roslaunch rviz_experiment experiment.launch user:=$1 condition:=$2
fi

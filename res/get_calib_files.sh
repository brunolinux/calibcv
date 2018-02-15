#!/bin/bash

echo "GETTING CALIBRATION FILES FROM DROPBOX"

FILES=( "https://www.dropbox.com/s/so7qjmpusrpmurr/calibration_ps3eyecam.avi" )

## get the files from dropbox
for file in "${FILES[@]}"
do

    echo "downloading: ${file}"
    wget "${file}"

done


echo "DONE"
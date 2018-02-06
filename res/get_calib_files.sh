#!/bin/bash

echo "GETTING CALIBRATION FILES FROM DROPBOX"

FILES=( "https://www.dropbox.com/s/yjip8kscakq257l/calibration_mslifecam.avi"  
        "https://www.dropbox.com/s/so7qjmpusrpmurr/calibration_ps3eyecam.avi"
        "https://www.dropbox.com/s/8p5nuvjcnyxkkv6/calibration_kinectv2.avi" )

## get the files from dropbox
for file in "${FILES[@]}"
do

    echo "downloading: ${file}"
    wget "${file}"

done


echo "DONE"
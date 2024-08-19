#!/bin/bash

blue="\033[1;34m"
yellow="\033[1;33m"
reset="\033[0m"

include_count=$(find include  -type f \( -name "*.cpp" -o -name "*.h" \) -exec cat {} \; | wc -l)
src_count=$(find src  -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.txt" \) -exec cat {} \; | wc -l)
total=$((include_count + src_count))

if [ ! -d "data/debug" ]; then
    mkdir data/debug
    touch data/debug/here_save_debug_images
fi

if [ ! -d "data/video" ]; then
    mkdir data/video
    touch data/video/here_save_video
fi

if [ ! -d "data/speed" ]; then
    mkdir data/speed
    touch data/speed/here_save_shoot_speed
fi

if [ ! -d "/etc/openrm" ]; then 
    mkdir /etc/openrm
    sudo cp -r data/uniconfig/* /etc/openrm/
    sudo chmod -R 777 /etc/openrm
fi

if [ ! -d "config" ]; then 
    ln -s /etc/openrm ./config
fi


if [ ! -d "build" ]; then 
    mkdir build
fi

imshow=0

while getopts ":rcg:ls" opt; do
    case $opt in
        r)
            echo -e "${yellow}<--- delete 'build' --->\n${reset}"
            sudo rm -rf build
            mkdir build
            shift
            ;;

        c)
            sudo cp -r data/uniconfig/* /etc/openrm/
            sudo chmod -R 777 /etc/openrm
            exit 0
            shift
            ;;
        g)
            git_message=$OPTARG
            echo -e "${yellow}\n<--- Git $git_message --->${reset}"
            git pull
            git add -A
            git commit -m "$git_message"
            git push
            exit 0
            shift
            ;;

        l)
            cd ../OpenRM
            sudo ./run.sh
            cd ../TJURM-2024
            exit 0
            shift
            ;;
        s)
            imshow=1
            shift
            ;;
        \?)
            echo -e "${red}\n--- Unavailable param: -$OPTARG ---\n${reset}"
            ;;
        :)
            echo -e "${red}\n--- param -$OPTARG need a value ---\n${reset}"
            ;;
        esac
    done


echo -e "${yellow}<--- Start CMake --->${reset}"
cd build
cmake ..


echo -e "${yellow}\n<--- Start Make --->${reset}"
max_threads=$(cat /proc/cpuinfo | grep "processor" | wc -l)
make -j "$max_threads"


echo -e "${yellow}\n<--- Total Lines --->${reset}"
echo -e "${blue}        $total${reset}"


echo -e "${yellow}\n<--- Run Code --->${reset}"
sudo rm /usr/local/bin/TJURM-2024
sudo cp TJURM-2024 /usr/local/bin/
sudo pkill TJURM-2024
sudo chmod 777 /dev/tty*

if [ $imshow = 1 ]; then
    TJURM-2024 -s
else
    TJURM-2024

fi

/etc/openrm/guard.sh

echo -e "${yellow}<----- OVER ----->${reset}"
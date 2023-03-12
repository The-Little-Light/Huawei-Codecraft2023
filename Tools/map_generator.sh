#/bin/bash

number=1000
if [ ! -d "./map" ]
then
    mkdir "./map"
fi

#帮助
if [ "$1" = "-?" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ] || [ "$1" = "/?" ] || [ "$1" = "/h" ] || [ "$1" = "/help" ] || [ $# -lt 1 ]
then
    echo "Usage: $0 <maps_number(>4)>"
    exit 1
fi


if [ $# -ge 1 ]
then
    if [ $1 -lt 5 ]
    then
        echo "Error: maps_number must be greater than 4"
        exit 1
    else
        # 如果generator.exe不存在
        if [ ! -f "./bin/generator.exe" ]
        then
            echo "generator.exe not found"
            echo "Please run build.sh first"
            exit 1
        fi
    fi
    number=$1
    for ((i=5; i<$number; i++))
    do
        ./bin/generator.exe >"./map/"$i.txt
    done
fi
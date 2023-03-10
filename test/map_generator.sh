#/bin/bash

number=1000
if [ ! -d "./map" ]
then
    mkdir "./map"
fi

#帮助
if [ $# -ge 1 ] && [ "$1" = "-?" ] || [ $# -lt 1 ]
then
    echo "Usage: $0 <maps_number(>4)>"
    exit 1
fi


if [ $# -ge 1 ]
then
    number=$1
    for ((i=5; i<$number; i++))
    do
        ./bin/generator.exe >"./map/"$i.txt
    done
fi
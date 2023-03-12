#/bin/bash
if [ "$1" = "-?" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ] || [ "$1" = "/?" ] || [ "$1" = "/h" ] || [ "$1" = "/help" ]
then
    echo "Usage: $0 [name]"
    exit 1
fi

id=1
pre=""


if [ $# -ge 1 ]
then
    pre=$1"-"
    while [ -d "./"$pre$id ]
    do
        rm -rf "./"$pre$id
        id=$((id+1))
    done
fi



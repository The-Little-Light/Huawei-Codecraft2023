#bin/bash

if [ "$1" = "-?" ] || [ "$1" = "-h" ] || [ "$1" = "--help" ] || [ "$1" = "/?" ] || [ "$1" = "/h" ] || [ "$1" = "/help" ] || [ $# -lt 1 ]
then
    echo "Usage: $0 <dir of source code> [name]"
    echo "Example: $0 .\src"
    echo "Example: $0 .\src x"
    exit 1
fi

source="$1"
id=1
pre=""
if [ $# -ge 2 ]
then
    pre=$2"-"
fi

while [ -d "./"$pre$id ]
do
    id=$((id+1))
done

target="./"$pre$id"/"
if [ ${source: -1} != "/" ]
then
    source=$source"/"
fi

mkdir $target
g++ -std=c++14 -O2 -o $target"main.exe" $source"main.cpp"
if [ $? -ne 0 ]
then
    echo "Compilation error"
    exit 1
fi

id=0
score=0
no=""
if [ -d map ]
then
    echo "" >${target}result.txt
    mkdir ${target}replay
    #对于map中的每个文件
    for file in ./map/*
    do
        suffix=`basename $file | cut -d . -f 1`
        ./bin/robot -f -s $RANDOM -r $target"replay/$suffix.rep" -m $file -c $target "main.exe" >>${target}result.txt
        tmp=`tail -n 1 ${target}result.txt`
        t1=`echo $tmp | jq .status`
        if [ $t1 = '"Successful"' ]
        then
            t2=`echo $tmp | jq .score`
            score=$((score+t2))
        else
            no=$no" "$suffix
        fi
        id=$((id+1))
    done

    tmp="Total: "$id
    echo $tmp >>${target}result.txt
    echo $tmp
    tmp="Average: "`awk "BEGIN{printf \\"%.4f\n\\",$score/$id}"`
    echo $tmp >>${target}result.txt
    echo $tmp
    tmp="Wrong: "$no
    echo $tmp >>${target}result.txt
    echo $tmp
fi

rm -r ./replay

#!/bin/sh

#for .cpp
ls *.dat | while read FILE_NAME
do
	echo 
	echo "extern const char mazeData_$(basename $FILE_NAME .dat)[N+1][N+1] = {"
	tail -n +4 $FILE_NAME | head -n -1 | sed -e 's/ //g' | while read line
do
	echo '		{"'$line'"},'
done
echo "};"
done



#for .h
echo

ls *.dat | while read FILE_NAME
do
	echo "extern const char mazeData_$(basename $FILE_NAME .dat)[N+1][N+1];"
done


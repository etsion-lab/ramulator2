#!/bin/bash

DIR=`dirname $0`

ECHO=""
#ECHO="echo"

benchs1="data-analytics-core-1 data-caching-core-1 data-serving-core-1 graph-analytics-core-1 in-memory-analytics-core-1 media-streaming-core-1 web-search-core-1 web-serving-core-1"
benchs8="data-analytics-core-8 data-caching-core-8 data-serving-core-8 graph-analytics-core-8 in-memory-analytics-core-8 media-streaming-core-8 web-search-core-8 web-serving-core-8"

#benchs8="web-search-core-8"

ninsts="100M"
#ninsts="1G"
ratio=1
cowslat=0
llc="2" # MB
drampage_kB=8 # KB

dirname="stats-8c-${ninsts}-LRU_nohit-ratio=1-cowslat=8-llc=${llc}MB-drampage=${drampage_kB}KB"
#dirname="base-8c-${ninsts}-nocows-no-zerocopy-accel"
#dirname="tst-8c-${ninsts}-LRU_nohit-ratio=${ratio}-cowslat=${cowslat}-llc=${llc}"
#dirname="tst-drampage=${drampage_kB}"

benchs=$benchs8

#benchs="in-memory-analytics-core-8"


for b in $benchs; do
   echo $b
   ${ECHO} ${DIR}/run.pl --bench $b --dir ${dirname} --name $b --config ${DIR}/../config_cows_ddr5.yaml &
done

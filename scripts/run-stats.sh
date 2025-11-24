#!/bin/bash

DIR=`dirname $0`
CONFIG="${DIR}/../config_cows_ddr5.yaml"

ECHO=""
#ECHO="echo"


# Usage: get_yaml_value KEY FILE
# Prints the value for the first matching KEY in a simple "key: value" YAML line.
get_yaml_value() {
  local key="$1"

  if [ -z "$key" ]; then
    echo "usage: get_yaml_value <key>" >&2
    return 1
  fi

  grep -v "^ *\#" ${CONFIG} | grep "${key}:" | awk '{print $2}'
}

freq_human() {
  local hz="$1"
  # Normalize to float; treat input as Hz unless clearly MHz/GHz already
  if [ $hz -ge 1000000000 ]; then
    printf "%.0fG\n" "$(calc "$hz/1e9")"
  elif [ $hz -ge 1000000 ]; then
    printf "%.0fM\n" "$(calc "$hz/1e6")"
  else
    out="${hz}Hz"
  fi
  printf "%s" "$out"
}

benchs1="data-analytics-core-1 data-caching-core-1 data-serving-core-1 graph-analytics-core-1 in-memory-analytics-core-1 media-streaming-core-1 web-search-core-1 web-serving-core-1"
benchs8="data-analytics-core-8 data-caching-core-8 data-serving-core-8 graph-analytics-core-8 in-memory-analytics-core-8 media-streaming-core-8 web-search-core-8 web-serving-core-8"

#benchs8="web-search-core-8"

#ninsts="100M"
ninsts=$(get_yaml_value "num_expected_insts")
ratio=$(get_yaml_value "cows_cache2llc_ratio")
cowslat=$(get_yaml_value "cows_cache_access_latency")
llc=$(get_yaml_value "llc_capacity_per_core")
drampage_kB=$(get_yaml_value "dram_page_bytes")

echo NINSTS=$ninsts

ninsts=$(freq_human $ninsts)

dirname="stats-8c-${ninsts}-LRU_nohit-ratio=${ratio}-cowslat=${cowslat}-llc=${llc}-drampage=${drampage_kB}"
#dirname="base-8c-${ninsts}-nocows-no-zerocopy-accel"
#dirname="tst-drampage=${drampage_kB}"

benchs=$benchs8

#benchs="in-memory-analytics-core-8"


for b in $benchs; do
   echo $b
   ${ECHO} ${DIR}/run.pl --bench $b --dir ${dirname} --name $b --config ${CONFIG} &
done

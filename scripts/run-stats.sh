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

insts_human() {
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

human_to_bytes() {
  local size="$1"
  if [[ "$size" =~ ^([0-9]+)([KMG]?)B?$ ]]; then
    local num="${BASH_REMATCH[1]}"
    local unit="${BASH_REMATCH[2]}"
    case "$unit" in
      K) echo $((num * 1024)) ;;
      M) echo $((num * 1024 * 1024)) ;;
      G) echo $((num * 1024 * 1024 * 1024)) ;;
      *) echo "$num" ;;
    esac
  else
    echo "Invalid size format: $size" >&2
    return 1
  fi
}

benchs1="data-analytics-core-1 data-caching-core-1 data-serving-core-1 graph-analytics-core-1 in-memory-analytics-core-1 media-streaming-core-1 web-search-core-1 web-serving-core-1"
benchs8="data-analytics-core-8 data-caching-core-8 data-serving-core-8 graph-analytics-core-8 in-memory-analytics-core-8 media-streaming-core-8 web-search-core-8 web-serving-core-8"

ncores=1
if [ "$ncores" -eq 1 ]; then
   benchs=$benchs1
else
   benchs=$benchs8
fi

#benchs="web-search-core-8"
#benchs="in-memory-analytics-core-8"

ninsts=$(get_yaml_value "num_expected_insts")
cows_enable=$(get_yaml_value "cows_enable")
ratio=$(get_yaml_value "cows_cache2llc_ratio")
cowslat=$(get_yaml_value "cows_cache_access_latency")
cowsdramlat=$(get_yaml_value "cows_dram_latency_on_translation")
cowsassoc=$(get_yaml_value "cows_cache_assoc")
llc=$(get_yaml_value "llc_capacity_per_core")
drampage_kB=$(get_yaml_value "dram_page_bytes")

# calc number of entries in cows cache
llc_bytes=$(human_to_bytes $llc)
line_size=$(get_yaml_value "llc_linesize")
cows_entries=$((llc_bytes / (line_size * ratio)))
echo llc_bytes=$llc_bytes, line_size=$line_size, ratio=$ratio, cowsassoc=$cowsassoc, cows_entries=$cows_entries

echo ">>>NINSTS=$ninsts<<<"

ninsts=$(insts_human $ninsts)

echo cows_enable=$cows_enable

suffix="${ninsts}-${ncores}c-llc=${llc}-drampage=${drampage_kB}"
if [ "$cows_enable" = "true" ]; then
  cows_suffix="ratio=${ratio}-cowsentries=${cows_entries}-cowslat=${cowslat}-cowsdramlat=${cowsdramlat}-cowsassoc=${cowsassoc}"
  suffix="cows-${suffix}---${cows_suffix}"
else
  suffix="nocows-${suffix}"
fi

#dirname="stats-${suffix}"
#dirname="base-8c-${ninsts}-nocows-no-zerocopy-accel"
#dirname="tst-${suffix}"
dirname="tst2-3200-${suffix}"
#dirname="tst2"

for b in $benchs; do
   echo $b
   ${ECHO} ${DIR}/run.pl --bench $b --dir ${dirname} --name $b --config ${CONFIG} &
done

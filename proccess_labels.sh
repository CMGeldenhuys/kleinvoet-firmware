#!/usr/bin/env bash
set -euo pipefail


path="$1"
rec="$path/REC.WAV"
ts="$path/TS.CSV"

# AWK Script
script=$(cat <<'EOF'
{
    if (NR != 1) {
        if ($2 != "") {
           print $1/fs "\t" $1/fs "\t" $2
        }
        else {
             print $1/fs "\t" $1/fs "\t" $4
        }
    }
}
EOF
)
# END OF SCRIPT

# Get sampling freq from WAV file
fs=`mediainfo -f $rec | grep 'Sampling rate' | head -n 1 | awk -F: '{print $2}'`
# Run above AWK script
awk -F, -v fs="$fs" "$script" "$ts"

#!/bin/bash

cfg_dir="./experiment_setups"
results_dir="./results"

for file in "$cfg_dir"/*.yaml; do
    echo "Beginning runs for setup: $file"

    for i in $(eval echo {1..$1}); do
        echo "Running experiment ${i}"
        python3 stompc.py -cfg "$file"
        
        echo "Sleepting for 10 seconds before next experiment"
        sleep 10
        
        printf "\n\n\n"
    done

    filename=$(basename "$file" .yaml)
    output_folder="$filename"
    mkdir -p "$output_folder"

    echo "Placing results from $filename into $output_folder"
    
    find "$results_dir" -mindepth 1 -maxdepth 1 -type d ! -name "$filename" -exec mv {} "$output_folder" \;
    cp "$results_dir/basic_data_analysis.ipynb" "$output_folder"
    zip -r "$results_dir/$filename.zip" "$output_folder"
    echo "All results succesfully placed in $output_folder.zip"

    rm -rf "$output_folder"
    echo "Deleted $output_folder"

done

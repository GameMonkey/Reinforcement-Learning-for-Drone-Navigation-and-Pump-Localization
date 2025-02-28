#!/bin/bash

cfg_dir="./experiment_setups"
results_dir="./results"

shopt -s nullglob
config_files=($cfg_dir/*.yaml)

if (( "${#config_files[@]}" )); then
    for file in "$cfg_dir"/*.yaml; do
        echo "Beginning runs for setup: $(basename "$file")"

        for i in $(eval echo {1..$1}); do
            echo "Running experiment ${i}"
            python3 stompc.py -cfg "$file"
            
            echo "Sleeping for 10 seconds before next experiment"
            sleep 10
            
            printf "\n\n\n"
        done

        filename=$(basename "$file" .yaml)
        output_folder="$filename"
        mkdir -p "$output_folder"

        echo "Placing results from $filename.yaml into $output_folder"
        
        find "$results_dir" -mindepth 1 -maxdepth 1 -type d ! -name "$filename" -exec mv {} "$output_folder" \;
        cp "$results_dir/basic_data_analysis.ipynb" "$output_folder"
        cp "$file" "$output_folder"
        cp "query.q" "$output_folder"

        zip -r "$results_dir/$filename.zip" "$output_folder"
        echo "All results succesfully placed in $output_folder.zip"

        rm -rf "$output_folder"
        echo "Deleted $output_folder"

        printf "\n\n\n"
    done
else
    echo "No YAML files found in $config_dir. Running stompc.py without -cfg."

    for i in $(eval echo {1..$1}); do
        echo "Run $i with default config"
        python3 stompc.py

        echo "Sleeping for 10 seconds before next experiment"
        sleep 10
        
        printf "\n\n\n"
    done

    output_folder="$results_dir/default_cfg"
    mkdir -p "$output_folder"

    echo "Organizing results into $output_folder"

    # Move only result folders (excluding .zip and non-directory files)
    find "$results_dir" -mindepth 1 -maxdepth 1 -type d -not -name "default_cfg" -exec mv {} "$output_folder" \;

    # Copy the .ipynb file into the new folder
    cp "$results_dir"/*.ipynb "$output_folder"

    # Zip the folder
    zip -r "$results_dir/default_cfg" "$output_folder"

    echo "Results for 'default_cfg' have been organized and zipped."

    # Delete the folder after zipping
    rm -rf "$output_folder"

    echo "Deleted folder: $output_folder"
fi
